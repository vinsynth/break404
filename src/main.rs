#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;

use panic_halt as _;

use rp2040_hal as hal;

use hal::clocks::{Clock, ClocksManager};
use hal::pll::common_configs::PLL_USB_48MHZ;
use hal::pll::PLLConfig;
use hal::pwm;

use hal::pac;
use pac::interrupt;

use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use embedded_hal::PwmPin;

use core::cell::RefCell;
use critical_section::Mutex;

use fugit::{HertzU32, RateExtU32};
use byteorder::{ByteOrder, LittleEndian};

fn blink_binary(
    pin: &mut dyn embedded_hal::digital::v2::OutputPin<Error = core::convert::Infallible>,
    delay: &mut dyn DelayMs<u32>,
    val: u32,
) {
    delay.delay_ms(1200);
    for b in (0..=31).rev() {
        if (val & (1 << b)) >> b == 1 {
            pin.set_high().unwrap();
            delay.delay_ms(1000);
            pin.set_low().unwrap();
            delay.delay_ms(200);
        } else {
            pin.set_high().unwrap();
            delay.delay_ms(200);
            pin.set_low().unwrap();
            delay.delay_ms(1000);
        }
    }
    delay.delay_ms(1200);
}

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

const XTAL_FREQ_HZ: u32 = 12_000_000u32;
// 110.25 MHz clock, multiple of 44.1 kHz sample rate
const PLL_SYS_110MHZ: PLLConfig = PLLConfig {
    vco_freq: HertzU32::MHz(882),
    refdiv: 2,
    post_div1: 4,
    post_div2: 2,
};

type PwmOut = pwm::Slice<pwm::Pwm0, pwm::FreeRunning>;
static PWMOUT: Mutex<RefCell<Option<PwmOut>>> = Mutex::new(RefCell::new(None));

static BREAK_BYTES: &[u8] = include_bytes!("../assets/lc8.wav");

#[rp2040_hal::entry]
fn main() -> ! {
    // let mut lc = [0; BREAK_BYTES.len() / 2 - 0x2c];
    // LittleEndian::read_u16_into(&BREAK_BYTES[0x2c..], &mut lc);
    info!("program start!");

    // init clocks
    info!("init clocks...");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    let xosc = hal::xosc::setup_xosc_blocking(pac.XOSC, XTAL_FREQ_HZ.Hz())
        .unwrap();
    watchdog.enable_tick_generation((XTAL_FREQ_HZ / 1_000_000) as u8);

    let mut clocks = ClocksManager::new(pac.CLOCKS);
    let pll_sys = hal::pll::setup_pll_blocking(
        pac.PLL_SYS,
        xosc.operating_frequency().into(),
        PLL_SYS_110MHZ,
        &mut clocks,
        &mut pac.RESETS,
    )
    .unwrap();
    let pll_usb = hal::pll::setup_pll_blocking(
        pac.PLL_USB,
        xosc.operating_frequency().into(),
        PLL_USB_48MHZ,
        &mut clocks,
        &mut pac.RESETS,
    )
    .unwrap();

    clocks
        .init_default(&xosc, &pll_sys, &pll_usb)
        .unwrap();
    let mut delay = cortex_m::delay::Delay::new(
        core.SYST, clocks.system_clock.freq().to_Hz()
    );

    info!("sysclk freq: {} Hz", clocks.system_clock.freq().to_Hz());

    // init pins
    info!("init pins...");
    let sio = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let mut led_pin = pins.gpio25.into_push_pull_output();

    // init pwm
    info!("init pwm...");
    let pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    // fPWM = fSYS / ((TOP + 1) * (CSR_PH_CORRECT + 1) * (DIV_INT + (DIV_FRAC / 16)))
    // 44.1 kHz = 110_250_000 Hz / ((2499 + 1) * (0 + 1) * (1 + (0 / 16)))
    let mut pwm: pwm::Slice<_, pwm::FreeRunning> = pwm_slices.pwm0.into_mode();
    pwm.default_config();
    pwm.set_top(2499);
    pwm.enable_interrupt();
    pwm.enable();

    info!(
        "pwm freq: {} Hz",
        clocks.system_clock.freq().to_Hz() / ((pwm.get_top() as u32 + 1) * (0 + 1) * (1 + (0 / 16)))
    );

    pwm.channel_a.output_to(pins.gpio16); // left channel
    pwm.channel_b.output_to(pins.gpio17); // right channel

    critical_section::with(|cs| {
        PWMOUT.borrow(cs).replace(Some(pwm));
    });
    unsafe { pac::NVIC::unmask(pac::Interrupt::PWM_IRQ_WRAP) };

    info!("loop start!");
    loop {
        led_pin.set_high().unwrap();
        delay.delay_ms(200);
        led_pin.set_low().unwrap();
        delay.delay_ms(200);
    }
}

#[interrupt]
fn PWM_IRQ_WRAP() {
    static mut PWMOUT_SNGL: Option<PwmOut> = None;
    static mut BREAK_ITER: Option<core::iter::Cycle<core::slice::Iter<'_, u8>>> =
        None;

    if PWMOUT_SNGL.is_none() {
        critical_section::with(|cs| {
            *PWMOUT_SNGL = PWMOUT.borrow(cs).take();
        });
    }
    if BREAK_ITER.is_none() {
        *BREAK_ITER = Some(BREAK_BYTES.iter().cycle());
    }

    if let (Some(pwm), Some(break_iter)) = (PWMOUT_SNGL, BREAK_ITER) {
        let val_a = ((*break_iter.next().unwrap() as u16) << 4) & 0xFFF;
        let val_b = ((*break_iter.next().unwrap() as u16) << 4) & 0xFFF;
        pwm.channel_a.set_duty(val_a);
        pwm.channel_b.set_duty(val_b);
        pwm.clear_interrupt();
    }
}

// end of file