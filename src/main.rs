#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;

use panic_halt as _;

use rp2040_hal as hal;

use hal::clocks::{Clock, ClocksManager};
use hal::gpio::{
    DynPinId,
    FunctionSioInput,
    Pin,
    PullUp,
};
use hal::pll::common_configs::PLL_USB_48MHZ;
use hal::pll::PLLConfig;
use hal::pwm;

use hal::pac;
use pac::interrupt;

use embedded_hal::digital::v2::InputPin;
use embedded_hal::PwmPin;

use core::cell::{Cell, RefCell};
use critical_section::Mutex;

use fugit::{HertzU32, RateExtU32};

// mod sequencer;

#[derive(Debug, PartialEq)]
enum SequencerState {
    Free,
    Retrig { to: usize, from: usize },
    Hold { to: usize },
}

#[derive(Debug, PartialEq)]
enum SequencerTrans {
    Trig { to: usize, from: usize },
    Jump { to: usize },
    Return,
    Resync,
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

static FREE_CURSOR: Mutex<Cell<usize>> = Mutex::new(Cell::new(0));
static MOD_CURSOR: Mutex<Cell<Option<usize>>> = Mutex::new(Cell::new(None));

#[rp2040_hal::entry]
fn main() -> ! {
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
    let sio = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

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
        clocks.system_clock.freq().to_Hz() /
            ((pwm.get_top() as u32 + 1) * (0 + 1) * (1 + (0 / 16)))
    );

    pwm.channel_a.output_to(pins.gpio16); // left channel
    pwm.channel_b.output_to(pins.gpio17); // right channel

    critical_section::with(|cs| {
        PWMOUT.borrow(cs).replace(Some(pwm));
    });
    unsafe { pac::NVIC::unmask(pac::Interrupt::PWM_IRQ_WRAP) };

    // init inputs
    info!("init inputs...");
    // joystick
    let mut joy_x_pin = hal::adc::AdcPin::new(pins.gpio26.into_floating_input());
    let mut joy_y_pin = hal::adc::AdcPin::new(pins.gpio27.into_floating_input());

    let mut adc = hal::Adc::new(pac.ADC, &mut pac.RESETS);
    let mut adc_fifo = adc
        .build_fifo()
        .clock_divider(47999, 1)
        .set_channel(&mut joy_x_pin)
        .round_robin((&mut joy_y_pin, &mut joy_x_pin))
        .start();

    let joy_c_pin = pins.gpio22.into_pull_up_input();

    let mut joy_x = 0;
    let mut joy_y = 0;

    // init sequencer
    info!("init sequencer buttons...");
    // sequencer state machine:
    //     Free => Jump ..
    //     Return* => Jump ..
    //     Jump => Hold => Return => Free..
    //     Jump => HoldR => ReturnR => Free..
    //     once: Jump*, Return*
    //     loop: Free, Hold*
    let seq_pins: [Pin<DynPinId, FunctionSioInput, PullUp>; 8] = [
        pins.gpio9.reconfigure().into_dyn_pin(),
        pins.gpio8.reconfigure().into_dyn_pin(),
        pins.gpio7.reconfigure().into_dyn_pin(),
        pins.gpio6.reconfigure().into_dyn_pin(),
        pins.gpio5.reconfigure().into_dyn_pin(),
        pins.gpio4.reconfigure().into_dyn_pin(),
        pins.gpio3.reconfigure().into_dyn_pin(),
        pins.gpio2.reconfigure().into_dyn_pin(),
    ];

    let mut seq_downs = [false; 8];
    let mut seq_vec = tinyvec::array_vec!([u8; 16]);

    let mut seq_state: SequencerState = SequencerState::Free;
    let mut seq_trans: Option<SequencerTrans> = None;

    let break_len = BREAK_BYTES[0x2c..].len();
    let steps_len = 16;

    info!("loop start!");
    loop {
        // sync inputs
        if adc_fifo.len() > 1 {
            joy_x = adc_fifo.read();
            joy_y = adc_fifo.read();
        }
        let joy_c_down = joy_c_pin.is_low().unwrap();

        // sync sequencer buttons
        let seq_vec_was = seq_vec.clone();
        for i in 0..seq_pins.len() {
            if seq_pins[i].is_low().unwrap() && !seq_vec.contains(&(i as u8)) {
                seq_vec.push(i as u8);
            } else if seq_pins[i].is_high().unwrap() && seq_vec.contains(&(i as u8)) {
                seq_vec.retain(|&x| x != i as u8);
            }
            seq_downs[i] = seq_pins[i].is_low().unwrap();
        }

        // process sequencer
        if seq_vec_was != seq_vec {
            if let Some(&t) = seq_vec.first() {
                if seq_vec.len() > 1  {
                    // init retrig
                    let to = break_len / steps_len * t as usize;

                    let mut from = to;
                    seq_downs.rotate_left(t as usize);
                    for i in 1..seq_downs.len() {
                        if seq_downs[i] {
                            from += break_len / steps_len / 32 * 2usize.pow(i as u32);
                        }
                    }
                    info!("input retrig to {} from {}!", to, from);
                    seq_trans = Some(SequencerTrans::Trig { to, from });
                } else if seq_vec_was.is_empty() {
                    // init jump
                    let to = break_len / steps_len * t as usize;

                    match (&seq_state, &seq_trans) {
                        (_, &Some(SequencerTrans::Trig { .. })) => (),
                        // (&SequencerState::Hold { to: t }, _) if t == to => (),
                        _ => {
                            info!("input jump to {}!", to);
                            seq_trans = Some(SequencerTrans::Jump { to });
                        }
                    }
                }
            }
        } 

        match (&seq_state, &seq_trans) {
            (&SequencerState::Free, None) => (),
            (_, &Some(SequencerTrans::Trig { to, from })) => critical_section::with(|cs| {
                if FREE_CURSOR.borrow(cs).get() % (break_len / steps_len / 32) < 2 {
                    info!("init retrig to {} from {}!", to, from);
                    MOD_CURSOR.borrow(cs).set(Some(to));
                    seq_state = SequencerState::Retrig { to, from };
                    seq_trans = None;
                }
            }),
            (_, &Some(SequencerTrans::Jump { to })) => critical_section::with(|cs| {
                if FREE_CURSOR.borrow(cs).get() % (break_len / steps_len) < 2 {
                    info!("jump to {}!", to);
                    MOD_CURSOR.borrow(cs).set(Some(to));
                    seq_state = SequencerState::Hold { to };
                    seq_trans = None;
                }
            }),
            (&SequencerState::Retrig { to, from }, None) => {
                if seq_vec.get(1).is_none() {
                    info!("input return!");
                    seq_trans = Some(SequencerTrans::Return);
                } else {
                    critical_section::with(|cs| {
                        if let Some(cursor) = MOD_CURSOR.borrow(cs).get() {
                            if cursor > from ||
                                cursor < to &&
                                cursor > (from + to) % break_len
                            {
                                info!("retrig to {} from {}!", to, from);
                                MOD_CURSOR.borrow(cs).set(Some(to));
                            }
                        }
                    });
                }
            }
            (&SequencerState::Retrig { .. }, &Some(SequencerTrans::Return)) => critical_section::with(|cs| {
                if FREE_CURSOR.borrow(cs).get() % (break_len / steps_len / 32) < 2 {
                    info!("return from retrig!");
                    MOD_CURSOR.borrow(cs).set(None);
                    seq_state = SequencerState::Free;
                    seq_trans = None;
                }
            }),
            (&SequencerState::Hold { .. }, None) => {
                if seq_vec.is_empty() {
                    info!("input resync!");
                    seq_trans = Some(SequencerTrans::Resync);
                }
            }
            (&SequencerState::Hold { .. }, &Some(SequencerTrans::Resync)) => critical_section::with(|cs| {
                if FREE_CURSOR.borrow(cs).get() % (break_len / steps_len) < 2 {
                    info!("resync from jump!");
                    MOD_CURSOR.borrow(cs).set(None);
                    seq_state = SequencerState::Free;
                    seq_trans = None;
                }
            }),
            _ => (),
        }
    }
}

#[interrupt]
fn PWM_IRQ_WRAP() {
    static mut PWMOUT_SNGL: Option<PwmOut> = None;

    if PWMOUT_SNGL.is_none() {
        critical_section::with(|cs| {
            *PWMOUT_SNGL = PWMOUT.borrow(cs).take();
        });
    }

    if let Some(pwm)= PWMOUT_SNGL {
        critical_section::with(|cs| {
            let cursor_a = MOD_CURSOR.borrow(cs).get().unwrap_or(
                FREE_CURSOR.borrow(cs).get()
            );
            pwm.channel_a.set_duty(
                ((BREAK_BYTES[0x2c + cursor_a] as u16) << 4) & 0xfff
            );
            pwm.channel_b.set_duty(
                ((BREAK_BYTES[0x2c + cursor_a + 1] as u16) << 4) & 0xfff
            );
            FREE_CURSOR.borrow(cs).set((FREE_CURSOR.borrow(cs).get() + 2) % BREAK_BYTES[0x2c..].len());
            if MOD_CURSOR.borrow(cs).get().is_some() {
                MOD_CURSOR.borrow(cs).set(
                    Some((cursor_a + 2) % BREAK_BYTES[0x2c..].len())
                );
            }
        });
        pwm.clear_interrupt();
    }
}

// end of file
