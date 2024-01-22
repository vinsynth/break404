#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;

use panic_halt as _;

use cortex_m::delay::Delay;

use rp2040_hal as hal;

use hal::clocks::{Clock, ClocksManager};
use hal::gpio::{
    DynPinId,
    FunctionSio,
    FunctionSioInput,
    FunctionSpi,
    Pin,
    PullDown,
    PullNone,
    PullUp,
    SioOutput,
};
use hal::pll::common_configs::PLL_USB_48MHZ;
use hal::pll::PLLConfig;
use hal::pwm;
use hal::gpio::bank0::*;
use hal::Spi;
use hal::spi::Enabled;

use hal::pac;
use pac::interrupt;
use pac::SPI1;

use embedded_hal::digital::v2::InputPin;
use embedded_hal::PwmPin;

use embedded_sdmmc::{
    File,
    Mode,
    SdCard,
    TimeSource,
    Timestamp,
    VolumeIdx,
    VolumeManager,
};

use fugit::{HertzU32, RateExtU32};

use critical_section::Mutex;
use core::cell::RefCell;
use alloc::collections::VecDeque;
use alloc::vec;
use alloc::boxed::Box;

use itertools::Itertools;

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

struct Grain {
    queue: VecDeque<u8>,
    upper_buf: Box<[u8]>,
    pub seek_to: Option<u32>,
}

impl Grain {
    pub fn new(samples: &[u8], upper_buf: Box<[u8]>) -> Self {
        Self {
            queue: VecDeque::from(samples.to_vec()),
            upper_buf,
            seek_to: None,
        }
    }

    pub fn is_empty(&self) -> bool {
        self.queue.is_empty()
    }

    pub fn take(&mut self) -> Option<u8> {
        self.queue.pop_front()
    }

    pub fn upper_buf(&self) -> Box<[u8]> {
        self.upper_buf.clone()
    }
}

type SdBlockDevice = SdCard<Spi<Enabled, SPI1, (
    Pin<Gpio11, FunctionSpi, PullNone>,
    Pin<Gpio12, FunctionSpi, PullUp>,
    Pin<Gpio10, FunctionSpi, PullNone>), 8>,
    Pin<Gpio13, FunctionSio<SioOutput>, PullDown>,
    Delay>;
type SdVolManager = VolumeManager<SdBlockDevice, DummyTimesource, 4, 4, 1>;

struct Grains<'a> {
    vol_mgr: &'a Mutex<RefCell<Option<SdVolManager>>>,
    file: File,
    grain: Grain,
    grain_len: usize,
    speed: f32,
}

impl<'a> Grains<'a> {
    pub fn new(vol_mgr: &'a Mutex<RefCell<Option<SdVolManager>>>, file: File, grain_len: usize, speed: f32) -> Self {
        if grain_len == 0 {
            crate::panic!("grain_len should be > 0");
        }
        Self {
            vol_mgr,
            file,
            grain: Grain::new(&[], Box::new([])),
            grain_len,
            speed,
        }
    }

    pub fn set_speed(&mut self, speed: f32) {
        self.speed = speed
    }

    pub fn seek_to(&mut self, pcm_index: u32) {
        self.grain.seek_to = Some(pcm_index % self.pcm_len());
        info!("seek_to: {}", pcm_index);
    }

    pub fn pcm_len(&self) -> u32 {
        critical_section::with(|cs| {
            self.vol_mgr.borrow_ref(cs).as_ref().unwrap().file_length(self.file).unwrap() -
                0x2c
        })
    }

    fn is_zero_crossing(sample_a: &u8, sample_b: &u8) -> bool {
        (u8::MAX / 2).cmp(sample_a) != (u8::MAX / 2).cmp(sample_b)
    }

    fn left(&self) -> u32 {
        critical_section::with(|cs| {
            self.vol_mgr.borrow_ref(cs).as_ref().unwrap().file_length(self.file).unwrap_or(0) -
                self.vol_mgr.borrow_ref(cs).as_ref().unwrap().file_offset(self.file).unwrap_or(0)
        })
    }
}

impl<'a> Iterator for Grains<'a> {
    type Item = u8;

    // TODO: investigate slow audio
    // - likely due to vol_mgr.read() being slow
    // TODO: fix crash
    fn next(&mut self) -> Option<u8> {
        // next grain
        while self.grain.is_empty() {
            critical_section::with(|cs| {
                if let Some(i) = self.grain.seek_to {
                    self.vol_mgr.borrow_ref_mut(cs).as_mut().unwrap().file_seek_from_start(self.file, 0x2c + i);
                }
                if self.vol_mgr.borrow_ref(cs).as_ref().unwrap().file_offset(self.file).unwrap() < 0x2c ||
                    self.vol_mgr.borrow_ref(cs).as_ref().unwrap().file_eof(self.file).unwrap()
                {
                    self.vol_mgr.borrow_ref_mut(cs).as_mut().unwrap().file_seek_from_start(self.file, 0x2c);
                };
            });

            let lower_buf = if self.grain.upper_buf().is_empty() {
                info!("new lower buf!");
                let mut buf = vec![
                    0;
                    self.grain_len.min(self.left() as usize)
                ].into_boxed_slice();
                critical_section::with(|cs| {
                    self.vol_mgr.borrow_ref_mut(cs).as_mut().unwrap().read(self.file, &mut buf);
                });
                buf
            } else {
                self.grain.upper_buf()
            };
            let lower_xing = lower_buf
                .iter()
                .rev()
                .tuple_windows::<(_, _)>()
                .enumerate()
                .find_or_last(|(_, (a, b))| Self::is_zero_crossing(a, b))
                .map(|(i, _)| lower_buf.len() - i)
                .unwrap_or(lower_buf.len());

            let mut upper_buf = vec![
                0;
                self.grain_len.min(self.left() as usize)
            ].into_boxed_slice();
            critical_section::with(|cs| {
                self.vol_mgr.borrow_ref_mut(cs).as_mut().unwrap().read(self.file, &mut upper_buf);
            });
            let upper_xing = upper_buf
                .iter()
                .tuple_windows::<(_, _)>()
                .enumerate()
                .find_or_last(|(_, (a, b))| Self::is_zero_crossing(a, b))
                .map(|(i, _)| upper_buf.len() + i)
                .unwrap_or(upper_buf.len());

            let lower_diff = (lower_buf.len()).abs_diff(lower_xing);
            let upper_diff = (upper_buf.len()).abs_diff(upper_xing);
            let xing = if upper_diff < lower_diff {
                upper_xing
            } else {
                lower_xing
            };

            let count = if self.speed == 0. {
                1
            } else {
                ((self.left() as usize / self.grain_len + 1) as f32 / self.speed) as usize -
                    ((self.left() as usize / self.grain_len) as f32 / self.speed) as usize
            };

            let samples = [lower_buf, upper_buf].concat();
            let lower_samples = samples.get(..xing).unwrap_or(&[]);
            let upper_samples = samples.get(xing..).unwrap_or(&[]);
            let samples = lower_samples.repeat(count).into_boxed_slice();
            // info!("left: {}, count: {}, s_len: {}, l_len: {}, u_len: {}",
            //     self.left(),
            //     count,
            //     samples.len(),
            //     lower_samples.len(),
            //     upper_samples.len()
            // );

            self.grain = Grain::new(
                &samples,
                if self.speed == 0. { lower_samples.into() } else { upper_samples.into() }
            );
        }
        self.grain.take()
    }
}

#[derive(Default)]
pub struct DummyTimesource();

impl TimeSource for DummyTimesource {
    fn get_timestamp(&self) -> Timestamp {
        Timestamp {
            year_since_1970: 0,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

static GRAINS: Mutex<RefCell<Option<Grains>>> = Mutex::new(RefCell::new(None));
static VOL_MGR: Mutex<RefCell<Option<SdVolManager>>> = Mutex::new(RefCell::new(None));

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

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

extern crate alloc;
use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

#[rp2040_hal::entry]
fn main() -> ! {
    // init heap
    info!("init heap...");
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 250000;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }
    
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
        xosc.operating_frequency(),
        PLL_SYS_110MHZ,
        &mut clocks,
        &mut pac.RESETS,
    )
    .unwrap();
    let pll_usb = hal::pll::setup_pll_blocking(
        pac.PLL_USB,
        xosc.operating_frequency(),
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

    // init spi
    info!("init spi...");
    let spi_sclk: Pin<_, FunctionSpi, PullNone> = pins.gpio10.reconfigure();
    let spi_mosi: Pin<_, FunctionSpi, PullNone> = pins.gpio11.reconfigure();
    let spi_miso: Pin<_, FunctionSpi, PullUp> = pins.gpio12.reconfigure();
    let spi_cs = pins.gpio13.into_push_pull_output();

    let spi = hal::spi::Spi::<_, _, _, 8>::new(pac.SPI1, (spi_mosi, spi_miso, spi_sclk));

    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        400.kHz(),
        embedded_hal::spi::MODE_0,
    );

    let sdcard = SdCard::new(spi, spi_cs, delay);
    let mut vol_mgr = VolumeManager::new(sdcard, DummyTimesource::default());

    match vol_mgr.device().num_bytes() {
        Ok(size) => info!("card size is {} bytes", size),
        Err(e) => crate::panic!("failed to retrieve card size: {}", Debug2Format(&e)),
    }
    vol_mgr
        .device()
        .spi(|spi| spi.set_baudrate(clocks.peripheral_clock.freq(), 16.MHz()));

    let volume = match vol_mgr.open_volume(VolumeIdx(0)) {
        Ok(v) => v,
        Err(e) => crate::panic!("failed to open volume 0: {}", Debug2Format(&e)),
    };
    let root_dir = match vol_mgr.open_root_dir(volume) {
        Ok(d) => d,
        Err(e) => crate::panic!("failed to open root directory: {}", Debug2Format(&e)),
    };
    let file = match vol_mgr.open_file_in_dir(root_dir, "funkier.wav", Mode::ReadOnly) {
        Ok(f) => f,
        Err(e) => crate::panic!("failed to open file: {}", Debug2Format(&e)),
    };

    critical_section::with(|cs| {
        VOL_MGR.borrow_ref_mut(cs).replace(vol_mgr);
    });

    critical_section::with(|cs| {
        GRAINS
            .borrow_ref_mut(cs)
            .replace(Grains::new(
                &VOL_MGR,
                file,
                1024,
                1.
            ));
    });

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
            (pwm.get_top() as u32 + 1)
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
        .round_robin((&mut joy_x_pin, &mut joy_y_pin))
        .start();

    let joy_c_pin = pins.gpio22.into_pull_up_input();

    // within 0..4096
    let (mut joy_x, mut joy_y) = (0, 0);

    info!("loop start!");
    loop {
        // sync inputs
        let (joy_x_was, joy_y_was) = (joy_x, joy_y);
        if adc_fifo.len() > 1 {
            joy_x = adc_fifo.read();
            joy_y = adc_fifo.read();
        }
        let joy_c_down = joy_c_pin.is_low().unwrap();

        if joy_x != joy_x_was {
            let speed = if joy_x < 100 {
                0.
            } else if joy_x < 1000 {
                libm::cbrtf(joy_x as f32 / 2047.)
            } else if joy_x > 2080 {
                2. * libm::cbrtf(joy_x as f32 / 2047. - 2.) + 3.
            } else {
                1.
            };
            critical_section::with(|cs| {
                GRAINS.borrow_ref_mut(cs).as_mut().unwrap().set_speed(speed);
            });
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
    if let Some(pwm) = PWMOUT_SNGL {
        critical_section::with(|cs| {
            pwm.channel_a.set_duty(((GRAINS
                .borrow_ref_mut(cs)
                .as_mut()
                .expect("failed  to get ref_mut for grains")
                .next()
                .unwrap_or(u8::MAX / 2) as u16) << 4) &0xfff
            );
        });
        pwm.clear_interrupt();
    }
}

// end of file
