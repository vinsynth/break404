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
use core::cell::{Cell, RefCell};
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
    u_buffer: Box<[u8]>,
    pub seek_to: Option<u32>,
}

impl Grain {
    pub fn new(samples: &[u8], u_buffer: Box<[u8]>) -> Self {
        Self {
            queue: VecDeque::from(samples.to_vec()),
            u_buffer,
            seek_to: None,
        }
    }

    pub fn is_empty(&self) -> bool {
        self.queue.is_empty()
    }

    pub fn take(&mut self) -> Option<u8> {
        self.queue.pop_front()
    }

    pub fn u_buffer(&self) -> Box<[u8]> {
        self.u_buffer.clone()
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

    fn next(&mut self) -> Option<u8> {
        // next grain
        while self.grain.is_empty() {
            if let Some(i) = self.grain.seek_to {
                critical_section::with(|cs| {
                    self.vol_mgr.borrow_ref_mut(cs).as_mut().unwrap().file_seek_from_start(self.file, 0x2c + i);
                });
            }
            critical_section::with(|cs| {
                if self.vol_mgr.borrow_ref(cs).as_ref().unwrap().file_eof(self.file).unwrap() {
                    self.vol_mgr.borrow_ref_mut(cs).as_mut().unwrap().file_seek_from_start(self.file, 0x2c);
                }
            });
            let grain_len = if self.speed == 0. {
                self.grain_len
            } else {
                (self.grain_len as f32 / self.speed) as usize
            };

            let l_len = grain_len.min(self.left() as usize);
            let l_buffer = if self.grain.u_buffer().is_empty() {
                let mut buf = vec![0; l_len].into_boxed_slice();
                critical_section::with(|cs| {
                    self.vol_mgr.borrow_ref_mut(cs).as_mut().unwrap().read(self.file, &mut buf);
                });
                buf
            } else {
                self.grain.u_buffer()
            };
            let lower = l_buffer
                    .iter()
                    .rev()
                    .tuple_windows::<(_, _)>()
                    .enumerate()
                    .find_or_last(|(_, (a, b))| Self::is_zero_crossing(a, b))
                    .map(|(i, _)| l_len - i)
                    .unwrap_or(l_len);

            let u_len = grain_len.min(self.left() as usize);
            let mut u_buffer = vec![0; u_len].into_boxed_slice();
            critical_section::with(|cs| {
                self.vol_mgr.borrow_ref_mut(cs).as_mut().unwrap().read(self.file, &mut u_buffer);
            });
            let upper = u_buffer
                    .iter()
                    .tuple_windows::<(_, _)>()
                    .enumerate()
                    .find_or_last(|(_, (a, b))| Self::is_zero_crossing(a, b))
                    .map(|(i, _)| u_len + i)
                    .unwrap_or(u_len);

            let u_diff = (grain_len).abs_diff(upper);
            let l_diff = (grain_len).abs_diff(lower);
            let end = if u_diff < l_diff {
                upper
            } else {
                lower
            };

            let samples = [l_buffer.clone(), u_buffer.clone()].concat();

            self.grain = Grain::new(
                samples.get(..end).unwrap_or(&[]),
                if self.speed == 0. { l_buffer } else { u_buffer }
            );
        }
        // next sample
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

static FREE_CURSOR: Mutex<Cell<usize>> = Mutex::new(Cell::new(0));
static MOD_CURSOR: Mutex<Cell<Option<usize>>> = Mutex::new(Cell::new(None));

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
    let file = match vol_mgr.open_file_in_dir(root_dir, "lc_mono8.wav", Mode::ReadOnly) {
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
        .set_channel(&mut joy_x_pin)
        .round_robin((&mut joy_y_pin, &mut joy_x_pin))
        .start();

    let joy_c_pin = pins.gpio22.into_pull_up_input();

    // within [0..4096]
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
        pins.gpio2.reconfigure().into_dyn_pin(),
        pins.gpio3.reconfigure().into_dyn_pin(),
        pins.gpio4.reconfigure().into_dyn_pin(),
        pins.gpio5.reconfigure().into_dyn_pin(),
        pins.gpio6.reconfigure().into_dyn_pin(),
        pins.gpio7.reconfigure().into_dyn_pin(),
        pins.gpio8.reconfigure().into_dyn_pin(),
        pins.gpio9.reconfigure().into_dyn_pin(),
    ];

    let mut seq_downs = [false; 8];
    let mut seq_vec = tinyvec::array_vec!([u8; 8]);

    let mut seq_state: SequencerState = SequencerState::Free;
    let mut seq_trans: Option<SequencerTrans> = None;

    let break_len = 16;
    // let break_len = BREAK_BYTES[0x2c..].len();
    let steps_len = 16;

    info!("loop start!");
    loop {
        // sync inputs
        let joy_x_was = joy_x;
        let joy_y_was = joy_y;
        if adc_fifo.len() > 1 {
            joy_x = adc_fifo.read();
            joy_y = adc_fifo.read();
        }
        let joy_c_down = joy_c_pin.is_low().unwrap();

        // sync sequencer buttons
        let seq_vec_was = seq_vec;
        for i in 0..seq_pins.len() {
            if seq_pins[i].is_low().unwrap() && !seq_vec.contains(&(i as u8)) {
                seq_vec.push(i as u8);
            } else if seq_pins[i].is_high().unwrap() && seq_vec.contains(&(i as u8)) {
                seq_vec.retain(|&x| x != i as u8);
            }
            seq_downs[i] = seq_pins[i].is_low().unwrap();
        }

        if joy_x != joy_x_was {
            let speed = if joy_x < 100 {
                0.
            } else if joy_x < 1900 {
                libm::cbrtf(joy_x as f32 / 2047.)
            } else if joy_x > 2080 {
                2. * libm::cbrtf(joy_x as f32 / 2047. - 2.) + 3.
            } else {
                1.
            };
            critical_section::with(|cs| {
                if GRAINS.borrow_ref(cs).is_some() {
                    GRAINS
                        .borrow_ref_mut(cs)
                        .as_mut()
                        .expect("failed to take grains as mut")
                        .set_speed(speed);
                }
            });
        }

        // process sequencer
        if seq_vec_was != seq_vec {
            if let Some(&t) = seq_vec.first() {
                if seq_vec.len() > 1  {
                    // init retrig
                    let to = break_len / steps_len * t as usize;

                    let mut from = to;
                    seq_downs.rotate_left(t as usize);
                    for (i, &d) in seq_downs.iter().enumerate().skip(1) {
                        if d {
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
        if !seq_vec.is_empty() && seq_vec_was.is_empty() {
            critical_section::with(|cs| {
                if GRAINS.borrow_ref(cs).is_some() {
                    let len = GRAINS
                        .borrow_ref(cs)
                        .as_ref()
                        .map_or(0, |g| g.pcm_len());
                    GRAINS
                        .borrow_ref_mut(cs)
                        .as_mut()
                        .expect("failed to take grains as mut")
                        .seek_to(len / 2);
                }
            });
        }

        match (&seq_state, &seq_trans) {
            (&SequencerState::Free, None) => (),
            (_, &Some(SequencerTrans::Trig { mut to, mut from })) => critical_section::with(|cs| {
                if FREE_CURSOR.borrow(cs).get() % (break_len / steps_len / 32) < 2 {
                    if FREE_CURSOR.borrow(cs).get() > break_len / 2 {
                        to += break_len / 2;
                        from += break_len / 2;
                    }
                    info!("init retrig to {} from {}!", to, from);
                    MOD_CURSOR.borrow(cs).set(Some(to));
                    seq_state = SequencerState::Retrig { to, from };
                    seq_trans = None;
                }
            }),
            (_, &Some(SequencerTrans::Jump { mut to })) => critical_section::with(|cs| {
                if FREE_CURSOR.borrow(cs).get() % (break_len / steps_len) < 2 {
                    if FREE_CURSOR.borrow(cs).get() > break_len / 2 {
                        to += break_len / 2;
                    }
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
