#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;

use libm::sqrtf;
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

use fugit::{HertzU32, RateExtU32};

use critical_section::Mutex;
use core::cell::{Cell, RefCell};
use alloc::collections::VecDeque;

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

// struct Grain<'a> {
//     samples: &'a [u8],
//     queue: VecDeque<u8>,
//     len: usize,
//     end: usize,
// }

// impl<'a> Grain<'a> {
//     pub fn new(samples: &'a [u8], len: usize, end: usize) -> Self {
//         let queue = VecDeque::from(samples.clone().to_vec());
//         Self {
//             samples,
//             queue,
//             len,
//             end,
//         }
//     }

//     pub fn count(&self) -> usize {
//         self.queue.len()
//     }

//     pub fn len(&self) -> usize {
//         self.len.min(self.samples.len())
//     }

//     pub fn take_raw(&mut self) -> Option<u8> {
//         self.queue.pop_front()
//     }

//     pub fn take_mixed(&mut self, other: Option<&mut Grain>) -> Option<u8> {
//         if let Some(s) = self.take_raw() {
//             if let Some(other) = other {
//                 if let Some(o) = other.take_raw() {
//                     //                  pi(self.len()-self.queue.len())
//                     // 1/2(o-s+(o-s)cos(-------------------------------))
//                     //                  min(self.len(), other.len())
//                     // Some(
//                     //     ((o + s) as f32 + (o - s) as f32 * libm::cosf(
//                     //         core::f32::consts::PI * (self.len() - self.count()) as f32 /
//                     //             self.len().min(other.len()) as f32
//                     //     )) as u8 / 2
//                     // );
//                     let s = s as f32;
//                     let o = o as f32;
//                     return Some((o + (o - s) / self.len().min(other.len()) as f32 * (self.len() as f32 - self.count() as f32)) as u8);
//                 }
//             }
//             return Some(s);
//         }
//         None
//         // self.take_raw()
//     }
// }

// struct Grains<'a> {
//     samples: &'a [u8],
//     grains: ConstGenericRingBuffer<Grain<'a>, 2>,
//     grain_len: usize,
//     smear: f32,
//     speed: f32,
// }

// impl<'a> Grains<'a> {
//     pub fn new(samples: &'a [u8], grain_len: usize) -> Self {
//         let speed = 1.;
//         let mut smear = 1_f32;
//         let mut grains = ConstGenericRingBuffer::new();
//         grains.push(Grain::new(&samples[..grain_len], grain_len, grain_len));
//         grains.push(Grain::new(&samples[grain_len..2 * grain_len], grain_len, 2 * grain_len));
//         Self {
//             samples,
//             grains,
//             grain_len,
//             smear: smear.clamp(0., 1.),
//             speed,
//         }
//     }
// }

// impl<'a> Iterator for Grains<'a> {
//     type Item = u8;

//     fn next(&mut self) -> Option<u8> {
//         if let Some(b) = self.grains.back() {
//             let smear_len = (self.grain_len as f32 / self.speed * self.smear) as usize;
//             if b.count() < smear_len {
//                 let grain_end = b.end + self.grain_len;
//                 let smear_end = b.end + (self.grain_len as f32 / self.speed) as usize + smear_len;
//                 self.grains.push(Grain::new(&self.samples[b.end..smear_end], self.grain_len, grain_end));
//             }
//         }
//         let mut g = self.grains.iter_mut();
//         g.next_back().and_then(|b| b.take_mixed(g.next_back()))
//     }
// }

struct Grain {
    queue: VecDeque<u8>,
    n: usize
}

impl Grain {
    pub fn new(samples: &[u8], n: usize) -> Self {
        Self {
            queue: VecDeque::from(samples.to_vec()),
            n
        }
    }

    pub fn len(&self) -> usize {
        self.queue.len()
    }

    pub fn n(&self) -> usize {
        self.n
    }

    pub fn take(&mut self) -> Option<u8> {
        self.queue.pop_front()
    }
}

struct Grains<'a> {
    samples: &'a [u8],
    grain: Grain,
    grain_len: usize,
    speed: f32,
}

impl<'a> Grains<'a> {
    pub fn new(samples: &'a [u8], grain_len: usize, speed: f32) -> Self {
        if grain_len == 0 {
            crate::panic!("grain_len should be > 0");
        }
        Self {
            samples,
            grain: Grain::new(&[], 0),
            grain_len,
            speed,
        }
    }

    pub fn set_speed(&mut self, speed: f32) {
        self.speed = speed
    }

    fn is_zero_crossing(sample_a: &u8, sample_b: &u8) -> bool {
        (u8::MAX / 2).cmp(sample_a) != (u8::MAX / 2).cmp(sample_b)
    }
}

impl<'a> Iterator for Grains<'a> {
    type Item = u8;

    fn next(&mut self) -> Option<u8> {
        // next grain
        while self.grain.len() == 0 {
            let n = self.grain.n() % (self.samples.len() / self.grain_len);
            let start = n * self.grain_len;

            // let end = if self.speed == 0. {
            //     self.samples.get(start..)
            //         .and_then(|s| s
            //             .iter()
            //             .tuple_windows::<(_, _)>()
            //             .enumerate()
            //             .find_or_last(|(_, (a, b))| Self::is_zero_crossing(a, b))
            //             .map(|(i, _)| start + i)
            //         ).unwrap_or(self.samples.len())
            let grain_len = if self.speed == 0. {
                self.grain_len
            } else {
                (self.grain_len as f32 / self.speed) as usize
            };

            let upper = self.samples.get(start + grain_len..)
                .and_then(|s| s
                    .iter()
                    .tuple_windows::<(_, _)>()
                    .enumerate()
                    .find_or_last(|(_, (a, b))| Self::is_zero_crossing(a, b))
                    .map(|(i, _)| start + grain_len + i)
                ).unwrap_or(self.samples.len());
            let lower = self.samples.get(start..start + grain_len)
                .and_then(|s| s
                    .iter()
                    .rev()
                    .tuple_windows::<(_, _)>()
                    .enumerate()
                    .find_or_last(|(_, (a, b))| Self::is_zero_crossing(a, b))
                    .map(|(i, _)| start + grain_len - i)
                ).unwrap_or(upper);

            let u_diff = (start + grain_len).abs_diff(upper);
            let l_diff = (start + grain_len).abs_diff(lower);
            let end = if u_diff < l_diff {
                upper
            } else {
                lower
            };

            // let r = ((n + 1) as f32 / self.speed) as usize -
            //      (n as f32 / self.speed) as usize;

            self.grain = Grain::new(
                &self.samples[start..end],
                if self.speed == 0. { n } else { n + 1 }
            );
        }
        // next sample
        self.grain.take()
    }
}

trait SliceExt {
    fn grains(&self, grain_len: usize, speed: f32) -> Grains<'_>;
}

impl SliceExt for [u8] {
    fn grains(&self, grain_len: usize, speed: f32) -> Grains<'_> {
        Grains::new(self, grain_len, speed)
    }
}

static BREAK_BYTES: &[u8] = include_bytes!("../assets/lc_mono8.wav");
static GRAINS: Mutex<RefCell<Option<Grains>>> = Mutex::new(RefCell::new(None));

static FREE_CURSOR: Mutex<Cell<usize>> = Mutex::new(Cell::new(0));
static MOD_CURSOR: Mutex<Cell<Option<usize>>> = Mutex::new(Cell::new(None));
static SPEED: Mutex<Cell<usize>> = Mutex::new(Cell::new(1));

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

extern crate alloc;
use alloc::vec::Vec;
use alloc::vec;
use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

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
    let mut joy_x_was = joy_x;
    let mut joy_y_was = joy_y;

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
                    GRAINS.
                        borrow_ref_mut(cs)
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
    critical_section::with(|cs| {
        if GRAINS.borrow_ref(cs).is_none() {
            GRAINS
                .borrow_ref_mut(cs)
                .replace(BREAK_BYTES[0x2c..].grains(1024, 1.));
        }
    });
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
        // critical_section::with(|cs| {
            // precalculate number grains
            // if GRAIN_N.borrow(cs).get().is_none() {
            //     let mut cursor = 0;
            //     'l: loop {
            //         GRAIN_N.borrow(cs).set(GRAIN_N.borrow(cs).get());
            //         while !zero_crossed(0x2c + cursor) {
            //             cursor += 1;
            //             if cursor >= BREAK_BYTES.len() - 1 {
            //                 break 'l
            //             }
            //         }
            //     }
            // }

            // let cursor = MOD_CURSOR.borrow(cs).get().unwrap_or(
            //     FREE_CURSOR.borrow(cs).get()
            // );

            // calculate current grain
            // if zero_crossing(cursor) {
                // GRAIN.borrow(cs).get_mut().start = cursor;
                // GRAIN.borrow(cs).get_mut().end = cursor + 1;
                // while !zero_crossing(0x2c + GRAIN.borrow(cs).get().end) {
                    // GRAIN.borrow(cs).get_mut().end = GRAIN.borrow(cs).get().end;
                // }
            // }

            // pwm.channel_a.set_duty(
            //     ((BREAK_BYTES[0x2c + cursor] as u16) << 4) & 0xfff
            // );
            // pwm.channel_b.set_duty(
            //     ((BREAK_BYTES[0x2c + cursor + 1] as u16) << 4) & 0xfff
            // );
            // FREE_CURSOR.borrow(cs).set((FREE_CURSOR.borrow(cs).get() + 2) % BREAK_BYTES[0x2c..].len());

            // if cursor >= GRAIN.borrow(cs).get().end {
            //     FREE_CURSOR.borrow(cs).set(GRAIN.borrow(cs).get().start);
            // }
            // if FREE_CURSOR.borrow(cs).get() >= GRAIN_END.borrow(cs).get() {
            //         FREE_CURSOR.borrow(cs).set(GRAIN_START.borrow(cs).get());
            //         GRAIN_N.borrow(cs).set(GRAIN_N.borrow(cs).get() + 1);

            //         GRAIN_END.borrow(cs).set(usize::MAX);
            // }
            // if MOD_CURSOR.borrow(cs).get().is_some() {
            //     MOD_CURSOR.borrow(cs).set(
            //         Some((cursor + 2) % BREAK_BYTES[0x2c..].len())
            //     );
            // }
        // });
}

// end of file
