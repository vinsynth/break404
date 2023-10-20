#![no_std]
#![no_main]

use defmt::*;
use defmt::panic;
use defmt_rtt as _;

use hal::dma::SingleChannel;
use panic_halt as _;

use rp2040_hal as hal;

use hal::clocks::{Clock, ClocksManager};
use hal::gpio::{
    DynPinId,
    FunctionSioInput,
    FunctionSpi,
    Pin,
    PullDown,
    PullNone,
    PullUp,
};
use hal::pll::common_configs::PLL_USB_48MHZ;
use hal::pll::PLLConfig;
use hal::pwm;

use hal::pac;
use pac::interrupt;

use embedded_hal::digital::v2::InputPin;
use embedded_hal::PwmPin;

use embedded_sdmmc::{
    SdCard,
    TimeSource,
    Timestamp,
    VolumeIdx,
    VolumeManager,
};
use embedded_sdmmc::filesystem;

use core::cell::{Cell, RefCell};
use critical_section::Mutex;

use fugit::{HertzU32, RateExtU32};

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

#[derive(Debug, PartialEq)]
enum CursorState {
    Free,
    Jump,
    Retrig,
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
static AUDIO: Mutex<RefCell<Option<core::iter::Cycle<core::slice::Iter<'_, u8>>>>> =
    Mutex::new(RefCell::new(None));
// static STREAM: Mutex<Option<core::iter::Cycle<core::iter::Iterator::Iter<(dyn Iterator + 'static), u8>>>> = Mutex::new(None);

static FREE_CURSOR: Mutex<Cell<usize>> = Mutex::new(Cell::new(0));
static MOD_CURSOR: Mutex<Cell<Option<usize>>> = Mutex::new(Cell::new(None));
// static JUMP_CURSOR: Mutex<Cell<Option<usize>>> = Mutex::new(Cell::new(None));
// static RETRIG_CURSOR: Mutex<Cell<Option<usize>>> = Mutex::new(Cell::new(None));

extern crate alloc;
use alloc::vec::Vec;
use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

#[rp2040_hal::entry]
fn main() -> ! {
    // init allocator
    info!("init allocator...");
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 0x1000;
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

    // init spi pins
    info!("init spi pins...");
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
        Err(e) => panic!("error retrieving card size: {}", Debug2Format(&e)),
    }

    vol_mgr
        .device()
        .spi(|spi| spi.set_baudrate(clocks.peripheral_clock.freq(), 16.MHz()));

    let mut vol0 = match vol_mgr.get_volume(VolumeIdx(0)) {
        Ok(v) => v,
        Err(e) => panic!("error getting volume 0: {}", Debug2Format(&e)),
    };
    let root = match vol_mgr.open_root_dir(&vol0) {
        Ok(dir) => dir,
        Err(e) => panic!("error opening root directory: {}", Debug2Format(&e)),
    };

    let mut dirs = Vec::new();
    vol_mgr
        .iterate_dir(&vol0, &root, |ent| {
            if ent.attributes.is_directory() {
                dirs.push(
                    alloc::string::String::from_utf8(
                        ent.name.base_name().to_vec()
                    ).unwrap()
                );
            }
        })
        .ok();
    let dir_set0 = match vol_mgr.open_dir(&vol0, &root, "set0") {
        Ok(dir) => dir,
        Err(e) => panic!("error opening `set0` directory: {}", Debug2Format(&e)),
    };

    vol_mgr
        .iterate_dir(&vol0, &dir_set0, |ent| {
            info!(
                "/{}.{}",
                core::str::from_utf8(ent.name.base_name()).unwrap(),
                core::str::from_utf8(ent.name.extension()).unwrap()
            );
        })
        .ok();
    if let Ok(mut file) = vol_mgr.open_file_in_dir(&mut vol0, &dir_set0, "lc8.wav", filesystem::Mode::ReadOnly) {
        let mut buf = [0u8; 32];
        // critical_section::with(|cs| {
        //     let buf = fixed_slice_vec::FixedSliceVec::new();
        //     if let Ok(mut audio) = AUDIO.borrow(cs).try_borrow_mut() {
        //         vol_mgr.read(&vol0, &mut file, &mut AUDIO).iter().cycle();
        //     }
        // });
        let read_count = vol_mgr.read(&vol0, &mut file, &mut buf).unwrap();
        vol_mgr.close_file(&vol0, file).unwrap();

        info!("read {} bytes: {}", read_count, buf);
    }
    vol_mgr.free();

    // init AUDIO
    critical_section::with(|cs| {
        if let Ok(mut audio) = AUDIO.borrow(cs).try_borrow_mut() {
            audio.replace(BREAK_BYTES.iter().cycle());
        }
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
            ((pwm.get_top() as u32 + 1) * (0 + 1) * (1 + (0 / 16)))
    );

    pwm.channel_a.output_to(pins.gpio16); // left channel
    pwm.channel_b.output_to(pins.gpio17); // right channel

    critical_section::with(|cs| {
        PWMOUT.borrow(cs).replace(Some(pwm));
    });
    unsafe { pac::NVIC::unmask(pac::Interrupt::PWM_IRQ_WRAP) };

    // init sequencer
    info!("init sequencer buttons...");
    let seq_pins: [Pin<DynPinId, FunctionSioInput, PullDown>; 4] = [
        pins.gpio18.reconfigure().into_dyn_pin(),
        pins.gpio19.reconfigure().into_dyn_pin(),
        pins.gpio20.reconfigure().into_dyn_pin(),
        pins.gpio21.reconfigure().into_dyn_pin(),
    ];

    let mut seq_highs = [false; 4];

    let mut seq_vec = tinyvec::array_vec!([u8; 16]);
    let mut cursor_state = CursorState::Free;

    let break_len = BREAK_BYTES[0x2c..].len();

    info!("loop start!");
    loop {
        // sync sequencer buttons
        for i in 0..seq_pins.len() {
            if seq_pins[i].is_high().unwrap() && !seq_vec.contains(&(i as u8)) {
                info!("push {}", i);
                seq_vec.push(i as u8);
            } else if seq_pins[i].is_low().unwrap() && seq_vec.contains(&(i as u8)) {
                info!("pop {}", i);
                seq_vec.retain(|&x| x != i as u8);
            }
            seq_highs[i] = seq_pins[i].is_high().unwrap();
        }

        // process sequencer
        if let Some(&j) = seq_vec.first() {
            // process retrigger
            if seq_vec.len() > 1 {
                cursor_state = CursorState::Retrig;
                let retrig_to = break_len / 4 * j as usize;

                critical_section::with(|cs| {
                    if MOD_CURSOR.borrow(cs).get().is_none() {
                        MOD_CURSOR.borrow(cs).set(Some(retrig_to))
                    }
                });

                let mut retrig_from = retrig_to;
                seq_highs.rotate_left(j as usize);
                for i in 1..seq_highs.len() {
                    if seq_highs[i] {
                        retrig_from += break_len / 256 * 2usize.pow(i as u32);
                    }
                }

                critical_section::with(|cs| {
                    let cursor = MOD_CURSOR.borrow(cs).get().unwrap();
                    if cursor > retrig_from ||
                        cursor < retrig_to &&
                        cursor > (retrig_to + retrig_from) % break_len
                    {
                        info!("retrig to {} from {}", retrig_to, retrig_from);
                        MOD_CURSOR.borrow(cs).set(Some(retrig_to));
                    }
                });
            } else if cursor_state != CursorState::Jump {
                cursor_state = CursorState::Jump;

                critical_section::with(|cs| {
                    MOD_CURSOR.borrow(cs).set(
                        Some(BREAK_BYTES[0x2c..].len() / 4 * j as usize)
                    );
                });
                info!("jump to {}", j);
            }
        } else { // process jump
            cursor_state = CursorState::Free;

            critical_section::with(|cs| {
                if MOD_CURSOR.borrow(cs).get().is_some() {
                    info!("free");
                }
                MOD_CURSOR.borrow(cs).set(None);
            });
        }
    }
}

#[interrupt]
fn PWM_IRQ_WRAP() {
    static mut PWMOUT_SNGL: Option<PwmOut> = None;
    static mut AUDIO_SNGL: Option<core::iter::Cycle<core::slice::Iter<'_, u8>>> =
        None;

    if PWMOUT_SNGL.is_none() {
        critical_section::with(|cs| {
            *PWMOUT_SNGL = PWMOUT.borrow(cs).take();
        });
    }
    if AUDIO_SNGL.is_none() {
        critical_section::with(|cs| {
            *AUDIO_SNGL = AUDIO.borrow(cs).take();
        });
    }

    if let (Some(pwm), Some(break_iter)) = (PWMOUT_SNGL, AUDIO_SNGL) {
        // let (mut val_a, mut val_b) = (0, 0);
        // critical_section::with(|cs| {
        //     if let Ok(mut audio) = AUDIO.borrow(cs).try_borrow_mut() {
        //         audio.replace(BREAK_BYTES.iter().cycle());
        //     }
        // });
        // critical_section::with(|cs| {
            // val_a = (*AUDIO.borrow(cs).try_borrow().unwrap()).unwrap().next().unwrap() as u16;
            // if let Some(a) = (*AUDIO.borrow(cs).try_borrow().unwrap()).as_mut().unwrap().next() {

            // }
            // val_a = STREAM.borrow(cs).unwrap().next().unwrap().clone();
            // val_a = *AUDIO.borrow(cs).try_borrow().unwrap().unwrap().next().unwrap();
            // if let Ok(mut audio) = AUDIO.borrow(cs).try_borrow() {
            //     val_a = ((audio.as_mut().unwrap().next().unwrap() as u16) << 4) & 0xfff;
                // val_a = ((*audio.as_mut().unwrap().next().unwrap() as u16) << 4) & 0xfff;
            // }
        // });
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
        // let val_a = ((*break_iter.next().unwrap() as u16) << 4) & 0xFFF;
        // let val_b = ((*break_iter.next().unwrap() as u16) << 4) & 0xFFF;
        // pwm.channel_a.set_duty(val_a);
        // pwm.channel_b.set_duty(val_b);
        pwm.clear_interrupt();
    }
}

// end of file