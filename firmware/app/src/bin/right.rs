#![no_std]
#![no_main]

use core::cell::RefCell;
use core::convert::Infallible;

use cortex_m::interrupt::Mutex;
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m_rt::entry;
use cortex_m_rt::exception;
use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::ToggleableOutputPin;
use embedded_hal::prelude::_embedded_hal_serial_Write;
use embedded_time::duration::Milliseconds;
use embedded_time::Clock;
use log::{info, LevelFilter};
use nb::block;
use panic_persist as _;
use rp_pico::hal::clocks::{self, ClocksManager};
use rp_pico::hal::gpio::PullUpInput;
use rp_pico::hal::gpio::{FunctionUart, Pin};
use rp_pico::hal::uart::{self, UartPeripheral};
use rp_pico::hal::{self, Clock as _};
use rp_pico::{
    hal::{
        gpio::DynPin,
        pac::{self},
        sio::Sio,
        watchdog::Watchdog,
    },
    Pins,
};
use ssd1306::{prelude::*, size::DisplaySize128x32, I2CDisplayInterface, Ssd1306};

use app::keyboard::*;
use app::rotary_enc::RotaryEncoder;
use app::SyncTimerClock;

type TimerShared = (
    DiodePinMatrix<DynPin, DynPin>,
    RotaryEncoder<
        Pin<hal::gpio::pin::bank0::Gpio14, PullUpInput>,
        Pin<hal::gpio::pin::bank0::Gpio15, PullUpInput>,
        Infallible,
    >,
);

static TIMER_SHARED: Mutex<RefCell<Option<TimerShared>>> = Mutex::new(RefCell::new(None));

const INPUT_SAMPLE: Milliseconds = Milliseconds(10);

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut core = pac::CorePeripherals::take().unwrap();

    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    let clocks: ClocksManager = clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let sio = Sio::new(pac.SIO);
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    static mut CLOCK: Option<SyncTimerClock> = None;
    let clock = unsafe {
        CLOCK = Some(SyncTimerClock::new(hal::Timer::new(
            pac.TIMER,
            &mut pac.RESETS,
        )));
        CLOCK.as_ref().unwrap()
    };

    //Init display
    let scl_pin = pins.gpio17.into_mode::<rp_pico::hal::gpio::FunctionI2C>();
    let sda_pin = pins.gpio16.into_mode::<rp_pico::hal::gpio::FunctionI2C>();

    let i2c = rp_pico::hal::i2c::I2C::new_controller(
        pac.I2C0,
        sda_pin,
        scl_pin,
        embedded_time::rate::Extensions::kHz(400),
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
    );

    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate90)
        .into_buffered_graphics_mode();
    display.init().unwrap();
    display.flush().unwrap();
    let mut oled_display = app::oled_display::OledDisplay::new(display, clock);

    app::check_for_persisted_panic(&mut oled_display);

    log::set_max_level(LevelFilter::Info);

    // Enable the USB interrupt
    unsafe {
        pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
    };

    let cols: [DynPin; 6] = [
        pins.gpio5.into_pull_down_input().into(),
        pins.gpio6.into_pull_down_input().into(),
        pins.gpio7.into_pull_down_input().into(),
        pins.gpio9.into_pull_down_input().into(),
        pins.gpio10.into_pull_down_input().into(),
        pins.gpio11.into_pull_down_input().into(),
    ];

    let mut rows: [DynPin; 6] = [
        pins.gpio28.into_push_pull_output().into(),
        pins.gpio27.into_push_pull_output().into(),
        pins.gpio26.into_push_pull_output().into(),
        pins.gpio22.into_push_pull_output().into(),
        pins.gpio21.into_push_pull_output().into(),
        pins.gpio20.into_push_pull_output().into(),
    ];

    for p in &mut rows {
        p.set_low().unwrap();
    }

    let rot_button = pins.gpio8.into_pull_up_input();

    let rot_b = pins.gpio15.into_pull_up_input();
    let rot_a = pins.gpio14.into_pull_up_input();

    let rot_enc = RotaryEncoder::new(rot_a, rot_b);

    let mut uart = UartPeripheral::<_, _>::new(pac.UART0, &mut pac.RESETS)
        .enable(
            uart::common_configs::_19200_8_N_1,
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    let _tx_pin = pins.gpio12.into_mode::<FunctionUart>();
    let _rx_pin = pins.gpio13.into_mode::<FunctionUart>();

    // Splash screen
    oled_display.draw_text_screen("Starting...").unwrap();

    info!("Starting");

    //100us timer
    let reload_value = 100 - 1;
    core.SYST.set_reload(reload_value);
    core.SYST.clear_current();
    //External clock, driven by the Watchdog - 1 tick per us
    core.SYST.set_clock_source(SystClkSource::External);
    core.SYST.enable_interrupt();
    core.SYST.enable_counter();

    let matrix = DiodePinMatrix::new(rows, cols);

    cortex_m::interrupt::free(|cs| {
        TIMER_SHARED.borrow(cs).replace(Some((matrix, rot_enc)));
    });

    let mut input_timer = clock
        .new_timer(INPUT_SAMPLE)
        .into_periodic()
        .start()
        .unwrap();

    let mut led = pins.led.into_push_pull_output();

    info!("Running main loop");
    loop {
        //10ms
        if input_timer.period_complete().unwrap() {
            //100Hz or slower
            let (keys, rot) = cortex_m::interrupt::free(|cs| {
                let mut timer_ref = TIMER_SHARED.borrow(cs).borrow_mut();

                let (ref mut matrix, ref mut rot_enc) = timer_ref.as_mut().unwrap();
                (
                    matrix.keys().expect("Failed to get matrix keys"),
                    rot_enc.rel_value(),
                )
            });

            let mut pressed_keys: arrayvec::ArrayVec<usize, 64> = keys
                .iter()
                .enumerate()
                .filter_map(|(i, &k)| k.pressed.then(|| i))
                .collect();

            if rot_button.is_low().unwrap() {
                pressed_keys.push(36);
            }
            if rot < 0 {
                pressed_keys.push(37)
            } else if rot > 0 {
                pressed_keys.push(38);
            }

            led.toggle().unwrap();
            for &k in &pressed_keys {
                block!(uart.write(k as u8)).unwrap();
            }
            block!(uart.write(0xFF)).unwrap();
            led.toggle().unwrap();

            oled_display.draw_right_display(&pressed_keys[..]).ok();
        }
    }
}

#[exception]
fn SysTick() {
    cortex_m::interrupt::free(|cs| {
        let mut timer_ref = TIMER_SHARED.borrow(cs).borrow_mut();
        if timer_ref.is_none() {
            return;
        }

        let (ref mut matrix, ref mut rot_enc) = timer_ref.as_mut().unwrap();
        rot_enc.update();
        matrix.update().expect("Failed to update keys");
    });

    cortex_m::asm::sev();
}
