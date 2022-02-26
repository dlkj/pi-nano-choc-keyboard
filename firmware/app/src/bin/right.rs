#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::prelude::_embedded_hal_serial_Write;
use embedded_time::duration::Milliseconds;
use embedded_time::Clock;
use log::{info, LevelFilter};
use nb::block;
use panic_persist as _;
use rp_pico::hal::clocks::{self, ClocksManager};
use rp_pico::hal::gpio::FunctionUart;
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
use app::SyncTimerClock;

const INPUT_SAMPLE: Milliseconds = Milliseconds(10);
const PIN_SAMPLE: Milliseconds = Milliseconds(1);

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();

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

    // let rot_pin_a = DebouncedPin::<DynPin>::new(pins.gpio15.into_pull_up_input().into(), true);
    // let rot_pin_b = DebouncedPin::<DynPin>::new(pins.gpio14.into_pull_up_input().into(), true);

    // let mut rot_enc = RotaryEncoder::new(rot_pin_a, rot_pin_b);

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

    let mut matrix = DiodePinMatrix::new(rows, cols);

    let mut pin_sample_timer = clock.new_timer(PIN_SAMPLE).into_periodic().start().unwrap();

    let mut input_timer = clock
        .new_timer(INPUT_SAMPLE)
        .into_periodic()
        .start()
        .unwrap();

    info!("Running main loop");
    loop {
        //1ms scan the keys and debounce (was .1 ms)
        if pin_sample_timer.period_complete().unwrap() {
            // let (p_a, p_b) = rot_enc.pins_borrow_mut();
            // p_a.update().expect("Failed to update rot a debouncer");
            // p_b.update().expect("Failed to update rot b debouncer");

            //todo: move onto an interupt timer
            //rot_enc.update();

            matrix.update().expect("Failed to update keys");
        }

        //10ms
        if input_timer.period_complete().unwrap() {
            //100Hz or slower
            let keys = matrix.keys().expect("Failed to get matrix keys");

            let pressed_keys: arrayvec::ArrayVec<usize, 36> = keys
                .iter()
                .enumerate()
                .filter_map(|(i, &k)| k.pressed.then(|| i))
                .collect();

            for &k in &pressed_keys {
                block!(uart.write(k as u8)).unwrap();
            }
            block!(uart.write(0xFF)).unwrap();

            oled_display.draw_right_display(&pressed_keys[..]).ok();
        }
    }
}
