#![no_std]
#![no_main]

use core::cell::RefCell;
use core::default::Default;

use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;
use embedded_time::duration::Milliseconds;
use embedded_time::Clock;
use frunk::HList;
use log::{info, LevelFilter};
use panic_persist as _;
use rp_pico::hal::clocks::{self, ClocksManager};
use rp_pico::hal::gpio::DynPin;
use rp_pico::hal::gpio::FunctionUart;
use rp_pico::hal::uart::{self, UartPeripheral};
use rp_pico::hal::{self, Clock as _};
use rp_pico::{
    hal::{
        pac::{self, interrupt},
        sio::Sio,
        watchdog::Watchdog,
    },
    Pins,
};
use ssd1306::{prelude::*, size::DisplaySize128x32, I2CDisplayInterface, Ssd1306};
use usb_device::class_prelude::*;
use usb_device::prelude::*;
use usbd_hid_devices::device::consumer::{ConsumerControlInterface, MultipleConsumerReport};
use usbd_hid_devices::device::keyboard::{KeyboardLedsReport, NKROBootKeyboardInterface};
use usbd_hid_devices::device::mouse::{WheelMouseInterface, WheelMouseReport};
use usbd_hid_devices::hid_class::UsbHidClass;
use usbd_hid_devices::page::Consumer;
use usbd_hid_devices::prelude::*;

use app::keyboard::*;
use app::oled_display::OledDisplay;
use app::SyncTimerClock;

// TODO:
// * Serial logging
// * Heart beat from left to right for screen saver control
// * Tidy and refactor - move keycode out to hid library
// * Pin args to &dyn InputPin and OutputPin
// * Rotary encoders
// * Consumer control support

type UsbShared = (
    UsbDevice<'static, hal::usb::UsbBus>,
    UsbHidClass<
        hal::usb::UsbBus,
        HList!(
            ConsumerControlInterface<'static, hal::usb::UsbBus>,
            WheelMouseInterface<'static, hal::usb::UsbBus>,
            NKROBootKeyboardInterface<'static, hal::usb::UsbBus, SyncTimerClock>,
        ),
    >,
    KeyboardLedsReport,
);

static IRQ_SHARED: Mutex<RefCell<Option<UsbShared>>> = Mutex::new(RefCell::new(None));

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
    let scl_pin = pins.gpio15.into_mode::<hal::gpio::FunctionI2C>();
    let sda_pin = pins.gpio14.into_mode::<hal::gpio::FunctionI2C>();

    let i2c = hal::I2C::i2c1(
        pac.I2C1,
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

    let mut oled_display = OledDisplay::new(display, clock);

    app::check_for_persisted_panic(&mut oled_display);

    log::set_max_level(LevelFilter::Info);

    //Init USB
    static mut USB_ALLOC: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;

    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    unsafe {
        USB_ALLOC = Some(usb_bus);
    }

    let usb_alloc = unsafe { USB_ALLOC.as_ref().unwrap() };

    let usb_keyboard = UsbHidClassBuilder::new()
        .add_interface(
            usbd_hid_devices::device::keyboard::NKROBootKeyboardInterface::default_config(clock),
        )
        .add_interface(usbd_hid_devices::device::mouse::WheelMouseInterface::default_config())
        .add_interface(
            usbd_hid_devices::device::consumer::ConsumerControlInterface::default_config(),
        )
        //Build
        .build(usb_alloc);

    // Create a USB device with https://pid.code VID and a test PID
    //https://pid.codes
    let usb_device = UsbDeviceBuilder::new(usb_alloc, UsbVidPid(0x1209, 0x0004))
        .manufacturer("DLKJ")
        .product("pi-choc-nano")
        .serial_number("1")
        .supports_remote_wakeup(false)
        .build();

    cortex_m::interrupt::free(|cs| {
        IRQ_SHARED
            .borrow(cs)
            .replace(Some((usb_device, usb_keyboard, Default::default())));
    });

    let input_cols: [DynPin; 6] = [
        pins.gpio20.into_pull_down_input().into(),
        pins.gpio21.into_pull_down_input().into(),
        pins.gpio22.into_pull_down_input().into(),
        pins.gpio26.into_pull_down_input().into(),
        pins.gpio27.into_pull_down_input().into(),
        pins.gpio28.into_pull_down_input().into(),
    ];

    let mut output_rows: [DynPin; 6] = [
        pins.gpio5.into_push_pull_output().into(),
        pins.gpio6.into_push_pull_output().into(),
        pins.gpio7.into_push_pull_output().into(),
        pins.gpio9.into_push_pull_output().into(),
        pins.gpio10.into_push_pull_output().into(),
        pins.gpio11.into_push_pull_output().into(),
    ];

    for p in &mut output_rows {
        p.set_low().unwrap();
    }

    let uart = UartPeripheral::<_, _>::new(pac.UART0, &mut pac.RESETS)
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

    // Enable the USB interrupt
    unsafe {
        pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
    };

    let mut keyboard = Keyboard::new(
        SplitMatrix {
            matrix1: DiodePinMatrix::new(output_rows, input_cols),
            matrix2: UartMatrix::new(uart),
        },
        LayerdKeyboardLayout::new(app::key_map::KEY_MAP),
    );

    let mut pin_sample_timer = clock.new_timer(PIN_SAMPLE).into_periodic().start().unwrap();

    let mut input_timer = clock
        .new_timer(INPUT_SAMPLE)
        .into_periodic()
        .start()
        .unwrap();

    let mut last_mouse_buttons = 0;
    let mut mouse_report = WheelMouseReport::default();
    let mut last_consumer_report = MultipleConsumerReport::default();

    info!("Running main loop");
    loop {
        //1ms scan the keys and debounce (was .1ms)
        if pin_sample_timer.period_complete().unwrap() {
            // let (p_a, p_b) = rot_enc.pins_borrow_mut();
            // p_a.update().expect("Failed to update rot a debouncer");
            // p_b.update().expect("Failed to update rot b debouncer");

            //todo: move onto an interrupt timer
            //rot_enc.update();

            keyboard.update().expect("Failed to update keyboard");
        }

        //10ms
        if input_timer.period_complete().unwrap() {
            //100Hz or slower
            let state = keyboard.state().unwrap();

            let (leds, usb_state) = cortex_m::interrupt::free(|cs| {
                let mut usb_ref = IRQ_SHARED.borrow(cs).borrow_mut();
                let (ref mut usb_device, ref mut composite, ref leds) = usb_ref.as_mut().unwrap();

                let keyboard = composite.interface::<NKROBootKeyboardInterface<'_, _, _>, _>();
                match keyboard.write_report(&state.keycodes) {
                    Err(UsbHidError::WouldBlock) => {}
                    Err(UsbHidError::Duplicate) => {}
                    Ok(_) => {}
                    Err(e) => {
                        panic!("Failed to write keyboard report: {:?}", e)
                    }
                };

                if mouse_report.buttons != last_mouse_buttons
                    || mouse_report.x != 0
                    || mouse_report.y != 0
                {
                    let mouse = composite.interface::<WheelMouseInterface<'_, _>, _>();
                    match mouse.write_report(&mouse_report) {
                        Err(UsbHidError::WouldBlock) => {}
                        Ok(_) => {
                            last_mouse_buttons = mouse_report.buttons;
                            mouse_report = Default::default();
                        }
                        Err(e) => {
                            panic!("Failed to write mouse report: {:?}", e)
                        }
                    };
                }

                let consumer_report = MultipleConsumerReport {
                    codes: [
                        Consumer::Unassigned,
                        Consumer::Unassigned,
                        Consumer::Unassigned,
                        Consumer::Unassigned,
                    ],
                };

                if last_consumer_report != consumer_report {
                    let consumer = composite.interface::<ConsumerControlInterface<'_, _>, _>();
                    match consumer.write_report(&consumer_report) {
                        Err(UsbError::WouldBlock) => {}
                        Ok(_) => {
                            last_consumer_report = consumer_report;
                        }
                        Err(e) => {
                            panic!("Failed to write consumer report: {:?}", e)
                        }
                    };
                }

                //Get status
                (*leds, usb_device.state())
            });

            oled_display
                .draw_left_display(leds, &state.keycodes, state.layer, usb_state)
                .ok();
        }
    }
}

#[allow(non_snake_case)]
#[interrupt]
fn USBCTRL_IRQ() {
    cortex_m::interrupt::free(|cs| {
        let mut usb_ref = IRQ_SHARED.borrow(cs).borrow_mut();
        if usb_ref.is_none() {
            return;
        }

        let (ref mut usb_device, ref mut composite, ref mut leds) = usb_ref.as_mut().unwrap();
        if usb_device.poll(&mut [composite]) {
            let keyboard = composite.interface::<NKROBootKeyboardInterface<'_, _, _>, _>();
            match keyboard.read_report() {
                Err(UsbError::WouldBlock) => {}
                Err(e) => {
                    panic!("Failed to read keyboard report: {:?}", e)
                }
                Ok(l) => {
                    *leds = l;
                }
            }
        }
    });

    cortex_m::asm::sev();
}
