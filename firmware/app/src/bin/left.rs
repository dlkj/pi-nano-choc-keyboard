#![no_std]
#![no_main]

use app::keyboard::keycode::*;
use app::keyboard::*;
use app::oled_display::OledDisplay;
use arrayvec::ArrayVec;
use core::cell::Cell;
use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use cortex_m::prelude::*;
use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;
use embedded_time::duration::Extensions;
use log::{info, LevelFilter};
use panic_persist as _;
use rp_pico::hal::clocks::{self, ClocksManager};
use rp_pico::hal::gpio::DynPin;
use rp_pico::hal::gpio::FunctionUart;
use rp_pico::hal::uart::{self, UartPeripheral};
use rp_pico::hal::{self, Clock};
use rp_pico::{
    hal::{
        pac::{self, interrupt},
        sio::Sio,
        timer::Timer,
        watchdog::Watchdog,
    },
    Pins,
};
use ssd1306::{prelude::*, size::DisplaySize128x32, I2CDisplayInterface, Ssd1306};
use usb_device::class_prelude::*;
use usb_device::device::UsbDeviceState;
use usb_device::prelude::*;
use usbd_hid_devices::hid::UsbHidClass;
use usbd_hid_devices::keyboard::HidKeyboard;

// TODO:
// * Serial logging
// * Heart beat from left to right for screen saver control
// * Tidy and refactor - move keycode out to hid library
// * Pin args to &dyn InputPin and OutputPin
// * Rotary encoders
// * Consumer control support

type UsbDevices = (
    UsbDevice<'static, hal::usb::UsbBus>,
    UsbHidClass<'static, hal::usb::UsbBus, usbd_hid_devices::keyboard::HidBootKeyboard>,
);

static USB: Mutex<Cell<Option<UsbDevices>>> = Mutex::new(Cell::new(None));

static KEYBOARD_STATUS: Mutex<Cell<(Leds, UsbDeviceState)>> =
    Mutex::new(Cell::new((Leds::empty(), UsbDeviceState::Default)));
static KEYBOARD_STATE: Mutex<RefCell<ArrayVec<KeyCode, 72>>> =
    Mutex::new(RefCell::new(ArrayVec::new_const()));

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

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);

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

    let mut oled_display = OledDisplay::new(display, &timer);

    if let Some(msg) = panic_persist::get_panic_message_utf8() {
        oled_display.draw_text_screen(msg).ok();
        loop {
            cortex_m::asm::nop();
        }
    }

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

    let usb_keyboard = usbd_hid_devices::hid::UsbHidClass::new(
        usb_alloc,
        usbd_hid_devices::keyboard::HidBootKeyboard::default(),
    );
    // Create a USB device with https://pid.code VID and a test PID
    //https://pid.codes
    let usb_device = UsbDeviceBuilder::new(usb_alloc, UsbVidPid(0x1209, 0x0004))
        .manufacturer("DLKJ")
        .product("pi-choc-nano")
        .serial_number("TEST")
        .device_class(3) // HID - from: https://www.usb.org/defined-class-codes
        .composite_with_iads()
        .supports_remote_wakeup(true)
        .build();

    cortex_m::interrupt::free(|cs| {
        USB.borrow(cs).set(Some((usb_device, usb_keyboard)));
    });

    let cols: [DynPin; 6] = [
        pins.gpio20.into_pull_down_input().into(),
        pins.gpio21.into_pull_down_input().into(),
        pins.gpio22.into_pull_down_input().into(),
        pins.gpio26.into_pull_down_input().into(),
        pins.gpio27.into_pull_down_input().into(),
        pins.gpio28.into_pull_down_input().into(),
    ];

    let mut rows: [DynPin; 6] = [
        pins.gpio5.into_push_pull_output().into(),
        pins.gpio6.into_push_pull_output().into(),
        pins.gpio7.into_push_pull_output().into(),
        pins.gpio9.into_push_pull_output().into(),
        pins.gpio10.into_push_pull_output().into(),
        pins.gpio11.into_push_pull_output().into(),
    ];

    for p in &mut rows {
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
            matrix1: DiodePinMatrix::new(rows, cols),
            matrix2: UartMatrix::new(uart),
        },
        LayerdKeyboardLayout::new(app::key_map::KEY_MAP),
    );

    let mut fast_countdown = timer.count_down();
    fast_countdown.start(100.nanoseconds());

    let mut slow_countdown = timer.count_down();
    slow_countdown.start(20.milliseconds());

    info!("Running main loop");
    loop {
        //0.1ms scan the keys and debounce
        if fast_countdown.wait().is_ok() {
            // let (p_a, p_b) = rot_enc.pins_borrow_mut();
            // p_a.update().expect("Failed to update rot a debouncer");
            // p_b.update().expect("Failed to update rot b debouncer");

            //todo: move onto an interupt timer
            //rot_enc.update();

            keyboard.update().expect("Failed to update keyboard");
        }

        //10ms
        if slow_countdown.wait().is_ok() {
            //led_pin.toggle().unwrap();

            //100Hz or slower
            let state = keyboard.state().unwrap();
            let (leds, usb_state) = cortex_m::interrupt::free(|cs| {
                //update keyboard state
                KEYBOARD_STATE
                    .borrow(cs)
                    .borrow_mut()
                    .clone_from(&state.keycodes);

                //Get status
                KEYBOARD_STATUS.borrow(cs).get()
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
    static mut USB_DEVICES: Option<(
        UsbDevice<'static, hal::usb::UsbBus>,
        UsbHidClass<'static, hal::usb::UsbBus, usbd_hid_devices::keyboard::HidBootKeyboard>,
    )> = None;

    if USB_DEVICES.is_none() {
        cortex_m::interrupt::free(|cs| {
            *USB_DEVICES = USB.borrow(cs).take();
        });
    }

    if let Some((ref mut usb_device, ref mut usb_keyboard)) = USB_DEVICES.as_mut() {
        let mut leds = if usb_device.state() != UsbDeviceState::Configured {
            Some(Leds::empty())
        } else {
            None
        };

        if usb_device.poll(&mut [usb_keyboard]) {
            leds = match usb_keyboard.read_leds() {
                Ok(leds) => Some(leds.into()),
                Err(UsbError::WouldBlock) => None,
                Err(_) => Some(Leds::empty()),
            };
            cortex_m::interrupt::free(|cs| {
                usb_keyboard
                    .write_keycodes(KEYBOARD_STATE.borrow(cs).borrow().iter().map(|&k| k as u8))
                    .ok()
            });
        }

        cortex_m::interrupt::free(|cs| {
            let (last_leds, _) = KEYBOARD_STATUS.borrow(cs).get();

            //update leds
            KEYBOARD_STATUS
                .borrow(cs)
                .set((leds.unwrap_or(last_leds), usb_device.state()));
        });
    }

    cortex_m::asm::sev();
}
