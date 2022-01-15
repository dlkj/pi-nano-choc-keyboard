#![no_std]
#![no_main]

use app::keyboard::keycode::*;
use app::keyboard::*;
use app::oled_display::OledDisplay;
use core::borrow::BorrowMut;
use core::cell::Cell;
use core::cell::RefCell;
use core::cell::UnsafeCell;
use core::fmt::Write;
use core::panic::PanicInfo;
use core::sync::atomic::{self, Ordering};
use cortex_m::interrupt::Mutex;
use cortex_m::prelude::*;
use cortex_m_rt::entry;
use embedded_graphics::mono_font::iso_8859_1::FONT_4X6;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::Rectangle;
use embedded_hal::digital::v2::OutputPin;
use embedded_text::alignment::HorizontalAlignment;
use embedded_text::style::{HeightMode, TextBoxStyleBuilder};
use embedded_text::TextBox;
use embedded_time::duration::Extensions;
use log::{error, info, LevelFilter};
use nb::block;
use rp_pico::hal::clocks::{self, ClocksManager};
use rp_pico::hal::gpio::{bank0::*, DynPin, Function};
use rp_pico::hal::gpio::{FunctionUart, Pin, I2C};
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
use ssd1306::mode::BufferedGraphicsMode;
use ssd1306::{prelude::*, size::DisplaySize128x32, I2CDisplayInterface, Ssd1306};
use usb_device::class::UsbClass;
use usb_device::class_prelude::*;
use usb_device::device::UsbDeviceState;
use usb_device::prelude::*;
use usbd_hid_devices::hid::UsbHidClass;
use usbd_hid_devices::keyboard::HidKeyboard;

// TODO:
// * Panic handler to screen in a less horrible way - Ram panic then show on boot
// * Serial logging
// * Heart beat from left to right for screen saver control
// * Tidy and refactor - move keycode out to hid library

static USB: Mutex<
    Cell<
        Option<(
            UsbDevice<'static, hal::usb::UsbBus>,
            UsbHidClass<'static, hal::usb::UsbBus, usbd_hid_devices::keyboard::HidBootKeyboard>,
        )>,
    >,
> = Mutex::new(Cell::new(None));

static KEYBOARD_STATUS: Mutex<Cell<(Leds, UsbDeviceState)>> =
    Mutex::new(Cell::new((Leds::empty(), UsbDeviceState::Default)));
static KEYBOARD_STATE: Mutex<Cell<Option<KeyboardState<72>>>> = Mutex::new(Cell::new(None));

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
        &usb_alloc,
        usbd_hid_devices::keyboard::HidBootKeyboard::default(),
    );
    // Create a USB device with https://pid.code VID and a test PID
    //https://pid.codes
    let usb_device = UsbDeviceBuilder::new(&usb_alloc, UsbVidPid(0x1209, 0x0004))
        .manufacturer("DLKJ")
        .product("pichocnano")
        .serial_number("TEST")
        .device_class(3) // HID - from: https://www.usb.org/defined-class-codes
        .composite_with_iads()
        .supports_remote_wakeup(true)
        .build();

    cortex_m::interrupt::free(|cs| {
        USB.borrow(cs).replace(Some((usb_device, usb_keyboard)));
    });

    log::set_max_level(LevelFilter::Info);

    // Enable the USB interrupt
    // unsafe {
    //     pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
    // };

    // let rot_pin_a = DebouncedPin::<DynPin>::new(pins.gpio16.into_pull_up_input().into(), true);
    // let rot_pin_b = DebouncedPin::<DynPin>::new(pins.gpio17.into_pull_up_input().into(), true);

    // let mut rot_enc = RotaryEncoder::new(rot_pin_a, rot_pin_b);

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

    let mut oled_display = OledDisplay::new(display, &timer);

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
            let keycodes = state.keycodes.clone();
            let layer = state.layer;

            let (leds, usb_state) =
                cortex_m::interrupt::free(|cs| KEYBOARD_STATUS.borrow(cs).get());

            //update keyboard state
            cortex_m::interrupt::free(|cs| KEYBOARD_STATE.borrow(cs).replace(Some(state)));

            oled_display
                .draw_left_display(leds, &keycodes, layer, usb_state)
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
            *USB_DEVICES = USB.borrow(cs).replace(None);
        });
    }

    if let Some((ref mut usb_device, ref mut usb_keyboard)) = USB_DEVICES.as_mut() {
        let mut leds = None;
        if usb_device.state() != UsbDeviceState::Configured {
            leds = Some(Leds::empty());
        }

        if usb_device.poll(&mut [usb_keyboard]) {
            leds = match usb_keyboard.read_leds() {
                Ok(leds) => Some(leds.into()),

                Err(UsbError::WouldBlock) => None,
                Err(_) => Some(Leds::empty()),
            };

            cortex_m::interrupt::free(|cs| {
                if let Some(state) = KEYBOARD_STATE.borrow(cs).replace(None) {
                    if usb_keyboard
                        .write_keycodes(state.keycodes.iter().map(|&k| k as u8))
                        .is_err()
                    {
                        KEYBOARD_STATE.borrow(cs).replace(Some(state));
                    }
                }
            });
        }

        cortex_m::interrupt::free(|cs| {
            //update leds
            if let Some(new_leds) = leds {
                KEYBOARD_STATUS
                    .borrow(cs)
                    .replace((new_leds, usb_device.state()));
            }
        });
    }
    cortex_m::asm::sev();
}

#[inline(never)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    error!("{}", info);

    let mut output = arrayvec::ArrayString::<1024>::new();
    if write!(&mut output, "{}", info).ok().is_some() {
        // cortex_m::interrupt::free(|cs| {
        //     let mut display_ref = OLED_DISPLAY.borrow(cs).borrow_mut();
        //     if let Some(display) = display_ref.as_mut() {
        //         display.clear();
        //         let character_style = MonoTextStyle::new(&FONT_4X6, BinaryColor::On);
        //         let textbox_style = TextBoxStyleBuilder::new()
        //             .height_mode(HeightMode::FitToText)
        //             .alignment(HorizontalAlignment::Left)
        //             .build();
        //         let bounds = Rectangle::new(Point::zero(), Size::new(32, 0));
        //         let text_box = TextBox::with_textbox_style(
        //             output.as_str(),
        //             bounds,
        //             character_style,
        //             textbox_style,
        //         );

        //         text_box.draw(display)?;
        //         display.flush()
        //     } else {
        //         Ok(())
        //     }
        // })
        // .ok();
    }

    loop {
        atomic::compiler_fence(Ordering::SeqCst);
    }
}
