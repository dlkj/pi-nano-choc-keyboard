#![no_std]
#![no_main]

use app::keyboard::keycode::*;
use app::keyboard::*;
use app::oled_display::OledDisplay;
use core::cell::RefCell;
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
use rp_pico::hal::clocks::{self, ClocksManager};
use rp_pico::hal::gpio::{bank0::*, DynPin, Function};
use rp_pico::hal::gpio::{FunctionUart, Pin, I2C};
use rp_pico::hal::uart::{self, UartPeripheral};
use rp_pico::hal::{self, Clock};
use rp_pico::{
    hal::{
        pac::{self},
        sio::Sio,
        timer::Timer,
        watchdog::Watchdog,
    },
    Pins,
};
use ssd1306::mode::BufferedGraphicsMode;
use ssd1306::{prelude::*, size::DisplaySize128x32, I2CDisplayInterface, Ssd1306};
use usb_device::class_prelude::*;
use usb_device::prelude::UsbDeviceBuilder;
use usb_device::prelude::UsbVidPid;
use usbd_hid_devices::keyboard::HidKeyboard;

type BufferedSsd1306 = Ssd1306<
    I2CInterface<hal::I2C<pac::I2C1, (Pin<Gpio14, Function<I2C>>, Pin<Gpio15, Function<I2C>>)>>,
    DisplaySize128x32,
    BufferedGraphicsMode<DisplaySize128x32>,
>;

static OLED_DISPLAY: Mutex<RefCell<Option<BufferedSsd1306>>> = Mutex::new(RefCell::new(None));

const BASE_MAP: [KeyAction; 72] = [
    //row 0
    KeyAction::Key {
        code: KeyCode::Escape,
    },
    KeyAction::Key { code: KeyCode::Kb1 },
    KeyAction::Key { code: KeyCode::Kb2 },
    KeyAction::Key { code: KeyCode::Kb3 },
    KeyAction::Key { code: KeyCode::Kb4 },
    KeyAction::Key { code: KeyCode::Kb5 },
    //row 1
    KeyAction::Key { code: KeyCode::Tab },
    KeyAction::Key { code: KeyCode::Q },
    KeyAction::Key { code: KeyCode::W },
    KeyAction::Key { code: KeyCode::E },
    KeyAction::Key { code: KeyCode::R },
    KeyAction::Key { code: KeyCode::T },
    //row 2
    KeyAction::Key {
        code: KeyCode::BackslashISO,
    },
    KeyAction::Key { code: KeyCode::A },
    KeyAction::Key { code: KeyCode::S },
    KeyAction::Key { code: KeyCode::D },
    KeyAction::Key { code: KeyCode::F },
    KeyAction::Key { code: KeyCode::G },
    //row 3
    KeyAction::Key {
        code: KeyCode::LeftShift,
    },
    KeyAction::Key { code: KeyCode::Z },
    KeyAction::Key { code: KeyCode::X },
    KeyAction::Key { code: KeyCode::C },
    KeyAction::Key { code: KeyCode::V },
    KeyAction::Key { code: KeyCode::B },
    //row 4
    KeyAction::Key {
        code: KeyCode::LeftControl,
    },
    KeyAction::Key {
        code: KeyCode::LeftGUI,
    },
    KeyAction::Key {
        code: KeyCode::Grave,
    },
    KeyAction::None,
    KeyAction::None,
    KeyAction::Key {
        code: KeyCode::LeftBracket,
    },
    //row 5
    KeyAction::Key {
        code: KeyCode::None,
    },
    KeyAction::Key {
        code: KeyCode::LeftAlt,
    },
    KeyAction::Function {
        function: KeyFunction::Hyper,
    },
    KeyAction::Key {
        code: KeyCode::Spacebar,
    },
    KeyAction::Key {
        code: KeyCode::Spacebar,
    },
    KeyAction::Layer { n: 1 },
    //right hand

    //row 0
    KeyAction::Key { code: KeyCode::Kb6 },
    KeyAction::Key { code: KeyCode::Kb7 },
    KeyAction::Key { code: KeyCode::Kb8 },
    KeyAction::Key { code: KeyCode::Kb9 },
    KeyAction::Key { code: KeyCode::Kb0 },
    KeyAction::Key {
        code: KeyCode::Minus,
    },
    //row 1
    KeyAction::Key { code: KeyCode::Y },
    KeyAction::Key { code: KeyCode::U },
    KeyAction::Key { code: KeyCode::I },
    KeyAction::Key { code: KeyCode::O },
    KeyAction::Key { code: KeyCode::P },
    KeyAction::Key {
        code: KeyCode::Equals,
    },
    //row 2
    KeyAction::Key { code: KeyCode::H },
    KeyAction::Key { code: KeyCode::J },
    KeyAction::Key { code: KeyCode::K },
    KeyAction::Key { code: KeyCode::L },
    KeyAction::Key {
        code: KeyCode::Semicolon,
    },
    KeyAction::Key {
        code: KeyCode::Apostrophy,
    },
    //row 3
    KeyAction::Key { code: KeyCode::N },
    KeyAction::Key { code: KeyCode::M },
    KeyAction::Key {
        code: KeyCode::Comma,
    },
    KeyAction::Key { code: KeyCode::Dot },
    KeyAction::Key {
        code: KeyCode::ForwardSlash,
    },
    KeyAction::Key {
        code: KeyCode::RightShift,
    },
    //row 4
    KeyAction::Key {
        code: KeyCode::RightBracket,
    },
    KeyAction::None,
    KeyAction::Key {
        code: KeyCode::Delete,
    },
    KeyAction::Key {
        code: KeyCode::Hash,
    },
    KeyAction::Key {
        code: KeyCode::Application,
    },
    KeyAction::Key {
        code: KeyCode::RightControl,
    },
    //row 5
    KeyAction::Layer { n: 2 },
    KeyAction::Key {
        code: KeyCode::Enter,
    },
    KeyAction::Key {
        code: KeyCode::Backspace,
    },
    KeyAction::Function {
        function: KeyFunction::Meh,
    },
    KeyAction::Key {
        code: KeyCode::RightAlt,
    },
    KeyAction::Key {
        code: KeyCode::None,
    },
];
const UPPER_MAP: [KeyAction; 72] = [
    //row 0
    KeyAction::FallThrough,
    KeyAction::None,
    KeyAction::None,
    KeyAction::None,
    KeyAction::None,
    KeyAction::None,
    //row 1
    KeyAction::FallThrough,
    KeyAction::None,
    KeyAction::None,
    KeyAction::None,
    KeyAction::None,
    KeyAction::None,
    //row 2
    KeyAction::FallThrough,
    KeyAction::None,
    KeyAction::None,
    KeyAction::None,
    KeyAction::None,
    KeyAction::None,
    //row 3
    KeyAction::FallThrough,
    KeyAction::None,
    KeyAction::None,
    KeyAction::None,
    KeyAction::None,
    KeyAction::None,
    //row 4
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    //row 5
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    //right hand
    //row 0
    KeyAction::None,
    KeyAction::Key {
        code: KeyCode::KpNumLock,
    },
    KeyAction::Key {
        code: KeyCode::KpBackslash,
    },
    KeyAction::Key {
        code: KeyCode::KpAsterisk,
    },
    KeyAction::Key {
        code: KeyCode::KpMinus,
    },
    KeyAction::Key {
        code: KeyCode::None,
    },
    //row 1
    KeyAction::None,
    KeyAction::Key { code: KeyCode::Kp7 },
    KeyAction::Key { code: KeyCode::Kp8 },
    KeyAction::Key { code: KeyCode::Kp9 },
    KeyAction::Key {
        code: KeyCode::KpPlus,
    },
    KeyAction::Key {
        code: KeyCode::None,
    },
    //row 2
    KeyAction::None,
    KeyAction::Key { code: KeyCode::Kp4 },
    KeyAction::Key { code: KeyCode::Kp5 },
    KeyAction::Key { code: KeyCode::Kp6 },
    KeyAction::Key {
        code: KeyCode::KpEnter,
    },
    KeyAction::Key {
        code: KeyCode::None,
    },
    //row 3
    KeyAction::None,
    KeyAction::Key { code: KeyCode::Kp1 },
    KeyAction::Key { code: KeyCode::Kp2 },
    KeyAction::Key { code: KeyCode::Kp3 },
    KeyAction::None,
    KeyAction::FallThrough,
    //row 4
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::Key { code: KeyCode::Kp0 },
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    //row 5
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::Key { code: KeyCode::Dot },
    KeyAction::FallThrough,
];

const LOWER_MAP: [KeyAction; 72] = [
    //row 0
    KeyAction::FallThrough,
    KeyAction::Key { code: KeyCode::F1 },
    KeyAction::Key { code: KeyCode::F2 },
    KeyAction::Key { code: KeyCode::F3 },
    KeyAction::Key { code: KeyCode::F4 },
    KeyAction::Key { code: KeyCode::F5 },
    //row 1
    KeyAction::FallThrough,
    KeyAction::None,
    KeyAction::None,
    KeyAction::None,
    KeyAction::None,
    KeyAction::None,
    //row 2
    KeyAction::FallThrough,
    KeyAction::None,
    KeyAction::None,
    KeyAction::None,
    KeyAction::None,
    KeyAction::None,
    //row 3
    KeyAction::FallThrough,
    KeyAction::None,
    KeyAction::None,
    KeyAction::None,
    KeyAction::None,
    KeyAction::None,
    //row 4
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    //row 5
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    //right hand
    //row 0
    KeyAction::Key { code: KeyCode::F6 },
    KeyAction::Key { code: KeyCode::F7 },
    KeyAction::Key { code: KeyCode::F8 },
    KeyAction::Key { code: KeyCode::F9 },
    KeyAction::Key { code: KeyCode::F10 },
    KeyAction::Key { code: KeyCode::F11 },
    //row 1
    KeyAction::None,
    KeyAction::Key {
        code: KeyCode::PageUp,
    },
    KeyAction::Key {
        code: KeyCode::UpArrow,
    },
    KeyAction::Key {
        code: KeyCode::PageDown,
    },
    KeyAction::None,
    KeyAction::Key { code: KeyCode::F12 },
    //row 2
    KeyAction::None,
    KeyAction::Key {
        code: KeyCode::LeftArrow,
    },
    KeyAction::Key {
        code: KeyCode::DownArrow,
    },
    KeyAction::Key {
        code: KeyCode::RightArrow,
    },
    KeyAction::None,
    KeyAction::Key {
        code: KeyCode::Pause,
    },
    //row 3
    KeyAction::None,
    KeyAction::Key {
        code: KeyCode::Home,
    },
    KeyAction::None,
    KeyAction::Key { code: KeyCode::End },
    KeyAction::None,
    KeyAction::FallThrough,
    //row 4
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    //row 5
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::FallThrough,
];

const KEY_MAP: [[KeyAction; 72]; 3] = [BASE_MAP, LOWER_MAP, UPPER_MAP];

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

    cortex_m::interrupt::free(|cs| {
        OLED_DISPLAY.borrow(cs).replace(Some(display));
    });

    //Init USB
    let usb_alloc = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    // USB_MANAGER.borrow(cs).replace(Some(UsbManager::new(
    //     usb_alloc.as_ref().unwrap(),
    //     //https://pid.codes
    //     0x0002,
    // )));

    let mut usb_keyboard = usbd_hid_devices::hid::UsbHidClass::new(
        &usb_alloc,
        usbd_hid_devices::keyboard::HidBootKeyboard::default(),
    );
    // Create a USB device with https://pid.code VID and a test PID
    let mut usb_device = UsbDeviceBuilder::new(&usb_alloc, UsbVidPid(0x1209, 0x02))
        .manufacturer("DLKJ")
        .product("pi-nano-choc")
        .serial_number("TEST")
        .device_class(0x03) //HID - from: https://www.usb.org/defined-class-codes
        // .composite_with_iads()
        // .supports_remote_wakeup(true)
        .build();

    //Ensure the host knows a restart has occured
    usb_device.force_reset().ok();

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

    let mut oled_display = OledDisplay::new(&OLED_DISPLAY, &timer);

    // Splash screen
    oled_display.draw_text_screen("Starting...").unwrap();

    info!("Starting");

    let mut keyboard = Keyboard::new(
        SplitMatrix {
            matrix1: DiodePinMatrix::new(rows, cols),
            matrix2: UartMatrix::new(uart),
        },
        LayerdKeyboardLayout::new(KEY_MAP),
    );

    let mut fast_countdown = timer.count_down();
    fast_countdown.start(100.nanoseconds());

    let mut slow_countdown = timer.count_down();
    slow_countdown.start(20.milliseconds());

    //let mut led_pin = pins.led.into_readable_output();

    let mut leds = 0;

    let mut keyboard_state = None;

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

            oled_display
                .draw_left_display(leds.into(), &state.keycodes, state.layer)
                .ok();

            keyboard_state = Some(state);
        }

        if usb_device.poll(&mut [&mut usb_keyboard]) {
            leds = match usb_keyboard.read_leds() {
                Ok(leds) => leds,

                Err(UsbError::WouldBlock) => leds,
                Err(_) => 0,
            };

            if let Some(state) = keyboard_state.as_ref() {
                if usb_keyboard
                    .write_keycodes(state.keycodes.iter().map(|&k| k as u8))
                    .is_ok()
                {
                    keyboard_state = None;
                }
            }
        }
    }
}

#[inline(never)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    error!("{}", info);

    let mut output = arrayvec::ArrayString::<1024>::new();
    if write!(&mut output, "{}", info).ok().is_some() {
        cortex_m::interrupt::free(|cs| {
            let mut display_ref = OLED_DISPLAY.borrow(cs).borrow_mut();
            if let Some(display) = display_ref.as_mut() {
                display.clear();
                let character_style = MonoTextStyle::new(&FONT_4X6, BinaryColor::On);
                let textbox_style = TextBoxStyleBuilder::new()
                    .height_mode(HeightMode::FitToText)
                    .alignment(HorizontalAlignment::Left)
                    .build();
                let bounds = Rectangle::new(Point::zero(), Size::new(32, 0));
                let text_box = TextBox::with_textbox_style(
                    output.as_str(),
                    bounds,
                    character_style,
                    textbox_style,
                );

                text_box.draw(display)?;
                display.flush()
            } else {
                Ok(())
            }
        })
        .ok();
    }

    loop {
        atomic::compiler_fence(Ordering::SeqCst);
    }
}
