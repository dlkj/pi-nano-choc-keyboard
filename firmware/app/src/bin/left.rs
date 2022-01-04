#![no_std]
#![no_main]

//USB serial console (minicom -b 115200 -o -D /dev/ttyACM0)

use app::keyboard::keycode::KeyCode;
use app::keyboard::*;
use app::usb::UsbManager;
use core::cell::RefCell;
use core::panic::PanicInfo;
use core::sync::atomic::{self, Ordering};
use core::{fmt, fmt::Write};
use cortex_m::interrupt::Mutex;
use cortex_m::prelude::_embedded_hal_timer_CountDown;
use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;
use embedded_time::duration::Extensions;
use log::{error, info, LevelFilter};
use log::{Level, Metadata, Record};
use nb::block;
use rp_pico::hal::clocks::{self, ClocksManager};
use rp_pico::hal::gpio::{FunctionUart, I2C};
use rp_pico::hal::uart::{self, UartPeripheral};
use rp_pico::hal::{self, Clock};
use rp_pico::{
    hal::{
        gpio::{bank0::*, DynPin, Function, Pin},
        pac::{self, interrupt},
        sio::Sio,
        timer::Timer,
        watchdog::Watchdog,
    },
    Pins,
};
use ssd1306::{prelude::*, size::DisplaySize128x32, I2CDisplayInterface, Ssd1306};
use usb_device::class_prelude::*;
use usbd_hid::descriptor::KeyboardReport;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GD25Q64CS;

type OledDisplay = app::oled_display::OledDisplay<
    I2CInterface<hal::I2C<pac::I2C1, (Pin<Gpio14, Function<I2C>>, Pin<Gpio15, Function<I2C>>)>>,
    DisplaySize128x32,
>;

static USB_MANAGER: Mutex<RefCell<Option<app::usb::UsbManager<hal::usb::UsbBus>>>> =
    Mutex::new(RefCell::new(None));
static LOGGER: KeyboardLogger = KeyboardLogger;
static OLED_DISPLAY: Mutex<RefCell<Option<OledDisplay>>> = Mutex::new(RefCell::new(None));

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
const LOWER_MAP: [KeyAction; 72] = [
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

const UPPER_MAP: [KeyAction; 72] = [
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

    let external_xtal_freq_hz = 12_000_000u32;
    let clocks: ClocksManager = clocks::init_clocks_and_plls(
        external_xtal_freq_hz,
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

    cortex_m::interrupt::free(|cs| {
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

        OLED_DISPLAY
            .borrow(cs)
            .replace(Some(OledDisplay::new(display)));

        //Init USB
        static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;

        // Note (safety): interupts not yet enabled
        unsafe {
            USB_BUS = Some(UsbBusAllocator::new(hal::usb::UsbBus::new(
                pac.USBCTRL_REGS,
                pac.USBCTRL_DPRAM,
                clocks.usb_clock,
                true,
                &mut pac.RESETS,
            )));

            USB_MANAGER
                .borrow(cs)
                .replace(Some(UsbManager::new(USB_BUS.as_ref().unwrap())));

            log::set_logger_racy(&LOGGER).unwrap();
        }
    });

    log::set_max_level(LevelFilter::Info);

    // Enable the USB interrupt
    unsafe {
        pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
    };

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

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);

    let uart = UartPeripheral::<_, _>::new(pac.UART0, &mut pac.RESETS)
        .enable(
            uart::common_configs::_19200_8_N_1,
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    let _tx_pin = pins.gpio12.into_mode::<FunctionUart>();
    let _rx_pin = pins.gpio13.into_mode::<FunctionUart>();

    start(timer, uart, KEY_MAP, rows, cols);
}

fn start<U>(
    timer: Timer,
    uart: U,
    keymaps: [[KeyAction; 72]; 3],
    rows: [DynPin; 6],
    cols: [DynPin; 6],
) -> !
where
    U: embedded_hal::serial::Read<u8>,
{
    // Splash screen
    cortex_m::interrupt::free(|cs| {
        let mut oled_display_ref = OLED_DISPLAY.borrow(cs).borrow_mut();
        let oled_display = oled_display_ref.as_mut().unwrap();
        oled_display.draw_text_screen("Starting...").unwrap();
    });

    let mut cd = timer.count_down();
    cd.start(2.seconds());
    block!(cd.wait()).unwrap();

    info!("macropad starting");

    let mut keyboard = Keyboard::new(
        SplitMatrix {
            matrix1: DiodePinMatrix::new(rows, cols),
            matrix2: UartMatrix::new(uart),
        },
        LayerdKeyboardLayout::new(keymaps),
    );

    let mut fast_countdown = timer.count_down();
    fast_countdown.start(100.nanoseconds());

    let mut slow_countdown = timer.count_down();
    slow_countdown.start(20.milliseconds());

    //let mut led_pin = pins.led.into_readable_output();

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
            let keyboard_state = keyboard.state().unwrap();
            let keyboard_report = get_hid_report(&keyboard_state);

            //todo - spin lock until usb ready to recive, reset timers
            cortex_m::interrupt::free(|cs| {
                let mut usb_ref = USB_MANAGER.borrow(cs).borrow_mut();
                if let Some(usb) = usb_ref.as_mut() {
                    usb.keyboard_borrow_mut().push_input(&keyboard_report).ok();
                }
            });

            let mut output = arrayvec::ArrayString::<1024>::new();
            if write!(
                &mut output,
                "k:\ns{:#04X?}\n\nm:\n{:08b}",
                keyboard_report.keycodes, keyboard_report.modifier
            )
            .ok()
            .is_some()
            {
                cortex_m::interrupt::free(|cs| {
                    let mut display_ref = OLED_DISPLAY.borrow(cs).borrow_mut();
                    if let Some(display) = display_ref.as_mut() {
                        display.draw_text_screen(output.as_str()).ok();
                    }
                });
            }
        }
    }
}

fn get_hid_report<const N: usize>(state: &KeyboardState<N>) -> KeyboardReport {
    //get first 6 current keypresses and send to usb
    let mut keycodes: [u8; 6] = [0, 0, 0, 0, 0, 0];

    let mut keycodes_it = keycodes.iter_mut();

    for k in &state.keycodes {
        match keycodes_it.next() {
            Some(kc) => {
                *kc = *k as u8;
            }
            None => {
                keycodes.fill(0x01); //Error roll over
                break;
            }
        }
    }

    KeyboardReport {
        modifier: state.modifiers.bits(),
        leds: 0,
        reserved: 0,
        keycodes,
    }
}

#[allow(non_snake_case)]
#[interrupt]
fn USBCTRL_IRQ() {
    cortex_m::interrupt::free(|cs| {
        let mut usb_ref = USB_MANAGER.borrow(cs).borrow_mut();
        if let Some(usb) = usb_ref.as_mut() {
            usb.service_irq();
        }
    });
    cortex_m::asm::sev();
}

pub struct KeyboardLogger;

impl fmt::Write for KeyboardLogger {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        cortex_m::interrupt::free(|cs| {
            let mut usb_ref = USB_MANAGER.borrow(cs).borrow_mut();
            if let Some(usb) = usb_ref.as_mut() {
                usb.serial_port_borrow_mut()
                    .write(s.as_bytes())
                    .map_or_else(
                        |_error| fmt::Result::Err(fmt::Error),
                        |_c| fmt::Result::Ok(()),
                    )
            } else {
                fmt::Result::Ok(())
            }
        })
    }
}

impl log::Log for KeyboardLogger {
    fn enabled(&self, metadata: &Metadata) -> bool {
        metadata.level() <= Level::Info
    }

    fn log(&self, record: &Record) {
        if self.enabled(record.metadata()) {
            let mut writer = KeyboardLogger;
            //Errors are likely due to serial port not connected, better to swallow failures than panic
            write!(&mut writer, "{} - {}\r\n", record.level(), record.args()).ok();
        }
    }

    fn flush(&self) {}
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
                display.draw_text_screen(output.as_str()).ok();
            }
        });
    }

    loop {
        atomic::compiler_fence(Ordering::SeqCst);
    }
}
