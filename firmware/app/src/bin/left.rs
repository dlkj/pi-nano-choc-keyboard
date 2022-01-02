#![no_std]
#![no_main]

//USB serial console (minicom -b 115200 -o -D /dev/ttyACM0)

use app::keyboard::keycode::KeyCode;
use app::usb::UsbManager;
use app::{keyboard::*, KeyboardRuntime};
use core::cell::RefCell;
use core::panic::PanicInfo;
use core::sync::atomic::{self, Ordering};
use core::{fmt, fmt::Write};
use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;
use log::{error, LevelFilter};
use log::{Level, Metadata, Record};
use rp_pico::hal;
use rp_pico::hal::clocks::{self, ClocksManager};
use rp_pico::hal::gpio::I2C;
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

//keypad, final row: '0', '.', 'enter'
const KEY_MAP: [KeyAction; 36] = [
    KeyAction::Key {
        code: KeyCode::Escape,
    },
    KeyAction::Key { code: KeyCode::Kb1 },
    KeyAction::Key { code: KeyCode::Kb2 },
    KeyAction::Key { code: KeyCode::Kb3 },
    KeyAction::Key { code: KeyCode::Kb4 },
    KeyAction::Key { code: KeyCode::Kb5 },
    KeyAction::Key { code: KeyCode::Tab },
    KeyAction::Key { code: KeyCode::Q },
    KeyAction::Key { code: KeyCode::W },
    KeyAction::Key { code: KeyCode::E },
    KeyAction::Key { code: KeyCode::R },
    KeyAction::Key { code: KeyCode::T },
    KeyAction::Key {
        code: KeyCode::BackslashISO,
    },
    KeyAction::Key { code: KeyCode::A },
    KeyAction::Key { code: KeyCode::S },
    KeyAction::Key { code: KeyCode::D },
    KeyAction::Key { code: KeyCode::F },
    KeyAction::Key { code: KeyCode::G },
    KeyAction::Key {
        code: KeyCode::LeftShift,
    },
    KeyAction::Key { code: KeyCode::Z },
    KeyAction::Key { code: KeyCode::X },
    KeyAction::Key { code: KeyCode::C },
    KeyAction::Key { code: KeyCode::V },
    KeyAction::Key { code: KeyCode::B },
    KeyAction::Key {
        code: KeyCode::LeftControl,
    },
    KeyAction::Key {
        code: KeyCode::LeftGUI,
    },
    KeyAction::Key { code: KeyCode::Kb7 },
    KeyAction::Key { code: KeyCode::Kb8 },
    KeyAction::Key {
        code: KeyCode::Backspace,
    },
    KeyAction::Key {
        code: KeyCode::LeftBracket,
    },
    KeyAction::Key {
        code: KeyCode::None,
    },
    KeyAction::Key {
        code: KeyCode::LeftAlt,
    },
    KeyAction::Key { code: KeyCode::Kb9 },
    KeyAction::Key { code: KeyCode::Kb0 },
    KeyAction::Key {
        code: KeyCode::Spacebar,
    },
    KeyAction::Key { code: KeyCode::Y },
];

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
            clocks.peripheral_clock,
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

    KeyboardRuntime::run(&OLED_DISPLAY, &USB_MANAGER, timer, KEY_MAP, rows, cols);
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
