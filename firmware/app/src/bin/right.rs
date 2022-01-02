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
    I2CInterface<hal::I2C<pac::I2C0, (Pin<Gpio16, Function<I2C>>, Pin<Gpio17, Function<I2C>>)>>,
    DisplaySize128x32,
>;

static USB_MANAGER: Mutex<RefCell<Option<UsbManager<hal::usb::UsbBus>>>> =
    Mutex::new(RefCell::new(None));
static LOGGER: Logger = Logger {};
static OLED_DISPLAY: Mutex<RefCell<Option<OledDisplay>>> = Mutex::new(RefCell::new(None));

const KEY_MAP: [KeyAction; 36] = [
    KeyAction::Key { code: KeyCode::Kb6 },
    KeyAction::Key { code: KeyCode::Kb7 },
    KeyAction::Key { code: KeyCode::Kb8 },
    KeyAction::Key { code: KeyCode::Kb9 },
    KeyAction::Key { code: KeyCode::Kb0 },
    KeyAction::Key {
        code: KeyCode::RightBracket,
    },
    KeyAction::Key { code: KeyCode::Y },
    KeyAction::Key { code: KeyCode::U },
    KeyAction::Key { code: KeyCode::I },
    KeyAction::Key { code: KeyCode::O },
    KeyAction::Key { code: KeyCode::P },
    KeyAction::Key {
        code: KeyCode::LeftBracket,
    },
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
    KeyAction::Key {
        code: KeyCode::RightBracket,
    },
    KeyAction::Key { code: KeyCode::Kp1 },
    KeyAction::Key { code: KeyCode::Kb2 },
    KeyAction::Key { code: KeyCode::Kb3 },
    KeyAction::Key {
        code: KeyCode::Application,
    },
    KeyAction::Key {
        code: KeyCode::RightControl,
    },
    KeyAction::Key { code: KeyCode::Kp5 },
    KeyAction::Key {
        code: KeyCode::Enter,
    },
    KeyAction::Key {
        code: KeyCode::Backspace,
    },
    KeyAction::Key { code: KeyCode::Kb6 },
    KeyAction::Key {
        code: KeyCode::RightAlt,
    },
    KeyAction::Key {
        code: KeyCode::None,
    },
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
        let scl_pin = pins.gpio17.into_mode::<rp_pico::hal::gpio::FunctionI2C>();
        let sda_pin = pins.gpio16.into_mode::<rp_pico::hal::gpio::FunctionI2C>();

        let i2c = rp_pico::hal::i2c::I2C::new_controller(
            pac.I2C0,
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
            .replace(Some(app::oled_display::OledDisplay::new(display)));

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

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);

    KeyboardRuntime::run(&OLED_DISPLAY, &USB_MANAGER, timer, KEY_MAP, rows, cols);
}

#[allow(non_snake_case)]
#[interrupt]
fn USBCTRL_IRQ() {
    cortex_m::interrupt::free(|cs| {
        if let Some(usb) = USB_MANAGER.borrow(cs).borrow_mut().as_mut() {
            usb.service_irq();
        }
    });
    cortex_m::asm::sev();
}

pub struct Logger {}

impl fmt::Write for Logger {
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

impl log::Log for Logger {
    fn enabled(&self, metadata: &Metadata) -> bool {
        metadata.level() <= Level::Info
    }

    fn log(&self, record: &Record) {
        if self.enabled(record.metadata()) {
            let mut writer = Logger {};
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
