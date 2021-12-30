#![no_std]
#![no_main]

//USB serial console (minicom -b 115200 -o -D /dev/ttyACM0)

mod keyboard;
mod logger;
mod panic;
mod rotary_enc;
mod usb;
mod debounce;


use embedded_hal::digital::v2::ToggleableOutputPin;
use embedded_time::duration::Extensions;
use embedded_time::fixed_point::FixedPoint;
use rp_pico::{
    hal::{
        self as rp2040_hal,
        Clock,
        pac::{self, interrupt},
        sio::Sio,
        timer::Timer,
        watchdog::Watchdog,
    },
    Pins,
};
use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;
use embedded_hal::prelude::*;
use keyboard::keycode::KeyCode;
use keyboard::Keyboard;
use log::{info, LevelFilter};
use rp2040_hal::gpio::dynpin::DynPin;
use usb_device::class_prelude::*;
use usbd_hid::descriptor::KeyboardReport;
use embedded_hal::digital::v2::OutputPin;


#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GD25Q64CS;

static USB_MANAGER: Mutex<RefCell<Option<usb::UsbManager<rp2040_hal::usb::UsbBus>>>> =
    Mutex::new(RefCell::new(None));
static LOGGER: logger::MacropadLogger = logger::MacropadLogger;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    let external_xtal_freq_hz = 12_000_000u32;
    let clocks: rp2040_hal::clocks::ClocksManager = rp2040_hal::clocks::init_clocks_and_plls(
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

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);

    let sio = Sio::new(pac.SIO);
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    
    cortex_m::interrupt::free(|cs| {
        // Note (safety): interupts not yet enabled

        //Init USB
        static mut USB_BUS: Option<UsbBusAllocator<rp2040_hal::usb::UsbBus>> = None;

        unsafe {
            USB_BUS = Some(UsbBusAllocator::new(rp2040_hal::usb::UsbBus::new(
                pac.USBCTRL_REGS,
                pac.USBCTRL_DPRAM,
                clocks.usb_clock,
                true,
                &mut pac.RESETS,
            )));

            USB_MANAGER
                .borrow(cs)
                .replace(Some(usb::UsbManager::new(USB_BUS.as_ref().unwrap())));

            log::set_logger_racy(&LOGGER).unwrap();
        }
    });

    log::set_max_level(LevelFilter::Info);

    // Enable the USB interrupt
    unsafe {
        pac::NVIC::unmask(rp2040_hal::pac::Interrupt::USBCTRL_IRQ);
    };

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());
    delay.delay_ms(2000);

    info!("macropad starting");

    let rot_pin_a =
        debounce::DebouncedPin::<DynPin>::new(pins.gpio16.into_pull_up_input().into(), true);
    let rot_pin_b =
        debounce::DebouncedPin::<DynPin>::new(pins.gpio17.into_pull_up_input().into(), true);

    let mut rot_enc = rotary_enc::RotaryEncoder::new(rot_pin_a, rot_pin_b);

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

    //keypad, final row: '0', '.', 'enter'
    const KEY_MAP: [keyboard::KeyAction; 36] = [
        keyboard::KeyAction::Key { code: KeyCode::Escape },
        keyboard::KeyAction::Key { code: KeyCode::Kb1 },
        keyboard::KeyAction::Key { code: KeyCode::Kb2 },
        keyboard::KeyAction::Key { code: KeyCode::Kb3 },
        keyboard::KeyAction::Key { code: KeyCode::Kb4 },
        keyboard::KeyAction::Key { code: KeyCode::Kb5 },
        keyboard::KeyAction::Key { code: KeyCode::Tab },
        keyboard::KeyAction::Key { code: KeyCode::Q },
        keyboard::KeyAction::Key { code: KeyCode::W },
        keyboard::KeyAction::Key { code: KeyCode::E },
        keyboard::KeyAction::Key { code: KeyCode::R },
        keyboard::KeyAction::Key { code: KeyCode::T },
        keyboard::KeyAction::Key { code: KeyCode::BackslashISO },
        keyboard::KeyAction::Key { code: KeyCode::A },
        keyboard::KeyAction::Key { code: KeyCode::S },
        keyboard::KeyAction::Key { code: KeyCode::D },
        keyboard::KeyAction::Key { code: KeyCode::F },
        keyboard::KeyAction::Key { code: KeyCode::G },
        keyboard::KeyAction::Key { code: KeyCode::LeftShift },
        keyboard::KeyAction::Key { code: KeyCode::Z },
        keyboard::KeyAction::Key { code: KeyCode::X },
        keyboard::KeyAction::Key { code: KeyCode::C },
        keyboard::KeyAction::Key { code: KeyCode::V },
        keyboard::KeyAction::Key { code: KeyCode::B },
        keyboard::KeyAction::Key { code: KeyCode::LeftControl },
        keyboard::KeyAction::Key { code: KeyCode::LeftGUI },
        keyboard::KeyAction::Key { code: KeyCode::Kb7 },
        keyboard::KeyAction::Key { code: KeyCode::Kb8 },
        keyboard::KeyAction::Key { code: KeyCode::Backspace },
        keyboard::KeyAction::Key { code: KeyCode::LeftBracket },
        keyboard::KeyAction::Key { code: KeyCode::None },
        keyboard::KeyAction::Key { code: KeyCode::LeftAlt },
        keyboard::KeyAction::Key { code: KeyCode::Kb9 },
        keyboard::KeyAction::Key { code: KeyCode::Kb0 },
        keyboard::KeyAction::Key { code: KeyCode::Spacebar },
        keyboard::KeyAction::Key { code: KeyCode::Y },

    ];

    let mut keyboard = Keyboard::new(
        keyboard::DiodePinMatrix::new(rows, cols),
        keyboard::BasicKeyboardLayout::new(KEY_MAP),
    );

    let mut fast_countdown = timer.count_down();
    fast_countdown.start(100.nanoseconds());

    let mut slow_countdown = timer.count_down();
    slow_countdown.start(20.milliseconds());

    let mut led_pin = pins.led.into_readable_output();
    
    info!("Running main loop");
    loop {
        //0.1ms scan the keys and debounce
        if fast_countdown.wait().is_ok() {

            let (p_a, p_b) = rot_enc.pins_borrow_mut();
            p_a.update().expect("Failed to update rot a debouncer");
            p_b.update().expect("Failed to update rot b debouncer");

            //todo: move onto an interupt timer
            rot_enc.update();

            keyboard.update().expect("Failed to update keyboard");
        }

        //10ms
        if slow_countdown.wait().is_ok() {
            led_pin.toggle().unwrap();

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
        }
    }
}

fn get_hid_report<const N: usize>(state: &keyboard::KeyboardState<N>) -> KeyboardReport {
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
}
