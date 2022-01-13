#![no_std]
#![no_main]

//USB serial console (minicom -b 115200 -o -D /dev/ttyACM0)

use app::keyboard::*;
use app::oled_display::OledDisplay;
use app::usb::UsbManager;
use core::cell::RefCell;
use core::fmt::Write;
use core::panic::PanicInfo;
use core::sync::atomic::{self, Ordering};
use cortex_m::interrupt::Mutex;
use cortex_m::prelude::_embedded_hal_timer_CountDown;
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
use ssd1306::mode::BufferedGraphicsMode;
use ssd1306::{prelude::*, size::DisplaySize128x32, I2CDisplayInterface, Ssd1306};
use usb_device::class_prelude::*;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GD25Q64CS;

type BufferedSsd1306 = Ssd1306<
    I2CInterface<hal::I2C<pac::I2C0, (Pin<Gpio16, Function<I2C>>, Pin<Gpio17, Function<I2C>>)>>,
    DisplaySize128x32,
    BufferedGraphicsMode<DisplaySize128x32>,
>;

static USB_MANAGER: Mutex<RefCell<Option<UsbManager<hal::usb::UsbBus>>>> =
    Mutex::new(RefCell::new(None));
static OLED_DISPLAY: Mutex<RefCell<Option<BufferedSsd1306>>> = Mutex::new(RefCell::new(None));

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

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);

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
            clocks.peripheral_clock.freq(),
        );

        let interface = I2CDisplayInterface::new(i2c);
        let mut display = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate90)
            .into_buffered_graphics_mode();
        display.init().unwrap();
        display.flush().unwrap();

        OLED_DISPLAY.borrow(cs).replace(Some(display));

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

            USB_MANAGER.borrow(cs).replace(Some(UsbManager::new(
                USB_BUS.as_ref().unwrap(),
                //https://pid.codes
                0x0003,
            )));
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

    let uart = UartPeripheral::<_, _>::new(pac.UART0, &mut pac.RESETS)
        .enable(
            uart::common_configs::_19200_8_N_1,
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    let _tx_pin = pins.gpio12.into_mode::<FunctionUart>();
    let _rx_pin = pins.gpio13.into_mode::<FunctionUart>();

    start(timer, uart, rows, cols);
}

fn start<U>(timer: Timer, mut uart: U, rows: [DynPin; 6], cols: [DynPin; 6]) -> !
where
    U: embedded_hal::serial::Write<u8>,
    U::Error: core::fmt::Debug,
{
    let mut oled_display = OledDisplay::new(&OLED_DISPLAY, &timer);

    // Splash screen
    oled_display.draw_text_screen("Starting...").unwrap();

    let mut cd = timer.count_down();
    cd.start(2.seconds());
    block!(cd.wait()).unwrap();

    info!("macropad starting");

    let mut matrix = DiodePinMatrix::new(rows, cols);

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

            matrix.update().expect("Failed to update keys");
        }

        //10ms
        if slow_countdown.wait().is_ok() {
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
