#![no_std]

#[macro_use(block)]
extern crate nb;

use core::{cell::RefCell, fmt::Write};
use cortex_m::{interrupt::Mutex, prelude::*};
use display_interface::WriteOnlyDataCommand;
use embedded_time::duration::Extensions;
use keyboard::KeyboardState;
use log::info;
use oled_display::OledDisplay;
use rp_pico::hal::{gpio::DynPin, Timer};
use ssd1306::size::DisplaySize128x32;
use usb::UsbManager;
use usbd_hid::descriptor::KeyboardReport;

use crate::keyboard::{DiodePinMatrix, KeyAction, Keyboard, LayerdKeyboardLayout};

pub mod debounce;
pub mod keyboard;
pub mod oled_display;
pub mod rotary_enc;
pub mod usb;

pub struct KeyboardRuntime {}
//use a builder pattern?

impl KeyboardRuntime {
    pub fn run<W>(
        display: &Mutex<RefCell<Option<OledDisplay<W, DisplaySize128x32>>>>,
        usb_manager: &Mutex<RefCell<Option<UsbManager<rp_pico::hal::usb::UsbBus>>>>,
        timer: Timer,
        keymaps: [[KeyAction; 36]; 3],
        rows: [DynPin; 6],
        cols: [DynPin; 6],
    ) -> !
    where
        W: WriteOnlyDataCommand,
    {
        // Slash screen
        cortex_m::interrupt::free(|cs| {
            let mut oled_display_ref = display.borrow(cs).borrow_mut();
            let oled_display = oled_display_ref.as_mut().unwrap();
            oled_display.draw_text_screen("Starting...").unwrap();
        });

        let mut cd = timer.count_down();
        cd.start(2.seconds());
        block!(cd.wait()).unwrap();

        info!("macropad starting");

        let mut keyboard = Keyboard::new(
            DiodePinMatrix::new(rows, cols),
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
                    let mut usb_ref = usb_manager.borrow(cs).borrow_mut();
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
                        let mut display_ref = display.borrow(cs).borrow_mut();
                        if let Some(display) = display_ref.as_mut() {
                            display.draw_text_screen(output.as_str()).ok();
                        }
                    });
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
    }
}
