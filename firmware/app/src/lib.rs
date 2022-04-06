#![no_std]

use cortex_m::interrupt::Mutex;
use display_interface::DisplayError;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::DrawTarget;
use embedded_time::duration::Fraction;
use embedded_time::Instant;
use rp_pico::hal::Timer;

use crate::oled_display::FlushableDisplay;

pub mod debounce;
pub mod key_map;
pub mod keyboard;
pub mod oled_display;
pub mod rotary_enc;

pub fn check_for_persisted_panic<D>(display: &mut oled_display::OledDisplay<'_, D>)
where
    D: DrawTarget<Color = BinaryColor, Error = DisplayError> + FlushableDisplay,
{
    if let Some(msg) = panic_persist::get_panic_message_utf8() {
        display.draw_text_screen(msg).ok();
        //USB boot with pin 25 for usb activity
        //Screen will continue to show panic message
        rp_pico::hal::rom_data::reset_to_usb_boot(0x1 << 25, 0x0);
    }
}

pub struct SyncTimerClock {
    timer: Mutex<Timer>,
}

impl SyncTimerClock {
    pub fn new(timer: Timer) -> Self {
        Self {
            timer: Mutex::new(timer),
        }
    }
}

impl<'a> embedded_time::clock::Clock for SyncTimerClock {
    //using u64 to avoid clock wrapping issues
    type T = u64;

    const SCALING_FACTOR: Fraction = Fraction::new(1, 1_000_000u32);

    fn try_now(&self) -> Result<Instant<Self>, embedded_time::clock::Error> {
        cortex_m::interrupt::free(|cs| Ok(Instant::new(self.timer.borrow(cs).get_counter())))
    }
}
