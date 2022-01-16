#![no_std]

use crate::oled_display::FlushableDisplay;
use display_interface::DisplayError;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::DrawTarget;

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
