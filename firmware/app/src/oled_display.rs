use core::cell::RefCell;
use core::fmt::Write;
use cortex_m::interrupt::Mutex;
use display_interface::DisplayError;
use embedded_graphics::primitives::RoundedRectangle;
use embedded_graphics::{
    image::{Image, ImageRawLE},
    mono_font::{ascii::*, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
    text::{Alignment, Text},
};
use embedded_text::{
    alignment::HorizontalAlignment,
    style::{HeightMode, TextBoxStyleBuilder},
    TextBox,
};
use rand::prelude::SmallRng;
use rand::{RngCore, SeedableRng};
use rp_pico::hal::Timer;

use crate::keyboard;

pub trait FlushableDisplay {
    fn flush(&mut self) -> Result<(), DisplayError>;
}

impl<DI, SIZE> FlushableDisplay
    for ssd1306::Ssd1306<DI, SIZE, ssd1306::mode::BufferedGraphicsMode<SIZE>>
where
    DI: display_interface::WriteOnlyDataCommand,
    SIZE: ssd1306::size::DisplaySize,
{
    fn flush(&mut self) -> Result<(), display_interface::DisplayError> {
        self.flush()
    }
}

pub struct OledDisplay<'a, D> {
    display: &'a Mutex<RefCell<Option<D>>>,
    timer: &'a Timer,
    last_active: u64,
    rng: SmallRng,
}

impl<'a, D> OledDisplay<'a, D>
where
    D: DrawTarget<Color = BinaryColor, Error = DisplayError> + FlushableDisplay,
{
    pub fn new(display: &'a Mutex<RefCell<Option<D>>>, timer: &'a Timer) -> OledDisplay<'a, D> {
        let now = timer.get_counter();

        OledDisplay {
            display,
            timer,
            last_active: now,
            rng: SmallRng::seed_from_u64(now),
        }
    }

    fn exclusive<F>(&self, f: F) -> Result<(), DisplayError>
    where
        F: FnOnce(&mut D) -> Result<(), DisplayError>,
    {
        cortex_m::interrupt::free(|cs| {
            let mut display_ref = self.display.borrow(cs).borrow_mut();
            let display = display_ref.as_mut();
            f(display.unwrap())
        })
    }

    pub fn draw_image(&mut self, data: &[u8], width: u32) -> Result<(), DisplayError> {
        self.exclusive(|display| {
            display.clear(BinaryColor::Off)?;

            let img: ImageRawLE<BinaryColor> = ImageRawLE::new(data, width);
            Image::new(&img, Point::new(32, 0)).draw(display)?;

            display.flush()
        })
    }

    fn draw_led_indicator(
        display: &mut D,
        top_left: Point,
        text: &str,
    ) -> Result<(), DisplayError> {
        let led_text_style = MonoTextStyle::new(&FONT_6X13_BOLD, BinaryColor::Off);
        let fill = PrimitiveStyle::with_fill(BinaryColor::On);

        let bounding_box = Rectangle::new(top_left, Size::new(8, 11));

        RoundedRectangle::with_equal_corners(bounding_box, Size::new(2, 2))
            .into_styled(fill)
            .draw(display)?;

        Text::with_alignment(
            text,
            bounding_box.center() + Point::new(0, 4),
            led_text_style,
            Alignment::Center,
        )
        .draw(display)?;
        Ok(())
    }

    fn draw_layer_indicator(
        display: &mut D,
        top_left: Point,
        layer: usize,
    ) -> Result<(), DisplayError> {
        let text_style = MonoTextStyle::new(&FONT_4X6, BinaryColor::On);

        let bounding_box = Rectangle::new(top_left, Size::new(32, 6));

        let mut buffer = arrayvec::ArrayString::<256>::new();
        let text = if layer == 0 {
            "Base"
        } else {
            write!(&mut buffer, "Layer {}", layer).unwrap();
            &buffer[..]
        };

        Text::with_alignment(
            text,
            bounding_box.center() + Point::new(0, 4),
            text_style,
            Alignment::Center,
        )
        .draw(display)?;
        Ok(())
    }

    fn draw_keycode_indicator(
        display: &mut D,
        top_left: Point,
        modifier: keyboard::keycode::Modifiers,
        keycodes: [u8; 6],
    ) -> Result<(), DisplayError> {
        let thin_stroke = PrimitiveStyle::with_stroke(BinaryColor::On, 1);
        let bounding_box = Rectangle::new(top_left, Size::new(32, 16));
        let fill = PrimitiveStyle::with_fill(BinaryColor::On);

        if !modifier.is_empty() {
            RoundedRectangle::with_equal_corners(
                Rectangle::with_center(bounding_box.center(), Size::new(16, 16)),
                Size::new(3, 3),
            )
            .into_styled(thin_stroke)
            .draw(display)?;
        }

        if keycodes.iter().any(|&k| k != 0) {
            RoundedRectangle::with_equal_corners(
                Rectangle::with_center(bounding_box.center(), Size::new(12, 12)),
                Size::new(2, 2),
            )
            .into_styled(fill)
            .draw(display)?;
        }

        Ok(())
    }

    pub fn draw_left_display(
        &mut self,
        leds: keyboard::keycode::Leds,
        modifier: keyboard::keycode::Modifiers,
        keycodes: [u8; 6],
        layer: usize,
    ) -> Result<(), DisplayError> {
        let now = self.timer.get_counter();

        if !modifier.is_empty() || keycodes.iter().any(|&k| k != 0) {
            self.last_active = now;
        }

        if now - self.last_active > 10_000_000 {
            self.draw_screen_saver()
        } else {
            self.exclusive(|display| {
                display.clear(BinaryColor::Off)?;

                //Led indicators
                if leds.contains(keyboard::keycode::Leds::CAP_LOCK) {
                    Self::draw_led_indicator(display, Point::new(0, 0), "C")?;
                }

                if leds.contains(keyboard::keycode::Leds::NUM_LOCK) {
                    Self::draw_led_indicator(display, Point::new(12, 0), "1")?;
                }

                if leds.contains(keyboard::keycode::Leds::SCROLL_LOCK) {
                    Self::draw_led_indicator(display, Point::new(24, 0), "S")?;
                }

                //Layer
                Self::draw_layer_indicator(display, Point::new(0, 13), layer)?;

                //Keycode indicator
                Self::draw_keycode_indicator(display, Point::new(0, 112), modifier, keycodes)?;

                display.flush()
            })
        }
    }

    pub fn draw_right_display(&mut self, pressed_keys: &[usize]) -> Result<(), DisplayError> {
        let now = self.timer.get_counter();

        if !pressed_keys.len() != 0 {
            self.last_active = now;
        }

        if now - self.last_active > 10_000_000 {
            self.draw_screen_saver()
        } else {
            let mut output = arrayvec::ArrayString::<1024>::new();
            if write!(&mut output, "k:\ns{:#02?}\n", &pressed_keys)
                .ok()
                .is_some()
            {
                self.draw_text_screen(output.as_str())?
            }

            Ok(())
        }
    }

    fn draw_screen_saver(&mut self) -> Result<(), DisplayError> {
        let pixels = [Pixel(
            Point::new(
                self.rng.next_u32() as i32 % 32,
                self.rng.next_u32() as i32 % 128,
            ),
            BinaryColor::On,
        )];

        self.exclusive(|display| {
            display.clear(BinaryColor::Off)?;
            display.draw_iter(pixels)?;
            display.flush()
        })
    }

    pub fn draw_text_screen(&mut self, text: &str) -> Result<(), DisplayError> {
        self.exclusive(|display| {
            display.clear(BinaryColor::Off)?;
            let character_style = MonoTextStyle::new(&FONT_4X6, BinaryColor::On);
            let textbox_style = TextBoxStyleBuilder::new()
                .height_mode(HeightMode::FitToText)
                .alignment(HorizontalAlignment::Left)
                .build();
            let bounds = Rectangle::new(Point::zero(), Size::new(32, 0));
            let text_box =
                TextBox::with_textbox_style(text, bounds, character_style, textbox_style);

            text_box.draw(display)?;
            display.flush()?;

            Ok(())
        })
    }
}
