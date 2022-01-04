use core::fmt::Write;
use display_interface::DisplayError;
use embedded_graphics::primitives::{Line, RoundedRectangle};
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
use ssd1306::mode::BufferedGraphicsMode;
use ssd1306::{prelude::*, Ssd1306};

use crate::keyboard;

pub struct OledDisplay<DI, SIZE>
where
    DI: WriteOnlyDataCommand,
    SIZE: ssd1306::mode::TerminalDisplaySize,
{
    display: Ssd1306<DI, SIZE, BufferedGraphicsMode<SIZE>>,
    rng: SmallRng,
}

impl<DI, SIZE> OledDisplay<DI, SIZE>
where
    DI: WriteOnlyDataCommand,
    SIZE: ssd1306::mode::TerminalDisplaySize,
{
    pub fn new(
        display: Ssd1306<DI, SIZE, BufferedGraphicsMode<SIZE>>,
        rng_seed: u64,
    ) -> OledDisplay<DI, SIZE> {
        OledDisplay {
            display,
            rng: SmallRng::seed_from_u64(rng_seed),
        }
    }

    pub fn draw_image(&mut self, data: &[u8], width: u32) -> Result<(), DisplayError> {
        self.display.clear();

        let img: ImageRawLE<BinaryColor> = ImageRawLE::new(data, width);
        Image::new(&img, Point::new(32, 0))
            .draw(&mut self.display)
            .unwrap();

        self.display.flush()?;

        Ok(())
    }

    fn draw_led_indicator(
        display: &mut Ssd1306<DI, SIZE, BufferedGraphicsMode<SIZE>>,
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
        display: &mut Ssd1306<DI, SIZE, BufferedGraphicsMode<SIZE>>,
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
        display: &mut Ssd1306<DI, SIZE, BufferedGraphicsMode<SIZE>>,
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

    pub fn draw_right_display(
        &mut self,
        leds: keyboard::keycode::Leds,
        modifier: keyboard::keycode::Modifiers,
        keycodes: [u8; 6],
        layer: usize,
    ) -> Result<(), DisplayError> {
        self.display.clear();

        //Led indicators
        if leds.contains(keyboard::keycode::Leds::NUM_LOCK) {
            Self::draw_led_indicator(&mut self.display, Point::new(0, 0), "1")?;
        }

        if leds.contains(keyboard::keycode::Leds::CAP_LOCK) {
            Self::draw_led_indicator(&mut self.display, Point::new(12, 0), "C")?;
        }

        if leds.contains(keyboard::keycode::Leds::SCROLL_LOCK) {
            Self::draw_led_indicator(&mut self.display, Point::new(24, 0), "S")?;
        }

        //Layer
        Self::draw_layer_indicator(&mut self.display, Point::new(0, 13), layer)?;

        //Keycode indicator
        Self::draw_keycode_indicator(&mut self.display, Point::new(0, 112), modifier, keycodes)?;

        self.display.flush()?;

        Ok(())
    }

    pub fn draw_screen_saver(&mut self) -> Result<(), DisplayError> {
        self.display.clear();

        let pixels = [Pixel(
            Point::new(
                self.rng.next_u32() as i32 % 32,
                self.rng.next_u32() as i32 % 128,
            ),
            BinaryColor::On,
        )];
        self.display.draw_iter(pixels)?;

        self.display.flush()?;
        Ok(())
    }

    pub fn draw_text_screen(&mut self, text: &str) -> Result<(), DisplayError> {
        self.display.clear();
        let character_style = MonoTextStyle::new(&FONT_4X6, BinaryColor::On);
        let textbox_style = TextBoxStyleBuilder::new()
            .height_mode(HeightMode::FitToText)
            .alignment(HorizontalAlignment::Left)
            .build();
        let bounds = Rectangle::new(Point::zero(), Size::new(32, 0));
        let text_box = TextBox::with_textbox_style(text, bounds, character_style, textbox_style);

        text_box.draw(&mut self.display).unwrap();
        self.display.flush()?;

        Ok(())
    }

    pub fn draw_pin_log(&mut self, data: &[bool]) {
        self.display.clear();
        for (i, d) in data.iter().enumerate() {
            Line::new(
                Point::new(i as i32, 64),
                Point::new(i as i32, if *d { 60 } else { 62 }),
            )
            .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
            .draw(&mut self.display)
            .unwrap();
        }
        self.display.flush().unwrap();
    }
}
