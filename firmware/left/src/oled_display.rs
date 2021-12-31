use core::fmt::Write;
use display_interface::DisplayError;
use embedded_graphics::primitives::Line;
use embedded_graphics::{
    image::{Image, ImageRawLE},
    mono_font::{ascii::FONT_4X6, ascii::FONT_6X10, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{
        Circle, PrimitiveStyle, PrimitiveStyleBuilder, Rectangle, StrokeAlignment, Triangle,
    },
    text::{Alignment, Text},
};
use embedded_text::{
    alignment::HorizontalAlignment,
    style::{HeightMode, TextBoxStyleBuilder},
    TextBox,
};
use ssd1306::mode::BufferedGraphicsMode;
use ssd1306::{prelude::*, Ssd1306};

pub struct OledDisplay<DI, SIZE>
where
    DI: WriteOnlyDataCommand,
    SIZE: ssd1306::mode::TerminalDisplaySize,
{
    display: Ssd1306<DI, SIZE, BufferedGraphicsMode<SIZE>>,
    //Ssd1306<ssd1306::prelude::I2CInterface<rp_pico::rp2040_hal::I2C<rp_pico::rp2040_pac::I2C1, (rp_pico::rp2040_hal::gpio::Pin<rp_pico::rp2040_hal::gpio::bank0::Gpio14, rp_pico::rp2040_hal::gpio::Function<rp_pico::rp2040_hal::gpio::I2C>>, rp_pico::rp2040_hal::gpio::Pin<rp_pico::rp2040_hal::gpio::bank0::Gpio15, rp_pico::rp2040_hal::gpio::Function<rp_pico::rp2040_hal::gpio::I2C>>)>>, ssd1306::prelude::DisplaySize128x32, ssd1306::mode::TerminalMode>
}

impl<DI, SIZE> OledDisplay<DI, SIZE>
where
    DI: WriteOnlyDataCommand,
    SIZE: ssd1306::mode::TerminalDisplaySize,
{
    pub fn new(display: Ssd1306<DI, SIZE, BufferedGraphicsMode<SIZE>>) -> OledDisplay<DI, SIZE> {
        OledDisplay { display }
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

    pub fn draw_numpad(&mut self, enc_value: i32) -> Result<(), DisplayError> {
        self.display.clear();
        let mut output = arrayvec::ArrayString::<256>::new();
        write!(
            &mut output,
            "7 8 9\n4 5 6\n1 2 3\n0 . E\nEnc: {}",
            enc_value
        )
        .unwrap();
        self.draw_text_screen(output.as_str())
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

    #[allow(dead_code)]
    pub fn draw_test(&mut self) -> Result<(), DisplayError> {
        self.display.clear();

        // Create styles used by the drawing operations.
        let thin_stroke = PrimitiveStyle::with_stroke(BinaryColor::On, 1);
        let thick_stroke = PrimitiveStyle::with_stroke(BinaryColor::On, 3);
        let border_stroke = PrimitiveStyleBuilder::new()
            .stroke_color(BinaryColor::On)
            .stroke_width(3)
            .stroke_alignment(StrokeAlignment::Inside)
            .build();
        let fill = PrimitiveStyle::with_fill(BinaryColor::On);
        let character_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);

        let yoffset = 10;

        // Draw a 3px wide outline around the display.
        self.display
            .bounding_box()
            .into_styled(border_stroke)
            .draw(&mut self.display)
            .unwrap();

        // Draw a triangle.
        Triangle::new(
            Point::new(16, 16 + yoffset),
            Point::new(16 + 16, 16 + yoffset),
            Point::new(16 + 8, yoffset),
        )
        .into_styled(thin_stroke)
        .draw(&mut self.display)
        .unwrap();

        // Draw a filled square
        Rectangle::new(Point::new(52, yoffset), Size::new(16, 16))
            .into_styled(fill)
            .draw(&mut self.display)
            .unwrap();

        // Draw a circle with a 3px wide stroke.
        Circle::new(Point::new(88, yoffset), 17)
            .into_styled(thick_stroke)
            .draw(&mut self.display)
            .unwrap();

        // Draw centered text.
        let text = "embedded-graphics";
        Text::with_alignment(
            text,
            self.display.bounding_box().center() + Point::new(0, 15),
            character_style,
            Alignment::Center,
        )
        .draw(&mut self.display)
        .unwrap();

        self.display.flush()?;

        Ok(())
    }
}
