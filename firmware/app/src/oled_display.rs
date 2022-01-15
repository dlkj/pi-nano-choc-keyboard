use crate::keyboard::keycode::KeyCode;
use core::fmt::Write;
use display_interface::DisplayError;
use embedded_graphics::primitives::PrimitiveStyleBuilder;
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
use usb_device::prelude::UsbDeviceState;

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
    display: D,
    timer: &'a Timer,
    last_active: u64,
    rng: SmallRng,
    leds: keyboard::keycode::Leds,
    usb_state: UsbDeviceState,
    layer: usize,
    screen_saver_stars: [Point; 32],
}

impl<'a, D> OledDisplay<'a, D>
where
    D: DrawTarget<Color = BinaryColor, Error = DisplayError> + FlushableDisplay,
{
    pub fn new(display: D, timer: &'a Timer) -> OledDisplay<'a, D> {
        let now = timer.get_counter();

        let mut rng = SmallRng::seed_from_u64(now);

        let mut points = [Point::default(); 32];
        for p in points.iter_mut() {
            p.x = (rng.next_u32() as i32) % 320;
            p.y = (rng.next_u32() as i32) % 1280;
        }

        OledDisplay {
            display,
            timer,
            last_active: now,
            rng,
            layer: 0,
            usb_state: UsbDeviceState::Default,
            screen_saver_stars: points,
            leds: keyboard::keycode::Leds::empty(),
        }
    }

    pub fn draw_image(&mut self, data: &[u8], width: u32) -> Result<(), DisplayError> {
        self.display.clear(BinaryColor::Off)?;

        let img: ImageRawLE<BinaryColor> = ImageRawLE::new(data, width);
        Image::new(&img, Point::new(32, 0)).draw(&mut self.display)?;

        self.display.flush()
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
        modifier: bool,
        keycodes: bool,
    ) -> Result<(), DisplayError> {
        let thin_stroke = PrimitiveStyle::with_stroke(BinaryColor::On, 1);
        let bounding_box = Rectangle::new(top_left, Size::new(32, 16));
        let fill = PrimitiveStyle::with_fill(BinaryColor::On);

        if modifier {
            RoundedRectangle::with_equal_corners(
                Rectangle::with_center(bounding_box.center(), Size::new(16, 16)),
                Size::new(3, 3),
            )
            .into_styled(thin_stroke)
            .draw(display)?;
        }

        if keycodes {
            RoundedRectangle::with_equal_corners(
                Rectangle::with_center(bounding_box.center(), Size::new(12, 12)),
                Size::new(2, 2),
            )
            .into_styled(fill)
            .draw(display)?;
        }

        Ok(())
    }

    fn draw_usb_indicator(
        display: &mut D,
        top_left: Point,
        usb_state: UsbDeviceState,
    ) -> Result<(), DisplayError> {
        let bounding_box = Rectangle::new(top_left, Size::new(32, 4));
        let shape = RoundedRectangle::with_equal_corners(
            Rectangle::with_center(bounding_box.center(), Size::new(32, 4)),
            Size::new(2, 2),
        );

        match usb_state {
            UsbDeviceState::Default => {}
            UsbDeviceState::Addressed => {
                let inner = PrimitiveStyleBuilder::new()
                    .fill_color(BinaryColor::On)
                    .stroke_color(BinaryColor::Off)
                    .stroke_width(1)
                    .build();

                shape.into_styled(inner).draw(display)?;
            }
            UsbDeviceState::Configured => {
                let fill = PrimitiveStyle::with_fill(BinaryColor::On);

                shape.into_styled(fill).draw(display)?;
            }
            UsbDeviceState::Suspend => {
                let hollow = PrimitiveStyle::with_stroke(BinaryColor::On, 1);

                shape.into_styled(hollow).draw(display)?;
            }
        }
        Ok(())
    }

    pub fn draw_left_display(
        &mut self,
        leds: keyboard::keycode::Leds,
        keycodes: &[KeyCode],
        layer: usize,
        usb_state: UsbDeviceState,
    ) -> Result<(), DisplayError> {
        let now = self.timer.get_counter();

        let (modifier_active, key_active) = keycodes.iter().fold((false, false), |(am, ak), &k| {
            (
                am || k.is_modifier(),
                ak || !k.is_modifier() && k >= KeyCode::A,
            )
        });

        //Keyboard is active if any key is pressed, layer change, led changes or usb status changes
        if modifier_active
            || key_active
            || leds != self.leds
            || usb_state != self.usb_state
            || layer != self.layer
        {
            self.last_active = now;
            self.leds = leds;
            self.usb_state = usb_state;
            self.layer = layer;
        }

        if now - self.last_active > 60_000_000 {
            self.draw_screen_saver()
        } else {
            self.display.clear(BinaryColor::Off)?;

            //Led indicators
            if leds.contains(keyboard::keycode::Leds::CAP_LOCK) {
                Self::draw_led_indicator(&mut self.display, Point::new(0, 0), "C")?;
            }

            if leds.contains(keyboard::keycode::Leds::NUM_LOCK) {
                Self::draw_led_indicator(&mut self.display, Point::new(12, 0), "1")?;
            }

            if leds.contains(keyboard::keycode::Leds::SCROLL_LOCK) {
                Self::draw_led_indicator(&mut self.display, Point::new(24, 0), "S")?;
            }

            //Layer
            Self::draw_layer_indicator(&mut self.display, Point::new(0, 17), layer)?;

            //Keycode indicator
            Self::draw_keycode_indicator(
                &mut self.display,
                Point::new(0, 100),
                modifier_active,
                key_active,
            )?;

            //Usb status
            Self::draw_usb_indicator(&mut self.display, Point::new(0, 124), usb_state)?;

            self.display.flush()
        }
    }

    pub fn draw_right_display(&mut self, pressed_keys: &[usize]) -> Result<(), DisplayError> {
        let now = self.timer.get_counter();

        if !pressed_keys.is_empty() {
            self.last_active = now;
        }

        if now - self.last_active > 60_000_000 {
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
        let pixels = self
            .screen_saver_stars
            .iter()
            .map(|&p| Pixel(p / 10, BinaryColor::On));

        self.display.clear(BinaryColor::Off)?;
        self.display.draw_iter(pixels)?;
        self.display.flush()?;

        for p in self.screen_saver_stars.iter_mut() {
            let tmp = *p;

            let velocity = (tmp - Point::new(160, 640)) / 20;

            *p = tmp
                + if velocity.x == 0 && velocity.y == 0 {
                    Point::new(1, 1)
                } else {
                    velocity
                };

            if !Rectangle::new(Point::zero(), Size::new(320, 1280)).contains(*p) {
                *p = Point::new(
                    self.rng.next_u32() as i32 % 320,
                    self.rng.next_u32() as i32 % 1280,
                );
            }
        }

        Ok(())
    }

    pub fn draw_text_screen(&mut self, text: &str) -> Result<(), DisplayError> {
        self.display.clear(BinaryColor::Off)?;
        let character_style = MonoTextStyle::new(&FONT_4X6, BinaryColor::On);
        let textbox_style = TextBoxStyleBuilder::new()
            .height_mode(HeightMode::FitToText)
            .alignment(HorizontalAlignment::Left)
            .build();
        let bounds = Rectangle::new(Point::zero(), Size::new(32, 0));
        let text_box = TextBox::with_textbox_style(text, bounds, character_style, textbox_style);

        text_box.draw(&mut self.display)?;
        self.display.flush()?;

        Ok(())
    }
}
