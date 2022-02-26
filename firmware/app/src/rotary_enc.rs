use embedded_hal::digital::v2::InputPin;

pub struct RotaryEncoder<'a, E> {
    pin_a: &'a dyn InputPin<Error = E>,
    pin_b: &'a dyn InputPin<Error = E>,
    state: u8,
    quarter_idx: i8,
    value: i32,
    last_rel_ref: i32,
}

impl<'a, E> RotaryEncoder<'a, E>
where
    E: core::fmt::Debug,
{
    pub fn new(pin_a: &'a dyn InputPin<Error = E>, pin_b: &'a dyn InputPin<Error = E>) -> Self {
        RotaryEncoder {
            pin_a,
            pin_b,
            state: 3,
            quarter_idx: 0,
            value: 0,
            last_rel_ref: 0,
        }
    }

    pub fn update(&mut self) {
        const ENCODER_STATES: [i8; 16] = [0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0];

        let new_state = self.pin_a.is_high().expect("Unable to read pin_a") as u8
            | (self.pin_b.is_high().expect("unable to read pin_b") as u8 * 2);

        let transition = ENCODER_STATES[((new_state << 2) | self.state) as usize];
        self.state = new_state;
        self.quarter_idx += transition;

        if self.quarter_idx > 3 {
            self.value -= 1;
            self.quarter_idx -= 4;
        } else if self.quarter_idx < -3 {
            self.value += 1;
            self.quarter_idx += 4;
        }
    }

    pub fn abs_value(&self) -> i32 {
        self.value
    }
    pub fn rel_value(&mut self) -> i32 {
        let rel = self.value - self.last_rel_ref;
        self.last_rel_ref = self.value;
        rel
    }
}
