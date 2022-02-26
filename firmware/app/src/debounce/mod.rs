use core::cell::Cell;

pub struct DebouncedPin<P> {
    pin: P,
    last: Cell<bool>,
    history: Cell<u8>,
}

impl<P, E> DebouncedPin<P>
where
    P: embedded_hal::digital::v2::InputPin<Error = E>,
{
    pub fn new(pin: P, default_state: bool) -> DebouncedPin<P> {
        DebouncedPin {
            pin,
            last: Cell::new(default_state),
            history: Cell::new(if default_state { u8::MAX } else { 0 }),
        }
    }

    pub fn update(&self) -> Result<(), E> {
        const MASK: u8 = 0b11100000; //look for 5 stable values

        let history = (self.history.get() << 1) | if self.pin.is_high()? { 1 } else { 0 } | MASK;
        self.history.set(history);

        match history {
            u8::MAX => self.last.set(true),
            MASK => self.last.set(false),
            _ => (),
        };

        Ok(())
    }
}

impl<P> core::borrow::Borrow<P> for DebouncedPin<P> {
    fn borrow(&self) -> &P {
        &self.pin
    }
}

impl<P> embedded_hal::digital::v2::InputPin for DebouncedPin<P>
where
    P: embedded_hal::digital::v2::InputPin,
{
    type Error = P::Error;

    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(self.last.get())
    }
    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(!self.last.get())
    }
}

#[cfg(test)]
mod tests;
