use embedded_hal::digital::v2::OutputPin;
use crate::keyboard::keycode::KeyCode;
use crate::keyboard::keycode::Modifiers;
use arrayvec::ArrayVec;
use crate::debounce::DebouncedPin;
use embedded_hal::digital::v2::InputPin;

pub mod keycode;

pub enum KeyAction {
    Key { code: KeyCode },
}

#[derive(Default, Copy, Clone)]
pub struct KeyState {
    pub pressed: bool,
}

pub trait KeyboardMatrix<const KEY_COUNT: usize> {
    type Error;
    fn update(&mut self) -> Result<(), Self::Error>;
    fn keys(&mut self) -> Result<[KeyState; KEY_COUNT], Self::Error>;
}

pub struct DirectPinMatrix<P, const N: usize> {
    pins: [DebouncedPin<P>; N],
}

impl<P, const N: usize> DirectPinMatrix<P, N> {
    pub fn new(pins: [P; N]) -> DirectPinMatrix<P, N>
    where
        P: InputPin,
    {
        DirectPinMatrix {
            pins: pins.map(|p| DebouncedPin::new(p, false)),
        }
    }
}

impl<P, const N: usize> KeyboardMatrix<N> for DirectPinMatrix<P, N>
where
    P: InputPin,
{
    type Error = P::Error;

    fn keys(&mut self) -> Result<[KeyState; N], Self::Error> {
        let mut keystates = [KeyState::default(); N];

        for (i, p) in self.pins.iter().enumerate() {
            keystates[i].pressed = p.is_high()?;
        }
        Ok(keystates)
    }

    fn update(&mut self) -> Result<(), Self::Error> {
        for p in &mut self.pins {
            p.update()?;
        }
        Ok(())
    }
}

pub struct DiodePinMatrix<PO, P> {
    rows: [PO; 6],
    cols: [P; 6],
}

impl<PO, P> DiodePinMatrix<PO, P> {
    pub fn new(mut rows: [PO; 6], cols: [P;6]) -> DiodePinMatrix<PO, P>
    where
        P: InputPin,
        PO: OutputPin,
    {
        for r in &mut rows {
            r.set_low().ok();
        }

        DiodePinMatrix {
            rows,
            cols
        }
    }
}

impl<PO, P> KeyboardMatrix<36> for DiodePinMatrix<PO, P>
where
    P: InputPin,
    PO: OutputPin,
    P::Error: core::fmt::Debug,
    PO::Error: core::fmt::Debug,
{
    type Error = P::Error;

    fn keys(&mut self) -> Result<[KeyState; 36], Self::Error> {
        let mut keystates = [KeyState::default(); 36];

        let mut i = 0;

        for r in &mut self.rows {

            r.set_high().unwrap();

            //allow time for output to stabilise beore scanning columns
            for _ in 0..10 {
                cortex_m::asm::nop();
            }

            for c in &self.cols {
                keystates[i].pressed = c.is_high().unwrap();
                i+=1;
            }
            r.set_low().unwrap();
        }
        Ok(keystates)
    }

    fn update(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

pub struct KeyboardLayoutState<const KEY_COUNT: usize> {
    pub modifiers: Modifiers,
    pub keycodes: ArrayVec<KeyCode, KEY_COUNT>,
}

pub trait KeyboardLayout<const N: usize> {
    fn state(&self, keys: &[KeyState; N]) -> KeyboardLayoutState<N>;
}

pub struct BasicKeyboardLayout<const N: usize> {
    keymap: [KeyAction; N],
}

impl<const N: usize> BasicKeyboardLayout<N> {
    pub fn new(keymap: [KeyAction; N]) -> BasicKeyboardLayout<N> {
        BasicKeyboardLayout { keymap }
    }
}

impl<const N: usize> KeyboardLayout<N> for BasicKeyboardLayout<N> {
    fn state(&self, keys: &[KeyState; N]) -> KeyboardLayoutState<N> {
        let mut modifiers = Modifiers::empty();
        let mut keycodes = arrayvec::ArrayVec::new();

        for (i, _) in keys.iter().enumerate().filter(|(_, k)| k.pressed) {
            match self.keymap[i] {
                KeyAction::Key { code } => {
                    if code.is_modifier() {
                        modifiers |= Modifiers::from(code);
                    } else {
                        keycodes.push(code);
                    }
                }
            }
        }

        KeyboardLayoutState {
            modifiers,
            keycodes,
        }
    }
}

pub struct KeyboardState<const KEY_COUNT: usize> {
    pub modifiers: Modifiers,
    pub keycodes: ArrayVec<KeyCode, KEY_COUNT>,
    pub keys: [KeyState; KEY_COUNT],
}

pub struct Keyboard<KM, KL, const KEY_COUNT: usize> {
    matrix: KM,
    layout: KL,
}

impl<KM, KL, const KEY_COUNT: usize> Keyboard<KM, KL, KEY_COUNT>
where
    KM: KeyboardMatrix<KEY_COUNT>,
    KL: KeyboardLayout<KEY_COUNT>,
{
    pub fn new(matrix: KM, layout: KL) -> Keyboard<KM, KL, KEY_COUNT> {
        Keyboard { matrix, layout }
    }
    pub fn update(&mut self) -> Result<(), KM::Error> {
        self.matrix.update()
    }
    pub fn state(&mut self) -> Result<KeyboardState<KEY_COUNT>, KM::Error> {
        let keys = self.matrix.keys()?;
        let layout_state = self.layout.state(&keys);

        Ok(KeyboardState {
            modifiers: layout_state.modifiers,
            keycodes: layout_state.keycodes,
            keys,
        })
    }
}
