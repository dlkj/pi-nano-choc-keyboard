use arrayvec::ArrayVec;
use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;
use usbd_human_interface_device::page::{Consumer, Keyboard as KeyCode};

use crate::debounce::DebouncedPin;

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum KeyAction {
    Key { code: KeyCode },
    None,
    FallThrough,
    Function { function: KeyFunction },
    Consumer(Consumer),
    MouseWheel(i8),
    Layer { n: usize },
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum KeyFunction {
    Hyper,
    Meh,
    //Todo modifer + keycode
}

#[derive(Debug, Default, Copy, Clone)]
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
    pub fn new(mut rows: [PO; 6], cols: [P; 6]) -> DiodePinMatrix<PO, P>
    where
        P: InputPin,
        PO: OutputPin,
    {
        for r in &mut rows {
            r.set_low().ok();
        }

        DiodePinMatrix { rows, cols }
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

            //allow time for output to stabilize before scanning columns
            for _ in 0..10 {
                cortex_m::asm::nop();
            }

            for c in &self.cols {
                keystates[i].pressed = c.is_high().unwrap();
                i += 1;
            }
            r.set_low().unwrap();
        }
        Ok(keystates)
    }

    fn update(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

pub struct SplitMatrix<M1, M2> {
    pub matrix1: M1,
    pub matrix2: M2,
}

impl<M1, M2, E1, E2> KeyboardMatrix<75> for SplitMatrix<M1, M2>
where
    M1: KeyboardMatrix<36, Error = E1>,
    M2: KeyboardMatrix<39, Error = E2>,
    E1: core::fmt::Debug,
    E2: core::fmt::Debug,
{
    type Error = core::convert::Infallible;

    fn update(&mut self) -> Result<(), Self::Error> {
        self.matrix1.update().unwrap();
        self.matrix2.update().unwrap();
        Ok(())
    }

    fn keys(&mut self) -> Result<[KeyState; 75], Self::Error> {
        let k1 = self.matrix1.keys().unwrap();
        let k2 = self.matrix2.keys().unwrap();

        let mut keys = [KeyState::default(); 75];

        keys[..36].clone_from_slice(&k1);
        keys[36..75].clone_from_slice(&k2);

        Ok(keys)
    }
}

pub struct UartMatrix<U> {
    uart: U,
    sync: bool,
    current: arrayvec::ArrayVec<u8, 128>,
    next: arrayvec::ArrayVec<u8, 128>,
}

impl<U> UartMatrix<U> {
    pub fn new(uart: U) -> Self {
        Self {
            uart,
            sync: false,
            current: arrayvec::ArrayVec::new(),
            next: arrayvec::ArrayVec::new(),
        }
    }
}

impl<U> KeyboardMatrix<39> for UartMatrix<U>
where
    U: embedded_hal::serial::Read<u8>,
{
    type Error = core::convert::Infallible;

    fn update(&mut self) -> Result<(), Self::Error> {
        while let Ok(x) = self.uart.read() {
            if !self.sync {
                if x == 0xFF {
                    self.sync = true;
                } else {
                    continue;
                }
            } else if x == 0xFF {
                self.current = self.next.take();
            } else if x < 39 {
                self.next.push(x);
            }
        }

        Ok(())
    }

    fn keys(&mut self) -> Result<[KeyState; 39], Self::Error> {
        let mut keys = [KeyState::default(); 39];

        for &k in &self.current {
            keys[k as usize].pressed = true;
        }

        Ok(keys)
    }
}

#[derive(Debug)]
pub struct KeyboardLayoutState<const KEY_COUNT: usize> {
    pub keycodes: ArrayVec<KeyCode, KEY_COUNT>,
    pub consumer: ArrayVec<Consumer, KEY_COUNT>,
    pub mouse_wheel: i8,
    pub layer: usize,
}

pub trait KeyboardLayout<const N: usize> {
    fn state(&self, keys: &[KeyState; N]) -> KeyboardLayoutState<N>;
}

pub struct LayerdKeyboardLayout<const N: usize, const L: usize> {
    keymaps: [[KeyAction; N]; L],
    layer_keys: arrayvec::ArrayVec<usize, 6>,
}

impl<const N: usize, const L: usize> LayerdKeyboardLayout<N, L> {
    pub fn new(keymaps: [[KeyAction; N]; L]) -> LayerdKeyboardLayout<N, L> {
        //todo - assert no layer keys on non base layer

        let mut layer_keys = arrayvec::ArrayVec::new();
        for (i, k) in keymaps[0].iter().enumerate() {
            if let KeyAction::Layer { .. } = k {
                layer_keys.push(i);
            }
        }

        LayerdKeyboardLayout {
            keymaps,
            layer_keys,
        }
    }
}

impl<const N: usize, const L: usize> KeyboardLayout<N> for LayerdKeyboardLayout<N, L> {
    fn state(&self, keys: &[KeyState; N]) -> KeyboardLayoutState<N> {
        let mut active_layers = [false; L];
        let mut max_layer = 0;
        active_layers[0] = true; //base layer is always active

        //find all the pressed layer keys
        for &l in &self.layer_keys {
            if keys[l].pressed {
                if let KeyAction::Layer { n } = self.keymaps[0][l] {
                    active_layers[n] = true;
                    max_layer = max_layer.max(n)
                }
            }
        }

        let mut keycodes = arrayvec::ArrayVec::new();
        let mut consumer = arrayvec::ArrayVec::new();
        let mut mouse_wheel = 0;

        for (i, _) in keys.iter().enumerate().filter(|(_, &k)| k.pressed) {
            let mut key = KeyAction::FallThrough;

            //cascade through the keymaps
            for l in (0..L).rev() {
                if key != KeyAction::FallThrough {
                    break;
                }

                if active_layers[l] {
                    key = self.keymaps[l][i];
                }
            }

            match key {
                KeyAction::Key { code } => {
                    keycodes.push(code);
                }
                KeyAction::None => {
                    //do nothing
                }
                KeyAction::Function { function: f } => match f {
                    KeyFunction::Hyper => {
                        keycodes.extend([
                            KeyCode::LeftShift,
                            KeyCode::LeftControl,
                            KeyCode::LeftAlt,
                            KeyCode::LeftGUI,
                        ]);
                    }
                    KeyFunction::Meh => {
                        keycodes.extend([
                            KeyCode::LeftShift,
                            KeyCode::LeftControl,
                            KeyCode::LeftAlt,
                        ]);
                    }
                },
                KeyAction::Layer { .. } => {
                    //should already be applied before mapping keys to keycodes
                }
                KeyAction::FallThrough => {
                    //can't fall any lower, same as None
                }
                KeyAction::Consumer(c) => {
                    consumer.push(c);
                }
                KeyAction::MouseWheel(i) => {
                    mouse_wheel += i;
                }
            }
        }

        KeyboardLayoutState {
            keycodes,
            consumer,
            mouse_wheel,
            layer: max_layer,
        }
    }
}

#[derive(Debug)]
pub struct KeyboardState<const KEY_COUNT: usize> {
    pub keycodes: ArrayVec<KeyCode, KEY_COUNT>,
    pub consumer: ArrayVec<Consumer, KEY_COUNT>,
    pub mouse_wheel: i8,
    pub keys: [KeyState; KEY_COUNT],
    pub layer: usize,
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
            keycodes: layout_state.keycodes,
            keys,
            layer: layout_state.layer,
            consumer: layout_state.consumer,
            mouse_wheel: layout_state.mouse_wheel,
        })
    }
}
