use usbd_hid_devices::page::Keyboard as KeyCode;

use crate::keyboard::KeyAction;
use crate::keyboard::KeyFunction;

const BASE_MAP: [KeyAction; 72] = [
    //row 0
    KeyAction::Key {
        code: KeyCode::Escape,
    },
    KeyAction::Key {
        code: KeyCode::Keyboard1,
    },
    KeyAction::Key {
        code: KeyCode::Keyboard2,
    },
    KeyAction::Key {
        code: KeyCode::Keyboard3,
    },
    KeyAction::Key {
        code: KeyCode::Keyboard4,
    },
    KeyAction::Key {
        code: KeyCode::Keyboard5,
    },
    //row 1
    KeyAction::Key { code: KeyCode::Tab },
    KeyAction::Key { code: KeyCode::Q },
    KeyAction::Key { code: KeyCode::W },
    KeyAction::Key { code: KeyCode::E },
    KeyAction::Key { code: KeyCode::R },
    KeyAction::Key { code: KeyCode::T },
    //row 2
    KeyAction::Key {
        code: KeyCode::NonUSBackslash,
    },
    KeyAction::Key { code: KeyCode::A },
    KeyAction::Key { code: KeyCode::S },
    KeyAction::Key { code: KeyCode::D },
    KeyAction::Key { code: KeyCode::F },
    KeyAction::Key { code: KeyCode::G },
    //row 3
    KeyAction::Key {
        code: KeyCode::LeftShift,
    },
    KeyAction::Key { code: KeyCode::Z },
    KeyAction::Key { code: KeyCode::X },
    KeyAction::Key { code: KeyCode::C },
    KeyAction::Key { code: KeyCode::V },
    KeyAction::Key { code: KeyCode::B },
    //row 4
    KeyAction::Key {
        code: KeyCode::LeftControl,
    },
    KeyAction::Key {
        code: KeyCode::LeftGUI,
    },
    KeyAction::Key {
        code: KeyCode::Grave,
    },
    KeyAction::None,
    KeyAction::None,
    KeyAction::Key {
        code: KeyCode::LeftBrace,
    },
    //row 5
    KeyAction::Key {
        code: KeyCode::NoEventIndicated,
    },
    KeyAction::Key {
        code: KeyCode::LeftAlt,
    },
    KeyAction::Function {
        function: KeyFunction::Hyper,
    },
    KeyAction::Key {
        code: KeyCode::Space,
    },
    KeyAction::Key {
        code: KeyCode::Space,
    },
    KeyAction::Layer { n: 1 },
    //right hand

    //row 0
    KeyAction::Key {
        code: KeyCode::Keyboard6,
    },
    KeyAction::Key {
        code: KeyCode::Keyboard7,
    },
    KeyAction::Key {
        code: KeyCode::Keyboard8,
    },
    KeyAction::Key {
        code: KeyCode::Keyboard9,
    },
    KeyAction::Key {
        code: KeyCode::Keyboard0,
    },
    KeyAction::Key {
        code: KeyCode::Minus,
    },
    //row 1
    KeyAction::Key { code: KeyCode::Y },
    KeyAction::Key { code: KeyCode::U },
    KeyAction::Key { code: KeyCode::I },
    KeyAction::Key { code: KeyCode::O },
    KeyAction::Key { code: KeyCode::P },
    KeyAction::Key {
        code: KeyCode::Equal,
    },
    //row 2
    KeyAction::Key { code: KeyCode::H },
    KeyAction::Key { code: KeyCode::J },
    KeyAction::Key { code: KeyCode::K },
    KeyAction::Key { code: KeyCode::L },
    KeyAction::Key {
        code: KeyCode::Semicolon,
    },
    KeyAction::Key {
        code: KeyCode::Apostrophe,
    },
    //row 3
    KeyAction::Key { code: KeyCode::N },
    KeyAction::Key { code: KeyCode::M },
    KeyAction::Key {
        code: KeyCode::Comma,
    },
    KeyAction::Key { code: KeyCode::Dot },
    KeyAction::Key {
        code: KeyCode::ForwardSlash,
    },
    KeyAction::Key {
        code: KeyCode::RightShift,
    },
    //row 4
    KeyAction::Key {
        code: KeyCode::RightBrace,
    },
    KeyAction::None,
    KeyAction::Key {
        code: KeyCode::DeleteForward,
    },
    KeyAction::Key {
        code: KeyCode::NonUSHash,
    },
    KeyAction::Key {
        code: KeyCode::Application,
    },
    KeyAction::Key {
        code: KeyCode::RightControl,
    },
    //row 5
    KeyAction::Layer { n: 1 },
    KeyAction::Key {
        code: KeyCode::ReturnEnter,
    },
    KeyAction::Key {
        code: KeyCode::DeleteBackspace,
    },
    KeyAction::Function {
        function: KeyFunction::Meh,
    },
    KeyAction::Key {
        code: KeyCode::RightAlt,
    },
    KeyAction::Key {
        code: KeyCode::NoEventIndicated,
    },
];
const UPPER_MAP: [KeyAction; 72] = [
    //row 0
    KeyAction::FallThrough,
    KeyAction::None,
    KeyAction::None,
    KeyAction::None,
    KeyAction::None,
    KeyAction::None,
    //row 1
    KeyAction::FallThrough,
    KeyAction::None,
    KeyAction::None,
    KeyAction::None,
    KeyAction::None,
    KeyAction::None,
    //row 2
    KeyAction::FallThrough,
    KeyAction::None,
    KeyAction::None,
    KeyAction::None,
    KeyAction::None,
    KeyAction::None,
    //row 3
    KeyAction::FallThrough,
    KeyAction::None,
    KeyAction::None,
    KeyAction::None,
    KeyAction::None,
    KeyAction::None,
    //row 4
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    //row 5
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    //right hand
    //row 0
    KeyAction::None,
    KeyAction::Key {
        code: KeyCode::LockingNumLock,
    },
    KeyAction::Key {
        code: KeyCode::KeypadDivide,
    },
    KeyAction::Key {
        code: KeyCode::KeypadMultiply,
    },
    KeyAction::Key {
        code: KeyCode::KeypadSubtract,
    },
    KeyAction::Key {
        code: KeyCode::NoEventIndicated,
    },
    //row 1
    KeyAction::None,
    KeyAction::Key {
        code: KeyCode::Keypad7,
    },
    KeyAction::Key {
        code: KeyCode::Keypad8,
    },
    KeyAction::Key {
        code: KeyCode::Keypad9,
    },
    KeyAction::Key {
        code: KeyCode::KeypadAdd,
    },
    KeyAction::Key {
        code: KeyCode::NoEventIndicated,
    },
    //row 2
    KeyAction::None,
    KeyAction::Key {
        code: KeyCode::Keypad4,
    },
    KeyAction::Key {
        code: KeyCode::Keypad5,
    },
    KeyAction::Key {
        code: KeyCode::Keypad6,
    },
    KeyAction::Key {
        code: KeyCode::KeypadEnter,
    },
    KeyAction::Key {
        code: KeyCode::NoEventIndicated,
    },
    //row 3
    KeyAction::None,
    KeyAction::Key {
        code: KeyCode::Keypad1,
    },
    KeyAction::Key {
        code: KeyCode::Keypad2,
    },
    KeyAction::Key {
        code: KeyCode::Keypad3,
    },
    KeyAction::None,
    KeyAction::FallThrough,
    //row 4
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::Key {
        code: KeyCode::Keypad0,
    },
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    //row 5
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::Key { code: KeyCode::Dot },
    KeyAction::FallThrough,
];

const LOWER_MAP: [KeyAction; 72] = [
    //row 0
    KeyAction::FallThrough,
    KeyAction::Key { code: KeyCode::F1 },
    KeyAction::Key { code: KeyCode::F2 },
    KeyAction::Key { code: KeyCode::F3 },
    KeyAction::Key { code: KeyCode::F4 },
    KeyAction::Key { code: KeyCode::F5 },
    //row 1
    KeyAction::FallThrough,
    KeyAction::None,
    KeyAction::None,
    KeyAction::None,
    KeyAction::None,
    KeyAction::None,
    //row 2
    KeyAction::FallThrough,
    KeyAction::None,
    KeyAction::None,
    KeyAction::None,
    KeyAction::None,
    KeyAction::None,
    //row 3
    KeyAction::FallThrough,
    KeyAction::None,
    KeyAction::None,
    KeyAction::None,
    KeyAction::None,
    KeyAction::None,
    //row 4
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    //row 5
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    //right hand
    //row 0
    KeyAction::Key { code: KeyCode::F6 },
    KeyAction::Key { code: KeyCode::F7 },
    KeyAction::Key { code: KeyCode::F8 },
    KeyAction::Key { code: KeyCode::F9 },
    KeyAction::Key { code: KeyCode::F10 },
    KeyAction::Key { code: KeyCode::F11 },
    //row 1
    KeyAction::None,
    KeyAction::Key {
        code: KeyCode::PageUp,
    },
    KeyAction::Key {
        code: KeyCode::UpArrow,
    },
    KeyAction::Key {
        code: KeyCode::PageDown,
    },
    KeyAction::None,
    KeyAction::Key { code: KeyCode::F12 },
    //row 2
    KeyAction::None,
    KeyAction::Key {
        code: KeyCode::LeftArrow,
    },
    KeyAction::Key {
        code: KeyCode::DownArrow,
    },
    KeyAction::Key {
        code: KeyCode::RightArrow,
    },
    KeyAction::None,
    KeyAction::Key {
        code: KeyCode::Pause,
    },
    //row 3
    KeyAction::None,
    KeyAction::Key {
        code: KeyCode::Home,
    },
    KeyAction::None,
    KeyAction::Key { code: KeyCode::End },
    KeyAction::None,
    KeyAction::FallThrough,
    //row 4
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    //row 5
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::FallThrough,
    KeyAction::FallThrough,
];

pub const KEY_MAP: [[KeyAction; 72]; 3] = [BASE_MAP, LOWER_MAP, UPPER_MAP];
