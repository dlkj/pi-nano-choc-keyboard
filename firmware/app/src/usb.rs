use usb_device::class_prelude::*;
use usb_device::prelude::*;
use usbd_hid_devices::hid::UsbHidClass;
use usbd_hid_devices::keyboard::*;

pub struct UsbManager<'a, B>
where
    B: usb_device::bus::UsbBus,
{
    usb_device: UsbDevice<'a, B>,
    keyboard: UsbHidClass<'a, B, HidBootKeyboard>,
    keyboard_led: u8,
}

impl<'a, B> UsbManager<'a, B>
where
    B: usb_device::bus::UsbBus,
{
    pub fn new(usb_bus: &'a UsbBusAllocator<B>, pid: u16) -> UsbManager<'a, B> {
        let keyboard = usbd_hid_devices::hid::UsbHidClass::new(
            usb_bus,
            usbd_hid_devices::keyboard::HidBootKeyboard::default(),
        );

        // Create a USB device with https://pid.code VID and a test PID
        let usb_device = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x1209, pid))
            .manufacturer("DLKJ")
            .product("pi-nano-choc")
            .serial_number("TEST")
            .device_class(0x03) //HID - from: https://www.usb.org/defined-class-codes
            // .composite_with_iads()
            // .supports_remote_wakeup(true)
            .build();

        UsbManager {
            keyboard,
            usb_device,
            keyboard_led: 0,
        }
    }
    pub fn keyboard_borrow_mut(&mut self) -> &mut UsbHidClass<'a, B, HidBootKeyboard> {
        &mut self.keyboard
    }

    pub fn keyboard_led(&self) -> u8 {
        self.keyboard_led
    }

    pub fn service_irq(&mut self) {
        if self.usb_device.poll(&mut [&mut self.keyboard]) {
            if let Ok(leds) = self.keyboard.read_leds() {
                self.keyboard_led = leds;
            }
        }
    }
}
