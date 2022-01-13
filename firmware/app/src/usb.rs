use usb_device::class_prelude::*;
use usb_device::prelude::*;
use usbd_hid_devices::hid::UsbHidClass;
use usbd_hid_devices::keyboard::*;
use usbd_serial::SerialPort;

pub struct UsbManager<'a, B>
where
    B: usb_device::bus::UsbBus,
{
    usb_device: UsbDevice<'a, B>,
    serial_port: SerialPort<'a, B>,
    keyboard: UsbHidClass<'a, B, HidBootKeyboard>,
    keyboard_led: u8,
}

impl<'a, B> UsbManager<'a, B>
where
    B: usb_device::bus::UsbBus,
{
    pub fn new(usb_bus: &'a UsbBusAllocator<B>, pid: u16) -> UsbManager<'a, B> {
        let serial_port = SerialPort::new(usb_bus);
        let keyboard = usbd_hid_devices::hid::UsbHidClass::new(
            usb_bus,
            usbd_hid_devices::keyboard::HidBootKeyboard::default(),
        );

        // Create a USB device with https://pid.code VID and a test PID
        let usb_device = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x1209, pid))
            .manufacturer("DLKJ")
            .product("Pi-Nano-Choc")
            .serial_number("TEST")
            .device_class(0x00) // from: https://www.usb.org/defined-class-codes
            .composite_with_iads()
            .supports_remote_wakeup(true)
            .build();

        UsbManager {
            serial_port,
            keyboard,
            usb_device,
            keyboard_led: 0,
        }
    }
    pub fn keyboard_borrow_mut(&mut self) -> &mut UsbHidClass<'a, B, HidBootKeyboard> {
        &mut self.keyboard
    }

    pub fn serial_port_borrow_mut(&mut self) -> &mut SerialPort<'a, B> {
        &mut self.serial_port
    }

    pub fn keyboard_led(&self) -> u8 {
        self.keyboard_led
    }

    pub fn service_irq(&mut self) {
        // Poll the USB driver with all of our supported USB Classes
        if self
            .usb_device
            .poll(&mut [&mut self.serial_port, &mut self.keyboard])
        {
            let mut buf = [0u8; 64];
            self.serial_port.read(&mut buf).ok(); //discard incoming data

            if let Ok(leds) = self.keyboard.read_leds() {
                self.keyboard_led = leds;
            }
        }
    }
}
