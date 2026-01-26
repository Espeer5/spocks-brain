//! UART over USB logging to the dev PC device that the Pico is connected to.
//! 
//! Provides a logging facility visible to the developer which writes data over
//! UART to a host machine connected to the pico dev port's USB UART port.
//! Activated by the compile feature "usb-log".
//! 
//! When the usb-log feature is not active, all logger functions are nops.

#[allow(dead_code)]

#[cfg(feature = "usb-log")]
mod imp {

    use rp_pico::hal::{
        clocks::UsbClock,
        pac,
        usb::UsbBus
    };
    use usb_device::{
        class_prelude::*,
        descriptor::lang_id::LangID,
        device::{StringDescriptors, UsbDeviceBuilder, UsbVidPid},
    };
    use usbd_serial::SerialPort;

    /// USB logging handle
    /// 
    /// Contains both the handle for the usb device and serial port
    pub struct UsbLogger {
        usb_dev: usb_device::device::UsbDevice<'static, UsbBus>,
        serial:  SerialPort<'static, UsbBus>,
    }

    /// Static allocator for the USB bus
    static mut USB_BUS: Option<UsbBusAllocator<UsbBus>> = None;

    impl UsbLogger {
        /// Initialize the USB CDC device and logging
        /// 
        /// NOTE: Relies on the HAL while most of spock's brain firware is
        /// written below the HAL.
        pub fn new(
            usbctrl_regs:  pac::USBCTRL_REGS,
            usbctrl_dpram: pac::USBCTRL_DPRAM,
            resets:   &mut pac::RESETS,
            usb_clock:     UsbClock,
        ) -> Self {
            let bus = UsbBusAllocator::new(UsbBus::new(
                usbctrl_regs,
                usbctrl_dpram,
                usb_clock,
                true,
                resets,
            ));

            unsafe {
                USB_BUS = Some(bus);
            }

            // Initialize serial output
            // bus_ref used only once during startup, so is safe.
            #[allow(static_mut_refs)]
            let bus_ref = unsafe { USB_BUS.as_ref().unwrap() };
            let mut serial = SerialPort::new(bus_ref);
            let strings = StringDescriptors::new(LangID::EN_US)
                .manufacturer("TOS Tricorder")
                .product("USB Log")
                .serial_number("0001");

            // Initialize USB device
            let usb_dev = UsbDeviceBuilder::new(bus_ref,
                UsbVidPid(0x2E8A, 0x000A))
                    .strings(&[strings])
                    .unwrap()
                    .device_class(usbd_serial::USB_CLASS_CDC)
                    .build();

            let _ = serial.write(b"\r\n[usb_log] init\r\n");

            Self { usb_dev, serial }
        }

        /// Polling of the logging USB device
        /// 
        /// Must be called in main loop of running firwmware when logs enabled
        pub fn poll(&mut self) {
            let mut buf = [0u8; 64];
            if self.usb_dev.poll(&mut [&mut self.serial]) {
                let _ = self.serial.read(&mut buf);
            }
        }

        /// Write log to serial device to the best of our abilities. Do not 
        /// block
        pub fn write(&mut self, bytes: &[u8]) {
            // Attempt poll first
            self.poll();

            let mut off = 0usize;
            while off < bytes.len() {
                match self.serial.write(&bytes[off..]) {
                    Ok(n) if n > 0 => off += n,
                    _ => break, // Error or WOULDBLOCK
                }
            }
        }

        pub fn dtr(&mut self) -> bool {
            self.poll();
            self.serial.dtr()
        }
    }

    /// Helper function for writing log strings
    pub fn write_str(log: &mut UsbLogger, s: &str) {
        log.write(s.as_bytes());
    }
}

#[cfg(feature = "usb-log")]
pub use imp::*;
