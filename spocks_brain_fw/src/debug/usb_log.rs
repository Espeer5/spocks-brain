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

    use rp_pico::hal::{clocks::UsbClock, pac, usb::UsbBus};
    use usb_device::{
        class_prelude::*,
        descriptor::lang_id::LangID,
        device::{StringDescriptors, UsbDeviceBuilder, UsbVidPid},
        UsbError,
    };
    use usbd_serial::SerialPort;

    /// USB logging handle
    ///
    /// Contains both the handle for the usb device and serial port
    pub struct UsbLogger {
        usb_dev: usb_device::device::UsbDevice<'static, UsbBus>,
        serial: SerialPort<'static, UsbBus>,
    }

    /// Static allocator for the USB bus
    static mut USB_BUS: Option<UsbBusAllocator<UsbBus>> = None;

    impl UsbLogger {
        /// Initialize the USB CDC device and logging
        ///
        /// NOTE: Relies on the HAL while most of spock's brain firware is
        /// written below the HAL.
        pub fn new(
            usbctrl_regs: pac::USBCTRL_REGS,
            usbctrl_dpram: pac::USBCTRL_DPRAM,
            resets: &mut pac::RESETS,
            usb_clock: UsbClock,
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
            let serial = SerialPort::new(bus_ref);
            let strings = StringDescriptors::new(LangID::EN_US)
                .manufacturer("TOS Tricorder")
                .product("USB Log")
                .serial_number("0001");

            // Initialize USB device
            // Default EP0 max packet size is 8; hosts can fail enumeration
            // during the first control transfer if it is too small
            // (see rp2040-hal `usb` docs).
            let usb_dev = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x2E8A, 0x000A))
                .strings(&[strings])
                .unwrap()
                .device_class(usbd_serial::USB_CLASS_CDC)
                .max_packet_size_0(64)
                .unwrap()
                .build();

            // Do not write here: the device is not configured yet and nothing
            // has called `usb_dev.poll()`. Bytes would sit in CDC buffers until
            // the host completes setup.

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
            if bytes.is_empty() {
                return;
            }

            let mut off = 0usize;
            let mut spins = 0usize;
            // CDC needs `usb_dev.poll()` between attempts; `WouldBlock` is normal until the host
            // picks up IN tokens. Abort after a bound so a disconnected cable cannot spin forever.
            const MAX_SPINS: usize = 500_000;

            while off < bytes.len() && spins < MAX_SPINS {
                self.poll();
                match self.serial.write(&bytes[off..]) {
                    Ok(n) => off += n,
                    Err(UsbError::WouldBlock) => {}
                    Err(_) => break,
                }
                spins += 1;
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

    /// Emit a compact text snapshot of the global GNSS state (from `nmea::Nmea`).
    pub fn write_gnss_state(log: &mut UsbLogger) {
        use core::fmt::Write;

        use heapless::String;

        crate::app::gnss_state::with_store(|nmea, stats| {
            let mut buf = String::<768>::new();
            let _ = writeln!(&mut buf, "--- GNSS ---");
            let _ = writeln!(
                &mut buf,
                "stats: applied:{} errors:{}",
                stats.sentences_applied, stats.parse_errors
            );
            let _ = writeln!(
                &mut buf,
                "time:{:?} date:{:?} fix:{:?}",
                nmea.fix_timestamp(),
                nmea.fix_date,
                nmea.fix_type()
            );
            let _ = writeln!(
                &mut buf,
                "lat:{:?} lon:{:?} alt_msl_m:{:?} geoid_sep_m:{:?}",
                nmea.latitude(),
                nmea.longitude(),
                nmea.altitude(),
                nmea.geoid_separation
            );
            let _ = writeln!(
                &mut buf,
                "sats_used:{:?} hdop:{:?} pdop:{:?} vdop:{:?}",
                nmea.fix_satellites(),
                nmea.hdop(),
                nmea.pdop,
                nmea.vdop
            );
            let _ = writeln!(
                &mut buf,
                "sog_kn:{:?} cog_true_deg:{:?}",
                nmea.speed_over_ground, nmea.true_course
            );
            let _ = writeln!(&mut buf, "satellites ({}):", nmea.satellites().len());
            for sat in nmea.satellites().iter().take(24) {
                let _ = writeln!(&mut buf, "  {}", sat);
            }
            log.write(buf.as_bytes());
            log.write(b"\r\n");
        });
    }
}

#[cfg(feature = "usb-log")]
pub use imp::*;
