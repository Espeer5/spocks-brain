//! Bounded event queue and NMEA line assembly (main thread only).

use core::cell::RefCell;
use core::mem;

use cortex_m::interrupt::{self, Mutex};
use heapless::{Deque, Vec};

use crate::platform::constants::{APP_EVENT_QUEUE_CAP, NMEA_LINE_CAP};
use crate::platform::uart;

#[cfg(feature = "usb-log")]
use crate::debug::usb_log::UsbLogger;

/// Application events (extend as new sensors are added).
#[derive(Debug)]
pub enum Event {
    /// A full NMEA line (text without trailing `\r\n`).
    #[allow(dead_code)]
    GnssNmeaLineReady(Vec<u8, NMEA_LINE_CAP>),
}

static EVENT_QUEUE: Mutex<RefCell<Deque<Event, APP_EVENT_QUEUE_CAP>>> =
    Mutex::new(RefCell::new(Deque::new()));

/// Lines dropped because the event queue was full.
static DROPPED_EVENTS: Mutex<RefCell<u32>> = Mutex::new(RefCell::new(0));

#[allow(dead_code)]
pub fn dropped_event_count() -> u32 {
    interrupt::free(|cs| *DROPPED_EVENTS.borrow(cs).borrow())
}

fn push_event(ev: Event) {
    interrupt::free(|cs| {
        let mut q = EVENT_QUEUE.borrow(cs).borrow_mut();
        if q.push_back(ev).is_err() {
            let mut n = DROPPED_EVENTS.borrow(cs).borrow_mut();
            *n = n.wrapping_add(1);
        }
    });
}

pub fn try_pop_event() -> Option<Event> {
    interrupt::free(|cs| EVENT_QUEUE.borrow(cs).borrow_mut().pop_front())
}

/// NMEA line state (main thread only).
pub struct NmeaLineAssembler {
    buf: Vec<u8, NMEA_LINE_CAP>,
}

impl NmeaLineAssembler {
    pub const fn new() -> Self {
        Self { buf: Vec::new() }
    }

    /// Feed UART bytes; completed lines are pushed to the event queue.
    pub fn feed_bytes(&mut self) {
        while let Some(b) = uart::try_pop_rx_byte() {
            if b == b'\n' {
                let mut line = mem::replace(&mut self.buf, Vec::new());
                while line.last() == Some(&b'\r') {
                    line.pop();
                }
                if !line.is_empty() {
                    push_event(Event::GnssNmeaLineReady(line));
                }
                continue;
            }
            if self.buf.is_full() {
                self.buf.clear();
                continue;
            }
            let _ = self.buf.push(b);
        }
    }
}

/// Handle one event (e.g. forward GNSS lines to USB when enabled).
#[cfg(feature = "usb-log")]
pub fn dispatch_event(ev: Event, usb: &mut UsbLogger) {
    match ev {
        Event::GnssNmeaLineReady(line) => {
            usb.write(&line);
            usb.write(b"\r\n");
        }
    }
}

#[cfg(not(feature = "usb-log"))]
pub fn dispatch_event(ev: Event) {
    match ev {
        Event::GnssNmeaLineReady(_) => {}
    }
}
