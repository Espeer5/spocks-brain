//! Global GNSS navigation state from NMEA 0183, backed by the `nmea` crate.

use core::cell::RefCell;

use cortex_m::interrupt::{self, Mutex};
use nmea::Nmea;

/// Monotonic counters for integration visibility.
#[derive(Debug, Clone, Default)]
pub struct GnssParseStats {
    pub sentences_applied: u32,
    pub parse_errors: u32,
}

struct GnssStore {
    /// Lazily allocated: `Nmea::default()` is not `const`, so we init on first use.
    nmea: Option<Nmea>,
    stats: GnssParseStats,
}

static GNSS_STORE: Mutex<RefCell<GnssStore>> = Mutex::new(RefCell::new(GnssStore {
    nmea: None,
    stats: GnssParseStats {
        sentences_applied: 0,
        parse_errors: 0,
    },
}));

/// Snapshot type for readers: the `nmea` crate’s aggregated navigation state.
#[allow(dead_code)]
pub type GnssState = Nmea;

/// Feed one complete NMEA line (no `\r\n`). ASCII/UTF-8 only.
pub fn feed_line(raw: &[u8]) {
    if raw.is_empty() {
        return;
    }
    let text = match core::str::from_utf8(raw) {
        Ok(t) => t,
        Err(_) => {
            interrupt::free(|cs| {
                let mut g = GNSS_STORE.borrow(cs).borrow_mut();
                g.stats.parse_errors = g.stats.parse_errors.wrapping_add(1);
            });
            return;
        }
    };

    interrupt::free(|cs| {
        let mut g = GNSS_STORE.borrow(cs).borrow_mut();
        let nmea = g.nmea.get_or_insert_with(Nmea::default);
        match nmea.parse(text) {
            Ok(_) => {
                g.stats.sentences_applied = g.stats.sentences_applied.wrapping_add(1);
            }
            Err(_) => {
                g.stats.parse_errors = g.stats.parse_errors.wrapping_add(1);
            }
        }
    });
}

/// Read the live parser and stats under the global mutex.
#[allow(dead_code)]
pub fn with_store<R>(f: impl FnOnce(&Nmea, &GnssParseStats) -> R) -> R {
    interrupt::free(|cs| {
        let mut g = GNSS_STORE.borrow(cs).borrow_mut();
        let stats_snap = g.stats.clone();
        let nmea = g.nmea.get_or_insert_with(Nmea::default);
        f(nmea, &stats_snap)
    })
}
