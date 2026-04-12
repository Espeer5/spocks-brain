//! Global GNSS navigation state from NMEA 0183, backed by the `nmea` crate.
//!
//! **Threading:** this module is only used from the main loop. UART and timer IRQ
//! handlers must never call into it. We use [`MainThreadOnly`] ([`RefCell`]) so
//! NMEA parsing and reads do **not** run inside [`cortex_m::interrupt::free`].
//! Masking interrupts for tens of milliseconds was starving `TIMER_IRQ_0` (LED)
//! and delaying UART RX service, which showed up as slow blinking and lagging UTC.
//!
//! **UTC anchor:** we latch [`last_anchor_utc`] + [`utc_anchor_timelr`] only when
//! the merged parser time moves **strictly forward** in wall-clock order. GGA/RMC
//! and similar sentences can disagree for the same navigation epoch; re-anchoring on
//! every `H:M:S` change reset [`utc_anchor_timelr`] every line so elapsed time
//! never reached 1 s and the LCD second appeared frozen for many seconds.

use core::cell::RefCell;

use chrono::{NaiveDate, NaiveDateTime, NaiveTime, Timelike};
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
    /// Wall time (second resolution) last used to anchor the LCD tick.
    last_anchor_utc: Option<NaiveDateTime>,
    /// `timelr()` when [`last_anchor_utc`] last advanced — restarts the local tick.
    /// `0` means no anchor yet; LCD shows raw fix time without advancing seconds.
    utc_anchor_timelr: u32,
}

/// `RefCell` in a `static` for firmware that only ever mutates from the main thread.
///
/// # Safety
///
/// Must not be borrowed from any interrupt handler or second core.
struct MainThreadOnly<T>(RefCell<T>);
unsafe impl<T> Sync for MainThreadOnly<T> {}

static GNSS_STORE: MainThreadOnly<GnssStore> = MainThreadOnly(RefCell::new(GnssStore {
    nmea: None,
    stats: GnssParseStats {
        sentences_applied: 0,
        parse_errors: 0,
    },
    last_anchor_utc: None,
    utc_anchor_timelr: 0,
}));

/// Snapshot type for readers: the `nmea` crate’s aggregated navigation state.
#[allow(dead_code)]
pub type GnssState = Nmea;

fn synthetic_epoch_date() -> NaiveDate {
    NaiveDate::from_ymd_opt(2000, 1, 1).unwrap()
}

fn candidate_anchor_utc(nmea: &Nmea) -> Option<NaiveDateTime> {
    let t = nmea.fix_timestamp()?;
    let t0 = NaiveTime::from_hms_opt(t.hour(), t.minute(), t.second())?;
    Some(match nmea.fix_date {
        Some(d) => d.and_time(t0),
        None => synthetic_epoch_date().and_time(t0),
    })
}

/// Feed one complete NMEA line (no `\r\n`). ASCII/UTF-8 only.
///
/// `timelr_now` must be [`crate::platform::timers::timelr_now`] from the same timer
/// used for LCD UTC ticking.
pub fn feed_line(raw: &[u8], timelr_now: u32) {
    if raw.is_empty() {
        return;
    }
    // GSV (satellite sky view) is verbose and not needed for time/position/fix on this UI.
    // Skipping it avoids burning CPU on dozens of long sentences per epoch at 9600 baud.
    if raw.len() >= 6 && raw[0] == b'$' && raw.get(3..6) == Some(b"GSV") {
        return;
    }
    let text = match core::str::from_utf8(raw) {
        Ok(t) => t,
        Err(_) => {
            let mut g = GNSS_STORE.0.borrow_mut();
            g.stats.parse_errors = g.stats.parse_errors.wrapping_add(1);
            return;
        }
    };

    let mut g = GNSS_STORE.0.borrow_mut();

    let (applied, err_inc) = {
        let nmea = g.nmea.get_or_insert_with(Nmea::default);
        match nmea.parse(text) {
            Ok(_) => (true, false),
            Err(_) => (false, true),
        }
    };

    if applied {
        g.stats.sentences_applied = g.stats.sentences_applied.wrapping_add(1);

        let nmea = g.nmea.as_ref().expect("applied implies nmea exists");
        if let Some(dt_new) = candidate_anchor_utc(nmea) {
            let forward = match g.last_anchor_utc {
                None => true,
                Some(dt_prev) => dt_new > dt_prev,
            };
            if forward {
                g.last_anchor_utc = Some(dt_new);
                g.utc_anchor_timelr = timelr_now;
            }
        }
    }
    if err_inc {
        g.stats.parse_errors = g.stats.parse_errors.wrapping_add(1);
    }
}

/// Read the live parser, stats, UTC anchor tick, and latched anchor datetime.
#[allow(dead_code)]
pub fn with_store<R>(
    f: impl FnOnce(&Nmea, &GnssParseStats, u32, Option<NaiveDateTime>) -> R,
) -> R {
    let mut g = GNSS_STORE.0.borrow_mut();
    let stats_snap = g.stats.clone();
    let anchor_timelr = g.utc_anchor_timelr;
    let anchor_dt = g.last_anchor_utc;
    let nmea = g.nmea.get_or_insert_with(Nmea::default);
    f(nmea, &stats_snap, anchor_timelr, anchor_dt)
}
