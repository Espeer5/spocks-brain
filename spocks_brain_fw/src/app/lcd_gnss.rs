//! 20×4 LCD rows: GNSS UTC time, latitude, longitude, fix summary.
//!
//! **UTC line:** [`crate::app::gnss_state`] latches a forward-only UTC anchor from
//! NMEA. The display adds [`crate::platform::timers::timelr`] elapsed time so a late
//! paint still shows the correct second.
//!
//! The main loop calls [`paint_utc_line_if_changed`] after **each** parsed NMEA line (and again
//! when idle) so UTC can advance while GSV/GSA bursts are processed. Row 0 only writes I²C
//! when the formatted text changes, so there is no per-line flicker cost.

use core::fmt::Write;

use chrono::{Datelike, Duration, NaiveDateTime, Timelike};
use heapless::String;
use nmea::Nmea;

use crate::app::gnss_state::GnssParseStats;
use crate::platform::i2c::I2cError;
use crate::platform::lcd_hd44780_i2c::LcdHd44780I2c;

/// Fixed-width rows for the LCD (ASCII space padding).
pub type LcdFourLines = [[u8; 20]; 4];

fn tick_display_from_anchor(
    anchor_dt: NaiveDateTime,
    utc_anchor_timelr: u32,
    now_timelr: u32,
) -> NaiveDateTime {
    let elapsed_us = now_timelr.wrapping_sub(utc_anchor_timelr) as i64;
    anchor_dt + Duration::microseconds(elapsed_us)
}

/// First LCD row only (20×1), padded with spaces.
pub fn format_gnss_line0(
    nmea: &Nmea,
    anchor_dt: Option<NaiveDateTime>,
    utc_anchor_timelr: u32,
    now_timelr: u32,
) -> [u8; 20] {
    let mut row = [b' '; 20];
    let mut l0 = String::<32>::new();

    if let Some(anchor) = anchor_dt.filter(|_| utc_anchor_timelr != 0) {
        let dt = tick_display_from_anchor(anchor, utc_anchor_timelr, now_timelr);
        if nmea.fix_date.is_some() {
            let d = dt.date();
            let _ = write!(
                &mut l0,
                "{:04}-{:02}-{:02} {:02}:{:02}:{:02}",
                d.year(),
                d.month(),
                d.day(),
                dt.hour(),
                dt.minute(),
                dt.second(),
            );
        } else {
            let _ = write!(
                &mut l0,
                "{:02}:{:02}:{:02} UTC",
                dt.hour(),
                dt.minute(),
                dt.second(),
            );
        }
    } else {
        match (nmea.fix_date, nmea.fix_timestamp()) {
            (Some(d), Some(t)) => {
                let _ = write!(
                    &mut l0,
                    "{:04}-{:02}-{:02} {:02}:{:02}:{:02}",
                    d.year(),
                    d.month(),
                    d.day(),
                    t.hour(),
                    t.minute(),
                    t.second(),
                );
            }
            (None, Some(t)) => {
                let _ = write!(
                    &mut l0,
                    "{:02}:{:02}:{:02} UTC",
                    t.hour(),
                    t.minute(),
                    t.second(),
                );
            }
            _ => {
                let _ = write!(&mut l0, "no GNSS time yet");
            }
        }
    }
    copy_trimmed(&mut row, l0.as_bytes());
    row
}

pub fn format_gnss_lines(
    nmea: &Nmea,
    stats: &GnssParseStats,
    anchor_dt: Option<NaiveDateTime>,
    utc_anchor_timelr: u32,
    now_timelr: u32,
) -> LcdFourLines {
    let mut out = [[b' '; 20]; 4];

    out[0] = format_gnss_line0(nmea, anchor_dt, utc_anchor_timelr, now_timelr);

    let mut l1 = String::<32>::new();
    match nmea.latitude() {
        Some(lat) => {
            let h = if lat >= 0.0 { 'N' } else { 'S' };
            let _ = write!(&mut l1, "Lat {:8.5} {}", lat.abs(), h);
        }
        None => {
            let _ = write!(&mut l1, "Lat ----");
        }
    }
    copy_trimmed(&mut out[1], l1.as_bytes());

    let mut l2 = String::<32>::new();
    match nmea.longitude() {
        Some(lon) => {
            let h = if lon >= 0.0 { 'E' } else { 'W' };
            let _ = write!(&mut l2, "Lon {:8.5} {}", lon.abs(), h);
        }
        None => {
            let _ = write!(&mut l2, "Lon ----");
        }
    }
    copy_trimmed(&mut out[2], l2.as_bytes());

    let mut l3 = String::<32>::new();
    let _ = write!(
        &mut l3,
        "fix:{:?} sats:{:?} ok:{} e:{}",
        nmea.fix_type(),
        nmea.fix_satellites(),
        stats.sentences_applied,
        stats.parse_errors
    );
    copy_trimmed(&mut out[3], l3.as_bytes());

    out
}

fn copy_trimmed(dst: &mut [u8; 20], src: &[u8]) {
    let n = src.len().min(20);
    dst[..n].copy_from_slice(&src[..n]);
}

/// Static boot text so the panel is not blank before the first NMEA line.
pub fn show_boot_splash(lcd: &mut LcdHd44780I2c<'_>, mut yield_fn: impl FnMut()) {
    const LINES: [&[u8]; 4] = [
        b"Spock brain         ",
        b"Waiting for NMEA... ",
        b"                    ",
        b"                    ",
    ];
    for row in 0..4u8 {
        yield_fn();
        let _ = lcd.write_line(row, LINES[row as usize]);
    }
    yield_fn();
}

/// If the UTC line content changed vs `prev_painted`, write row 0 and update `prev_painted`.
///
/// Call this **every main-loop iteration** so the second rolls as soon as `timelr` crosses a
/// 1 s boundary — not on a slow or coupled timer.
pub fn paint_utc_line_if_changed(
    lcd: &mut LcdHd44780I2c<'_>,
    now_timelr: u32,
    prev_painted: &mut [u8; 20],
    mut yield_fn: impl FnMut(),
) -> bool {
    let row = crate::app::gnss_state::with_store(|nmea, _stats, anchor_tl, anchor_dt| {
        format_gnss_line0(nmea, anchor_dt, anchor_tl, now_timelr)
    });
    if row == *prev_painted {
        return false;
    }
    yield_fn();
    match lcd.write_line(0, &row) {
        Ok(()) => {
            *prev_painted = row;
            yield_fn();
            true
        }
        Err(I2cError::TransferTimeout | I2cError::EnableTimeout | I2cError::NackOrArbitration) => {
            // Do not cache — LCD may still show old text; retry next loop.
            yield_fn();
            false
        }
    }
}

/// Rows 1–3 (lat / lon / stats). Row 0 is driven by [`paint_utc_line_if_changed`].
pub fn refresh_lcd_data_rows_with_yield(
    lcd: &mut LcdHd44780I2c<'_>,
    now_timelr: u32,
    mut yield_fn: impl FnMut(),
) {
    let lines = crate::app::gnss_state::with_store(|nmea, stats, anchor_tl, anchor_dt| {
        format_gnss_lines(nmea, stats, anchor_dt, anchor_tl, now_timelr)
    });
    for row in 1..4u8 {
        yield_fn();
        let _ = lcd.write_line(row, &lines[row as usize]);
    }
    yield_fn();
}
