//! Thin adapter: raw NMEA line bytes → [`crate::app::gnss_state`].

/// Process one inbound NMEA line and update global GNSS state.
#[inline]
pub fn process_nmea_line(line: &[u8], timelr_now: u32) {
    crate::app::gnss_state::feed_line(line, timelr_now);
}
