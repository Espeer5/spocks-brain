//! Platform constants for Spock's brain

////////////////////////////////////////////////////////////////////////////////
// CLOCK FREQUENCIES
////////////////////////////////////////////////////////////////////////////////

/// System clock frequency in Hz
pub const SYS_CLOCK_HZ: u32 = 125_000_000;

/// USB clock frequency in Hz
#[allow(dead_code)]
pub const USB_CLOCK_HZ: u32 = 48_000_000;

/// Peripheral clock frequency in Hz
#[allow(dead_code)]
pub const PERI_CLOCK_HZ: u32 = SYS_CLOCK_HZ;

////////////////////////////////////////////////////////////////////////////////
// PERIPHERAL BAUD RATES
////////////////////////////////////////////////////////////////////////////////

/// UART0 baud rate for GNSS module (many modules default to 9600; change if yours
/// uses 38400 or another rate).
pub const UART0_BAUD_RATE: u32 = 9600;

/// Byte ring from UART0 ISR to main (NMEA bursts are short at 9600 baud).
pub const UART0_RX_RING_CAP: usize = 512;

/// Max NMEA sentence length (incl. `$` … `\r\n`) with margin for multi-GNSS GSA/GSV.
pub const NMEA_LINE_CAP: usize = 128;

/// Bounded application event queue depth.
pub const APP_EVENT_QUEUE_CAP: usize = 16;

////////////////////////////////////////////////////////////////////////////////
// SOFTWARE TIMERS
////////////////////////////////////////////////////////////////////////////////

/// Maximum concurrent generic software timers (see `platform::timers`).
/// Increase when more simultaneous `schedule_soft` timers are needed.
pub const SW_TIMER_MAX_HEAP: usize = 10;

/// One second in microseconds (RP2040 timer tick interval 1 MHz).
#[allow(dead_code)]
pub const ONE_SECOND_US: u32 = 1_000_000;

/// First soft-timer expiry after scheduling (microseconds).
pub const LED_BLINK_INITIAL_DELAY_US: u32 = 10_000;

/// Soft-timer period for onboard LED blink demo (microseconds). 250 ms edge-to-edge → 2 Hz toggle.
pub const LED_BLINK_PERIOD_US: u32 = 250_000;
