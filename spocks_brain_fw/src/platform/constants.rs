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

/// UART0 baud rate for GNSS module
#[allow(dead_code)]
pub const UART0_BAUD_RATE: u32 = 9600;

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
