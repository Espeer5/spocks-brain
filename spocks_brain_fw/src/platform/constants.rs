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
/// GSV bursts can enqueue many lines per navigation epoch; keep headroom so RMC/GGA
/// lines are not dropped while the main thread is busy.
pub const APP_EVENT_QUEUE_CAP: usize = 48;

/// Max NMEA lines to dequeue per **outer** main-loop pass (before `feed_bytes` runs again).
/// Row 0 (UTC) is repainted after **each** dispatched line so GSV/GSA bursts cannot delay
/// the clock until a full batch finishes. This cap only bounds how much we drain before
/// other work (`usb_log`, full LCD rows) in the same pass.
pub const MAX_NMEA_EVENTS_PER_LOOP: usize = 12;

/// How often to redraw lat/lon/stats rows (microseconds). UTC row 0 is repainted every
/// main-loop iteration when the formatted text changes.
pub const LCD_GNSS_FULL_REFRESH_PERIOD_US: u32 = 500_000;

/// With `usb-log`, how often to emit a GNSS text snapshot over USB. **Not** every NMEA line —
/// dumping on every line blocked the main loop for so long the LCD only advanced in huge jumps.
#[cfg(feature = "usb-log")]
pub const USB_GNSS_DEBUG_DUMP_PERIOD_US: u32 = 2_000_000;

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

////////////////////////////////////////////////////////////////////////////////
// I2C1 — HD44780 + PCF8574 backpack (Pico: GP2 = SDA, GP3 = SCL)
////////////////////////////////////////////////////////////////////////////////

/// I²C SCL frequency (Hz). 400 kHz fast mode is widely supported on PCF8574 backpacks.
pub const I2C1_SCL_HZ: u32 = 400_000;

/// 7-bit address of the PCF8574 backpack (this module uses `0x27`).
pub const I2C_LCD_ADDR_7BIT: u8 = 0x27;

/// GPIO index for I2C1 SDA.
pub const I2C1_SDA_GPIO: usize = 2;

/// GPIO index for I2C1 SCL.
pub const I2C1_SCL_GPIO: usize = 3;

/// How the PCF8574 outputs are wired to the HD44780 on your backpack.
///
/// Most **ebay / “Arduino”** 2004 I²C modules follow the `LiquidCrystal_I2C`
/// sketch library: **RS=P0, RW=P1, E=P2, BL=P3**, **D4–D7=P4–P7** (data on the
/// **upper** nibble of the expander byte).
///
/// Some boards (e.g. certain SparkFun-style backpacks) use **D4–D7 on P0–P3**
/// and **RS/RW/E/BL on P4–P7** instead — use [`Pcf8574Hd44780Wiring::DataLowControlsHigh`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Pcf8574Hd44780Wiring {
    /// D4–D7 on P0–P3; RS=P4, RW=P5, E=P6, BL=P7 (active high on P7).
    #[allow(dead_code)]
    DataLowControlsHigh,
    /// Same as common `LiquidCrystal_I2C` / FM backpacks.
    #[allow(dead_code)]
    LiquidCrystalI2cLib,
}

/// Change this if the LCD shows solid blocks or garbage with correct I²C ACKs.
pub const LCD8574_WIRING: Pcf8574Hd44780Wiring = Pcf8574Hd44780Wiring::LiquidCrystalI2cLib;

impl Pcf8574Hd44780Wiring {
    /// Safe idle byte for I²C address probes: backlight “on”, E/RS/RW low, no data strobes.
    pub const fn probe_idle_byte(self) -> u8 {
        match self {
            Self::DataLowControlsHigh => 0x80,
            Self::LiquidCrystalI2cLib => 0x08,
        }
    }

    /// Enable pulse bit on the expander (not part of `pack_nibble`).
    pub const fn e_mask(self) -> u8 {
        match self {
            Self::DataLowControlsHigh => 1 << 6,
            Self::LiquidCrystalI2cLib => 1 << 2,
        }
    }

    /// Port pattern with E low: RS, backlight, D4–D7 nibble (logical lower nibble of HD44780 byte).
    pub fn pack_nibble(self, rs: bool, nibble: u8) -> u8 {
        let n = nibble & 0x0F;
        match self {
            Self::DataLowControlsHigh => {
                (if rs { 1 << 4 } else { 0 }) | (1 << 7) | n
            }
            Self::LiquidCrystalI2cLib => {
                (if rs { 1 } else { 0 }) | (1 << 3) | (n << 4)
            }
        }
    }
}
