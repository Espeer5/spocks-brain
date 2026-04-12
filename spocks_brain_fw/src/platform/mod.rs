//! The platform module is the application-specific abstraction layer for the
//! RP2040 for Spock's Brain. I've opted not not use the typical HAL for most
//! purposes for learning purposes. Most functionality in the platform module
//! can be subbed out for the HAL.
//!
//! ## Second core (RP2040 is dual Cortex-M0+)
//! The chip is not “too slow” for NMEA + HD44780 — typical bottlenecks are **blocking USB CDC
//! writes**, **bit-banged I²C timing**, or **parsing every GSV line**. Offloading work to core 1
//! ([`rp_pico::hal::multicore`]) can help CPU-bound parsing, but **UART, USB, and a given I²C bus
//! must still be owned by one core** (or guarded). A common split is core 0: peripherals + LCD,
//! core 1: NMEA parse from an SPSC queue — not wired up in this tree yet.

pub mod clocks;
pub mod constants;
pub mod gpio;
pub mod i2c;
pub mod lcd_hd44780_i2c;
pub mod resets;
pub mod timers;
pub mod uart;

use rp_pico::hal::pac::{IO_BANK0, PADS_BANK0, SIO, UART0};

/// GPIO + UART after the system clock runs at 125 MHz. Clock bring-up is done
/// in [`crate::main`] via [`crate::platform::clocks::init_12mhz_xosc_plls`].
pub fn init(io: &mut IO_BANK0, pads: &mut PADS_BANK0, sio: &mut SIO, uart0: &mut UART0) {
    gpio::init_gpios(io, pads);
    gpio::init_onboard_led(io, pads, sio);
    uart::init_uart0(uart0, constants::UART0_BAUD_RATE);
    // RX IRQ masking on the UART block + NVIC unmask: see `main` after USB/LCD
    // bring-up so `usb_log.poll()` is not starved during enumeration.
}
