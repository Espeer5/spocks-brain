//! The platform module is the application-specific abstraction layer for the
//! RP2040 for Spock's Brain. I've opted not not use the typical HAL for most
//! purposes for learning purposes. Most functionality in the platform module
//! can be subbed out for the HAL.

pub mod clocks;
pub mod constants;
pub mod gpio;
pub mod resets;
pub mod timers;
pub mod uart;

use rp_pico::hal::pac::{IO_BANK0, PADS_BANK0, SIO, UART0};

/// GPIO + UART after the system clock runs at 125 MHz. Clock bring-up is done
/// in [`crate::main`] via [`clocks::init_12mhz_xosc_plls`].
///
/// Takes register blocks individually so this still compiles after `pac` is
/// partially moved by [`rp2040_hal::clocks::init_clocks_and_plls`].
pub fn init(
    io: &mut IO_BANK0,
    pads: &mut PADS_BANK0,
    sio: &mut SIO,
    uart0: &mut UART0,
) {
    gpio::init_gpios(io, pads);
    gpio::init_onboard_led(io, pads, sio);
    uart::init_uart0(uart0, constants::UART0_BAUD_RATE);
}
