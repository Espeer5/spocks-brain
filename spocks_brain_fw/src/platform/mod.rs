//! The platform module is the application-specific abstraction layer for the
//! RP2040 for Spock's Brain. I've opted not not use the typical HAL for most
//! purposes for learning purposes. Most functionality in the platform module
//! can be subbed out for the HAL.

pub mod clocks;
pub mod constants;
pub mod gpio;
pub mod resets;
pub mod uart;

use rp_pico::hal::pac::Peripherals;

/// Initialize the hardware. We use the available register map since recreating
/// it offers little learning other than how to read a datasheet.
pub fn init(pac: &mut Peripherals) {
    // Release resets for the blocks needed during init
    resets::clear_pll_resets(pac);
    resets::clear_io_resets(pac);
    
    // Bringup the system clock and peripheral clocks
    clocks::init_12mhz_xosc_plls(pac);

    // GP0 = UART0 TX, GP1 = UART0 RX for GNSS
    gpio::init_gpios(pac);

    // Initialize UART0 for GNSS
    uart::init_uart0(pac, constants::UART0_BAUD_RATE);
}
