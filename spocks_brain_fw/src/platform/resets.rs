//! Internal reset handling for the RP2040

use rp_pico::hal::pac::Peripherals;

////////////////////////////////////////////////////////////////////////////////
/// USED RESET BITS
////////////////////////////////////////////////////////////////////////////////

/// Mask bit for IO bank 0
const IO_BANK0_RESET_BIT: u32 = 5;

/// Mask bit for pads bank 0
const PAD_BANK0_RESET_BIT: u32 = 8;

/// Mask bit for timer
const TIMER_RESET_BIT: u32 = 21;

/// Mask bit for UART0
const UART0_RESET_BIT: u32 = 22;

/// Mask bit for system clock PLL
const PLL_SYS_RESET_BIT: u32 = 12;

/// Mask bit for usb clock PLL
const PLL_USB_RESET_BIT: u32 = 13;

////////////////////////////////////////////////////////////////////////////////
/// RESET HELPERS
////////////////////////////////////////////////////////////////////////////////

/// Clears all internal reset bits indicated in the given mask
fn clear_reset(pac: &mut Peripherals, mask: u32)
{
    // Clear bits in RESETS.reset (active-high reset control)
    pac.RESETS.reset().modify(|r, w| unsafe {
        w.bits(r.bits() & !mask)
    });
    // Wait until reset_done shows the block is out of reset
    while (pac.RESETS.reset_done().read().bits() & mask) != mask {}
}

////////////////////////////////////////////////////////////////////////////////
/// PUB RESET ROUTINES
////////////////////////////////////////////////////////////////////////////////

/// Clear reset bits for the system and USB clocks
pub fn clear_pll_resets(pac: &mut Peripherals) {
    const PLL_MASK: u32 = PLL_SYS_RESET_BIT | PLL_USB_RESET_BIT;
    clear_reset(pac, PLL_MASK);
}

/// Clear reset bits for IO components
pub fn clear_io_resets(pac: &mut Peripherals) {
    const IO_MASK: u32 = IO_BANK0_RESET_BIT  |
                         PAD_BANK0_RESET_BIT |
                         TIMER_RESET_BIT     |
                         UART0_RESET_BIT;
    clear_reset(pac, IO_MASK);
}
