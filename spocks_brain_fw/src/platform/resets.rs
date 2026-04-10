//! Internal reset handling for the RP2040

use rp_pico::hal::pac::Peripherals;

////////////////////////////////////////////////////////////////////////////////
// USED RESET MASKS
////////////////////////////////////////////////////////////////////////////////

const IO_BANK0_RESET_MASK:  u32 = 1 << 5;
const PAD_BANK0_RESET_MASK: u32 = 1 << 8;
const TIMER_RESET_MASK:     u32 = 1 << 21;
const UART0_RESET_MASK:     u32 = 1 << 22;
const PLL_SYS_RESET_MASK:   u32 = 1 << 12;
const PLL_USB_RESET_MASK:   u32 = 1 << 13;

////////////////////////////////////////////////////////////////////////////////
// RESET HELPERS
////////////////////////////////////////////////////////////////////////////////

/// Clear reset bits in `mask` and wait until `reset_done` matches
fn clear_reset_blocking(pac: &mut Peripherals, mask: u32) {
    pac.RESETS.reset().modify(|r, w| unsafe {
        w.bits(r.bits() & !mask)
    });
    cortex_m::asm::dsb();
    cortex_m::asm::isb();

    while (pac.RESETS.reset_done().read().bits() & mask) != mask {
        cortex_m::asm::nop();
    }
}

////////////////////////////////////////////////////////////////////////////////
// PUB RESET ROUTINES
////////////////////////////////////////////////////////////////////////////////

/// Release PLL_SYS and PLL_USB from reset.
pub fn clear_pll_resets(pac: &mut Peripherals) {
    const PLL_MASK: u32 = PLL_SYS_RESET_MASK | PLL_USB_RESET_MASK;
    clear_reset_blocking(pac, PLL_MASK)
}

/// Clear GPIO + pads reset bits
pub fn clear_gpio_subsystem_resets(pac: &mut Peripherals) {
    const GPIO_MASK: u32 = IO_BANK0_RESET_MASK | PAD_BANK0_RESET_MASK;
    clear_reset_blocking(pac, GPIO_MASK)
}

/// Release **TIMER** from reset only.
pub fn clear_timer_reset(pac: &mut Peripherals) {
    clear_reset_blocking(pac, TIMER_RESET_MASK)
}

/// Release **UART0** from reset only.
///
/// Call after `clk_peri` is enabled (e.g. end of [`super::clocks::init_12mhz_xosc_plls`]);
/// otherwise `reset_done` for UART may not assert.
pub fn clear_uart0_reset(pac: &mut Peripherals) {
    clear_reset_blocking(pac, UART0_RESET_MASK)
}

/// Clear timer and UART0 resets
#[allow(dead_code)]
pub fn clear_timer_uart_resets(pac: &mut Peripherals) {
    clear_timer_reset(pac);
    clear_uart0_reset(pac);
}

/// Clear all reset bits needed for GPIO, pads, timer, and UART
#[allow(dead_code)]
pub fn clear_io_resets(pac: &mut Peripherals) {
    const IO_MASK: u32 = IO_BANK0_RESET_MASK
        | PAD_BANK0_RESET_MASK
        | TIMER_RESET_MASK
        | UART0_RESET_MASK;
    clear_reset_blocking(pac, IO_MASK)
}
