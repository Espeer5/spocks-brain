//! GPIO control for Spock's brain

use rp_pico::hal::pac::Peripherals;

////////////////////////////////////////////////////////////////////////////////
/// PUB GPIO INITIALIZATION ROUTINES
////////////////////////////////////////////////////////////////////////////////

/// Initialize GP0 and GP1 for UART0 (GNSS): GP0 = TX, GP1 = RX.
/// GNSS TX connects to RP2040 RX (GP1); RP2040 TX (GP0) connects to GNSS RX.
pub fn init_gpios(pac: &mut Peripherals) {
    // Pad settings: no pull, input enable for RX (GP1); TX (GP0) gets direction from UART.
    // PAC uses PADS_BANK0.gpio(n) and pad fields ie(), pue(), pde(), od().
    pac.PADS_BANK0.gpio(0).modify(|_, w| {
        w.pue().clear_bit();
        w.pde().clear_bit();
        w.od().clear_bit();
        w.ie().set_bit()
    });
    pac.PADS_BANK0.gpio(1).modify(|_, w| {
        w.pue().clear_bit();
        w.pde().clear_bit();
        w.od().clear_bit();
        w.ie().set_bit()
    });

    // IO function selection: GP0 and GP1 as UART0 (PAC: IO_BANK0.gpio(n).gpio_ctrl()).
    pac.IO_BANK0.gpio(0).gpio_ctrl().modify(|_, w| w.funcsel().uart());
    pac.IO_BANK0.gpio(1).gpio_ctrl().modify(|_, w| w.funcsel().uart());
}
