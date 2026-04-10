//! GPIO control for Spock's brain

use rp_pico::hal::pac::{IO_BANK0, PADS_BANK0, SIO};

/// Raspberry Pi Pico onboard LED (GP25), SIO output, active high.
pub const ONBOARD_LED_GPIO: usize = 25;

////////////////////////////////////////////////////////////////////////////////
// PUB GPIO INITIALIZATION ROUTINES
////////////////////////////////////////////////////////////////////////////////

/// Initialize GP0 and GP1 for UART0 (GNSS): GP0 = TX, GP1 = RX.
/// GNSS TX connects to RP2040 RX (GP1); RP2040 TX (GP0) connects to GNSS RX.
pub fn init_gpios(io: &mut IO_BANK0, pads: &mut PADS_BANK0) {
    // Pad settings: no pull, input enable for RX (GP1); TX (GP0) gets direction from UART.
    // PAC uses PADS_BANK0.gpio(n) and pad fields ie(), pue(), pde(), od().
    pads.gpio(0).modify(|_, w| {
        w.pue().clear_bit();
        w.pde().clear_bit();
        w.od().clear_bit();
        w.ie().set_bit()
    });
    pads.gpio(1).modify(|_, w| {
        w.pue().clear_bit();
        w.pde().clear_bit();
        w.od().clear_bit();
        w.ie().set_bit()
    });

    // IO function selection: GP0 and GP1 as UART0 (PAC: IO_BANK0.gpio(n).gpio_ctrl()).
    io.gpio(0).gpio_ctrl().modify(|_, w| w.funcsel().uart());
    io.gpio(1).gpio_ctrl().modify(|_, w| w.funcsel().uart());
}

/// Configure GP25 as SIO output for the Pico’s built-in LED (active high).
pub fn init_onboard_led(io: &mut IO_BANK0, pads: &mut PADS_BANK0, sio: &mut SIO) {
    let n = ONBOARD_LED_GPIO;
    let mask = 1u32 << n;

    pads.gpio(n).modify(|_, w| {
        w.pue().clear_bit();
        w.pde().clear_bit();
        w.od().clear_bit();
        w.ie().clear_bit()
    });
    io.gpio(n).gpio_ctrl().modify(|_, w| w.funcsel().sio());

    sio.gpio_oe_set().write(|w| unsafe { w.bits(mask) });
    // Start with LED off (output low).
    sio.gpio_out_clr().write(|w| unsafe { w.bits(mask) });
}

/// Drive the onboard LED on (`true`) or off (`false`).
#[allow(dead_code)]
pub fn set_onboard_led(sio: &mut SIO, on: bool) {
    let mask = 1u32 << ONBOARD_LED_GPIO;
    if on {
        sio.gpio_out_set().write(|w| unsafe { w.bits(mask) });
    } else {
        sio.gpio_out_clr().write(|w| unsafe { w.bits(mask) });
    }
}

/// Toggle the onboard LED output (XOR the pin in `GPIO_OUT`).
pub fn toggle_onboard_led(sio: &mut SIO) {
    let mask = 1u32 << ONBOARD_LED_GPIO;
    sio.gpio_out_xor().write(|w| unsafe { w.bits(mask) });
}

