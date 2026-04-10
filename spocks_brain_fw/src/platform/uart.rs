//! UART control for Spock's brain

use crate::platform::constants::PERI_CLOCK_HZ;
use rp_pico::hal::pac::Peripherals;

////////////////////////////////////////////////////////////////////////////////
/// UART INITIALIZATION HELPERS
////////////////////////////////////////////////////////////////////////////////

/// Calculate the baud divisors for the given baud rate
/// The baud rate divisor is calculated as PERI_CLOCK_HZ / (16 * baud_rate) and
/// must be expressed as a 16 bit integer and a 6 bit fractional part.
fn baud_divisors(baud_rate: u32) -> (u16, u8) {
    // Calculate the baud rate divisor using fixed point arithmetic to avoid
    // floating point operations
    let divisor = 16u32 * baud_rate;

    let integer_part = (PERI_CLOCK_HZ / divisor) as u16;
    let remainder    = PERI_CLOCK_HZ % divisor;

    // frac64 = round(rem * 64 / divisor)
    let frac64 = (remainder as u32 * 64 + divisor / 2) / divisor as u32;

    (integer_part, frac64 as u8)
}

////////////////////////////////////////////////////////////////////////////////
/// PUB UART INITIALIZATION ROUTINES
////////////////////////////////////////////////////////////////////////////////

/// Initialize UART0 for use with the GNSS module
pub fn init_uart0(pac: &mut Peripherals, baud_rate: u32) {
    // Set up the UART0 peripheral
    let uart0 = &pac.UART0;

    // Disable UART while configuring
    uart0.uartcr().modify(|_, w| w.uarten().clear_bit());

    // Clear UART interrupts -- bits 10:0 enable all interrupts
    uart0.uarticr().write(|w| unsafe { w.bits(0x7FF) });

    // Enable RX and TX FIFOs and set word len to 8 bits
    uart0.uartlcr_h().write(|w| unsafe {
        w.wlen().bits(0b11) // 8 bits
            .fen().set_bit() // enable FIFOs
    });

    // Calculate the baud divisors for the given baud rate
    let (integer_part, frac64) = baud_divisors(baud_rate);

    // Program the baud rate divisors
    uart0.uartibrd().write(|w| unsafe { w.bits(integer_part.into()) });
    uart0.uartfbrd().write(|w| unsafe { w.bits(frac64.into()) });

    // Enable TX, RX, UART
    uart0.uartcr().write(|w| {
        w.txe().set_bit();
        w.rxe().set_bit();
        w.uarten().set_bit()
    });

    // TODO: enable interrupts when needed
}