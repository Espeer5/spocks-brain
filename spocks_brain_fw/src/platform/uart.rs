//! UART control for Spock's brain

use core::cell::RefCell;

use cortex_m::interrupt::{self, Mutex};
use heapless::Deque;

use crate::platform::constants::{PERI_CLOCK_HZ, UART0_RX_RING_CAP};
use rp_pico::hal::pac::UART0;

////////////////////////////////////////////////////////////////////////////////
// UART INITIALIZATION HELPERS
////////////////////////////////////////////////////////////////////////////////

/// Calculate the baud divisors for the given baud rate
/// The baud rate divisor is calculated as PERI_CLOCK_HZ / (16 * baud_rate) and
/// must be expressed as a 16 bit integer and a 6 bit fractional part.
fn baud_divisors(baud_rate: u32) -> (u16, u8) {
    let divisor = 16u32 * baud_rate;

    let integer_part = (PERI_CLOCK_HZ / divisor) as u16;
    let remainder = PERI_CLOCK_HZ % divisor;

    let frac64 = (remainder * 64 + divisor / 2) / divisor;

    (integer_part, frac64 as u8)
}

////////////////////////////////////////////////////////////////////////////////
// RX ring (ISR producer, main consumer)
////////////////////////////////////////////////////////////////////////////////

static UART_RX_RING: Mutex<RefCell<Deque<u8, UART0_RX_RING_CAP>>> =
    Mutex::new(RefCell::new(Deque::new()));

/// Bytes lost because the RX ring was full (monotonic counter).
static UART_RX_DROPPED_BYTES: Mutex<RefCell<u32>> = Mutex::new(RefCell::new(0));

/// Total bytes pushed into the RX ring from `UART0_IRQ` (ISR → main), for diagnostics.
static UART_RX_BYTES_TOTAL: Mutex<RefCell<u32>> = Mutex::new(RefCell::new(0));

/// Push one byte from the UART ISR. Returns `false` if the ring was full.
pub fn push_rx_byte_from_isr(byte: u8) -> bool {
    interrupt::free(|cs| {
        let mut t = UART_RX_BYTES_TOTAL.borrow(cs).borrow_mut();
        *t = t.wrapping_add(1);

        let mut d = UART_RX_RING.borrow(cs).borrow_mut();
        if d.push_back(byte).is_ok() {
            return true;
        }
        let mut n = UART_RX_DROPPED_BYTES.borrow(cs).borrow_mut();
        *n = n.wrapping_add(1);
        false
    })
}

/// Pop one byte on the main thread (or under `interrupt::free`).
pub fn try_pop_rx_byte() -> Option<u8> {
    interrupt::free(|cs| UART_RX_RING.borrow(cs).borrow_mut().pop_front())
}

/// Monotonic count of bytes dropped due to a full RX ring.
#[allow(dead_code)]
pub fn rx_dropped_byte_count() -> u32 {
    interrupt::free(|cs| *UART_RX_DROPPED_BYTES.borrow(cs).borrow())
}

/// Total bytes read from the UART RX FIFO in `UART0_IRQ` (includes bytes lost if the ring was full).
/// Exposed for bring-up; call sites may omit it in release firmware.
#[allow(dead_code)]
pub fn rx_bytes_total() -> u32 {
    interrupt::free(|cs| *UART_RX_BYTES_TOTAL.borrow(cs).borrow())
}

////////////////////////////////////////////////////////////////////////////////
// NVIC
////////////////////////////////////////////////////////////////////////////////

/// Unmask `UART0_IRQ` in the NVIC. Call after UART and RX ring are ready.
pub fn enable_uart0_interrupt() {
    use cortex_m::asm;
    use cortex_m::peripheral::NVIC;
    use rp_pico::hal::pac::Interrupt;

    NVIC::unpend(Interrupt::UART0_IRQ);

    unsafe {
        NVIC::unmask(Interrupt::UART0_IRQ);
        asm::dsb();
        asm::isb();
        cortex_m::interrupt::enable();
    }
}

////////////////////////////////////////////////////////////////////////////////
// IRQ handler body (called from `#[interrupt] fn UART0_IRQ`)
////////////////////////////////////////////////////////////////////////////////

/// Service UART0 receive: drain FIFO into the RX ring, clear IRQ status. Call only from
/// `#[interrupt] fn UART0_IRQ` — the main loop does not poll the UART.
pub fn handle_uart0_rx_irq(uart0: &mut UART0) {
    let _ = uart0.uartmis().read();

    loop {
        let fr = uart0.uartfr().read();
        if fr.rxfe().bit() {
            break;
        }

        let dr = uart0.uartdr().read();
        if dr.fe().bit() || dr.pe().bit() || dr.be().bit() || dr.oe().bit() {
            uart0.uartrsr().write(|w| unsafe { w.bits(0xF) });
        }

        let b = dr.data().bits();
        let _ = push_rx_byte_from_isr(b);
    }

    // Clear sticky interrupt sources (write-one-to-clear bits per PL011).
    uart0.uarticr().write(|w| unsafe { w.bits(0x7FF) });
}

////////////////////////////////////////////////////////////////////////////////
// PUB UART INITIALIZATION ROUTINES
////////////////////////////////////////////////////////////////////////////////

/// Initialize UART0 for use with the GNSS module (8N1, FIFOs, baud). Does not
/// enable RX interrupts; call [`uart0_enable_rx_interrupts`] after setup.
pub fn init_uart0(uart0: &mut UART0, baud_rate: u32) {
    // Disable UART while configuring
    uart0.uartcr().modify(|_, w| w.uarten().clear_bit());

    // Clear UART interrupts
    uart0.uarticr().write(|w| unsafe { w.bits(0x7FF) });

    let (integer_part, frac64) = baud_divisors(baud_rate);

    // PL011 latches baud divisors on the next `uartlcr_h` write — program IBRD/FBRD *before*
    // LCR_H (see rp2040-hal `configure_baudrate`). Writing LCR_H first leaves divisors invalid.
    uart0
        .uartibrd()
        .write(|w| unsafe { w.bits(integer_part.into()) });
    uart0.uartfbrd().write(|w| unsafe { w.bits(frac64.into()) });

    uart0
        .uartlcr_h()
        .write(|w| unsafe { w.wlen().bits(0b11).fen().set_bit() });

    uart0.uartcr().write(|w| {
        w.txe().set_bit();
        w.rxe().set_bit();
        w.uarten().set_bit()
    });
}

/// RX FIFO trigger at 1/2 full, unmask RX + receive-timeout + error interrupts.
pub fn uart0_enable_rx_interrupts(uart0: &mut UART0) {
    uart0.uartifls().modify(|r, w| unsafe {
        let v = r.bits();
        w.bits((v & !(7 << 3)) | (0b010 << 3))
    });

    uart0.uartimsc().write(|w| {
        w.rxim().set_bit();
        w.rtim().set_bit();
        w.oeim().set_bit();
        w.feim().set_bit();
        w.peim().set_bit();
        w.beim().set_bit()
    });
}
