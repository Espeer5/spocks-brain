#![no_std]
#![no_main]

mod debug;

#[cfg(feature = "usb-log")]
use crate::debug::usb_log::UsbLogger;

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::OutputPin;
use panic_probe as _;

use rp_pico as bsp;

use bsp::hal::{
    clocks::{Clock, init_clocks_and_plls},
    gpio::FunctionUart,
    pac,
    sio::Sio,
    uart::{DataBits, StopBits, UartConfig, UartPeripheral},
    watchdog::Watchdog,
};

// For 9600.Hz()
use bsp::hal::fugit::RateExtU32;

#[entry]
fn main() -> ! {
    info!("Program start");

    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // Pico crystal is 12MHz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // Delay (kept from your working blinky)
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // Pins
    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // LED
    let mut led_pin = pins.led.into_push_pull_output();

    // ---- UART0 (GPS) on GP0/GP1 ----
    // GP1 is UART0 RX: connect GPS TX here
    let uart_pins = (
        pins.gpio0.into_function::<FunctionUart>(), // TX (optional)
        pins.gpio1.into_function::<FunctionUart>(), // RX
    );

    let uart_cfg = UartConfig::new(9600.Hz(), DataBits::Eight, None, StopBits::One);
    let uart = UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
        .enable(uart_cfg, clocks.peripheral_clock.freq())
        .unwrap();

    #[cfg(feature = "usb-log")]
    let mut log = UsbLogger::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        &mut pac.RESETS,
        clocks.usb_clock,
    );

    // Buffers

    // Track host-open state so we don't "miss" prints

    let mut uart_buf = [0u8; 64];

    let mut ms: u32 = 0;
    let mut hb: u32 = 0;
    let mut gps_bytes_total: u32 = 0;

    let mut line_buf = [0u8; 128];
    let mut line_len: usize = 0;

    let led_on = false;

    loop {
        #[cfg(feature = "usb-log")]
        log.poll();

        // Read GPS UART nonblocking and forward to USB (best-effort)
        if let Ok(n) = uart.read_raw(&mut uart_buf) {
            gps_bytes_total = gps_bytes_total.wrapping_add(n as u32);
            for &b in &uart_buf[..n] {
                if line_len < line_buf.len() {
                    line_buf[line_len] = b;
                    line_len += 1;
                } else {
                    #[cfg(feature = "usb-log")]
                    log.write(&line_buf[..line_len]);
                    line_len = 0;
                }

                #[cfg(feature = "usb-log")]
                if b == b'\n' {
                    // flush one full NMEA sentence
                    log.write(&line_buf[..line_len]);
                    line_len = 0;
                }
            }
        }

        // 1ms timebase (keeps USB polling happening constantly)
        delay.delay_ms(1u32);
        ms = ms.wrapping_add(1);

        // Blink LED at 1 Hz WITHOUT long blocking delays
        if ms.is_multiple_of(1000) {
            hb = hb.wrapping_add(1);

            // Unconditional heartbeat text (proves USB TX works)
            #[cfg(feature = "usb-log")]
            log.write(b"\r\nHB ");
            // minimal decimal print
            let mut num = hb;
            let mut digits = [0u8; 10];
            let mut i = 0usize;
            if num != 0 {
                while num > 0 && i < digits.len() {
                    digits[i] = b'0' + (num % 10) as u8;
                    num /= 10;
                    i += 1;
                }
                digits[..i].reverse();
            }
            #[cfg(feature = "usb-log")]
            log.write(&digits[..i]);

            #[cfg(feature = "usb-log")]
            log.write(b" | GPS bytes: ");
            let mut num2 = gps_bytes_total;
            let mut digits2 = [0u8; 10];
            let mut j = 0usize;
            if num2 != 0 {
                while num2 > 0 && j < digits2.len() {
                    digits2[j] = b'0' + (num2 % 10) as u8;
                    num2 /= 10;
                    j += 1;
                }
                digits2[..j].reverse();
            }
            #[cfg(feature = "usb-log")]
            log.write(&digits2[..j]);
            #[cfg(feature = "usb-log")]
            log.write(b"\r\n");

            // LED toggle
            if led_on {
                led_pin.set_high().ok();
            } else {
                led_pin.set_low().ok();
            }
        }
    }
}
