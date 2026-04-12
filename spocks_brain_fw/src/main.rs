#![no_std]
#![no_main]

mod app;
mod platform;

#[cfg(feature = "usb-log")]
mod debug;

use panic_halt as _;
use rp_pico::hal::clocks::ClocksManager;
use rp_pico::hal::pac::interrupt;
use rp_pico::{entry, hal::pac::Peripherals};

#[cfg(not(feature = "usb-log"))]
use cortex_m::asm;

use app::events::{self, NmeaLineAssembler};
use platform::constants::{
    I2C_LCD_ADDR_7BIT, LCD_GNSS_FULL_REFRESH_PERIOD_US, MAX_NMEA_EVENTS_PER_LOOP,
};
#[cfg(feature = "usb-log")]
use platform::constants::USB_GNSS_DEBUG_DUMP_PERIOD_US;
use platform::i2c::I2c1Master;
use platform::lcd_hd44780_i2c::LcdHd44780I2c;
use platform::uart;
#[cfg(feature = "usb-log")]
use debug::usb_log::{write_gnss_state, UsbLogger};

#[entry]
fn main() -> ! {
    let mut pac = Peripherals::take().unwrap();

    platform::resets::clear_gpio_subsystem_resets(&mut pac.RESETS);

    platform::gpio::init_onboard_led(&mut pac.IO_BANK0, &mut pac.PADS_BANK0, &mut pac.SIO);

    platform::resets::clear_timer_reset(&mut pac.RESETS);

    // Clock bring-up (see `platform::clocks`). PLL blocks must be out of reset
    // before programming.
    platform::resets::clear_pll_resets(&mut pac.RESETS);
    platform::clocks::init_12mhz_xosc_plls(&mut pac).unwrap();

    // Own the CLOCKS block after PAC PLL setup. Do **not** replace with
    // `mem::zeroed()` — that is UB and breaks the `UsbClock` handle so USB
    // never enumerates.
    let clocks_mgr = ClocksManager::new(pac.CLOCKS);

    platform::resets::clear_uart0_reset(&mut pac.RESETS);

    platform::init(
        &mut pac.IO_BANK0,
        &mut pac.PADS_BANK0,
        &mut pac.SIO,
        &mut pac.UART0,
    );

    platform::gpio::init_i2c1_pins(&mut pac.IO_BANK0, &mut pac.PADS_BANK0);

    #[cfg(feature = "usb-log")]
    let mut usb_log = UsbLogger::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        &mut pac.RESETS,
        clocks_mgr.usb_clock,
    );

    #[cfg(feature = "usb-log")]
    unsafe {
        UsbLogger::register_ptr_for_isr(&mut usb_log);
    }

    #[cfg(not(feature = "usb-log"))]
    let _clocks_mgr = clocks_mgr;

    // Timer + LED **before** I2C/LCD so a stuck bus or long HD44780 power-up
    // cannot starve the soft-timer blink. UART RX NVIC stays masked until after
    // USB + LCD so ISR noise cannot starve `usb_log.poll()` during enumeration.
    platform::timers::configure_irq_priorities();

    #[cfg(feature = "usb-log")]
    {
        use cortex_m::asm;
        use cortex_m::peripheral::NVIC;
        use rp_pico::hal::pac::Interrupt;
        NVIC::unpend(Interrupt::USBCTRL_IRQ);
        unsafe {
            NVIC::unmask(Interrupt::USBCTRL_IRQ);
            asm::dsb();
            asm::isb();
        }
    }
    platform::timers::init_timer_hardware(&mut pac.TIMER);
    platform::timers::enable_timer_interrupts();

    let _led_demo =
        platform::gpio::schedule_onboard_led_demo(&mut pac.TIMER).expect("LED demo failed");

    // Give USB a poll budget before I2C/LCD so enumeration is not starved.
    #[cfg(feature = "usb-log")]
    {
        for _ in 0..50_000 {
            usb_log.poll();
        }
    }

    let mut i2c1 = I2c1Master::new(&mut pac.I2C1, &mut pac.RESETS).expect("I2C1 controller");
    // Prefer configured address; many backpacks respond on 0x3F instead of 0x27.
    let lcd_addr = if i2c1.probe_ack(I2C_LCD_ADDR_7BIT) {
        I2C_LCD_ADDR_7BIT
    } else if i2c1.probe_ack(0x3F) {
        0x3F
    } else {
        I2C_LCD_ADDR_7BIT
    };

    #[cfg(feature = "usb-log")]
    {
        use core::fmt::Write;
        use heapless::String;
        let mut s = String::<64>::new();
        let _ = writeln!(&mut s, "I2C1: HD44780 at 7-bit addr 0x{:02x}", lcd_addr);
        debug::usb_log::write_str(&mut usb_log, s.as_str());
    }

    let mut lcd = LcdHd44780I2c::new_with_addr(&mut i2c1, lcd_addr);

    #[cfg(feature = "usb-log")]
    {
        lcd.init_with_yield(|| usb_log.poll()).expect("LCD init");
        app::lcd_gnss::show_boot_splash(&mut lcd, || usb_log.poll());
    }

    #[cfg(not(feature = "usb-log"))]
    {
        lcd.init().expect("LCD init");
        app::lcd_gnss::show_boot_splash(&mut lcd, || {});
    }

    uart::uart0_enable_rx_interrupts(&mut pac.UART0);
    uart::enable_uart0_interrupt();

    let mut nmea = NmeaLineAssembler::new();
    let t0 = platform::timers::timelr_now(&pac.TIMER);
    let mut next_full_lcd_us = t0;
    #[cfg(feature = "usb-log")]
    let mut last_usb_gnss_dump_us = t0;
    // Last UTC row bytes on the LCD; repainted whenever the formatted line differs.
    let mut lcd_line0_cache: [u8; 20] = [b' '; 20];

    loop {
        #[cfg(feature = "usb-log")]
        usb_log.poll();

        nmea.feed_bytes();

        // Parse up to MAX_NMEA_EVENTS_PER_LOOP lines per outer iteration. Paint UTC row 0 after
        // **each** line so heavy GSV/GSA bursts do not stall the clock until a batch completes.
        let mut nmea_events = 0usize;
        while nmea_events < MAX_NMEA_EVENTS_PER_LOOP {
            let Some(ev) = events::try_pop_event() else {
                break;
            };
            let timelr_now = platform::timers::timelr_now(&pac.TIMER);
            events::dispatch_event(ev, timelr_now);
            nmea_events += 1;

            let now = platform::timers::timelr_now(&pac.TIMER);
            #[cfg(feature = "usb-log")]
            let _ = app::lcd_gnss::paint_utc_line_if_changed(&mut lcd, now, &mut lcd_line0_cache, || {
                usb_log.poll()
            });
            #[cfg(not(feature = "usb-log"))]
            let _ = app::lcd_gnss::paint_utc_line_if_changed(&mut lcd, now, &mut lcd_line0_cache, || {});
        }

        #[cfg(feature = "usb-log")]
        {
            let now = platform::timers::timelr_now(&pac.TIMER);
            if platform::timers::time_reached(
                now,
                last_usb_gnss_dump_us.wrapping_add(USB_GNSS_DEBUG_DUMP_PERIOD_US),
            ) {
                last_usb_gnss_dump_us = now;
                write_gnss_state(&mut usb_log);
            }
        }

        let now = platform::timers::timelr_now(&pac.TIMER);
        #[cfg(feature = "usb-log")]
        let _ = app::lcd_gnss::paint_utc_line_if_changed(&mut lcd, now, &mut lcd_line0_cache, || {
            usb_log.poll()
        });
        #[cfg(not(feature = "usb-log"))]
        let _ = app::lcd_gnss::paint_utc_line_if_changed(&mut lcd, now, &mut lcd_line0_cache, || {});

        let now = platform::timers::timelr_now(&pac.TIMER);
        if platform::timers::time_reached(now, next_full_lcd_us) {
            next_full_lcd_us = now.wrapping_add(LCD_GNSS_FULL_REFRESH_PERIOD_US);
            #[cfg(feature = "usb-log")]
            app::lcd_gnss::refresh_lcd_data_rows_with_yield(&mut lcd, now, || usb_log.poll());
            #[cfg(not(feature = "usb-log"))]
            app::lcd_gnss::refresh_lcd_data_rows_with_yield(&mut lcd, now, || {});
        }

        // `wfi()` only wakes on *enabled* NVIC IRQs. We do not unmask
        // `USBCTRL_IRQ`, so USB bus activity does not wake the core — the
        // device would only see `usb_log.poll()` when the timer (or UART)
        // fires. Enumeration needs sub‑millisecond polling; sleeping here
        // breaks it.
        #[cfg(not(feature = "usb-log"))]
        asm::wfi();
    }
}

////////////////////////////////////////////////////////////////////////////////
// INTERRUPT HANDLERS
////////////////////////////////////////////////////////////////////////////////

#[interrupt]
fn UART0_IRQ() {
    let mut pac = unsafe { Peripherals::steal() };
    platform::uart::handle_uart0_rx_irq(&mut pac.UART0);
}

#[interrupt]
fn TIMER_IRQ_0() {
    let mut pac = unsafe { Peripherals::steal() };
    platform::timers::handle_alarm0_multiplex(&mut pac.TIMER);
}

#[interrupt]
fn TIMER_IRQ_1() {
    let mut pac = unsafe { Peripherals::steal() };
    platform::timers::handle_dedicated_alarm1(&mut pac.TIMER);
}

#[interrupt]
fn TIMER_IRQ_2() {
    let mut pac = unsafe { Peripherals::steal() };
    platform::timers::handle_dedicated_alarm2(&mut pac.TIMER);
}

#[interrupt]
fn TIMER_IRQ_3() {
    let mut pac = unsafe { Peripherals::steal() };
    platform::timers::handle_dedicated_alarm3(&mut pac.TIMER);
}

#[cfg(feature = "usb-log")]
#[interrupt]
fn USBCTRL_IRQ() {
    UsbLogger::poll_from_isr();
}
