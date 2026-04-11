#![no_std]
#![no_main]

mod app;
mod platform;

#[cfg(feature = "usb-log")]
mod debug;

use core::mem;

use panic_halt as _;
use rp_pico::hal::clocks::ClocksManager;
use rp_pico::hal::pac::interrupt;
use rp_pico::{entry, hal::pac::Peripherals};

#[cfg(not(feature = "usb-log"))]
use cortex_m::asm;

use app::events::{self, NmeaLineAssembler};
#[cfg(feature = "usb-log")]
use debug::usb_log::UsbLogger;

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

    // `UsbBus` from rp2040-hal expects an `UsbClock` handle; hardware is
    // already configured above. Replace `pac.CLOCKS` so `ClocksManager` owns
    // the block; leftover field is zeroed and unused.
    let clocks_block = mem::replace(&mut pac.CLOCKS, unsafe { mem::zeroed() });
    let clocks_mgr = ClocksManager::new(clocks_block);

    platform::resets::clear_uart0_reset(&mut pac.RESETS);

    platform::init(
        &mut pac.IO_BANK0,
        &mut pac.PADS_BANK0,
        &mut pac.SIO,
        &mut pac.UART0,
    );

    #[cfg(feature = "usb-log")]
    let mut usb_log = UsbLogger::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        &mut pac.RESETS,
        clocks_mgr.usb_clock,
    );

    #[cfg(not(feature = "usb-log"))]
    let _clocks_mgr = clocks_mgr;

    // Give USB a poll budget before UART RX IRQ so enumeration is not starved.
    #[cfg(feature = "usb-log")]
    {
        for _ in 0..12_000 {
            usb_log.poll();
        }
    }

    // Timer IRQs preempt UART0 RX (see `timers::configure_irq_priorities`).
    // Onboard LED demo uses ALARM0 soft timers so blink rate reflects IRQ
    // scheduling.
    platform::timers::configure_irq_priorities();
    platform::timers::init_timer_hardware(&mut pac.TIMER);
    platform::timers::enable_timer_interrupts();

    let _led_demo =
        platform::gpio::schedule_onboard_led_demo(&mut pac.TIMER).expect("LED demo failed");

    platform::uart::enable_uart0_interrupt();

    let mut nmea = NmeaLineAssembler::new();

    loop {
        #[cfg(feature = "usb-log")]
        usb_log.poll();

        nmea.feed_bytes();

        while let Some(ev) = events::try_pop_event() {
            #[cfg(feature = "usb-log")]
            events::dispatch_event(ev, &mut usb_log);
            #[cfg(not(feature = "usb-log"))]
            events::dispatch_event(ev);
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
