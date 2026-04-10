#![no_std]
#![no_main]

mod platform;

use panic_halt as _;
use rp_pico::hal::pac::interrupt;
use rp_pico::{entry, hal::pac::Peripherals};

use cortex_m::asm;

use platform::constants::{LED_BLINK_INITIAL_DELAY_US, LED_BLINK_PERIOD_US};
use platform::timers::{self, SoftMode};

/// Toggles the onboard LED from a soft-timer callback
fn led_blink_soft_timer(_id: timers::SoftTimerId) {
    let mut pac = unsafe { Peripherals::steal() };
    platform::gpio::toggle_onboard_led(&mut pac.SIO);
}

#[entry]
fn main() -> ! {
    let mut pac = Peripherals::take().unwrap();

    platform::resets::clear_gpio_subsystem_resets(&mut pac);

    platform::gpio::init_onboard_led(&mut pac.IO_BANK0, &mut pac.PADS_BANK0, &mut pac.SIO);

    platform::resets::clear_pll_resets(&mut pac);
    platform::resets::clear_timer_reset(&mut pac);

    platform::clocks::init_12mhz_xosc_plls(&mut pac).unwrap();

    platform::resets::clear_uart0_reset(&mut pac);

    platform::init(
        &mut pac.IO_BANK0,
        &mut pac.PADS_BANK0,
        &mut pac.SIO,
        &mut pac.UART0,
    );

    platform::timers::init_timer_hardware(&mut pac.TIMER);
    platform::timers::enable_timer_interrupts();

    timers::schedule_soft(
        &mut pac.TIMER,
        LED_BLINK_INITIAL_DELAY_US,
        SoftMode::Periodic {
            period_us: LED_BLINK_PERIOD_US,
        },
        led_blink_soft_timer,
    )
    .unwrap();

    loop {
        asm::wfi();
    }
}

////////////////////////////////////////////////////////////////////////////////
// TIMER INTERRUPT HANDLERS
////////////////////////////////////////////////////////////////////////////////

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
