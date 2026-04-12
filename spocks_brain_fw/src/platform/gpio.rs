//! GPIO control for Spock's brain

use rp_pico::hal::pac::{IO_BANK0, PADS_BANK0, Peripherals, SIO, TIMER};

use crate::platform::constants::{
    I2C1_SCL_GPIO, I2C1_SDA_GPIO, LED_BLINK_INITIAL_DELAY_US, LED_BLINK_PERIOD_US,
};
use crate::platform::timers::{self, SoftMode, SoftTimerId};

/// Raspberry Pi Pico onboard LED (GP25), SIO output, active high.
pub const ONBOARD_LED_GPIO: usize = 25;

////////////////////////////////////////////////////////////////////////////////
// PUB GPIO INITIALIZATION ROUTINES
////////////////////////////////////////////////////////////////////////////////

/// Initialize GP0 and GP1 for UART0 (GNSS): GP0 = TX, GP1 = RX.
/// GNSS TX connects to RP2040 RX (GP1); RP2040 TX (GP0) connects to GNSS RX.
/// Baud is [`crate::platform::constants::UART0_BAUD_RATE`]
pub fn init_gpios(io: &mut IO_BANK0, pads: &mut PADS_BANK0) {
    // Pad settings: GP1 (RX) has pull-up for idle-high; GP0 (TX) no pull. PAC:
    // PADS_BANK0.gpio(n) with ie(), pue(), pde(), od().
    pads.gpio(0).modify(|_, w| {
        w.pue().clear_bit();
        w.pde().clear_bit();
        w.od().clear_bit();
        w.ie().set_bit()
    });
    // RX must idle high (UART mark). Internal pull-up avoids a floating pin when
    // the GNSS is disconnected, otherwise noise triggers endless RX IRQs and
    // USB never gets polled
    pads.gpio(1).modify(|_, w| {
        w.pue().set_bit();
        w.pde().clear_bit();
        w.od().clear_bit();
        w.ie().set_bit()
    });

    // IO function selection: GP0 and GP1 as UART0
    io.gpio(0).gpio_ctrl().modify(|_, w| w.funcsel().uart());
    io.gpio(1).gpio_ctrl().modify(|_, w| w.funcsel().uart());
}

/// GP2 = I2C1 SDA, GP3 = I2C1 SCL: open-drain I2C with internal pull-ups.
pub fn init_i2c1_pins(io: &mut IO_BANK0, pads: &mut PADS_BANK0) {
    for &n in &[I2C1_SDA_GPIO, I2C1_SCL_GPIO] {
        pads.gpio(n).modify(|_, w| {
            w.pue().set_bit();
            w.pde().clear_bit();
            w.od().clear_bit();
            w.ie().set_bit()
        });
        io.gpio(n).gpio_ctrl().modify(|_, w| w.funcsel().i2c());
    }
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

/// Callback for [`schedule_onboard_led_demo`] (soft timer on ALARM0).
pub fn onboard_led_demo_soft_timer_cb(_id: SoftTimerId) {
    let mut pac = unsafe { Peripherals::steal() };
    toggle_onboard_led(&mut pac.SIO);
}

/// Start the onboard LED blink using the multiplexed soft timer (see `TIMER_IRQ_0`).
pub fn schedule_onboard_led_demo(timer: &mut TIMER) -> Result<SoftTimerId, ()> {
    timers::schedule_soft(
        timer,
        LED_BLINK_INITIAL_DELAY_US,
        SoftMode::Periodic {
            period_us: LED_BLINK_PERIOD_US,
        },
        onboard_led_demo_soft_timer_cb,
    )
}
