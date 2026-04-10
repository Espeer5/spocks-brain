#![no_std]
#![no_main]

mod platform;

use defmt_rtt as _;
use rp_pico::{entry, hal::pac::Peripherals};
use panic_probe as _;

#[entry]
fn main() -> ! {
    let mut pac = Peripherals::take().unwrap();

    platform::init(&mut pac);

    loop {
        cortex_m::asm::wfi();
    }
}