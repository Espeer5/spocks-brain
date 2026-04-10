//! Hardware clock control for Spock's brain

use rp_pico::hal::pac::Peripherals;

////////////////////////////////////////////////////////////////////////////////
/// CONSTANTS
////////////////////////////////////////////////////////////////////////////////

/// Magic value for XOSC CTRL ENABLE field (bits 23:12). Writing this enables
/// the crystal oscillator.
const XOSC_CTRL_ENABLE_MAGIC: u32 = 0xfab;

/// Offset for XOSC CTRL ENABLE field (bits 23:12)
const XOSC_CTRL_ENABLE_OFFSET: u32 = 12;

/// XOSC CTRL FREQ_RANGE for 1–15 MHz crystal (bits 11:0 = 0xAA0 per datasheet).
const XOSC_CTRL_FREQ_RANGE_1_15MHZ: u32 = 0xAA0;

/// For 12 MHz XOSC, delay value (bits 13:0) for 1 ms startup.
/// One unit = 256 * xtal_period ⇒ 1 ms = 0.001 * 12e6 / 256 ≈ 46.875 → 47.
const XOSC_STARTUP_DELAY_1MS_12MHZ: u16 = 47;

/// XOSC STATUS register bit for stable oscillator
const XOSC_STATUS_STABLE: u32 = 1 << 31;

////////////////////////////////////////////////////////////////////////////////
/// CLOCK INITIALIZATION HELPERS
////////////////////////////////////////////////////////////////////////////////

/// Initialize the external 12 MHz crystal oscillator by directly writing the 
/// ENABLE field of the XOSC CTRL register. Bits 23:12 must be 0xfab to enable.
/// STARTUP delay is set for 1 ms before the oscillator is enabled.
fn init_xosc_12mhz(pac: &mut Peripherals) {
    pac.XOSC
        .startup()
        .write(|w| unsafe { w.delay().bits(XOSC_STARTUP_DELAY_1MS_12MHZ) });

    // Enable XOSC and set FREQ_RANGE for 1–15 MHz (required for 12 MHz crystal).
    pac.XOSC.ctrl().write(|w| unsafe {
        w.bits((XOSC_CTRL_ENABLE_MAGIC << XOSC_CTRL_ENABLE_OFFSET)
            | XOSC_CTRL_FREQ_RANGE_1_15MHZ)
    });

    // Wait until STATUS indicates oscillator is running and stable (bit 31
    // STABLE).
    while pac.XOSC.status().read().bits() & XOSC_STATUS_STABLE == 0 {}
}

/// Initialize the system clock PLL to 125 MHz
fn init_pll_sys_125mhz(pac: &mut Peripherals) {
    // To drive the system clock at 125 MHz, we set the following parameters:
    // - refdiv   = 1   -- input clock is 12 MHz
    // - fbdiv    = 125 -- FREF * FBDIV = 12 MHz * 125 = 1500 MHz
    // - postdiv1 = 6   -- 1500 MHz / 6 = 250 MHz
    // - postdiv2 = 2   -- 250 MHz / 2  = 125 MHz

    // Turn off main PLL power, VCO power, and post-divider power
    pac.PLL_SYS
        .pwr()
        .write(|w| w.pd().set_bit().vcopd().set_bit().postdivpd().set_bit());

    // Program reference clock divider (bits 5:0 of CS register) to 1
    pac.PLL_SYS.cs().write(|w| unsafe { w.refdiv().bits(1) });

    // Program feedback divider (bits 11:0 of FBDIV_INT register) to 125
    pac.PLL_SYS.fbdiv_int().write(|w| unsafe { w.bits(125) });

    // Turn ON PLL main power and VCO power; keep post-divider off until PRIM is
    // programmed.
    pac.PLL_SYS
        .pwr()
        .write(|w|
            w.pd().clear_bit().vcopd().clear_bit().postdivpd().set_bit());

    // Wait for PLL to lock (bit 31 of CS register)
    while pac.PLL_SYS.cs().read().bits() & (1 << 31) == 0 {}

    // Program the 2 post-dividers (must be done while post-divider is powered
    // down).
    pac.PLL_SYS
        .prim()
        .write(|w| unsafe { w.postdiv1().bits(6).postdiv2().bits(2) });

    // Turn ON power to the post-dividers
    pac.PLL_SYS
        .pwr()
        .write(|w|
            w.pd().clear_bit().vcopd().clear_bit().postdivpd().clear_bit());
}

/// Initialize the usb clock PLL to 48 MHz
fn init_pll_usb_48mhz(pac: &mut Peripherals) {
    // To drive the usb clock at 48 MHz, we set the following parameters:
    // - refdiv   = 1   -- input clock is 12 MHz
    // - fbdiv    = 40  -- FREF * FBDIV = 12 MHz * 40 = 480 MHz
    // - postdiv1 = 5   -- 480 MHz / 5 = 96 MHz
    // - postdiv2 = 2   -- 96 MHz / 2 = 48 MHz

    // Turn off main PLL power, VCO power, and post-divider power (one write so
    // all bits stick).
    pac.PLL_USB
        .pwr()
        .write(|w| w.pd().set_bit().vcopd().set_bit().postdivpd().set_bit());

    // Program reference clock divider (bits 5:0 of CS register) to 1
    pac.PLL_USB.cs().write(|w| unsafe { w.refdiv().bits(1) });

    // Program feedback divider (bits 11:0 of FBDIV_INT register) to 40
    pac.PLL_USB.fbdiv_int().write(|w| unsafe { w.bits(40) });

    // Turn ON PLL main power and VCO power; keep post-divider off until PRIM is
    // programmed.
    pac.PLL_USB
        .pwr()
        .write(|w|
            w.pd().clear_bit().vcopd().clear_bit().postdivpd().set_bit());

    // Wait for PLL to lock (bit 31 of CS register)
    while pac.PLL_USB.cs().read().bits() & (1 << 31) == 0 {}

    // Program the 2 post-dividers (must be done while post-divider is powered
    // down).
    pac.PLL_USB
        .prim()
        .write(|w| unsafe { w.postdiv1().bits(5).postdiv2().bits(2) });

    // Turn ON power to the post-dividers
    pac.PLL_USB
        .pwr()
        .write(|w|
            w.pd().clear_bit().vcopd().clear_bit().postdivpd().clear_bit());
}


////////////////////////////////////////////////////////////////////////////////
/// PUB CLOCK INITIALIZATION ROUTINES
////////////////////////////////////////////////////////////////////////////////

/// Initialize PLLS for the external 12 MHz oscillator
pub fn init_12mhz_xosc_plls(pac: &mut Peripherals) {
    // Bring up the external 12 MHz oscillator
    init_xosc_12mhz(pac);

    // CLK_REF_CTRL bits 1:0 = 0x2 select XOSC glitchlessly.
    pac.CLOCKS
        .clk_ref_ctrl()
        .modify(|_, w| w.src().xosc_clksrc());

    // No divider needed for XOSC, so set CLK_REF_DIV to 1.
    pac.CLOCKS
        .clk_ref_div()
        .write(|w| unsafe { w.bits(1) });

    // Lock the sys reference PLL to 125 MHz
    init_pll_sys_125mhz(pac);

    // Lock the usb reference PLL to 48 MHz
    init_pll_usb_48mhz(pac);

    // Set the system clock to the PLL_SYS output. SRC selects ref vs aux;
    // AUXSRC selects which aux source (PLL_SYS). Glitchless mux uses aux when
    // SRC=1.
    pac.CLOCKS
        .clk_sys_ctrl()
        .modify(|_, w| w.auxsrc().clksrc_pll_sys().src().clksrc_clk_sys_aux());

    // Ensure system clock divider is 1
    pac.CLOCKS
        .clk_sys_div()
        .write(|w| unsafe { w.bits(1) });

    // Set the usb clock to the PLL_USB output (CLK_USB has only AUXSRC, no
    // SRC).
    pac.CLOCKS
        .clk_usb_ctrl()
        .modify(|_, w| w.auxsrc().clksrc_pll_usb());

    // Ensure usb clock divider is 1
    pac.CLOCKS
        .clk_usb_div()
        .write(|w| unsafe { w.bits(1) });

    // Set peripheral clock to follow clk_sys (125 MHz). Typical RP2040
    // reference design uses clk_peri = clk_sys so peripherals run at system
    // clock.
    pac.CLOCKS
        .clk_peri_ctrl()
        .modify(|_, w| w.auxsrc().clk_sys());
}
