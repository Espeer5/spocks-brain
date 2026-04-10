//! Hardware clock control for Spock's brain
//!
//! PAC-level XOSC + PLL + clock mux setup. Used from [`crate::main`]. Every wait
//! loop is bounded so a bad crystal or mux never **silently** freezes the CPU
//! (which looks like “boot pulse then LED dead”).

use core::fmt;

use rp_pico::hal::pac::Peripherals;

////////////////////////////////////////////////////////////////////////////////
// ERRORS
////////////////////////////////////////////////////////////////////////////////

/// Failure while bringing up XOSC / PLLs / glitchless muxes.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum ClockInitError {
    /// XOSC `STABLE` never set (bad crystal, solder, or wrong board).
    XoscNotStable,
    /// `CLK_REF_SELECTED` never showed XOSC (glitchless mux stuck).
    ClkRefNotXosc,
    /// `PLL_SYS` never reported lock.
    PllSysNotLocked,
    /// `PLL_USB` never reported lock.
    PllUsbNotLocked,
    /// `CLK_SYS_SELECTED` never showed aux / PLL path.
    ClkSysNotAux,
}

impl fmt::Debug for ClockInitError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::XoscNotStable => write!(f, "XoscNotStable"),
            Self::ClkRefNotXosc => write!(f, "ClkRefNotXosc"),
            Self::PllSysNotLocked => write!(f, "PllSysNotLocked"),
            Self::PllUsbNotLocked => write!(f, "PllUsbNotLocked"),
            Self::ClkSysNotAux => write!(f, "ClkSysNotAux"),
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
// CONSTANTS
////////////////////////////////////////////////////////////////////////////////

/// `CLK_REF_CTRL.SRC` index for XOSC (`SRC_A::XOSC_CLKSRC = 2`).
const CLK_REF_SEL_XOSC: u32 = 1 << 2;

/// `CLK_SYS_CTRL.SRC` index for the aux mux (`CLKSRC_CLK_SYS_AUX = 1`).
const CLK_SYS_SEL_AUX: u32 = 1 << 1;

/// Max inner-loop iterations for each hardware wait (enough at ROSC post-reset).
const SPIN_XOSC_STABLE: u32 = 5_000_000;
const SPIN_CLK_SELECTED: u32 = 5_000_000;
const SPIN_PLL_LOCK: u32 = 5_000_000;

////////////////////////////////////////////////////////////////////////////////
// SPIN HELPERS
////////////////////////////////////////////////////////////////////////////////

#[inline]
fn spin_until(
    mut done: impl FnMut() -> bool,
    max_iters: u32,
    err: ClockInitError,
) -> Result<(), ClockInitError> {
    for _ in 0..max_iters {
        if done() {
            return Ok(());
        }
        cortex_m::asm::nop();
    }
    Err(err)
}

fn wait_clk_ref_xosc_selected(pac: &Peripherals) -> Result<(), ClockInitError> {
    spin_until(
        || pac.CLOCKS.clk_ref_selected().read().bits() == CLK_REF_SEL_XOSC,
        SPIN_CLK_SELECTED,
        ClockInitError::ClkRefNotXosc,
    )
}

fn wait_clk_sys_aux_selected(pac: &Peripherals) -> Result<(), ClockInitError> {
    spin_until(
        || pac.CLOCKS.clk_sys_selected().read().bits() == CLK_SYS_SEL_AUX,
        SPIN_CLK_SELECTED,
        ClockInitError::ClkSysNotAux,
    )
}

/// Bring up the 12 MHz crystal. Sequence matches `rp2040_hal::xosc` (freq_range →
/// startup → enable); a single raw `ctrl` write or a too-short startup delay can
/// leave STABLE never set, hanging `init` forever.
fn init_xosc_12mhz(pac: &mut Peripherals) -> Result<(), ClockInitError> {
    const CRYSTAL_HZ: u32 = 12_000_000;
    const STARTUP_MULT: u32 = 64;

    pac.XOSC.ctrl().write(|w| {
        w.freq_range()._1_15mhz();
        w
    });

    let startup_delay = (CRYSTAL_HZ / (1000 * 256)) * STARTUP_MULT;
    let startup_delay: u16 = startup_delay.min(u32::from(u16::MAX)) as u16;

    pac.XOSC
        .startup()
        .write(|w| unsafe { w.delay().bits(startup_delay) });

    pac.XOSC.ctrl().write(|w| {
        w.freq_range()._1_15mhz();
        w.enable().enable();
        w
    });

    spin_until(
        || pac.XOSC.status().read().stable().bit_is_set(),
        SPIN_XOSC_STABLE,
        ClockInitError::XoscNotStable,
    )
}

/// Initialize the system clock PLL to 125 MHz
fn init_pll_sys_125mhz(pac: &mut Peripherals) -> Result<(), ClockInitError> {
    pac.PLL_SYS
        .pwr()
        .write(|w| w.pd().set_bit().vcopd().set_bit().postdivpd().set_bit());

    pac.PLL_SYS.cs().write(|w| unsafe { w.refdiv().bits(1) });
    pac.PLL_SYS.fbdiv_int().write(|w| unsafe { w.bits(125) });

    pac.PLL_SYS
        .pwr()
        .write(|w|
            w.pd().clear_bit().vcopd().clear_bit().postdivpd().set_bit());

    spin_until(
        || pac.PLL_SYS.cs().read().lock().bit_is_set(),
        SPIN_PLL_LOCK,
        ClockInitError::PllSysNotLocked,
    )?;

    pac.PLL_SYS
        .prim()
        .write(|w| unsafe { w.postdiv1().bits(6).postdiv2().bits(2) });

    pac.PLL_SYS
        .pwr()
        .write(|w|
            w.pd().clear_bit().vcopd().clear_bit().postdivpd().clear_bit());

    Ok(())
}

/// Initialize the usb clock PLL to 48 MHz
fn init_pll_usb_48mhz(pac: &mut Peripherals) -> Result<(), ClockInitError> {
    pac.PLL_USB
        .pwr()
        .write(|w| w.pd().set_bit().vcopd().set_bit().postdivpd().set_bit());

    pac.PLL_USB.cs().write(|w| unsafe { w.refdiv().bits(1) });
    pac.PLL_USB.fbdiv_int().write(|w| unsafe { w.bits(40) });

    pac.PLL_USB
        .pwr()
        .write(|w|
            w.pd().clear_bit().vcopd().clear_bit().postdivpd().set_bit());

    spin_until(
        || pac.PLL_USB.cs().read().lock().bit_is_set(),
        SPIN_PLL_LOCK,
        ClockInitError::PllUsbNotLocked,
    )?;

    pac.PLL_USB
        .prim()
        .write(|w| unsafe { w.postdiv1().bits(5).postdiv2().bits(2) });

    pac.PLL_USB
        .pwr()
        .write(|w|
            w.pd().clear_bit().vcopd().clear_bit().postdivpd().clear_bit());

    Ok(())
}

////////////////////////////////////////////////////////////////////////////////
// PUB CLOCK INITIALIZATION ROUTINES
////////////////////////////////////////////////////////////////////////////////

/// Initialize PLLs for the external 12 MHz oscillator. Returns [`Err`] if any
/// hardware wait times out instead of blocking forever.
pub fn init_12mhz_xosc_plls(pac: &mut Peripherals) -> Result<(), ClockInitError> {
    init_xosc_12mhz(pac)?;

    pac.CLOCKS
        .clk_ref_ctrl()
        .modify(|_, w| w.src().xosc_clksrc());

    pac.CLOCKS
        .clk_ref_div()
        .write(|w| unsafe { w.bits(0x0100) });

    wait_clk_ref_xosc_selected(pac)?;

    init_pll_sys_125mhz(pac)?;
    init_pll_usb_48mhz(pac)?;

    pac.CLOCKS
        .clk_sys_ctrl()
        .modify(|_, w| w.auxsrc().clksrc_pll_sys().src().clksrc_clk_sys_aux());

    pac.CLOCKS
        .clk_sys_div()
        .write(|w| unsafe { w.bits(0x0100) });

    wait_clk_sys_aux_selected(pac)?;

    pac.CLOCKS.clk_usb_ctrl().modify(|_, w| {
        w.auxsrc().clksrc_pll_usb();
        w.enable().set_bit()
    });

    pac.CLOCKS
        .clk_usb_div()
        .write(|w| unsafe { w.bits(0x0100) });

    pac.CLOCKS.clk_peri_ctrl().modify(|_, w| {
        w.auxsrc().clk_sys();
        w.enable().set_bit()
    });

    Ok(())
}
