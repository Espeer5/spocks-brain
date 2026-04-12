//! RP2040 I2C1 master (GP2 SDA / GP3 SCL).
//!
//! Register programming matches [`rp2040_hal::i2c::I2C::new_controller`] and
//! [`rp2040_hal::i2c::controller::write_internal`] — the previous PAC-only driver
//! diverged (e.g. `IC_TX_TL`, `IC_RX_FIFO_FULL_HLD_CTRL`, end-of-transfer waits)
//! and could ACK on the bus without driving the LCD reliably.

use heapless::Vec;

use rp_pico::hal::pac::{I2C1, RESETS};

use crate::platform::constants::{
    I2C1_SCL_HZ, I2C_LCD_ADDR_7BIT, LCD8574_WIRING, PERI_CLOCK_HZ,
};
use crate::platform::resets;

/// Same as `rp2040_hal::i2c::I2C::TX_FIFO_DEPTH`.
const TX_FIFO_DEPTH: u8 = 16;

/// Max entries for [`I2c1Master::scan_bus`] (covers 0x08..=0x77).
pub const I2C_SCAN_CAP: usize = 120;

const SPIN_ENABLE: usize = 50_000;
const SPIN_FIFO: usize = 500_000;
const SPIN_STOP: usize = 2_000_000;

/// Master handle for I2C1 (PAC block only; pins configured separately).
pub struct I2c1Master<'a> {
    i2c: &'a mut I2C1,
    /// Last value written to `IC_TAR` while enabled. Avoids [`setup_7bit`]'s full
    /// controller disable/enable on every 1-byte HD44780 nibble write.
    current_tar: Option<u8>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum I2cError {
    EnableTimeout,
    TransferTimeout,
    NackOrArbitration,
}

impl<'a> I2c1Master<'a> {
    /// Configure I2C1 like `rp2040_hal::i2c::I2C::i2c1` / `new_controller`, then enable the block.
    pub fn new(i2c: &'a mut I2C1, resets_block: &mut RESETS) -> Result<Self, I2cError> {
        resets::clear_i2c1_reset(resets_block);

        i2c.ic_enable().write(|w| w.enable().disabled());
        spin_while_enabled(i2c, false)?;

        let freq = I2C1_SCL_HZ;
        let freq_in = PERI_CLOCK_HZ;
        debug_assert!(freq <= 1_000_000 && freq > 0);

        let period = (freq_in + freq / 2) / freq;
        let lcnt = period * 3 / 5;
        let hcnt = (period - lcnt).max(8);
        let lcnt = lcnt.max(8);
        debug_assert!(hcnt <= 0xffff && lcnt <= 0xffff);

        let sda_tx_hold_count = if freq < 1_000_000 {
            ((freq_in * 3) / 10_000_000) + 1
        } else {
            debug_assert!(freq_in >= 32_000_000);
            ((freq_in * 3) / 25_000_000) + 1
        };
        let sda_tx_hold_count = sda_tx_hold_count.min(lcnt.saturating_sub(2)).max(1);

        i2c.ic_con().modify(|_, w| {
            w.speed().fast();
            w.master_mode().enabled();
            w.ic_slave_disable().slave_disabled();
            w.ic_restart_en().enabled();
            w.tx_empty_ctrl().enabled()
        });

        i2c.ic_tx_tl().write(|w| unsafe { w.tx_tl().bits(0) });
        i2c.ic_rx_tl().write(|w| unsafe { w.rx_tl().bits(0) });

        unsafe {
            i2c.ic_fs_scl_hcnt()
                .write(|w| w.ic_fs_scl_hcnt().bits(hcnt as u16));
            i2c.ic_fs_scl_lcnt()
                .write(|w| w.ic_fs_scl_lcnt().bits(lcnt as u16));
            let ticks = if lcnt < 16 { 1 } else { (lcnt / 16) as u8 };
            i2c.ic_fs_spklen().write(|w| w.ic_fs_spklen().bits(ticks));
            i2c.ic_sda_hold()
                .modify(|_, w| w.ic_sda_tx_hold().bits(sda_tx_hold_count as u16));

            i2c.ic_tx_tl()
                .write(|w| w.tx_tl().bits(TX_FIFO_DEPTH));
            i2c.ic_rx_tl().write(|w| w.rx_tl().bits(0));
            i2c.ic_con()
                .modify(|_, w| w.rx_fifo_full_hld_ctrl().enabled());
        }

        i2c.ic_con()
            .modify(|_, w| w.ic_10bitaddr_master().addr_7bits());

        i2c.ic_intr_mask().write(|w| unsafe { w.bits(0) });

        i2c.ic_enable().write(|w| w.enable().enabled());
        spin_while_enabled(i2c, true)?;

        Ok(Self {
            i2c,
            current_tar: None,
        })
    }

    /// 7-bit address, controller transmit with STOP after the buffer (HAL `write` / `write_internal`).
    pub fn write(&mut self, addr_7bit: u8, bytes: &[u8]) -> Result<(), I2cError> {
        if bytes.is_empty() {
            return Ok(());
        }
        if self.current_tar != Some(addr_7bit) {
            self.setup_7bit(addr_7bit).map_err(|e| {
                self.current_tar = None;
                e
            })?;
            self.current_tar = Some(addr_7bit);
        }

        let mut first_byte = true;
        for (i, &byte) in bytes.iter().enumerate() {
            let last = i + 1 == bytes.len();
            self.wait_tfnf().map_err(|e| {
                self.current_tar = None;
                e
            })?;

            self.i2c.ic_data_cmd().write(|w| {
                if first_byte {
                    first_byte = false;
                }
                w.stop().bit(last);
                unsafe { w.dat().bits(byte) }
            });
        }

        match self.wait_stop_finish() {
            Ok(()) => Ok(()),
            Err(e) => {
                self.current_tar = None;
                Err(e)
            }
        }
    }

    pub fn probe_ack(&mut self, addr_7bit: u8) -> bool {
        self.write(addr_7bit, &[LCD8574_WIRING.probe_idle_byte()])
            .is_ok()
    }

    #[allow(dead_code)]
    pub fn scan_bus(&mut self) -> Vec<u8, I2C_SCAN_CAP> {
        let mut out = Vec::new();
        for addr in 0x08u8..0x78 {
            if self.probe_ack(addr) {
                let _ = out.push(addr);
            }
        }
        out
    }

    #[allow(dead_code)]
    pub fn scan_bus_priority_fast<F: FnMut()>(&mut self, mut hook: F) -> Vec<u8, I2C_SCAN_CAP> {
        const ORDER: &[u8] = &[I2C_LCD_ADDR_7BIT, 0x27, 0x3F, 0x20, 0x21];
        let mut seen = [false; 128];
        let mut out = Vec::new();
        for &addr in ORDER {
            if addr >= 0x78 || seen[addr as usize] {
                continue;
            }
            seen[addr as usize] = true;
            hook();
            if self.probe_ack(addr) {
                let _ = out.push(addr);
            }
        }
        out
    }

    fn setup_7bit(&mut self, addr: u8) -> Result<(), I2cError> {
        self.i2c.ic_enable().write(|w| w.enable().disabled());
        spin_while_enabled(self.i2c, false)?;
        self.i2c
            .ic_con()
            .modify(|_, w| w.ic_10bitaddr_master().addr_7bits());
        self.i2c.ic_tar().write(|w| unsafe {
            w.ic_tar().bits(u16::from(addr))
        });
        self.i2c.ic_enable().write(|w| w.enable().enabled());
        spin_while_enabled(self.i2c, true)?;
        Ok(())
    }

    fn wait_tfnf(&mut self) -> Result<(), I2cError> {
        let mut spins = 0usize;
        loop {
            if self.i2c.ic_raw_intr_stat().read().tx_abrt().is_active() {
                let _ = self.i2c.ic_clr_tx_abrt().read();
                return Err(I2cError::NackOrArbitration);
            }
            if self.i2c.ic_status().read().tfnf().is_full() {
                spins += 1;
                if spins > SPIN_FIFO {
                    return Err(I2cError::TransferTimeout);
                }
                cortex_m::asm::nop();
                continue;
            }
            return Ok(());
        }
    }

    fn wait_stop_finish(&mut self) -> Result<(), I2cError> {
        let mut spins = 0usize;
        loop {
            if self.i2c.ic_raw_intr_stat().read().tx_abrt().is_active() {
                let _ = self.i2c.ic_clr_tx_abrt().read();
                return Err(I2cError::NackOrArbitration);
            }
            if self.i2c.ic_raw_intr_stat().read().stop_det().is_active() {
                let _ = self.i2c.ic_clr_stop_det().read();
                return Ok(());
            }
            spins += 1;
            if spins > SPIN_STOP {
                return Err(I2cError::TransferTimeout);
            }
            cortex_m::asm::nop();
        }
    }
}

fn spin_while_enabled(i2c: &mut I2C1, want_enabled: bool) -> Result<(), I2cError> {
    let mut spins = 0usize;
    loop {
        let en = i2c.ic_enable_status().read().ic_en().is_enabled();
        if en == want_enabled {
            return Ok(());
        }
        spins += 1;
        if spins > SPIN_ENABLE {
            return Err(I2cError::EnableTimeout);
        }
        cortex_m::asm::nop();
    }
}
