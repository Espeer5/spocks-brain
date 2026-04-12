//! HD44780 20×4 in 4-bit mode via a PCF8574 I²C backpack.
//!
//! Pin mapping is selected by [`crate::platform::constants::LCD8574_WIRING`].

use crate::platform::constants::{I2C_LCD_ADDR_7BIT, LCD8574_WIRING, SYS_CLOCK_HZ};
use crate::platform::i2c::{I2c1Master, I2cError};

/// DDRAM addresses for 20×4 with line starts (with bit 7 set for command).
const LINE_DD_RAM: [u8; 4] = [0x80, 0xC0, 0x94, 0xD4];

/// HD44780 + PCF8574 on I2C1 (GP2/GP3).
pub struct LcdHd44780I2c<'a> {
    i2c: &'a mut I2c1Master<'a>,
    addr: u8,
}

impl<'a> LcdHd44780I2c<'a> {
    #[allow(dead_code)]
    pub fn new(i2c: &'a mut I2c1Master<'a>) -> Self {
        Self::new_with_addr(i2c, I2C_LCD_ADDR_7BIT)
    }

    pub fn new_with_addr(i2c: &'a mut I2c1Master<'a>, addr_7bit: u8) -> Self {
        Self {
            i2c,
            addr: addr_7bit,
        }
    }

    #[cfg(not(feature = "usb-log"))]
    pub fn init(&mut self) -> Result<(), I2cError> {
        self.init_with_yield(|| {})
    }

    pub fn init_with_yield<F: FnMut()>(&mut self, mut yield_fn: F) -> Result<(), I2cError> {
        delay_ms_yield(50, &mut yield_fn);

        self.write_nibble(false, 0x03)?;
        delay_ms_yield(5, &mut yield_fn);
        self.write_nibble(false, 0x03)?;
        delay_us(200);
        self.write_nibble(false, 0x03)?;
        delay_us(200);
        self.write_nibble(false, 0x02)?;
        delay_us(200);

        self.write_byte(false, 0x28)?;
        self.write_byte(false, 0x0C)?;
        self.write_byte(false, 0x06)?;
        self.write_byte(false, 0x01)?;
        delay_ms_yield(2, &mut yield_fn);
        Ok(())
    }

    #[allow(dead_code)]
    pub fn clear(&mut self) -> Result<(), I2cError> {
        self.write_byte(false, 0x01)?;
        delay_ms(2);
        Ok(())
    }

    /// Row 0..3, ASCII padded/truncated to 20 columns.
    pub fn write_line(&mut self, row: u8, text: &[u8]) -> Result<(), I2cError> {
        let row = (row as usize).min(3);
        self.write_byte(false, LINE_DD_RAM[row])?;
        for i in 0..20 {
            let c = text.get(i).copied().unwrap_or(b' ');
            self.write_byte(true, c)?;
        }
        Ok(())
    }

    fn write_raw(&mut self, port: u8) -> Result<(), I2cError> {
        self.i2c.write(self.addr, &[port])
    }

    /// `port` must have **E low** (see [`LCD8574_WIRING::e_mask`]).
    fn pulse_enable(&mut self, port: u8) -> Result<(), I2cError> {
        let e = LCD8574_WIRING.e_mask();
        self.write_raw(port | e)?;
        delay_us(2);
        self.write_raw(port & !e)?;
        delay_us(50);
        Ok(())
    }

    fn write_nibble(&mut self, rs: bool, nibble: u8) -> Result<(), I2cError> {
        let base = LCD8574_WIRING.pack_nibble(rs, nibble);
        self.pulse_enable(base)
    }

    fn write_byte(&mut self, rs: bool, byte: u8) -> Result<(), I2cError> {
        self.write_nibble(rs, byte >> 4)?;
        self.write_nibble(rs, byte & 0x0F)
    }
}

fn delay_us(us: u32) {
    let cycles = (us as u64).saturating_mul(u64::from(SYS_CLOCK_HZ / 1_000_000));
    for _ in 0..cycles {
        cortex_m::asm::nop();
    }
}

fn delay_ms(ms: u32) {
    for _ in 0..ms {
        delay_us(1000);
    }
}

fn delay_ms_yield<F: FnMut()>(ms: u32, yield_fn: &mut F) {
    for _ in 0..ms {
        yield_fn();
        delay_us(1000);
    }
}
