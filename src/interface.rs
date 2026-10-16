//! Bus interfaces (SPI / I2C) the driver talks to the MFRC522 through.

use crate::Error;
use embedded_hal_async::{i2c, spi};

/// Default 7-bit I2C address of MFRC522 modules (`0x28`).
pub const DEFAULT_I2C_ADDRESS: u8 = 0x28;

/// Register access to the MFRC522. Implemented for SPI and I2C; implement it yourself for other
/// buses (e.g. UART) or for tests.
#[allow(async_fn_in_trait)]
pub trait Interface {
    /// Writes one register.
    async fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), Error>;
    /// Writes all of `values` to one register (used for the FIFO).
    async fn write_regs(&mut self, reg: u8, values: &[u8]) -> Result<(), Error>;
    /// Reads one register.
    async fn read_reg(&mut self, reg: u8) -> Result<u8, Error>;
    /// Fills `out` from one register (used for the FIFO). With `rx_align > 0` only the bits from
    /// position `rx_align` upwards of `out[0]` are replaced (bit-oriented anticollision frames).
    async fn read_regs(&mut self, reg: u8, out: &mut [u8], rx_align: u8) -> Result<(), Error>;
}

/// Keeps the low `rx_align` bits of the old `out[0]` (see [`Interface::read_regs`]).
fn merge_rx_align(old_first: u8, out: &mut [u8], rx_align: u8) {
    if let Some(first) = out.first_mut()
        && rx_align > 0
    {
        let mask = 0xFFu8 << (rx_align & 0x07);
        *first = (old_first & !mask) | (*first & mask);
    }
}

/// MFRC522 on an SPI bus (mode 0, ≤ 10 MHz).
pub struct SpiInterface<S> {
    spi: S,
}

impl<S: spi::SpiDevice> SpiInterface<S> {
    pub fn new(spi: S) -> Self {
        Self { spi }
    }

    /// Returns the SPI device.
    pub fn release(self) -> S {
        self.spi
    }
}

fn spi_err<E: spi::Error>(e: E) -> Error {
    Error::Spi(e.kind())
}

impl<S: spi::SpiDevice> Interface for SpiInterface<S> {
    async fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), Error> {
        self.spi
            .transaction(&mut [spi::Operation::Write(&[reg << 1, val])])
            .await
            .map_err(spi_err)
    }

    async fn write_regs(&mut self, reg: u8, values: &[u8]) -> Result<(), Error> {
        self.spi
            .transaction(&mut [
                spi::Operation::Write(&[reg << 1]),
                spi::Operation::Write(values),
            ])
            .await
            .map_err(spi_err)
    }

    async fn read_reg(&mut self, reg: u8) -> Result<u8, Error> {
        let mut read = [0; 1];
        self.spi
            .transaction(&mut [
                spi::Operation::Write(&[(reg << 1) | 0x80]),
                spi::Operation::Transfer(&mut read, &[0]),
            ])
            .await
            .map_err(spi_err)?;

        Ok(read[0])
    }

    async fn read_regs(&mut self, reg: u8, out: &mut [u8], rx_align: u8) -> Result<(), Error> {
        let Some((last, rest)) = out.split_last_mut() else {
            return Ok(());
        };

        // every clocked-out byte is the next read address, the final one ends the read
        let addr = 0x80 | (reg << 1);
        let old_first = rest.first().copied().unwrap_or(*last);
        rest.fill(addr);
        *last = 0;

        self.spi
            .transaction(&mut [
                spi::Operation::Write(&[addr]),
                spi::Operation::TransferInPlace(out),
            ])
            .await
            .map_err(spi_err)?;

        merge_rx_align(old_first, out, rx_align);
        Ok(())
    }
}

/// MFRC522 on an I2C bus.
pub struct I2cInterface<I> {
    i2c: I,
    address: u8,
}

impl<I: i2c::I2c> I2cInterface<I> {
    /// `address` is usually [`DEFAULT_I2C_ADDRESS`].
    pub fn new(i2c: I, address: u8) -> Self {
        Self { i2c, address }
    }

    /// Returns the I2C bus.
    pub fn release(self) -> I {
        self.i2c
    }
}

fn i2c_err<E: i2c::Error>(e: E) -> Error {
    Error::I2c(e.kind())
}

impl<I: i2c::I2c> Interface for I2cInterface<I> {
    async fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), Error> {
        self.i2c
            .write(self.address, &[reg, val])
            .await
            .map_err(i2c_err)
    }

    async fn write_regs(&mut self, reg: u8, values: &[u8]) -> Result<(), Error> {
        self.i2c
            .transaction(
                self.address,
                &mut [i2c::Operation::Write(&[reg]), i2c::Operation::Write(values)],
            )
            .await
            .map_err(i2c_err)
    }

    async fn read_reg(&mut self, reg: u8) -> Result<u8, Error> {
        let mut read = [0; 1];
        self.i2c
            .write_read(self.address, &[reg], &mut read)
            .await
            .map_err(i2c_err)?;

        Ok(read[0])
    }

    async fn read_regs(&mut self, reg: u8, out: &mut [u8], rx_align: u8) -> Result<(), Error> {
        let Some(&old_first) = out.first() else {
            return Ok(());
        };

        self.i2c
            .write_read(self.address, &[reg], out)
            .await
            .map_err(i2c_err)?;

        merge_rx_align(old_first, out, rx_align);
        Ok(())
    }
}
