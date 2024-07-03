#![no_std]

use consts::{PCDErrorCode, Uid, UidSize};
use embedded_hal::digital::OutputPin;

pub mod consts;
pub mod debug;
pub mod mifare;
pub mod pcd;
pub mod picc;

pub struct MFRC522<S, C>
where
    S: embedded_hal_async::spi::SpiBus,
    C: OutputPin,
{
    spi: S,
    cs: C,
    read_buff: [u8; 1],

    get_current_time: fn() -> u64,
}

impl<S, C> MFRC522<S, C>
where
    S: embedded_hal_async::spi::SpiBus,
    C: OutputPin,
{
    pub fn new(spi: S, cs: C, get_current_time: fn() -> u64) -> Self {
        Self {
            spi,
            cs,
            read_buff: [0],
            get_current_time,
        }
    }

    pub async fn get_card(&mut self, size: UidSize) -> Result<Uid, PCDErrorCode> {
        let mut uid = Uid {
            size: size.to_byte(),
            sak: 0,
            uid_bytes: [0; 10],
        };

        self.picc_select(&mut uid, 0).await?;
        Ok(uid)
    }

    pub async fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), PCDErrorCode> {
        self.cs.set_low().map_err(|_| PCDErrorCode::Unknown)?;
        self.spi_transfer(&[reg << 1]).await?;
        self.spi_transfer(&[val]).await?;
        self.cs.set_high().map_err(|_| PCDErrorCode::Unknown)?;

        Ok(())
    }

    pub async fn write_reg_buff(
        &mut self,
        reg: u8,
        count: usize,
        values: &[u8],
    ) -> Result<(), PCDErrorCode> {
        self.cs.set_low().map_err(|_| PCDErrorCode::Unknown)?;
        self.spi_transfer(&[reg << 1]).await?;

        for i in 0..count {
            self.spi_transfer(&[values[i]]).await?;
        }

        self.cs.set_high().map_err(|_| PCDErrorCode::Unknown)?;
        Ok(())
    }

    pub async fn read_reg(&mut self, reg: u8) -> Result<u8, PCDErrorCode> {
        self.cs.set_low().map_err(|_| PCDErrorCode::Unknown)?;
        self.spi_transfer(&[(reg << 1) | 0x80]).await?;
        self.spi_transfer(&[0]).await?;
        self.cs.set_high().map_err(|_| PCDErrorCode::Unknown)?;

        Ok(self.read_buff[0])
    }

    pub async fn read_reg_buff(
        &mut self,
        reg: u8,
        count: usize,
        output_buff: &mut [u8],
        rx_align: u8,
    ) -> Result<(), PCDErrorCode> {
        if count == 0 {
            return Ok(());
        }

        let addr = 0x80 | (reg << 1);
        let mut index = 0;

        self.cs.set_low().map_err(|_| PCDErrorCode::Unknown)?;
        self.spi_transfer(&[addr]).await?;

        if rx_align > 0 {
            let mask = (0xFF << rx_align) & 0xFF;
            self.spi_transfer(&[addr]).await?;

            output_buff[0] = (output_buff[0] & !mask) | (self.read_buff[0] & mask);
            index += 1;
        }

        while index < count - 1 {
            self.spi_transfer(&[addr]).await?;
            output_buff[index] = self.read_buff[0];
            index += 1;
        }

        self.spi_transfer(&[0]).await?;
        output_buff[index] = self.read_buff[0];

        self.cs.set_high().map_err(|_| PCDErrorCode::Unknown)?;
        Ok(())
    }

    async fn spi_transfer(&mut self, data: &[u8]) -> Result<(), PCDErrorCode> {
        self.spi
            .transfer(&mut self.read_buff, data)
            .await
            .map_err(|_| PCDErrorCode::Unknown)?;

        Ok(())
    }
}

#[inline(always)]
pub fn tif<T>(expr: bool, true_val: T, false_val: T) -> T {
    if expr {
        true_val
    } else {
        false_val
    }
}
