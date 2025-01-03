#![no_std]

use consts::{PCDErrorCode, Uid, UidSize};
use embedded_hal_async::spi::Operation;

pub mod consts;
pub mod debug;
pub mod mifare;
pub mod pcd;
pub mod picc;

pub struct MFRC522<S>
where
    S: embedded_hal_async::spi::SpiDevice,
{
    spi: S,
    get_current_time: fn() -> u64,
}

impl<S> MFRC522<S>
where
    S: embedded_hal_async::spi::SpiDevice,
{
    #[cfg(not(feature = "embassy-time"))]
    pub fn new(spi: S, get_current_time: fn() -> u64) -> Self {
        Self {
            spi,
            get_current_time,
        }
    }

    #[cfg(feature = "embassy-time")]
    pub fn new(spi: S) -> Self {
        Self {
            spi,
            get_current_time: || embassy_time::Instant::now().as_micros(),
        }
    }

    #[cfg(not(feature = "embassy-time"))]
    pub async fn sleep(&self, time_ms: u64) {
        let start_time = (self.get_current_time)(); // microseconds
        while (self.get_current_time)() - start_time < time_ms * 1_000 {}
    }

    #[cfg(feature = "embassy-time")]
    pub async fn sleep(&self, time_ms: u64) {
        embassy_time::Timer::after_millis(time_ms).await;
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
        self.spi
            .transaction(&mut [Operation::Write(&[reg << 1, val])])
            .await?;

        Ok(())
    }

    pub async fn write_reg_buff(
        &mut self,
        reg: u8,
        count: usize,
        values: &[u8],
    ) -> Result<(), PCDErrorCode> {
        self.spi
            .transaction(&mut [
                Operation::Write(&[reg << 1]),
                Operation::Write(&values[..count]),
            ])
            .await?;

        Ok(())
    }

    pub async fn read_reg(&mut self, reg: u8) -> Result<u8, PCDErrorCode> {
        let mut read = [0; 1];
        self.spi
            .transaction(&mut [
                Operation::Write(&[(reg << 1) | 0x80]),
                Operation::Transfer(&mut read, &[0]),
            ])
            .await?;

        Ok(read[0])
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
        let first_out_byte = output_buff[0];
        output_buff[..count - 1].fill(addr);
        output_buff[count - 1] = 0;

        self.spi
            .transaction(&mut [
                Operation::Write(&[addr]),
                Operation::TransferInPlace(&mut output_buff[..count]),
            ])
            .await?;

        if rx_align > 0 {
            let mask = (0xFF << rx_align) & 0xFF;
            output_buff[0] = (first_out_byte & !mask) | (output_buff[0] & mask);
        }

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
