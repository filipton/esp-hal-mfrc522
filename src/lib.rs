#![no_std]

use consts::{PCDErrorCode, Uid, UidSize};

pub mod consts;
pub mod debug;
pub mod drivers;
pub mod mifare;
pub mod pcd;
pub mod picc;

pub trait MfrcDriver {
    fn write_reg(&mut self, reg: u8, val: u8) -> impl Future<Output = Result<(), PCDErrorCode>>;
    fn write_reg_buff(
        &mut self,
        reg: u8,
        count: usize,
        values: &[u8],
    ) -> impl Future<Output = Result<(), PCDErrorCode>>;
    fn read_reg(&mut self, reg: u8) -> impl Future<Output = Result<u8, PCDErrorCode>>;
    fn read_reg_buff(
        &mut self,
        reg: u8,
        count: usize,
        output_buff: &mut [u8],
        rx_align: u8,
    ) -> impl Future<Output = Result<(), PCDErrorCode>>;
}

pub struct MFRC522<D>
where
    D: MfrcDriver,
{
    driver: D,
    get_current_time: fn() -> u64,
}

impl<D> MFRC522<D>
where
    D: MfrcDriver,
{
    #[cfg(not(feature = "embassy-time"))]
    pub fn new(driver: D, get_current_time: fn() -> u64) -> Self {
        Self {
            driver,
            get_current_time,
        }
    }

    #[cfg(feature = "embassy-time")]
    pub fn new(driver: D) -> Self {
        Self {
            driver,
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
        self.driver.write_reg(reg, val).await
    }

    pub async fn write_reg_buff(
        &mut self,
        reg: u8,
        count: usize,
        values: &[u8],
    ) -> Result<(), PCDErrorCode> {
        self.driver.write_reg_buff(reg, count, values).await
    }

    pub async fn read_reg(&mut self, reg: u8) -> Result<u8, PCDErrorCode> {
        self.driver.read_reg(reg).await
    }

    pub async fn read_reg_buff(
        &mut self,
        reg: u8,
        count: usize,
        output_buff: &mut [u8],
        rx_align: u8,
    ) -> Result<(), PCDErrorCode> {
        self.driver
            .read_reg_buff(reg, count, output_buff, rx_align)
            .await
    }
}

#[inline(always)]
pub fn tif<T>(expr: bool, true_val: T, false_val: T) -> T {
    if expr { true_val } else { false_val }
}
