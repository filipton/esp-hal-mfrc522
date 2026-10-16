#![doc = include_str!("../README.md")]
#![cfg_attr(not(test), no_std)]

mod consts;
mod debug;
pub mod interface;
mod mifare;
mod pcd;
mod picc;
mod types;

#[cfg(test)]
mod tests;

pub use interface::{DEFAULT_I2C_ADDRESS, I2cInterface, Interface, SpiInterface};
pub use mifare::access_bits;
pub use types::{AntennaGain, CardType, Error, Key, KeyType, Uid, Version};

/// MFRC522 driver over an [`Interface`].
pub struct Mfrc522<I> {
    interface: I,
    /// Monotonic clock in microseconds, used for timeouts.
    now_us: fn() -> u64,
}

impl<I: Interface> Mfrc522<I> {
    /// Creates the driver with your own microsecond clock (used for timeouts and, without the
    /// `embassy-time` feature, busy-wait delays).
    pub fn with_clock(interface: I, now_us: fn() -> u64) -> Self {
        Self { interface, now_us }
    }

    /// Creates the driver using `embassy-time` for timeouts and delays.
    #[cfg(feature = "embassy-time")]
    pub fn new(interface: I) -> Self {
        Self::with_clock(interface, || embassy_time::Instant::now().as_micros())
    }

    /// Returns the interface.
    pub fn release(self) -> I {
        self.interface
    }

    /// Reads a raw register (see the MFRC522 datasheet, section 9).
    pub async fn read_register(&mut self, reg: u8) -> Result<u8, Error> {
        self.interface.read_reg(reg).await
    }

    /// Writes a raw register (see the MFRC522 datasheet, section 9).
    pub async fn write_register(&mut self, reg: u8, val: u8) -> Result<(), Error> {
        self.interface.write_reg(reg, val).await
    }

    #[cfg(not(feature = "embassy-time"))]
    async fn sleep_ms(&self, ms: u64) {
        // shortcut: busy-waits (blocks the executor), enable `embassy-time` for a real async sleep
        let start = (self.now_us)();
        while self.elapsed_us(start) < ms.saturating_mul(1_000) {}
    }

    #[cfg(feature = "embassy-time")]
    async fn sleep_ms(&self, ms: u64) {
        embassy_time::Timer::after_millis(ms).await;
    }

    /// Microseconds since `start` (wrapping, so a clock wrap can't panic).
    fn elapsed_us(&self, start: u64) -> u64 {
        (self.now_us)().wrapping_sub(start)
    }

    async fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), Error> {
        self.interface.write_reg(reg, val).await
    }

    async fn write_regs(&mut self, reg: u8, values: &[u8]) -> Result<(), Error> {
        self.interface.write_regs(reg, values).await
    }

    async fn read_reg(&mut self, reg: u8) -> Result<u8, Error> {
        self.interface.read_reg(reg).await
    }

    async fn read_regs(&mut self, reg: u8, out: &mut [u8], rx_align: u8) -> Result<(), Error> {
        self.interface.read_regs(reg, out, rx_align).await
    }

    async fn clear_bits(&mut self, reg: u8, mask: u8) -> Result<(), Error> {
        let val = self.read_reg(reg).await?;
        self.write_reg(reg, val & !mask).await
    }

    async fn set_bits(&mut self, reg: u8, mask: u8) -> Result<(), Error> {
        let val = self.read_reg(reg).await?;
        self.write_reg(reg, val | mask).await
    }
}
