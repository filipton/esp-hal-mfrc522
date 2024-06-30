#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use consts::{PCDCommand, PCDRegister};
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use embedded_hal::digital::OutputPin;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    dma::Dma,
    dma_descriptors,
    gpio::{Io, Level, Output},
    peripherals::{Peripherals, SPI3},
    prelude::*,
    spi::{
        master::{dma::SpiDma, prelude::*, Spi},
        FullDuplexMode, SpiMode,
    },
    system::SystemControl,
    timer::timg::TimerGroup,
    Async,
};

mod consts;

#[main]
async fn main(_spawner: Spawner) {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::max(system.clock_control).freeze();
    esp_println::logger::init_logger_from_env();

    let timg0 = TimerGroup::new_async(peripherals.TIMG0, &clocks);
    esp_hal_embassy::init(&clocks, timg0);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let sck = io.pins.gpio4;
    let miso = io.pins.gpio2;
    let mosi = io.pins.gpio3;
    let cs = Output::new(io.pins.gpio5, Level::High);

    let dma = Dma::new(peripherals.DMA);
    let dma_chan = dma.channel0;
    let (mut descriptors, mut rx_descriptors) = dma_descriptors!(32000);
    let dma_chan = dma_chan.configure_for_async(
        false,
        &mut descriptors,
        &mut rx_descriptors,
        esp_hal::dma::DmaPriority::Priority0,
    );

    let spi = Spi::new(peripherals.SPI3, 4.MHz(), SpiMode::Mode0, &clocks);
    let spi: Spi<SPI3, FullDuplexMode> = spi.with_sck(sck).with_miso(miso).with_mosi(mosi);
    let spi: SpiDma<SPI3, _, FullDuplexMode, Async> = spi.with_dma(dma_chan);

    let mut mfrc522 = MFRC522::new(spi, cs);
    _ = mfrc522.pcd_init().await;
    esp_println::println!("PCD ver: {:?}", mfrc522.pcd_get_version().await);
}

pub struct MFRC522<S, C>
where
    S: embedded_hal_async::spi::SpiBus,
    C: OutputPin,
{
    spi: S,
    cs: C,
    read_buff: [u8; 1],
}

// TODO: implement own errors type (using for example thiserror-no-std or snafu)
impl<S, C> MFRC522<S, C>
where
    S: embedded_hal_async::spi::SpiBus,
    C: OutputPin,
{
    pub fn new(spi: S, cs: C) -> Self {
        Self {
            spi,
            cs,
            read_buff: [0],
        }
    }

    pub async fn pcd_init(&mut self) -> Result<(), ()> {
        self.pcd_reset().await?;

        self.write_reg(PCDRegister::TxModeReg, 0x00).await?;
        self.write_reg(PCDRegister::RxModeReg, 0x00).await?;

        self.write_reg(PCDRegister::ModWidthReg, 0x26).await?;

        self.write_reg(PCDRegister::TModeReg, 0x80).await?;
        self.write_reg(PCDRegister::TPrescalerReg, 0xA9).await?;
        self.write_reg(PCDRegister::TReloadRegH, 0x03).await?;
        self.write_reg(PCDRegister::TReloadRegL, 0xE8).await?;

        self.write_reg(PCDRegister::TxASKReg, 0x40).await?;
        self.write_reg(PCDRegister::ModeReg, 0x3D).await?;

        self.pcd_antenna_on().await?;
        Timer::after(Duration::from_millis(4)).await;

        Ok(())
    }

    pub async fn pcd_reset(&mut self) -> Result<(), ()> {
        self.write_reg(PCDRegister::CommandReg, PCDCommand::SoftReset)
            .await?;

        // max 3 tries
        for _ in 0..3 {
            let out = self.read_reg(PCDRegister::CommandReg).await;
            if let Ok(out) = out {
                let out = out & (1 << 4);
                if out == 0 {
                    break;
                }
            }

            Timer::after(Duration::from_millis(50)).await;
        }

        Ok(())
    }

    pub async fn pcd_antenna_on(&mut self) -> Result<(), ()> {
        let val = self.read_reg(PCDRegister::TxControlReg).await?;
        if (val & 0x03) != 0x03 {
            self.write_reg(PCDRegister::TxControlReg, val | 0x03)
                .await?;
        }

        Ok(())
    }

    pub async fn pcd_get_version(&mut self) -> Result<u8, ()> {
        self.read_reg(PCDRegister::VersionReg).await
    }

    /// Now it prints data to console, TODO: change this
    /// Always returns false (for now)
    pub async fn pcd_selftest(&mut self) -> Result<bool, ()> {
        self.write_reg(PCDRegister::FIFOLevelReg, 0x80).await?;
        self.write_reg_buff(PCDRegister::FIFODataReg, 25, &[0; 25])
            .await?;

        self.write_reg(PCDRegister::CommandReg, PCDCommand::Mem)
            .await?;
        self.write_reg(PCDRegister::AutoTestReg, 0x09).await?;
        self.write_reg(PCDRegister::FIFODataReg, 0x00).await?;
        self.write_reg(PCDRegister::CommandReg, PCDCommand::CalcCRC)
            .await?;

        for _ in 0..0xFF {
            let n = self.read_reg(PCDRegister::FIFOLevelReg).await?;
            if n >= 64 {
                break;
            }
        }

        self.write_reg(PCDRegister::CommandReg, PCDCommand::Idle)
            .await?;

        let mut res = [0; 64];
        self.read_reg_buff(PCDRegister::FIFODataReg, 64, &mut res, 0)
            .await?;
        self.write_reg(PCDRegister::AutoTestReg, 0x40 & 0x00)
            .await?;

        for i in 0..64 {
            if i % 8 == 0 {
                esp_println::print!("\n");
            }

            esp_println::print!("{:#04x} ", res[i]);
        }
        esp_println::print!("\n");

        self.pcd_init().await?;
        Ok(false)
    }

    pub async fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), ()> {
        _ = self.cs.set_low();
        _ = self.spi.transfer(&mut self.read_buff, &[reg << 1]).await;
        _ = self.spi.transfer(&mut self.read_buff, &[val]).await;
        _ = self.cs.set_high();

        Ok(())
    }

    pub async fn write_reg_buff(&mut self, reg: u8, count: usize, values: &[u8]) -> Result<(), ()> {
        _ = self.cs.set_low();
        _ = self.spi.transfer(&mut self.read_buff, &[reg << 1]).await;

        for i in 0..count {
            _ = self.spi.transfer(&mut self.read_buff, &[values[i]]).await;
        }

        _ = self.cs.set_high();
        Ok(())
    }

    pub async fn read_reg(&mut self, reg: u8) -> Result<u8, ()> {
        _ = self.cs.set_low();
        _ = self
            .spi
            .transfer(&mut self.read_buff, &[(reg << 1) | 0x80])
            .await;

        _ = self.spi.transfer(&mut self.read_buff, &[0]).await;
        _ = self.cs.set_high();

        Ok(self.read_buff[0])
    }

    pub async fn read_reg_buff(
        &mut self,
        reg: u8,
        count: usize,
        output_buff: &mut [u8],
        rx_align: u8,
    ) -> Result<(), ()> {
        if count == 0 {
            return Ok(());
        }

        let addr = 0x80 | (reg << 1);
        let mut index = 0;

        _ = self.cs.set_low();
        _ = self.spi.transfer(&mut self.read_buff, &[addr]).await;
        if rx_align > 0 {
            let mask = (0xFF << rx_align) & 0xFF;
            _ = self.spi.transfer(&mut self.read_buff, &[addr]).await;

            output_buff[0] = (output_buff[0] & !mask) | (self.read_buff[0] & mask);
            index += 1;
        }

        while index < count - 1 {
            _ = self.spi.transfer(&mut self.read_buff, &[addr]).await;
            output_buff[index] = self.read_buff[0];
            index += 1;
        }

        _ = self.spi.transfer(&mut self.read_buff, &[0]).await;
        output_buff[index] = self.read_buff[0];

        _ = self.cs.set_high();
        Ok(())
    }

    pub async fn clear_register_bit_mask(&mut self, reg: u8, mask: u8) -> Result<(), ()> {
        let tmp = self.read_reg(reg).await?;
        self.write_reg(reg, tmp & (!mask)).await?;

        Ok(())
    }

    pub async fn set_register_bit_mask(&mut self, reg: u8, mask: u8) -> Result<(), ()> {
        let tmp = self.read_reg(reg).await?;
        self.write_reg(reg, tmp | mask).await?;

        Ok(())
    }

    pub async fn calc_crc(&mut self, data: &[u8], length: u8, res: &mut [u8]) -> Result<(), ()> {
        self.write_reg(PCDRegister::CommandReg, PCDCommand::Idle)
            .await?;
        self.write_reg(PCDRegister::DivIrqReg, 0x04).await?;
        self.write_reg(PCDRegister::FIFOLevelReg, 0x80).await?;
        self.write_reg_buff(PCDRegister::FIFODataReg, length as usize, data)
            .await?;
        self.write_reg(PCDRegister::CommandReg, PCDCommand::CalcCRC)
            .await?;

        // TODO: make sth like timeout (this 5000 is only for waiting)
        for _ in 0..5000 {
            let n = self.read_reg(PCDRegister::DivIrqReg).await?;
            if n & 0x04 != 0 {
                self.write_reg(PCDRegister::CommandReg, PCDCommand::Idle)
                    .await?;

                res[0] = self.read_reg(PCDRegister::CRCResultRegL).await?;
                res[1] = self.read_reg(PCDRegister::CRCResultRegH).await?;
                return Ok(());
            }
        }

        // timeout?
        Err(())
    }
}
