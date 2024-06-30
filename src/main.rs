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
    let mut cs = Output::new(io.pins.gpio5, Level::High);

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
    let mut spi: SpiDma<SPI3, _, FullDuplexMode, Async> = spi.with_dma(dma_chan);

    esp_println::println!("PCD reset first");
    _ = write_register(
        &mut spi,
        &mut cs,
        PCDRegister::CommandReg,
        PCDCommand::SoftReset,
    )
    .await;

    // max 3 tries
    for _ in 0..3 {
        let out = read_register(&mut spi, &mut cs, PCDRegister::CommandReg).await;
        if let Ok(out) = out {
            let out = out & (1 << 4);
            if out == 0 {
                break;
            }
        }

        Timer::after(Duration::from_millis(50)).await;
    }

    esp_println::println!("PCD init");
    _ = write_register(&mut spi, &mut cs, PCDRegister::TxModeReg, 0x00).await;
    _ = write_register(&mut spi, &mut cs, PCDRegister::RxModeReg, 0x00).await;

    _ = write_register(&mut spi, &mut cs, PCDRegister::ModWidthReg, 0x26).await;

    _ = write_register(&mut spi, &mut cs, PCDRegister::TModeReg, 0x80).await;
    _ = write_register(&mut spi, &mut cs, PCDRegister::TPrescalerReg, 0xA9).await;
    _ = write_register(&mut spi, &mut cs, PCDRegister::TReloadRegH, 0x03).await;
    _ = write_register(&mut spi, &mut cs, PCDRegister::TReloadRegL, 0xE8).await;

    _ = write_register(&mut spi, &mut cs, PCDRegister::TxASKReg, 0x40).await;
    _ = write_register(&mut spi, &mut cs, PCDRegister::ModeReg, 0x3D).await;

    esp_println::println!("PCD antenna on");
    let val = read_register(&mut spi, &mut cs, PCDRegister::TxControlReg)
        .await
        .unwrap();

    if (val & 0x03) != 0x03 {
        _ = write_register(&mut spi, &mut cs, PCDRegister::TxControlReg, val | 0x03).await;
    }

    Timer::after(Duration::from_millis(4)).await;
    let ver = read_register(&mut spi, &mut cs, PCDRegister::VersionReg)
        .await
        .unwrap();

    esp_println::println!("PCD ver: {ver:?}");

    // SELFTEST CODE
    /*
    esp_println::println!("PCD selftest");

    _ = write_register(&mut spi, &mut cs, PCDRegister::FIFOLevelReg, 0x80).await;
    _ = write_registers(
        &mut spi,
        &mut cs,
        PCDRegister::FIFODataReg,
        25,
        &[
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        ],
    )
    .await;

    _ = write_register(&mut spi, &mut cs, PCDRegister::CommandReg, PCDCommand::Mem).await;
    _ = write_register(&mut spi, &mut cs, PCDRegister::AutoTestReg, 0x09).await;
    _ = write_register(&mut spi, &mut cs, PCDRegister::FIFODataReg, 0x00).await;
    _ = write_register(
        &mut spi,
        &mut cs,
        PCDRegister::CommandReg,
        PCDCommand::CalcCRC,
    )
    .await;

    for _ in 0..0xFF {
        let n = read_register(&mut spi, &mut cs, PCDRegister::FIFOLevelReg)
            .await
            .unwrap();

        if n >= 64 {
            break;
        }
    }

    _ = write_register(&mut spi, &mut cs, PCDRegister::CommandReg, PCDCommand::Idle).await;

    let mut res = [0; 64];
    _ = read_registers(&mut spi, &mut cs, PCDRegister::FIFODataReg, 64, &mut res, 0).await;
    _ = write_register(&mut spi, &mut cs, PCDRegister::AutoTestReg, 0x40 & 0x00).await;

    for i in 0..64 {
        if i % 8 == 0 {
            esp_println::print!("\n");
        }

        esp_println::print!("{:#04x} ", res[i]);
    }
    esp_println::print!("\n");
    */
}

async fn write_register(
    spi: &mut impl embedded_hal_async::spi::SpiBus,
    cs: &mut impl OutputPin,
    register: u8,
    value: u8,
) -> Result<(), ()> {
    let mut read_buff = [0; 1];

    _ = cs.set_low();
    _ = spi.transfer(&mut read_buff, &[register << 1]).await;

    _ = spi.transfer(&mut read_buff, &[value]).await;
    _ = cs.set_high();

    Ok(())
}

async fn write_registers(
    spi: &mut impl embedded_hal_async::spi::SpiBus,
    cs: &mut impl OutputPin,
    register: u8,
    count: usize,
    values: &[u8],
) -> Result<(), ()> {
    let mut read_buff = [0; 1];

    _ = cs.set_low();
    _ = spi.transfer(&mut read_buff, &[register << 1]).await;

    for i in 0..count {
        _ = spi.transfer(&mut read_buff, &[values[i]]).await;
    }

    _ = cs.set_high();

    Ok(())
}

// TODO: use ? to propagate errors up
async fn read_register(
    spi: &mut impl embedded_hal_async::spi::SpiBus,
    cs: &mut impl OutputPin,
    register: u8,
) -> Result<u8, ()> {
    let mut read_buff = [0; 1];

    _ = cs.set_low();
    _ = spi
        .transfer(&mut read_buff, &[(register << 1) | 0x80])
        .await;

    _ = spi.transfer(&mut read_buff, &[0]).await;
    _ = cs.set_high();

    Ok(read_buff[0])
}

async fn read_registers(
    spi: &mut impl embedded_hal_async::spi::SpiBus,
    cs: &mut impl OutputPin,
    register: u8,
    count: usize,
    buff: &mut [u8],
    rx_align: u8,
) -> Result<(), ()> {
    if count == 0 {
        return Ok(());
    }

    let addr = 0x80 | (register << 1);
    let mut index = 0;
    let mut read_buff = [0; 1];

    _ = cs.set_low();
    _ = spi.transfer(&mut read_buff, &[addr]).await;
    if rx_align > 0 {
        let mask = (0xFF << rx_align) & 0xFF;
        _ = spi.transfer(&mut read_buff, &[addr]).await;
        let val = read_buff[0];

        buff[0] = (buff[0] & !mask) | (val & mask);
        index += 1;
    }

    while index < count - 1 {
        _ = spi.transfer(&mut read_buff, &[addr]).await;
        buff[index] = read_buff[0];
        index += 1;
    }

    _ = spi.transfer(&mut read_buff, &[0]).await;
    buff[index] = read_buff[0];

    _ = cs.set_high();
    Ok(())
}

async fn clear_register_bit_mask(
    spi: &mut impl embedded_hal_async::spi::SpiBus,
    cs: &mut impl OutputPin,
    register: u8,
    mask: u8,
) -> Result<(), ()> {
    let tmp = read_register(spi, cs, register).await?;
    write_register(spi, cs, register, tmp & (!mask)).await?;

    Ok(())
}

async fn set_register_bit_mask(
    spi: &mut impl embedded_hal_async::spi::SpiBus,
    cs: &mut impl OutputPin,
    register: u8,
    mask: u8,
) -> Result<(), ()> {
    let tmp = read_register(spi, cs, register).await?;
    write_register(spi, cs, register, tmp | mask).await?;

    Ok(())
}

async fn calc_crc(
    spi: &mut impl embedded_hal_async::spi::SpiBus,
    cs: &mut impl OutputPin,
    data: &[u8],
    length: u8,
    res: &mut [u8],
) -> Result<(), ()> {
    write_register(spi, cs, PCDRegister::CommandReg, PCDCommand::Idle).await?;
    write_register(spi, cs, PCDRegister::DivIrqReg, 0x04).await?;
    write_register(spi, cs, PCDRegister::FIFOLevelReg, 0x80).await?;
    write_registers(spi, cs, PCDRegister::FIFODataReg, length as usize, data).await?;
    write_register(spi, cs, PCDRegister::CommandReg, PCDCommand::CalcCRC).await?;

    for i in (0..5000).rev() {
        let n = read_register(spi, cs, PCDRegister::DivIrqReg).await?;
        if n & 0x04 != 0 {
            write_register(spi, cs, PCDRegister::CommandReg, PCDCommand::Idle).await?;

            res[0] = read_register(spi, cs, PCDRegister::CRCResultRegL).await?;
            res[1] = read_register(spi, cs, PCDRegister::CRCResultRegH).await?;
            return Ok(());
        }
    }

    // timeout?
    Err(())
}
