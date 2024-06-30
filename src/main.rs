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
use esp_println::println;

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

    let spi = Spi::new(peripherals.SPI3, 4000000.Hz(), SpiMode::Mode0, &clocks);
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

    for i in 0..0xFF {
        let n = read_register(&mut spi, &mut cs, PCDRegister::FIFOLevelReg)
            .await
            .unwrap();

        esp_println::println!("N: {n}");
    }

    /*
      byte        n;
    for(uint8_t i   = 0; i < 0xFF; i++) {
      // The datasheet does not specify exact completion condition except
      // that FIFO buffer should contain 64 bytes.
      // While selftest is initiated by CalcCRC command
      // it behaves differently from normal CRC computation,
      // so one can't reliably use DivIrqReg to check for completion.
      // It is reported that some devices does not trigger CRCIRq flag
      // during selftest.
      n = _driver.PCD_ReadRegister(PCD_Register::FIFOLevelReg);
      if(n >= 64) {
        break;
      }
    }
    _driver.PCD_WriteRegister(PCD_Register::CommandReg, PCD_Command::PCD_Idle);
      */

    /*
    loop {
        _ = write_register(&mut spi, &mut cs, PCDRegister::TxModeReg, 0x00).await;
        _ = write_register(&mut spi, &mut cs, PCDRegister::RxModeReg, 0x00).await;
        _ = write_register(&mut spi, &mut cs, PCDRegister::ModWidthReg, 0x26).await;

        Timer::after(Duration::from_millis(30)).await;
    }
    */

    /*
        let test_val = 0x25;
        let mut pass = 0;

        let mut read_buff = [0; 1]; // read size: n = 1
        for i in test_val..(test_val + 2) {
            let res = write_register(&mut spi, &mut cs, RC522_MOD_WIDTH_REG, i).await;

            if res.is_ok() {
                // read:
                cs.set_low();
                let res = embedded_hal_async::spi::SpiBus::transfer(
                    &mut spi,
                    &mut read_buff,
                    &[(RC522_MOD_WIDTH_REG) | 0x80],
                )
                .await;
                esp_println::println!("read_write_res: {res:?}");

                let res =
                    embedded_hal_async::spi::SpiBus::transfer(&mut spi, &mut read_buff, &[0]).await;
                esp_println::println!("read_read_res: {res:?}");

                esp_println::println!("RES: {}, expected: {i}", read_buff[0]);
                cs.set_high();
                /*
                if res.is_ok() && read_buff[0] == i {

                }
                */
            }

            if pass != 1 {
                esp_println::println!("ERROR: (pass != 1)");
            }

            // write(RC522_MOD_WIDTH_REG, i)

            // if not err
            //  tmp = read(RC522_MOD_WIDTH_REG)
            //  if not err and tmp == i; then pass = 1
            //
            // if pass != 1
            //  Error!
        }
    */

    /*
    let send_buf = [0, 1, 2, 3, 4, 5, 6, 7];
    //let mut read_buf = [0; 16];
    loop {
        let mut buffer = [0; 8];
        esp_println::println!("Sending bytes");
        let res = embedded_hal_async::spi::SpiBus::transfer(&mut spi, &mut buffer, &send_buf).await;
        esp_println::println!("res: {res:?}");
        esp_println::println!("Bytes received: {:?}", buffer);
        //let res = spi.read(&mut read_buf);
        //spi.read
        //_ = embedded_hal_async::spi::SpiBus::write(&mut spi, &[]).await;
        //let res = embedded_hal_async::spi::SpiBus::read(&mut spi, &mut read_buf).await;
        //esp_println::println!("read: {res:?}");
        //esp_println::println!("read_buf: {read_buf:?}");

        Timer::after(Duration::from_millis(100)).await;
    }
    */
    /*
    loop {
        log::info!("Hello world!");
        Timer::after(Duration::from_millis(500)).await;
    }
    */
}

/*
        impl<'d, T, C, M> embedded_hal_async::spi::SpiBus for SpiDma<'d, T, C, M, crate::Async>
        where
            T: InstanceDma<C::Tx<'d>, C::Rx<'d>>,
            C: ChannelTypes,
            C::P: SpiPeripheral,
            M: IsFullDuplex,
*/

async fn write_register(
    spi: &mut impl embedded_hal_async::spi::SpiBus,
    cs: &mut impl OutputPin,
    register: u8,
    value: u8,
) -> Result<(), ()> {
    let mut read_buff = [0; 1];

    _ = cs.set_low();
    let res = spi.transfer(&mut read_buff, &[register << 1]).await;
    //esp_println::println!("write_addr_res: {res:?}");

    let res = spi.transfer(&mut read_buff, &[value]).await;
    //esp_println::println!("write_val_res: {res:?}");
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
    let res = spi.transfer(&mut read_buff, &[register << 1]).await;
    //esp_println::println!("write_addr_res: {res:?}");

    for i in 0..count {
        let res = spi.transfer(&mut read_buff, &[values[i]]).await;
        //esp_println::println!("write_val_res: {res:?}");
    }

    _ = cs.set_high();

    Ok(())
}

async fn read_register(
    spi: &mut impl embedded_hal_async::spi::SpiBus,
    cs: &mut impl OutputPin,
    register: u8,
) -> Result<u8, ()> {
    let mut read_buff = [0; 1];

    _ = cs.set_low();
    let res = spi
        .transfer(&mut read_buff, &[(register << 1) | 0x80])
        .await;
    //esp_println::println!("read_w_addr_res: {res:?}");

    let res = spi.transfer(&mut read_buff, &[0]).await;
    //esp_println::println!("read_r_val_res: {res:?}");
    _ = cs.set_high();

    Ok(read_buff[0])
}
