#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    dma::Dma,
    dma_descriptors,
    gpio::Io,
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

const RC522_MOD_WIDTH_REG: u8 = 0x24;

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
    let cs = io.pins.gpio5;

    let dma = Dma::new(peripherals.DMA);
    let dma_chan = dma.channel0;
    let (mut descriptors, mut rx_descriptors) = dma_descriptors!(32000);
    let dma_chan = dma_chan.configure_for_async(
        false,
        &mut descriptors,
        &mut rx_descriptors,
        esp_hal::dma::DmaPriority::Priority0,
    );

    let spi = Spi::new(peripherals.SPI3, 5000000.Hz(), SpiMode::Mode0, &clocks);
    let mut spi: SpiDma<SPI3, _, FullDuplexMode, Async> = spi.with_dma(dma_chan);

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
