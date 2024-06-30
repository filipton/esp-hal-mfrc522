#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use embedded_hal::spi::SpiBus;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    dma::*,
    dma_descriptors,
    gpio::Io,
    peripherals::Peripherals,
    prelude::*,
    spi::{
        master::{prelude::*, Spi},
        SpiMode,
    },
    system::SystemControl,
    timer::timg::TimerGroup,
};

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

    /*
    let dma = Dma::new(peripherals.DMA);
    let dma_chan = dma.channel0;

    let (descriptors, rx_descriptors) = dma_descriptors!(32000);
    */
    let mut spi = Spi::new(peripherals.SPI2, 100.kHz(), SpiMode::Mode0, &clocks).with_pins(
        Some(sck),
        Some(mosi),
        Some(miso),
        Some(cs),
    );

    //let send_buffer = [0, 1, 2, 3, 4, 5, 6, 7];
    let mut read_buf = [16];
    loop {
        let res = spi.read(&mut read_buf);
        esp_println::println!("read: {res:?}");
        esp_println::println!("read_buf: {read_buf:?}");

        //Timer::after(Duration::from_millis(100)).await;
    }
    /*
    loop {
        log::info!("Hello world!");
        Timer::after(Duration::from_millis(500)).await;
    }
    */
}
