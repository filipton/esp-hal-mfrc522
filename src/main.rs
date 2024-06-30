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
    let mut cs = Output::new(io.pins.gpio5, Level::High);

    Timer::after(Duration::from_millis(1000)).await;
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

    esp_println::println!("Performing RW test...");
    let test_val = 0x25;
    let mut pass = 0;

    for i in test_val..(test_val + 2) {
        // write:
        _ = embedded_hal_async::spi::SpiBus::flush(&mut spi).await;
        cs.set_low();
        let mut write_buff = [RC522_MOD_WIDTH_REG, i]; // ADDR, DATA..
        write_buff[0] = (write_buff[0] << 1) & 0x7E; // idk why but ok
        let res = embedded_hal_async::spi::SpiBus::write(&mut spi, &write_buff).await;
        esp_println::println!("write_res: {res:?}");
        cs.set_high();

        if res.is_ok() {
            // read:
            _ = embedded_hal_async::spi::SpiBus::flush(&mut spi).await;
            cs.set_low();
            let addr = ((RC522_MOD_WIDTH_REG << 1) & 0x7E) | 0x80;
            let res = embedded_hal_async::spi::SpiBus::write(&mut spi, &[addr]).await;
            esp_println::println!("read_write_res: {res:?}");
            cs.set_high();

            _ = embedded_hal_async::spi::SpiBus::flush(&mut spi).await;
            cs.set_low();
            let mut read_buff = [0; 1]; // read size: n = 1
            let res = embedded_hal_async::spi::SpiBus::read(&mut spi, &mut read_buff).await;
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
