#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use adv_shift_registers::wrappers::ShifterPin;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use embedded_hal::digital::OutputPin;
use esp_backtrace as _;
use esp_hal::{
    dma::{Dma, DmaRxBuf, DmaTxBuf},
    dma_buffers,
    gpio::{AnyPin, Io, Output},
    peripherals::DMA,
    prelude::*,
    spi::{master::Spi, SpiMode},
    timer::timg::TimerGroup,
};
use esp_hal_mfrc522::{consts::UidSize, debug::MFRC522Debug};
use log::{debug, error, info};

#[main]
async fn main(_spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    esp_println::logger::init_logger(log::LevelFilter::Trace);
    //esp_println::logger::init_logger_from_env();

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    #[cfg(feature = "esp32s3")]
    log::set_max_level(log::LevelFilter::Trace);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let data_pin = Output::new(io.pins.gpio10, esp_hal::gpio::Level::Low);
    let clk_pin = Output::new(io.pins.gpio21, esp_hal::gpio::Level::Low);
    let latch_pin = Output::new(io.pins.gpio1, esp_hal::gpio::Level::Low);

    // NOTE: change this to normal CS pin (im just testing my adv_shift_registers crate)
    let mut adv_shift_reg =
        adv_shift_registers::AdvancedShiftRegister::<8, _>::new(data_pin, clk_pin, latch_pin, 0);
    adv_shift_reg.update_shifters();
    let digits_shifters = adv_shift_reg.get_shifter_range_mut(2..8);
    digits_shifters.set_data(&[255; 6]);

    let mut cs_pin = adv_shift_reg.get_pin_mut(1, 0, true);
    _ = cs_pin.set_high();

    #[cfg(feature = "esp32s3")]
    let sck = io.pins.gpio4.degrade();
    #[cfg(feature = "esp32s3")]
    let miso = io.pins.gpio2.degrade();
    #[cfg(feature = "esp32s3")]
    let mosi = io.pins.gpio3.degrade();

    #[cfg(feature = "esp32c3")]
    let sck = io.pins.gpio4.degrade();
    #[cfg(feature = "esp32c3")]
    let miso = io.pins.gpio5.degrade();
    #[cfg(feature = "esp32c3")]
    let mosi = io.pins.gpio6.degrade();

    _ = _spawner.spawn(rfid_task(
        miso,
        mosi,
        sck,
        cs_pin,
        #[cfg(feature = "esp32s3")]
        peripherals.SPI3,
        #[cfg(feature = "esp32c3")]
        peripherals.SPI2,
        peripherals.DMA,
    ));

    loop {
        info!("main loop");
        Timer::after(Duration::from_millis(15000)).await;
    }
}

#[embassy_executor::task]
async fn rfid_task(
    miso: AnyPin,
    mosi: AnyPin,
    sck: AnyPin,
    cs_pin: ShifterPin,

    #[cfg(feature = "esp32s3")] spi: esp_hal::peripherals::SPI3,

    #[cfg(feature = "esp32c3")] spi: esp_hal::peripherals::SPI2,

    dma: DMA,
) {
    let dma = Dma::new(dma);
    let dma_chan = dma.channel0;
    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(32000);
    let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();
    let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();

    let dma_chan = dma_chan.configure_for_async(false, esp_hal::dma::DmaPriority::Priority0);

    //let cs = Output::new(cs, Level::High);
    let spi = Spi::new(spi, 5.MHz(), SpiMode::Mode0);
    let spi = spi.with_sck(sck).with_miso(miso).with_mosi(mosi);
    let spi = spi.with_dma(dma_chan).with_buffers(dma_rx_buf, dma_tx_buf);

    //esp_hal_mfrc522::MFRC522::new(spi, cs, || esp_hal::time::current_time().ticks());
    let mut mfrc522 = esp_hal_mfrc522::MFRC522::new(spi, cs_pin); // embassy-time feature is enabled,
                                                                  // so no need to pass current_time
                                                                  // function

    _ = mfrc522.pcd_init().await;
    _ = mfrc522.pcd_selftest().await;
    debug!("PCD ver: {:?}", mfrc522.pcd_get_version().await);

    if !mfrc522.pcd_is_init().await {
        error!("MFRC522 init failed! Try to power cycle to module!");
    }

    loop {
        if mfrc522.picc_is_new_card_present().await.is_ok() {
            let card = mfrc522.get_card(UidSize::Four).await;
            if let Ok(card) = card {
                info!("Card UID: {}", card.get_number());

                let mut buff = [0; 18];
                let mut byte_count = 18;
                _ = mfrc522.mifare_read(0, &mut buff, &mut byte_count).await;

                log::info!("{:02X?}", buff);

                //_ = mfrc522.debug_dump_card(&card).await;
            }

            _ = mfrc522.picc_halta().await;
        }

        Timer::after(Duration::from_millis(1)).await;
    }
}
