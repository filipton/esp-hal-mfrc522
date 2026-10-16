#![no_std]
#![no_main]

use adv_shift_registers::wrappers::ShifterPin;
use embassy_executor::Spawner;
use embassy_time::Delay;
use embassy_time::{Duration, Timer};
use embedded_hal::digital::OutputPin;
use embedded_hal_bus::spi::ExclusiveDevice;
use esp_backtrace as _;
use esp_hal::gpio::Pin;
use esp_hal::spi::Mode;
use esp_hal::time::Rate;
use esp_hal::{
    dma_rx_buffer, dma_tx_buffer,
    gpio::{AnyPin, Output},
    spi::master::Spi,
    timer::timg::TimerGroup,
};
use esp_hal_mfrc522::{Key, KeyType, Mfrc522, SpiInterface};
use log::{error, info};

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_hal::main]
async fn main(_spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    esp_println::logger::init_logger(log::LevelFilter::Trace);
    //esp_println::logger::init_logger_from_env();

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);

    #[cfg(feature = "esp32s3")]
    log::set_max_level(log::LevelFilter::Trace);

    let data_pin = Output::new(
        peripherals.GPIO10,
        esp_hal::gpio::Level::Low,
        Default::default(),
    );
    let clk_pin = Output::new(
        peripherals.GPIO21,
        esp_hal::gpio::Level::Low,
        Default::default(),
    );
    let latch_pin = Output::new(
        peripherals.GPIO1,
        esp_hal::gpio::Level::Low,
        Default::default(),
    );

    // NOTE: change this to normal CS pin (im just testing my adv_shift_registers crate)
    let mut adv_shift_reg =
        adv_shift_registers::AdvancedShiftRegister::<8, _>::new(data_pin, clk_pin, latch_pin, 0);
    adv_shift_reg.update_shifters();
    let digits_shifters = adv_shift_reg.get_shifter_range_mut(2..8);
    digits_shifters.set_data(&[255; 6]);

    let mut cs_pin = adv_shift_reg.get_pin_mut(1, 0, true);
    _ = cs_pin.set_high();

    #[cfg(feature = "esp32s3")]
    let sck = peripherals.GPIO4.degrade();
    #[cfg(feature = "esp32s3")]
    let miso = peripherals.GPIO2.degrade();
    #[cfg(feature = "esp32s3")]
    let mosi = peripherals.GPIO3.degrade();

    #[cfg(feature = "esp32c3")]
    let sck = peripherals.GPIO4.degrade();
    #[cfg(feature = "esp32c3")]
    let miso = peripherals.GPIO5.degrade();
    #[cfg(feature = "esp32c3")]
    let mosi = peripherals.GPIO6.degrade();

    _spawner.spawn(rfid_task(
        miso,
        mosi,
        sck,
        cs_pin,
        #[cfg(feature = "esp32s3")]
        peripherals.SPI3,
        #[cfg(feature = "esp32c3")]
        peripherals.SPI2,
        peripherals.DMA_CH0,
    ).unwrap());

    loop {
        info!("main loop");
        Timer::after(Duration::from_millis(15000)).await;
    }
}

#[embassy_executor::task]
async fn rfid_task(
    miso: AnyPin<'static>,
    mosi: AnyPin<'static>,
    sck: AnyPin<'static>,
    //cs_pin: AnyPin,
    cs_pin: ShifterPin,

    #[cfg(feature = "esp32s3")] spi: esp_hal::peripherals::SPI3<'static>,

    #[cfg(feature = "esp32c3")] spi: esp_hal::peripherals::SPI2<'static>,

    dma: esp_hal::peripherals::DMA_CH0<'static>,
) {
    let dma_rx_buf = dma_rx_buffer!(512).expect("Dma rx buf failed");
    let dma_tx_buf = dma_tx_buffer!(512).expect("Dma tx buf failed");

    //let cs = Output::new(cs, Level::High);
    let spi = Spi::new(
        spi,
        esp_hal::spi::master::Config::default()
            .with_frequency(Rate::from_khz(400))
            .with_mode(Mode::_0),
    )
    .unwrap();

    let spi = spi.with_sck(sck).with_miso(miso).with_mosi(mosi);
    let spi = spi
        .with_dma(dma)
        .with_buffers(dma_rx_buf, dma_tx_buf)
        .into_async();

    let dev = ExclusiveDevice::new(spi, cs_pin, Delay).unwrap();

    let mut rfid = Mfrc522::new(SpiInterface::new(dev)); // `embassy-time` feature

    if let Err(e) = rfid.init().await {
        error!("MFRC522 init failed: {e}");
    }
    info!("MFRC522 version: {:?}", rfid.version().await);
    if !matches!(rfid.is_initialized().await, Ok(true)) {
        error!("MFRC522 not responding! Try to power cycle the module!");
    }

    loop {
        if let Ok(true) = rfid.is_new_card_present().await {
            if let Ok(uid) = rfid.select().await {
                info!("Card UID: {:02X?} ({:?})", uid.as_bytes(), uid.card_type());

                // block 0 (manufacturer block) with the factory key
                match rfid.authenticate(KeyType::A, 0, &Key::DEFAULT, &uid).await {
                    Ok(()) => info!("Block 0: {:02X?}", rfid.read_block(0).await),
                    Err(e) => error!("Auth failed: {e}"),
                }
                //_ = rfid.dump_card(&uid).await;

                _ = rfid.halt().await;
                _ = rfid.stop_crypto1().await;
            }
        }

        Timer::after(Duration::from_millis(1)).await;
    }
}
