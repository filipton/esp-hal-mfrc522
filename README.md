# esp-hal-MFRC522
Non-blocking RFID library for esp-hal (maybe for other hal's in the future).
This project is just "port" of [this Arduino Library](https://github.com/OSSLibraries/Arduino_MFRC522v2).

[![crates.io](https://img.shields.io/crates/v/esp-hal-mfrc522.svg)](https://crates.io/crates/esp-hal-mfrc522)
[![MIT license](https://img.shields.io/github/license/mashape/apistatus.svg)]()

## Example
```rust
let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(512);
let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).expect("Dma tx buf failed");
let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).expect("Dma rx buf failed");

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

let driver = esp_hal_mfrc522::drivers::SpiDriver::new(dev);
//esp_hal_mfrc522::MFRC522::new(spi, cs, || esp_hal::time::current_time().ticks());
let mut mfrc522 = esp_hal_mfrc522::MFRC522::new(driver); // embassy-time feature is enabled,
                                                         // so no need to pass current_time
                                                         // function

_ = mfrc522.pcd_init().await;
_ = mfrc522.pcd_selftest().await;
log::debug!("PCD ver: {:?}", mfrc522.pcd_get_version().await);

if !mfrc522.pcd_is_init().await {
    log::error!("MFRC522 init failed! Try to power cycle to module!");
}

loop {
    if mfrc522.picc_is_new_card_present().await.is_ok() {
        let card = mfrc522.get_card(UidSize::Four).await;
        if let Ok(card) = card {
            log::info!("Card UID: {}", card.get_number());

            // this function dumps card blocks using log::debug
            // use mfrc522_esp_hal::debug::MFRC522Debug;
            //
            //_ = mfrc522.debug_dump_card(&card).await;
        }

        _ = mfrc522.picc_halta().await;
    }

    Timer::after(Duration::from_millis(1)).await;
}
```

## TODO
- [ ] Change some functions to be more "rust-like"
- [ ] Documentation in code
- [x] Crates.io publish
