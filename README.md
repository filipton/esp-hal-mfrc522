# esp-hal-MFRC522
Async, `no_std`, no-alloc MFRC522 RFID library for esp-hal (works with any HAL implementing `embedded-hal-async` SPI or I2C).
This project is just "port" of [this Arduino Library](https://github.com/OSSLibraries/Arduino_MFRC522v2).

[![crates.io](https://img.shields.io/crates/v/esp-hal-mfrc522.svg)](https://crates.io/crates/esp-hal-mfrc522)
[![MIT license](https://img.shields.io/github/license/mashape/apistatus.svg)]()

## Example
```rust
let dma_rx_buf = dma_rx_buffer!(512).expect("Dma rx buf failed");
let dma_tx_buf = dma_tx_buffer!(512).expect("Dma tx buf failed");

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

            // dumps card blocks using log::debug
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
