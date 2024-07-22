# esp-hal-MFRC522
Non-blocking RFID library for esp-hal (maybe for other hal's in the future).
This project is just "port" of [this Arduino Library](https://github.com/OSSLibraries/Arduino_MFRC522v2).

[![crates.io](https://img.shields.io/crates/v/esp-hal-mfrc522.svg)](https://crates.io/crates/esp-hal-mfrc522)
[![MIT license](https://img.shields.io/github/license/mashape/apistatus.svg)]()

## Example
```rust
let dma = Dma::new(peripherals.dma);
let dma_chan = dma.channel0;
let (descriptors, rx_descriptors) = dma_descriptors!(32000);
let dma_chan = dma_chan.configure_for_async(false, esp_hal::dma::DmaPriority::Priority0);

let cs = Output::new(io.pins.gpio5, Level::High);
let spi = Spi::new(peripherals.SPI3, 5.MHz(), SpiMode::Mode0, &clocks);
let spi: Spi<SPI3, FullDuplexMode> = spi.with_sck(io.pins.gpio4).with_miso(io.pins.gpio2).with_mosi(io.pins.gpio3);
let spi: SpiDma<SPI3, _, FullDuplexMode, Async> =
    spi.with_dma(dma_chan, descriptors, rx_descriptors);

//mfrc522_esp_hal::MFRC522::new(spi, cs, || esp_hal::time::current_time().ticks());
let mut mfrc522 = mfrc522_esp_hal::MFRC522::new(spi, cs); // embassy-time feature is enabled,
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
