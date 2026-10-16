# esp-hal-MFRC522
Async, `no_std`, no-alloc MFRC522 RFID library for esp-hal (works with any HAL implementing `embedded-hal-async` SPI or I2C).
This project is just "port" of [this Arduino Library](https://github.com/OSSLibraries/Arduino_MFRC522v2).

[![crates.io](https://img.shields.io/crates/v/esp-hal-mfrc522.svg)](https://crates.io/crates/esp-hal-mfrc522)
[![MIT license](https://img.shields.io/github/license/mashape/apistatus.svg)]()

## Example
```rust,no_run
use esp_hal_mfrc522::{Error, Interface, Key, KeyType, Mfrc522};

// `interface` is e.g. `SpiInterface::new(spi_device)` (any embedded-hal-async `SpiDevice`) or
// `I2cInterface::new(i2c, DEFAULT_I2C_ADDRESS)`.
async fn read_cards(interface: impl Interface, now_us: fn() -> u64) -> Result<(), Error> {
    // with the `embassy-time` feature: `Mfrc522::new(interface)`
    let mut rfid = Mfrc522::with_clock(interface, now_us);

    rfid.init().await?;
    log::info!("version: {:?}", rfid.version().await?);

    loop {
        if rfid.is_new_card_present().await? {
            let uid = rfid.select().await?;
            log::info!("UID: {:02X?} ({:?})", uid.as_bytes(), uid.card_type());

            rfid.authenticate(KeyType::A, 4, &Key::DEFAULT, &uid).await?;
            let block = rfid.read_block(4).await?;
            log::info!("block 4: {block:02X?}");
            // rfid.dump_card(&uid).await?; // logs the whole card with log::debug

            rfid.halt().await?;
            rfid.stop_crypto1().await?;
        }
        // wait a bit here (e.g. `embassy_time::Timer::after_millis(1).await`)
    }
}
```

Full firmware example (esp32s3 / esp32c3): [`examples/`](examples/).
Upgrading from 0.3: see [MIGRATING.md](MIGRATING.md).

## TODO
- [x] Change some functions to be more "rust-like"
- [x] Documentation in code
- [x] Crates.io publish
