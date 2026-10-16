# Migrating from 0.3 to 0.4

0.4 replaces the Arduino-style API (`pcd_*`, `picc_*`, `mifare_*`, raw command bytes,
out-parameters) with a typed one. The register sequences and timings are unchanged.

## Construction

| 0.3 | 0.4 |
|---|---|
| `esp_hal_mfrc522::MFRC522` | `esp_hal_mfrc522::Mfrc522` |
| `drivers::SpiDriver::new(spi)` | `SpiInterface::new(spi)` |
| `drivers::I2CDriver::new(i2c, 0x28)` | `I2cInterface::new(i2c, DEFAULT_I2C_ADDRESS)` |
| `MFRC522::new(driver)` (`embassy-time`) | `Mfrc522::new(interface)` (`embassy-time`) |
| `MFRC522::new(driver, now_us)` (no `embassy-time`) | `Mfrc522::with_clock(interface, now_us)` (always available) |
| trait `MfrcDriver` | trait `Interface` |

```rust
// 0.3
let mut mfrc522 = MFRC522::new(drivers::I2CDriver::new(i2c, 0x28));
// 0.4
let mut rfid = Mfrc522::new(I2cInterface::new(i2c, DEFAULT_I2C_ADDRESS));
```

`new` no longer changes its signature with the `embassy-time` feature (features must be additive):
enabling the feature now only adds `new`.

Custom `Interface` implementations: `write_reg_buff(reg, count, values)` became
`write_regs(reg, values)` and `read_reg_buff(reg, count, out, rx_align)` became
`read_regs(reg, out, rx_align)`; the slice length is the count.

## Errors

`consts::PCDErrorCode` is now `Error` (implements `Display` and `core::error::Error`).

| 0.3 | 0.4 |
|---|---|
| `Error` | `Communication` |
| `NoRoom` | `BufferTooSmall` |
| `Invalid` | `InvalidArgument` |
| `CrcWrong` | `Crc` |
| `MifareNack` | `Nack` |
| `InternalError` | `Internal` |
| `SpiError(kind)` / `I2cError(kind)` | `Spi(kind)` / `I2c(kind)` |
| `Unknown`, `DriverError` | removed (never returned) |
| `Collision`, `Timeout` | unchanged |

## Reader (PCD)

| 0.3 | 0.4 |
|---|---|
| `pcd_init()` | `init()` |
| `pcd_is_init() -> bool` | `is_initialized() -> Result<bool, Error>` |
| `pcd_reset()` | `reset()` |
| `pcd_antenna_on/off()` | `antenna_on/off()` |
| `pcd_get_antenna_gain() -> u8` | `antenna_gain() -> AntennaGain` |
| `pcd_set_antenna_gain(mask: u8)` | `set_antenna_gain(AntennaGain)` |
| `pcd_get_version() -> PCDVersion` | `version() -> Version` (`Unknown(u8)` keeps the raw byte) |
| `pcd_soft_power_down/up()` | `power_down/up()` |
| `pcd_stop_crypto1()` | `stop_crypto1()` |
| `pcd_ntag216_auth(pwd)` | `ntag_pwd_auth(pwd)` |
| `pcd_selftest()` | removed (always returned `false`) |
| `pcd_transceive_data`, `pcd_communicate_with_picc`, `pcd_mifare_transceive`, `pcd_calc_crc*`, `pcd_set/clear_register_bit_mask`, `write_reg*`/`read_reg*` | internal; `read_register(reg)` / `write_register(reg, val)` stay public for raw access |

```rust
// 0.3
if !mfrc522.pcd_is_init().await { /* ... */ }
// 0.4
if !matches!(rfid.is_initialized().await, Ok(true)) { /* ... */ }
```

## Cards (PICC)

| 0.3 | 0.4 |
|---|---|
| `picc_is_new_card_present() -> Result<(), _>` (`Err` = no card) | `is_new_card_present() -> Result<bool, Error>` (`Ok(false)` = no card, `Err` = bus error) |
| `get_card(UidSize::Four) -> Uid` | `select() -> Uid` (the card decides the UID length) |
| `picc_select(&mut uid, valid_bits)` | `select()` |
| `picc_request_a(&mut atqa, &mut size)` | `request_a() -> [u8; 2]` |
| `picc_wakeup_a(&mut atqa, &mut size)` | `wakeup_a() -> [u8; 2]` |
| `picc_halta()` | `halt()` |
| `consts::UidSize` | removed |

```rust
// 0.3
if mfrc522.picc_is_new_card_present().await.is_ok() {
    let card = mfrc522.get_card(UidSize::Four).await?;
}
// 0.4
if rfid.is_new_card_present().await? {
    let uid = rfid.select().await?;
}
```

### `Uid`

Fields are private now (a `Uid` always holds a valid 4/7/10-byte UID).

| 0.3 | 0.4 |
|---|---|
| `uid.uid_bytes[..uid.size as usize]` | `uid.as_bytes()` |
| `uid.sak` | `uid.sak()` |
| `PICCType::from_sak(uid.sak)` | `uid.card_type()` (`CardType`) |
| `uid.get_number()` | `uid.to_u128()` (same value) |

`PICCType::PiccType*` variants are `CardType::*` without the prefix (`PiccTypeMifareUL` →
`MifareUltralight`, `PiccTypeTnp3XXX` → `Tnp3xxx`; `PiccTypeMifareDesfire` was never produced and
is gone).

## MIFARE

| 0.3 | 0.4 |
|---|---|
| `pcd_authenticate(PICCCommand::PICC_CMD_MF_AUTH_KEY_A, block, &[u8], &uid)` | `authenticate(KeyType::A, block, &Key([..]), &uid)` |
| `mifare_read(block, &mut buf, &mut size)` (18-byte buffer incl. CRC) | `read_block(block) -> [u8; 16]` |
| `mifare_write(block, &buf, 16)` | `write_block(block, &[u8; 16])` |
| `mifare_ultralight_write(page, &mut buf, 4)` | `write_page(page, &[u8; 4])` |
| `mifare_get_value(block) -> u32` | `read_value(block) -> i32` |
| `mifare_set_value(block, u32)` | `write_value(block, i32)` |
| `mifare_increment/decrement(block, delta)` | `increment/decrement(block, delta)` |
| `mifare_restore(block)` / `mifare_transfer(block)` | `restore(block)` / `transfer(block)` |
| `mifare_two_step_helper(cmd, ..)` | internal |
| `mifare_calculate_access_bits(&mut buf, g0, g1, g2, g3)` | `access_bits(g0, g1, g2, g3) -> [u8; 3]` (free function) |
| `consts::PICCCommand`, `PCDCommand`, `PCDRegister` | internal |

```rust
// 0.3
mfrc522.pcd_authenticate(PICCCommand::PICC_CMD_MF_AUTH_KEY_A, 63, &[0xFF; 6], &card).await?;
let mut buf = [0; 18];
let mut size = 18;
mfrc522.mifare_read(62, &mut buf, &mut size).await?;
mfrc522.mifare_write(62, &buf, 16).await?;
// 0.4
rfid.authenticate(KeyType::A, 63, &Key::DEFAULT, &uid).await?;
let mut block = rfid.read_block(62).await?;
rfid.write_block(62, &block).await?;
```

## Debug dumps

The `MFRC522Debug` trait and the empty `test()` are gone; `debug_dump_card*` are plain methods:
`dump_card(&uid)`, `dump_card_details(&uid)` (no longer async), `dump_card_memory(&uid)`.

## Behaviour fixes

- `increment`, `decrement` and `restore` work now: the card never ACKs their operand step, 0.3
  treated that silence as a timeout error.
