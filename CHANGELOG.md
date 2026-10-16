# Changelog

## 0.4.0

Breaking: new typed API, see [MIGRATING.md](MIGRATING.md).

- `Mfrc522` with `init`, `is_new_card_present -> bool`, `select -> Uid`, `authenticate(KeyType, block, &Key, &Uid)`, `read_block -> [u8; 16]`, `write_block(&[u8; 16])`, `write_page`, value-block ops.
- `Error` enum with `Display`; `Uid`, `Key`, `KeyType`, `CardType`, `Version`, `AntennaGain` types.
- `SpiInterface` / `I2cInterface` implementing the `Interface` trait (slice-based register I/O).
- `Mfrc522::with_clock` always available; `Mfrc522::new` added by `embassy-time` (features are additive now).
- Fixed: `increment` / `decrement` / `restore` always failed with `Timeout`.
- Removed: `pcd_selftest` (stub), `UidSize`, `MFRC522Debug`, public raw command/register constants.

## 0.3.5

- Bounds checks everywhere (no panics on bad card/bus data), correct value-block encoding, collision counts as card present.
