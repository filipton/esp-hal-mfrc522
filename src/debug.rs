//! Card dumps to `log::debug!` (port of the Arduino library's `PICC_DumpToSerial`).

use core::fmt::Write;

use crate::{CardType, Error, Interface, Key, KeyType, Mfrc522, Uid};
use heapless::String;

impl<I: Interface> Mfrc522<I> {
    /// Logs UID/SAK/type, then the memory (Classic sectors with the default key, or Ultralight
    /// pages), then halts the card.
    pub async fn dump_card(&mut self, uid: &Uid) -> Result<(), Error> {
        self.dump_card_details(uid);
        log::debug!("");
        self.dump_card_memory(uid).await
    }

    /// Logs UID, SAK and card type.
    pub fn dump_card_details(&self, uid: &Uid) {
        let mut line: String<32> = String::new();
        for byte in uid.as_bytes() {
            _ = write!(line, " {byte:02X}");
        }

        log::debug!("Card UID:{line}");
        log::debug!("Card SAK: 0x{:02X}", uid.sak());
        log::debug!("PICC Type: {:?}", uid.card_type());
    }

    /// Logs the card memory and halts the card.
    pub async fn dump_card_memory(&mut self, uid: &Uid) -> Result<(), Error> {
        let sectors = match uid.card_type() {
            CardType::MifareMini => Some(5),
            CardType::Mifare1K => Some(16),
            CardType::Mifare4K => Some(40),
            CardType::MifareUltralight => None,
            CardType::Unknown | CardType::NotComplete => return Ok(()),
            other => {
                log::warn!("Dumping memory not implemented for: {other:?}");
                return self.halt().await;
            }
        };

        match sectors {
            Some(sectors) => {
                log::debug!(
                    "Sector Block   0  1  2  3   4  5  6  7   8  9 10 11  12 13 14 15  AccessBits"
                );
                for sector in (0..sectors).rev() {
                    if let Err(e) = self.dump_classic_sector(uid, &Key::DEFAULT, sector).await {
                        log::error!("Sector dump error ({sector}): {e:?}");
                    }
                }
                self.halt().await?;
                self.stop_crypto1().await
            }
            None => {
                if let Err(e) = self.dump_ultralight().await {
                    log::error!("Dump mifare ultralight failed: {e:?}");
                }
                self.halt().await
            }
        }
    }

    async fn dump_classic_sector(&mut self, uid: &Uid, key: &Key, sector: u8) -> Result<(), Error> {
        // sectors 0..32 have 4 blocks, 32..40 (4K only) have 16
        let (first_block, blocks) = match sector {
            0..32 => (sector * 4, 4),
            32..40 => (128 + (sector - 32) * 16, 16),
            _ => return Err(Error::InvalidArgument),
        };

        self.authenticate(KeyType::A, first_block, key, uid).await?;

        let mut groups = [0u8; 4];
        let mut inverted_error = false;
        let mut line: String<128> = String::new();
        for offset in (0..blocks).rev() {
            line.clear();
            let is_trailer = offset == blocks - 1;
            if is_trailer {
                _ = write!(line, "  {sector: >2}    ");
            } else {
                _ = line.push_str("        ");
            }

            let block = first_block + offset;
            _ = write!(line, "{block: >3}   ");
            let data = self.read_block(block).await?;
            for (i, byte) in data.iter().enumerate() {
                _ = write!(line, "{byte:02X} ");
                if i % 4 == 3 {
                    _ = line.push(' ');
                }
            }

            if is_trailer {
                let (c1, c2, c3) = (data[7] >> 4, data[8] & 0xF, data[8] >> 4);
                let (c1n, c2n, c3n) = (data[6] & 0xF, data[6] >> 4, data[7] & 0xF);
                inverted_error = c1 != (!c1n & 0xF) || c2 != (!c2n & 0xF) || c3 != (!c3n & 0xF);
                groups[0] = ((c1 & 1) << 2) | ((c2 & 1) << 1) | (c3 & 1);
                groups[1] = ((c1 & 2) << 1) | (c2 & 2) | ((c3 & 2) >> 1);
                groups[2] = (c1 & 4) | ((c2 & 4) >> 1) | ((c3 & 4) >> 2);
                groups[3] = ((c1 & 8) >> 1) | ((c2 & 8) >> 2) | ((c3 & 8) >> 3);
            }

            let (group, first_in_group) = if blocks == 4 {
                (offset, true)
            } else {
                let group = offset / 5;
                (group, group == 3 || group != (offset + 1) / 5)
            };
            let bits = groups[group as usize];

            if first_in_group {
                _ = write!(
                    line,
                    "[ {} {} {} ]",
                    (bits >> 2) & 1,
                    (bits >> 1) & 1,
                    bits & 1
                );
                if inverted_error {
                    _ = line.push_str(" Inverted access bits did not match! ");
                }
            }

            if group != 3 && (bits == 1 || bits == 6) {
                let value = i32::from_le_bytes([data[0], data[1], data[2], data[3]]);
                _ = write!(line, " Value=0x{value:X} Adr=0x{:X}", data[12]);
            }

            log::debug!("{line}");
        }

        Ok(())
    }

    async fn dump_ultralight(&mut self) -> Result<(), Error> {
        log::debug!("Page  0  1  2  3");
        let mut line: String<128> = String::new();
        for first_page in (0..16u8).step_by(4) {
            let data = self.read_block(first_page).await?;
            for (page, bytes) in (first_page..).zip(data.as_chunks::<4>().0) {
                line.clear();
                _ = write!(line, " {page: >2}  ");
                for byte in bytes {
                    _ = write!(line, " {byte:02X} ");
                }
                log::debug!("{line}");
            }
        }
        Ok(())
    }
}
