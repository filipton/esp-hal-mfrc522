use crate::{
    consts::{PCDErrorCode, PICCCommand},
    MFRC522,
};
use embedded_hal::digital::OutputPin;

#[allow(async_fn_in_trait)]
pub trait MFRC522Debug {
    async fn dump_to_serial(&mut self, uid: &[u8]) -> Result<(), PCDErrorCode>;
}

impl<S, C> MFRC522Debug for MFRC522<S, C>
where
    S: embedded_hal_async::spi::SpiBus,
    C: OutputPin,
{
    async fn dump_to_serial(&mut self, uid: &[u8]) -> Result<(), PCDErrorCode> {
        // for now only works onm MIFARE_1k
        // TODO: implement the rest
        if uid.len() != 4 {
            return Err(PCDErrorCode::Invalid);
        }

        let key = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF];
        let no_of_sectors = 16; // mifare 1k

        for sector in (0..no_of_sectors).rev() {
            let first_block;
            let no_of_blocks;

            if sector < 32 {
                no_of_blocks = 4;
                first_block = sector * no_of_blocks;
            } else if sector < 40 {
                no_of_blocks = 16;
                first_block = 128 + (sector - 32) * no_of_blocks;
            } else {
                return Err(PCDErrorCode::Invalid);
            }

            let mut is_sector_trailer = true;
            let mut buff = [0; 18];

            for block_offset in (0..no_of_blocks).rev() {
                let block_addr = first_block + block_offset;
                if is_sector_trailer {
                    self.pcd_authenticate(
                        PICCCommand::PICC_CMD_MF_AUTH_KEY_A,
                        first_block,
                        &key,
                        &uid,
                    )
                    .await?;
                }

                let mut byte_count = 18;
                self.mifare_read(block_addr, &mut buff, &mut byte_count)
                    .await?;
                log::debug!("S{sector} B{block_addr} {:?}", &buff[..16]);

                if is_sector_trailer {
                    is_sector_trailer = false;
                }
            }
        }

        // TODO: move this code from above to own function and then
        // if error occurs call picc_halta and pcd_stop_crypto1
        // to unlock next card read!
        self.picc_halta().await?;
        self.pcd_stop_crypto1().await?;
        Ok(())
    }
}
