//! MIFARE Classic / Ultralight block commands (authenticate first for Classic).

use crate::{Error, Interface, Mfrc522, consts::PICCCommand};

impl<I: Interface> Mfrc522<I> {
    /// Reads 16 bytes: one Classic block, or 4 Ultralight pages starting at `block`.
    pub async fn read_block(&mut self, block: u8) -> Result<[u8; 16], Error> {
        let mut cmd = [PICCCommand::PICC_CMD_MF_READ, block, 0, 0];
        let crc = self.calc_crc(&cmd[..2]).await?;
        cmd[2..].copy_from_slice(&crc);

        let mut back = [0; 18];
        let (len, _) = self.transceive(&cmd, 0, Some(&mut back), 0, true).await?;
        if len != 18 {
            return Err(Error::Communication);
        }

        let mut data = [0; 16];
        data.copy_from_slice(&back[..16]);
        Ok(data)
    }

    /// Writes one 16-byte Classic block (on Ultralight only the first 4 bytes are stored).
    pub async fn write_block(&mut self, block: u8, data: &[u8; 16]) -> Result<(), Error> {
        self.mifare_transceive(&[PICCCommand::PICC_CMD_MF_WRITE, block], false)
            .await?;
        self.mifare_transceive(data, false).await
    }

    /// Writes one 4-byte Ultralight / NTAG page.
    pub async fn write_page(&mut self, page: u8, data: &[u8; 4]) -> Result<(), Error> {
        let [a, b, c, d] = *data;
        self.mifare_transceive(&[PICCCommand::PICC_CMD_UL_WRITE, page, a, b, c, d], false)
            .await
    }

    /// Reads the value of a value block.
    pub async fn read_value(&mut self, block: u8) -> Result<i32, Error> {
        let data = self.read_block(block).await?;
        Ok(i32::from_le_bytes([data[0], data[1], data[2], data[3]]))
    }

    /// Formats `block` as a value block holding `value`.
    pub async fn write_value(&mut self, block: u8, value: i32) -> Result<(), Error> {
        self.write_block(block, &value_block(block, value)).await
    }

    /// Adds `delta` to a value block into the card's transfer buffer (commit with
    /// [`Self::transfer`]).
    pub async fn increment(&mut self, block: u8, delta: u32) -> Result<(), Error> {
        self.two_step(PICCCommand::PICC_CMD_MF_INCREMENT, block, delta)
            .await
    }

    /// Subtracts `delta` from a value block into the transfer buffer (commit with
    /// [`Self::transfer`]).
    pub async fn decrement(&mut self, block: u8, delta: u32) -> Result<(), Error> {
        self.two_step(PICCCommand::PICC_CMD_MF_DECREMENT, block, delta)
            .await
    }

    /// Copies a value block into the transfer buffer (commit with [`Self::transfer`]).
    pub async fn restore(&mut self, block: u8) -> Result<(), Error> {
        self.two_step(PICCCommand::PICC_CMD_MF_RESTORE, block, 0)
            .await
    }

    /// Writes the transfer buffer to `block`.
    pub async fn transfer(&mut self, block: u8) -> Result<(), Error> {
        self.mifare_transceive(&[PICCCommand::PICC_CMD_MF_TRANSFER, block], false)
            .await
    }

    async fn two_step(&mut self, cmd: u8, block: u8, operand: u32) -> Result<(), Error> {
        self.mifare_transceive(&[cmd, block], false).await?;
        // the card does not ACK the operand: silence is success
        self.mifare_transceive(&operand.to_le_bytes(), true).await
    }
}

/// Value block layout: value, !value, value, addr, !addr, addr, !addr.
pub(crate) fn value_block(block: u8, value: i32) -> [u8; 16] {
    let v = value.to_le_bytes();
    let inv = (!value).to_le_bytes();
    let mut data = [0; 16];
    data[0..4].copy_from_slice(&v);
    data[4..8].copy_from_slice(&inv);
    data[8..12].copy_from_slice(&v);
    data[12..16].copy_from_slice(&[block, !block, block, !block]);
    data
}

/// Access-condition bytes (6..9 of a sector trailer) for the access groups `g0..g3`
/// (each `0..=7`, bits `C1 C2 C3`).
pub fn access_bits(g0: u8, g1: u8, g2: u8, g3: u8) -> [u8; 3] {
    let c1 = ((g3 & 4) << 1) | (g2 & 4) | ((g1 & 4) >> 1) | ((g0 & 4) >> 2);
    let c2 = ((g3 & 2) << 2) | ((g2 & 2) << 1) | (g1 & 2) | ((g0 & 2) >> 1);
    let c3 = ((g3 & 1) << 3) | ((g2 & 1) << 2) | ((g1 & 1) << 1) | (g0 & 1);

    [
        ((!c2 & 0xF) << 4) | (!c1 & 0xF),
        (c1 << 4) | (!c3 & 0xF),
        (c3 << 4) | c2,
    ]
}
