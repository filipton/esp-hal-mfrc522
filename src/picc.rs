//! Card (PICC) side of ISO 14443-3A: request/wake-up, anticollision + select, halt.

use crate::{
    Error, Interface, Mfrc522, Uid,
    consts::{PCDRegister, PICCCommand},
};

impl<I: Interface> Mfrc522<I> {
    /// Whether a card in IDLE state is in the field (`REQA`). Halted cards are not reported.
    ///
    /// `Ok(false)` means no card answered; `Err` is a bus/chip error.
    pub async fn is_new_card_present(&mut self) -> Result<bool, Error> {
        self.write_reg(PCDRegister::TxModeReg, 0x00).await?;
        self.write_reg(PCDRegister::RxModeReg, 0x00).await?;
        self.write_reg(PCDRegister::ModWidthReg, 0x26).await?;

        match self.request_a().await {
            // a collision still means at least one card is there (select resolves it)
            Ok(_) | Err(Error::Collision) => Ok(true),
            Err(Error::Timeout) => Ok(false),
            Err(e) => Err(e),
        }
    }

    /// `REQA`: wakes cards in IDLE state; returns the ATQA.
    pub async fn request_a(&mut self) -> Result<[u8; 2], Error> {
        self.reqa_or_wupa(PICCCommand::PICC_CMD_REQA).await
    }

    /// `WUPA`: wakes cards in IDLE and HALT state; returns the ATQA.
    pub async fn wakeup_a(&mut self) -> Result<[u8; 2], Error> {
        self.reqa_or_wupa(PICCCommand::PICC_CMD_WUPA).await
    }

    async fn reqa_or_wupa(&mut self, cmd: u8) -> Result<[u8; 2], Error> {
        self.clear_bits(PCDRegister::CollReg, 0x80).await?;

        let mut atqa = [0; 2];
        // short frame: 7 bits
        let (len, valid_bits) = self
            .transceive(&[cmd], 7, Some(&mut atqa), 0, false)
            .await?;
        if len != 2 || valid_bits != 0 {
            return Err(Error::Communication);
        }
        Ok(atqa)
    }

    /// `HLTA`: puts the selected card into HALT state.
    pub async fn halt(&mut self) -> Result<(), Error> {
        let mut buf = [PICCCommand::PICC_CMD_HLTA, 0, 0, 0];
        let crc = self.calc_crc(&buf[..2]).await?;
        buf[2..].copy_from_slice(&crc);

        // success is the card staying silent
        match self.transceive(&buf, 0, None, 0, false).await {
            Ok(_) => Err(Error::Communication),
            Err(Error::Timeout) => Ok(()),
            Err(e) => Err(e),
        }
    }

    /// Anticollision + select of one card (any UID length); call after
    /// [`Self::is_new_card_present`] or [`Self::wakeup_a`].
    pub async fn select(&mut self) -> Result<Uid, Error> {
        let mut uid = Uid {
            bytes: [0; 10],
            len: 0,
            sak: 0,
        };
        let mut cascade_level = 1u8;
        let mut buff = [0u8; 9];

        self.clear_bits(PCDRegister::CollReg, 0x80).await?;

        loop {
            let uid_index: usize;
            (buff[0], uid_index) = match cascade_level {
                1 => (PICCCommand::PICC_CMD_SEL_CL1, 0),
                2 => (PICCCommand::PICC_CMD_SEL_CL2, 3),
                3 => (PICCCommand::PICC_CMD_SEL_CL3, 6),
                _ => return Err(Error::Internal),
            };

            let mut known_bits = 0u8;
            let mut tx_last_bits;
            let mut response_ptr;
            let mut response_len;
            loop {
                let buffer_used;
                if known_bits >= 32 {
                    // SELECT: NVB 0x70, full UID part, BCC, CRC
                    buff[1] = 0x70;
                    buff[6] = buff[2] ^ buff[3] ^ buff[4] ^ buff[5];
                    let crc = self.calc_crc(&buff[..7]).await?;
                    buff[7..9].copy_from_slice(&crc);

                    tx_last_bits = 0;
                    buffer_used = 9;
                    response_ptr = 6;
                    response_len = 3;
                } else {
                    // ANTICOLLISION with the bits known so far
                    tx_last_bits = known_bits % 8;
                    let index = 2 + known_bits / 8;
                    buff[1] = (index << 4) + tx_last_bits;
                    buffer_used = index + u8::from(tx_last_bits != 0);
                    response_ptr = index;
                    response_len = 9 - index;
                }

                let send = buff;
                let rx_align = tx_last_bits;
                let back = &mut buff[response_ptr as usize..(response_ptr + response_len) as usize];
                match self
                    .transceive(
                        &send[..buffer_used as usize],
                        tx_last_bits,
                        Some(back),
                        rx_align,
                        false,
                    )
                    .await
                {
                    Ok((len, valid_bits)) => {
                        response_len = len as u8;
                        tx_last_bits = valid_bits;
                        if known_bits >= 32 {
                            break;
                        }
                        known_bits = 32;
                    }
                    Err(Error::Collision) => {
                        let coll = self.read_reg(PCDRegister::CollReg).await?;
                        if coll & 0x20 != 0 {
                            // CollPosNotValid
                            return Err(Error::Collision);
                        }
                        let mut collision_pos = coll & 0x1F;
                        if collision_pos == 0 {
                            collision_pos = 32;
                        }
                        if collision_pos <= known_bits {
                            return Err(Error::Internal);
                        }

                        // pick the card with a 1 at the collision bit
                        known_bits = collision_pos;
                        let check_bit = (known_bits - 1) % 8;
                        let index = 1 + known_bits / 8 + u8::from(!known_bits.is_multiple_of(8));
                        buff[index as usize] |= 1 << check_bit;
                    }
                    Err(e) => return Err(e),
                }
            }

            // UID bytes of this level (skip the cascade tag)
            let (from, count) = if buff[2] == PICCCommand::PICC_CMD_CT {
                (3, 3)
            } else {
                (2, 4)
            };
            uid.bytes[uid_index..uid_index + count].copy_from_slice(&buff[from..from + count]);

            // SAK: 1 byte + CRC
            if response_len != 3 || tx_last_bits != 0 {
                return Err(Error::Communication);
            }
            let ptr = response_ptr as usize;
            let crc = self.calc_crc(&buff[ptr..ptr + 1]).await?;
            if crc != [buff[ptr + 1], buff[ptr + 2]] {
                return Err(Error::Crc);
            }

            let sak = buff[ptr];
            if sak & 0x04 != 0 {
                cascade_level += 1;
            } else {
                uid.sak = sak;
                uid.len = 3 * cascade_level + 1;
                return Ok(uid);
            }
        }
    }
}
