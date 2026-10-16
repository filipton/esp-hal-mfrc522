use crate::{
    MFRC522, MfrcDriver,
    consts::{PCDErrorCode, PICCCommand},
};

impl<D> MFRC522<D>
where
    D: MfrcDriver,
{
    pub async fn mifare_read(
        &mut self,
        block_addr: u8,
        buff: &mut [u8],
        buff_size: &mut u8,
    ) -> Result<(), PCDErrorCode> {
        if *buff_size < 18 || buff.len() < 18 {
            return Err(PCDErrorCode::NoRoom);
        }

        let mut cmd = [PICCCommand::PICC_CMD_MF_READ, block_addr, 0, 0];
        self.pcd_calc_crc_single_buf(&mut cmd, 2, 2).await?;

        self.pcd_transceive_data(&cmd, 4, buff, Some(buff_size), None, 0, true)
            .await
    }

    pub async fn mifare_write(
        &mut self,
        block_addr: u8,
        buff: &[u8],
        buff_size: u8,
    ) -> Result<(), PCDErrorCode> {
        if buff_size < 16 {
            return Err(PCDErrorCode::Invalid);
        }

        let cmd_buff = [PICCCommand::PICC_CMD_MF_WRITE, block_addr];
        self.pcd_mifare_transceive(&cmd_buff, 2, false).await?;
        self.pcd_mifare_transceive(buff, buff_size, false).await?;

        Ok(())
    }

    pub async fn mifare_ultralight_write(
        &mut self,
        page: u8,
        buff: &mut [u8],
        buff_size: u8,
    ) -> Result<(), PCDErrorCode> {
        let Some(data) = buff.get(..4).filter(|_| buff_size >= 4) else {
            return Err(PCDErrorCode::Invalid);
        };

        let mut cmd_buff = [0; 6];
        cmd_buff[0] = PICCCommand::PICC_CMD_UL_WRITE;
        cmd_buff[1] = page;
        cmd_buff[2..].copy_from_slice(data);

        self.pcd_mifare_transceive(&cmd_buff, 6, false).await?;
        Ok(())
    }

    pub async fn mifare_transfer(&mut self, block_addr: u8) -> Result<(), PCDErrorCode> {
        let cmd_buff = [PICCCommand::PICC_CMD_MF_TRANSFER, block_addr];
        self.pcd_mifare_transceive(&cmd_buff, 2, false).await?;

        Ok(())
    }

    pub async fn mifare_two_step_helper(
        &mut self,
        cmd: u8,
        block_addr: u8,
        data: u32,
    ) -> Result<(), PCDErrorCode> {
        let cmd_buff = [cmd, block_addr];
        self.pcd_mifare_transceive(&cmd_buff, 2, false).await?;
        self.pcd_mifare_transceive(&data.to_le_bytes(), 4, false)
            .await?;

        Ok(())
    }

    pub async fn mifare_decrement(
        &mut self,
        block_addr: u8,
        delta: u32,
    ) -> Result<(), PCDErrorCode> {
        self.mifare_two_step_helper(PICCCommand::PICC_CMD_MF_DECREMENT, block_addr, delta)
            .await
    }

    pub async fn mifare_increment(
        &mut self,
        block_addr: u8,
        delta: u32,
    ) -> Result<(), PCDErrorCode> {
        self.mifare_two_step_helper(PICCCommand::PICC_CMD_MF_INCREMENT, block_addr, delta)
            .await
    }

    pub async fn mifare_restore(&mut self, block_addr: u8) -> Result<(), PCDErrorCode> {
        self.mifare_two_step_helper(PICCCommand::PICC_CMD_MF_RESTORE, block_addr, 0)
            .await
    }

    pub async fn mifare_get_value(&mut self, block_addr: u8) -> Result<u32, PCDErrorCode> {
        let mut buff = [0; 18];
        let mut size = 18;

        self.mifare_read(block_addr, &mut buff, &mut size).await?;
        Ok(u32::from_le_bytes([buff[0], buff[1], buff[2], buff[3]]))
    }

    pub async fn mifare_set_value(
        &mut self,
        block_addr: u8,
        value: u32,
    ) -> Result<(), PCDErrorCode> {
        // value block: value, !value, value, addr, !addr, addr, !addr
        let mut buff = [0; 16];
        buff[0..4].copy_from_slice(&value.to_le_bytes());
        buff[4..8].copy_from_slice(&(!value).to_le_bytes());
        buff[8..12].copy_from_slice(&value.to_le_bytes());

        buff[12] = block_addr;
        buff[14] = block_addr;
        buff[13] = !block_addr;
        buff[15] = !block_addr;

        self.mifare_write(block_addr, &buff, 16).await
    }

    pub async fn mifare_calculate_access_bits(
        &mut self,
        buff: &mut [u8],
        g0: u8,
        g1: u8,
        g2: u8,
        g3: u8,
    ) -> Result<(), PCDErrorCode> {
        let Some(buff) = buff.get_mut(..3) else {
            return Err(PCDErrorCode::NoRoom);
        };

        let c1 = ((g3 & 4) << 1) | (g2 & 4) | ((g1 & 4) >> 1) | ((g0 & 4) >> 2);
        let c2 = ((g3 & 2) << 2) | ((g2 & 2) << 1) | (g1 & 2) | ((g0 & 2) >> 1);
        let c3 = ((g3 & 1) << 3) | ((g2 & 1) << 2) | ((g1 & 1) << 1) | (g0 & 1);

        buff[0] = (!c2 & 0xF) << 4 | (!c1 & 0xF);
        buff[1] = c1 << 4 | (!c3 & 0xF);
        buff[2] = c3 << 4 | c2;

        Ok(())
    }
}
