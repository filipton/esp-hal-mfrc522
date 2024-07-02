#![no_std]

use consts::{PCDCommand, PCDErrorCode, PCDRegister, PCDVersion, PICCCommand};
use embedded_hal::digital::OutputPin;
use heapless::String;

pub mod consts;

/// assert return boolean (false)
macro_rules! assert_rb {
    ($expr:expr, $expected:expr) => {
        if $expr != $expected {
            return false;
        }
    };
}

pub struct MFRC522<S, C>
where
    S: embedded_hal_async::spi::SpiBus,
    C: OutputPin,
{
    spi: S,
    cs: C,
    read_buff: [u8; 1],

    get_current_time: fn() -> u64,
}

impl<S, C> MFRC522<S, C>
where
    S: embedded_hal_async::spi::SpiBus,
    C: OutputPin,
{
    pub fn new(spi: S, cs: C, get_current_time: fn() -> u64) -> Self {
        Self {
            spi,
            cs,
            read_buff: [0],
            get_current_time,
        }
    }

    pub async fn pcd_init(&mut self) -> Result<(), PCDErrorCode> {
        self.pcd_reset().await?;

        self.write_reg(PCDRegister::TxModeReg, 0x00).await?;
        self.write_reg(PCDRegister::RxModeReg, 0x00).await?;
        self.write_reg(PCDRegister::ModWidthReg, 0x26).await?;

        self.write_reg(PCDRegister::TModeReg, 0x80).await?;
        self.write_reg(PCDRegister::TPrescalerReg, 0xA9).await?;
        self.write_reg(PCDRegister::TReloadRegH, 0x03).await?;
        self.write_reg(PCDRegister::TReloadRegL, 0xE8).await?;

        self.write_reg(PCDRegister::TxASKReg, 0x40).await?;
        self.write_reg(PCDRegister::ModeReg, 0x3D).await?;

        self.pcd_antenna_on().await?;

        let start_time = (self.get_current_time)(); // microseconds
        while (self.get_current_time)() - start_time < 4_000 {}
        Ok(())
    }

    pub async fn pcd_is_init(&mut self) -> bool {
        assert_rb!(self.read_reg(PCDRegister::TxModeReg).await, Ok(0x00));
        assert_rb!(self.read_reg(PCDRegister::RxModeReg).await, Ok(0x00));
        assert_rb!(self.read_reg(PCDRegister::ModWidthReg).await, Ok(0x26));

        assert_rb!(self.read_reg(PCDRegister::TModeReg).await, Ok(0x80));
        assert_rb!(self.read_reg(PCDRegister::TPrescalerReg).await, Ok(0xA9));
        assert_rb!(self.read_reg(PCDRegister::TReloadRegH).await, Ok(0x03));
        assert_rb!(self.read_reg(PCDRegister::TReloadRegL).await, Ok(0xE8));

        assert_rb!(self.read_reg(PCDRegister::TxASKReg).await, Ok(0x40));
        assert_rb!(self.read_reg(PCDRegister::ModeReg).await, Ok(0x3D));
        true
    }

    pub async fn pcd_reset(&mut self) -> Result<(), PCDErrorCode> {
        self.spi.flush().await.map_err(|_| PCDErrorCode::Unknown)?;

        self.write_reg(PCDRegister::CommandReg, PCDCommand::SoftReset)
            .await?;

        // max 3 tries
        for _ in 0..3 {
            let out = self.read_reg(PCDRegister::CommandReg).await;
            if let Ok(out) = out {
                let out = out & (1 << 4);
                if out == 0 {
                    break;
                }
            }

            let start_time = (self.get_current_time)(); // microseconds
            while (self.get_current_time)() - start_time < 50_000 {}
        }

        Ok(())
    }

    pub async fn pcd_antenna_on(&mut self) -> Result<(), PCDErrorCode> {
        let val = self.read_reg(PCDRegister::TxControlReg).await?;
        if (val & 0x03) != 0x03 {
            self.write_reg(PCDRegister::TxControlReg, val | 0x03)
                .await?;
        }

        Ok(())
    }

    pub async fn pcd_antenna_off(&mut self) -> Result<(), PCDErrorCode> {
        self.pcd_clear_register_bit_mask(PCDRegister::TxControlReg, 0x03)
            .await
    }

    pub async fn pcd_get_antenna_gain(&mut self) -> Result<u8, PCDErrorCode> {
        let res = self.read_reg(PCDRegister::RFCfgReg).await?;
        Ok(res & (0x07 << 4))
    }

    pub async fn pcd_set_antenna_gain(&mut self, mask: u8) -> Result<(), PCDErrorCode> {
        if self.pcd_get_antenna_gain().await? != mask {
            self.pcd_clear_register_bit_mask(PCDRegister::RFCfgReg, 0x07 << 4)
                .await?;

            self.pcd_set_register_bit_mask(PCDRegister::RFCfgReg, mask & (0x07 << 4))
                .await?;
        }

        Ok(())
    }

    pub async fn pcd_get_version(&mut self) -> Result<PCDVersion, PCDErrorCode> {
        Ok(PCDVersion::from_byte(
            self.read_reg(PCDRegister::VersionReg).await?,
        ))
    }

    pub async fn pcd_soft_power_down(&mut self) -> Result<(), PCDErrorCode> {
        let mut val = self.read_reg(PCDRegister::CommandReg).await?;
        val |= 1 << 4;
        self.write_reg(PCDRegister::CommandReg, val).await?;

        Ok(())
    }

    pub async fn pcd_soft_power_up(&mut self) -> Result<(), PCDErrorCode> {
        let mut val = self.read_reg(PCDRegister::CommandReg).await?;
        val &= !(1 << 4);
        self.write_reg(PCDRegister::CommandReg, val).await?;

        let start_time = (self.get_current_time)();
        while (self.get_current_time)() - start_time < 500_000 {
            let val = self.read_reg(PCDRegister::CommandReg).await?;
            if val & (1 << 4) == 0 {
                return Ok(());
            }
        }

        Err(PCDErrorCode::Timeout)
    }

    pub async fn pcd_stop_crypto1(&mut self) -> Result<(), PCDErrorCode> {
        self.pcd_clear_register_bit_mask(PCDRegister::Status2Reg, 0x08)
            .await
    }

    pub async fn pcd_mifare_transceive(
        &mut self,
        send_data: &[u8],
        mut send_len: u8,
        accept_timeout: bool,
    ) -> Result<(), PCDErrorCode> {
        let mut cmd_buff = [0; 18];
        if send_len > 16 {
            return Err(PCDErrorCode::Invalid);
        }

        cmd_buff[..send_len as usize].copy_from_slice(&send_data[..send_len as usize]);
        self.pcd_calc_crc(
            &cmd_buff.clone(), // i cant make it like in C :(
            send_len,
            &mut cmd_buff[send_len as usize..],
        )
        .await?;

        send_len += 2;

        let wait_irq = 0x30;
        let mut cmd_buff_size = 18;
        let mut valid_bits = 0;

        let res = self
            .pcd_communicate_with_picc(
                PCDCommand::Transceive,
                wait_irq,
                &cmd_buff.clone(),
                send_len,
                &mut cmd_buff,
                &mut cmd_buff_size,
                &mut valid_bits,
                0,
                false,
            )
            .await;

        match res {
            Err(PCDErrorCode::Timeout) => {
                if accept_timeout {
                    return Ok(());
                }
            }
            Err(e) => return Err(e),
            Ok(_) => {}
        }

        if cmd_buff_size != 1 || valid_bits != 4 {
            return Err(PCDErrorCode::Error);
        }

        if cmd_buff[0] != 0xA {
            // MIFARE_Misc::MF_ACK type
            return Err(PCDErrorCode::MifareNack);
        }

        Ok(())
    }

    pub async fn picc_is_new_card_present(&mut self) -> Result<(), PCDErrorCode> {
        let mut buffer_atqa = [0; 2];
        let mut buffer_size = 2;

        self.write_reg(PCDRegister::TxModeReg, 0x00).await?;
        self.write_reg(PCDRegister::RxModeReg, 0x00).await?;
        self.write_reg(PCDRegister::ModWidthReg, 0x26).await?;

        self.picc_request_a(&mut buffer_atqa, &mut buffer_size)
            .await?;

        Ok(())
    }

    pub async fn picc_halta(&mut self) -> Result<(), PCDErrorCode> {
        let mut buff = [0; 4];
        buff[0] = PICCCommand::PICC_CMD_HLTA;
        buff[1] = 0;

        self.pcd_calc_crc(&buff.clone(), 2, &mut buff[2..]).await?;
        let mut void = [0; 16];
        let mut len_void = 0;
        let mut valid_bits_void = 0;
        let res = self
            .pcd_transceive_data(
                &buff,
                4,
                &mut void,
                &mut len_void,
                &mut valid_bits_void,
                0,
                false,
            )
            .await;

        // yes error timeout here is only Ok here
        match res {
            Ok(_) => Err(PCDErrorCode::Error),
            Err(PCDErrorCode::Timeout) => Ok(()),
            Err(e) => Err(e),
        }
    }

    pub async fn get_card_uid_4b(&mut self) -> Result<u32, PCDErrorCode> {
        let res = self.picc_select(4, [0; 10], 0).await?;
        Ok(u32::from_le_bytes([res[0], res[1], res[2], res[3]]))
    }

    pub async fn get_card_uid_7b(&mut self) -> Result<u64, PCDErrorCode> {
        let res = self.picc_select(7, [0; 10], 0).await?;
        Ok(u64::from_le_bytes([
            res[0], res[1], res[2], res[3], res[4], res[5], res[6], 0,
        ]))
    }

    pub async fn get_card_uid_10b(&mut self) -> Result<u128, PCDErrorCode> {
        let res = self.picc_select(4, [0; 10], 0).await?;
        Ok(u128::from_le_bytes([
            res[0], res[1], res[2], res[3], res[4], res[5], res[6], res[7], res[8], res[9], 0, 0,
            0, 0, 0, 0,
        ]))
    }

    pub async fn picc_select(
        &mut self,
        uid_bytes_count: u8,
        known_uid_bytes: [u8; 10],
        valid_bits: u8,
    ) -> Result<[u8; 10], PCDErrorCode> {
        let mut uid_bytes = [0; 10];

        let mut uid_complete = false;
        let mut use_casdcade_tag;
        let mut cascade_level = 1u8;
        let mut count: u8;
        let mut check_bit: u8;
        let mut index: u8;
        let mut uid_index: u8;
        let mut current_level_known_bits: i8;
        let mut buff = [0; 9];
        let mut buffer_used: u8;
        let mut rx_align: u8;
        let mut tx_last_bits = 0u8;
        let mut response_buff_ptr = 0;
        let mut response_length = 0u8;

        if valid_bits > 80 {
            return Err(PCDErrorCode::Invalid);
        }

        self.pcd_clear_register_bit_mask(PCDRegister::CollReg, 0x80)
            .await?;

        while !uid_complete {
            match cascade_level {
                1 => {
                    buff[0] = PICCCommand::PICC_CMD_SEL_CL1;
                    uid_index = 0;
                    use_casdcade_tag = valid_bits != 0 && (uid_bytes_count > 4);
                }
                2 => {
                    buff[0] = PICCCommand::PICC_CMD_SEL_CL2;
                    uid_index = 3;
                    use_casdcade_tag = valid_bits != 0 && (uid_bytes_count > 7);
                }
                3 => {
                    buff[0] = PICCCommand::PICC_CMD_SEL_CL3;
                    uid_index = 6;
                    use_casdcade_tag = false;
                }
                _ => {
                    return Err(PCDErrorCode::InternalError);
                }
            }

            current_level_known_bits = valid_bits as i8 - (8i8 * uid_index as i8);
            if current_level_known_bits < 0 {
                current_level_known_bits = 0;
            }

            index = 2;
            if use_casdcade_tag {
                buff[index as usize] = PICCCommand::PICC_CMD_CT;
                index += 1;
            }

            let mut bytes_to_copy =
                current_level_known_bits / 8 + tif(current_level_known_bits % 8 != 0, 1, 0);

            if bytes_to_copy != 0 {
                let max_bytes = if use_casdcade_tag { 3 } else { 4 };
                if bytes_to_copy > max_bytes {
                    bytes_to_copy = max_bytes;
                }

                for count in 0..bytes_to_copy as usize {
                    buff[index as usize] = known_uid_bytes[uid_index as usize + count];
                    index += 1;
                }
            }

            if use_casdcade_tag {
                current_level_known_bits += 8;
            }

            let mut select_done = false;
            while !select_done {
                if current_level_known_bits >= 32 {
                    buff[1] = 0x70;
                    buff[6] = buff[2] ^ buff[3] ^ buff[4] ^ buff[5];

                    self.pcd_calc_crc(&buff.clone(), 7, &mut buff[7..]).await?;

                    tx_last_bits = 0;
                    buffer_used = 9;
                    response_buff_ptr = 6;
                    response_length = 3;
                } else {
                    tx_last_bits = (current_level_known_bits % 8) as u8;
                    count = (current_level_known_bits / 8) as u8;
                    index = 2 + count;
                    buff[1] = (index << 4) + tx_last_bits;
                    buffer_used = index + tif(tx_last_bits != 0, 1, 0);

                    response_length = 9 - index;
                    response_buff_ptr = index;
                }

                rx_align = tx_last_bits;
                self.write_reg(PCDRegister::BitFramingReg, (rx_align << 4) + tx_last_bits)
                    .await?;

                let res = self
                    .pcd_transceive_data(
                        &buff.clone(),
                        buffer_used,
                        &mut buff[response_buff_ptr as usize..],
                        &mut response_length,
                        &mut tx_last_bits,
                        rx_align,
                        false,
                    )
                    .await;

                match res {
                    Ok(_) => {
                        if current_level_known_bits >= 32 {
                            select_done = true;
                        } else {
                            current_level_known_bits = 32;
                        }
                    }
                    Err(PCDErrorCode::Collision) => {
                        let value_of_coll_reg = self.read_reg(PCDRegister::CollReg).await?;
                        if value_of_coll_reg & 0x20 != 0 {
                            return Err(PCDErrorCode::Collision);
                        }

                        let mut collision_pos = value_of_coll_reg & 0x1F;
                        if collision_pos == 0 {
                            collision_pos = 32;
                        }

                        if collision_pos as i8 <= current_level_known_bits {
                            return Err(PCDErrorCode::InternalError);
                        }

                        current_level_known_bits = collision_pos as i8;
                        count = (current_level_known_bits % 8) as u8;
                        check_bit = ((current_level_known_bits - 1) % 8) as u8;
                        index = 1 + (current_level_known_bits / 8) as u8 + tif(count != 0, 1, 0);

                        buff[index as usize] |= 1 << check_bit;
                    }
                    Err(e) => {
                        return Err(e);
                    }
                }
            }

            index = tif(buff[2] == PICCCommand::PICC_CMD_CT, 3, 2);
            let bytes_to_copy = tif(buff[2] == PICCCommand::PICC_CMD_CT, 3, 4);

            for i in 0..bytes_to_copy {
                uid_bytes[uid_index as usize + i] = buff[index as usize];
                index += 1;
            }

            if response_length != 3 || tx_last_bits != 0 {
                return Err(PCDErrorCode::Error);
            }

            self.pcd_calc_crc(&[buff[response_buff_ptr as usize]], 1, &mut buff[2..])
                .await?;

            if (buff[2] != buff[response_buff_ptr as usize + 1])
                || (buff[3] != buff[response_buff_ptr as usize + 2])
            {
                return Err(PCDErrorCode::CrcWrong);
            }

            if buff[response_buff_ptr as usize + 0] & 0x04 != 0 {
                cascade_level += 1;
            } else {
                uid_complete = true;
            }
        }

        Ok(uid_bytes)
    }

    pub async fn picc_wakeup_a(
        &mut self,
        buffer_atqa: &mut [u8],
        buffer_size: &mut u8,
    ) -> Result<(), PCDErrorCode> {
        self.picc_reqa_or_wupa(PICCCommand::PICC_CMD_WUPA, buffer_atqa, buffer_size)
            .await
    }

    pub async fn picc_request_a(
        &mut self,
        buffer_atqa: &mut [u8],
        buffer_size: &mut u8,
    ) -> Result<(), PCDErrorCode> {
        self.picc_reqa_or_wupa(PICCCommand::PICC_CMD_REQA, buffer_atqa, buffer_size)
            .await
    }

    pub async fn picc_reqa_or_wupa(
        &mut self,
        cmd: u8,
        buffer_atqa: &mut [u8],
        buffer_size: &mut u8,
    ) -> Result<(), PCDErrorCode> {
        if *buffer_size < 2 {
            return Err(PCDErrorCode::NoRoom);
        }

        self.pcd_clear_register_bit_mask(PCDRegister::CollReg, 0x80)
            .await?;

        let mut valid_bits = 7;
        self.pcd_transceive_data(
            &[cmd],
            1,
            buffer_atqa,
            buffer_size,
            &mut valid_bits,
            0,
            false,
        )
        .await?;

        if *buffer_size != 2 || valid_bits != 0 {
            return Err(PCDErrorCode::Error);
        }

        Ok(())
    }

    pub async fn pcd_transceive_data(
        &mut self,
        send_data: &[u8],
        send_len: u8,
        back_data: &mut [u8],
        back_len: &mut u8,
        valid_bits: &mut u8,
        rx_align: u8,
        check_crc: bool,
    ) -> Result<(), PCDErrorCode> {
        let wait_irq = 0x30;
        self.pcd_communicate_with_picc(
            PCDCommand::Transceive,
            wait_irq,
            send_data,
            send_len,
            back_data,
            back_len,
            valid_bits,
            rx_align,
            check_crc,
        )
        .await
    }

    pub async fn pcd_communicate_with_picc(
        &mut self,
        cmd: u8,
        wait_irq: u8,
        send_data: &[u8],
        send_len: u8,
        back_data: &mut [u8],
        back_len: &mut u8,
        valid_bits: &mut u8,
        rx_align: u8,
        check_crc: bool,
    ) -> Result<(), PCDErrorCode> {
        let tx_last_bits = *valid_bits;
        let bit_framing = (rx_align << 4) + tx_last_bits;

        self.write_reg(PCDRegister::CommandReg, PCDCommand::Idle)
            .await?;

        self.write_reg(PCDRegister::ComIrqReg, 0x7F).await?;
        self.write_reg(PCDRegister::FIFOLevelReg, 0x80).await?;
        self.write_reg_buff(PCDRegister::FIFODataReg, send_len as usize, send_data)
            .await?;

        self.write_reg(PCDRegister::BitFramingReg, bit_framing)
            .await?;

        self.write_reg(PCDRegister::CommandReg, cmd).await?;

        if cmd == PCDCommand::Transceive {
            self.pcd_set_register_bit_mask(PCDRegister::BitFramingReg, 0x80)
                .await?;
        }

        let start_time = (self.get_current_time)();
        loop {
            let n = self.read_reg(PCDRegister::ComIrqReg).await?;
            if n & wait_irq != 0 {
                break;
            }

            if n & 0x01 != 0 || (self.get_current_time)() - start_time >= 36_000 {
                return Err(PCDErrorCode::Timeout);
            }
        }

        let error_reg_value = self.read_reg(PCDRegister::ErrorReg).await?;
        if error_reg_value & 0x13 != 0 {
            return Err(PCDErrorCode::Error);
        }

        let mut _valid_bits = 0;
        if *back_len != 0 {
            let n = self.read_reg(PCDRegister::FIFOLevelReg).await?;
            if n > *back_len {
                return Err(PCDErrorCode::NoRoom);
            }

            *back_len = n;
            self.read_reg_buff(PCDRegister::FIFODataReg, n as usize, back_data, rx_align)
                .await?;

            _valid_bits = self.read_reg(PCDRegister::ControlReg).await? & 0x07;
            if *valid_bits != 0 {
                *valid_bits = _valid_bits;
            }
        }

        if error_reg_value & 0x08 != 0 {
            return Err(PCDErrorCode::Collision);
        }

        if *back_len != 0 && check_crc {
            if *back_len == 1 && _valid_bits == 4 {
                return Err(PCDErrorCode::MifareNack);
            }

            if *back_len < 2 || _valid_bits != 0 {
                return Err(PCDErrorCode::CrcWrong);
            }

            let mut control_buff = [0; 2];
            self.pcd_calc_crc(&back_data, *back_len - 2, &mut control_buff)
                .await?;

            if (back_data[*back_len as usize - 2] != control_buff[0])
                || (back_data[*back_len as usize - 1] != control_buff[1])
            {
                return Err(PCDErrorCode::CrcWrong);
            }
        }

        Ok(())
    }

    /// Now it prints data to console, TODO: change this
    /// Always returns false (for now)
    pub async fn pcd_selftest(&mut self) -> Result<bool, PCDErrorCode> {
        log::debug!("Running PCD_Selftest!\n");

        self.write_reg(PCDRegister::FIFOLevelReg, 0x80).await?;
        self.write_reg_buff(PCDRegister::FIFODataReg, 25, &[0; 25])
            .await?;

        self.write_reg(PCDRegister::CommandReg, PCDCommand::Mem)
            .await?;

        self.write_reg(PCDRegister::AutoTestReg, 0x09).await?;
        self.write_reg(PCDRegister::FIFODataReg, 0x00).await?;
        self.write_reg(PCDRegister::CommandReg, PCDCommand::CalcCRC)
            .await?;

        for _ in 0..0xFF {
            let n = self.read_reg(PCDRegister::FIFOLevelReg).await?;
            if n >= 64 {
                break;
            }
        }

        self.write_reg(PCDRegister::CommandReg, PCDCommand::Idle)
            .await?;

        let mut res = [0; 64];
        self.read_reg_buff(PCDRegister::FIFODataReg, 64, &mut res, 0)
            .await?;

        self.write_reg(PCDRegister::AutoTestReg, 0x40 & 0x00)
            .await?;

        let mut str: String<128> = String::new();
        for i in 0..64 {
            if i % 8 == 0 && str.len() != 0 {
                log::debug!("{}", str);
                str.clear();
            }

            _ = core::fmt::write(&mut str, format_args!("{:#04x} ", res[i]));
        }
        log::debug!("{}", str);

        log::debug!("PCD_Selftest Done!\n");
        self.pcd_init().await?;
        Ok(false)
    }

    pub async fn pcd_clear_register_bit_mask(
        &mut self,
        reg: u8,
        mask: u8,
    ) -> Result<(), PCDErrorCode> {
        let tmp = self.read_reg(reg).await?;
        self.write_reg(reg, tmp & (!mask)).await?;

        Ok(())
    }

    pub async fn pcd_set_register_bit_mask(
        &mut self,
        reg: u8,
        mask: u8,
    ) -> Result<(), PCDErrorCode> {
        let tmp = self.read_reg(reg).await?;
        self.write_reg(reg, tmp | mask).await?;

        Ok(())
    }

    pub async fn pcd_calc_crc(
        &mut self,
        data: &[u8],
        length: u8,
        res: &mut [u8],
    ) -> Result<(), PCDErrorCode> {
        self.write_reg(PCDRegister::CommandReg, PCDCommand::Idle)
            .await?;

        self.write_reg(PCDRegister::DivIrqReg, 0x04).await?;
        self.write_reg(PCDRegister::FIFOLevelReg, 0x80).await?;
        self.write_reg_buff(PCDRegister::FIFODataReg, length as usize, data)
            .await?;

        self.write_reg(PCDRegister::CommandReg, PCDCommand::CalcCRC)
            .await?;

        let start_time = (self.get_current_time)();
        while (self.get_current_time)() - start_time < 89_000 {
            let n = self.read_reg(PCDRegister::DivIrqReg).await?;
            if n & 0x04 != 0 {
                self.write_reg(PCDRegister::CommandReg, PCDCommand::Idle)
                    .await?;

                res[0] = self.read_reg(PCDRegister::CRCResultRegL).await?;
                res[1] = self.read_reg(PCDRegister::CRCResultRegH).await?;
                return Ok(());
            }
        }

        Err(PCDErrorCode::Timeout)
    }

    pub async fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), PCDErrorCode> {
        self.cs.set_low().map_err(|_| PCDErrorCode::Unknown)?;
        self.spi_transfer(&[reg << 1]).await?;
        self.spi_transfer(&[val]).await?;
        self.cs.set_high().map_err(|_| PCDErrorCode::Unknown)?;

        Ok(())
    }

    pub async fn write_reg_buff(
        &mut self,
        reg: u8,
        count: usize,
        values: &[u8],
    ) -> Result<(), PCDErrorCode> {
        self.cs.set_low().map_err(|_| PCDErrorCode::Unknown)?;
        self.spi_transfer(&[reg << 1]).await?;

        for i in 0..count {
            self.spi_transfer(&[values[i]]).await?;
        }

        self.cs.set_high().map_err(|_| PCDErrorCode::Unknown)?;
        Ok(())
    }

    pub async fn read_reg(&mut self, reg: u8) -> Result<u8, PCDErrorCode> {
        self.cs.set_low().map_err(|_| PCDErrorCode::Unknown)?;
        self.spi_transfer(&[(reg << 1) | 0x80]).await?;
        self.spi_transfer(&[0]).await?;
        self.cs.set_high().map_err(|_| PCDErrorCode::Unknown)?;

        Ok(self.read_buff[0])
    }

    pub async fn read_reg_buff(
        &mut self,
        reg: u8,
        count: usize,
        output_buff: &mut [u8],
        rx_align: u8,
    ) -> Result<(), PCDErrorCode> {
        if count == 0 {
            return Ok(());
        }

        let addr = 0x80 | (reg << 1);
        let mut index = 0;

        self.cs.set_low().map_err(|_| PCDErrorCode::Unknown)?;
        self.spi_transfer(&[addr]).await?;

        if rx_align > 0 {
            let mask = (0xFF << rx_align) & 0xFF;
            self.spi_transfer(&[addr]).await?;

            output_buff[0] = (output_buff[0] & !mask) | (self.read_buff[0] & mask);
            index += 1;
        }

        while index < count - 1 {
            self.spi_transfer(&[addr]).await?;
            output_buff[index] = self.read_buff[0];
            index += 1;
        }

        self.spi_transfer(&[0]).await?;
        output_buff[index] = self.read_buff[0];

        self.cs.set_high().map_err(|_| PCDErrorCode::Unknown)?;
        Ok(())
    }

    async fn spi_transfer(&mut self, data: &[u8]) -> Result<(), PCDErrorCode> {
        self.spi
            .transfer(&mut self.read_buff, data)
            .await
            .map_err(|_| PCDErrorCode::Unknown)?;

        Ok(())
    }
}

#[inline(always)]
fn tif<T>(expr: bool, true_val: T, false_val: T) -> T {
    if expr {
        true_val
    } else {
        false_val
    }
}
