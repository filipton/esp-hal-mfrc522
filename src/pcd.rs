//! Reader (PCD) control: init, antenna, power, authentication and the transceive core.

use crate::{
    AntennaGain, Error, Interface, Key, KeyType, Mfrc522, Uid, Version,
    consts::{PCDCommand, PCDRegister, PICCCommand},
};

/// Register values written by [`Mfrc522::init`] (checked by [`Mfrc522::is_initialized`]).
const INIT_REGS: [(u8, u8); 9] = [
    (PCDRegister::TxModeReg, 0x00),
    (PCDRegister::RxModeReg, 0x00),
    (PCDRegister::ModWidthReg, 0x26),
    // timer: 25 ms timeout (TPrescaler 0xA9 -> 40 kHz, reload 1000)
    (PCDRegister::TModeReg, 0x80),
    (PCDRegister::TPrescalerReg, 0xA9),
    (PCDRegister::TReloadRegH, 0x03),
    (PCDRegister::TReloadRegL, 0xE8),
    (PCDRegister::TxASKReg, 0x40), // 100% ASK
    (PCDRegister::ModeReg, 0x3D),  // CRC preset 0x6363
];

impl<I: Interface> Mfrc522<I> {
    /// Soft-resets the chip, configures timer/modulation and turns the antenna on.
    pub async fn init(&mut self) -> Result<(), Error> {
        self.reset().await?;
        for (reg, val) in INIT_REGS {
            self.write_reg(reg, val).await?;
        }
        self.antenna_on().await?;
        self.sleep_ms(4).await;
        Ok(())
    }

    /// Whether the chip still holds the configuration written by [`Self::init`] (detects a
    /// module that lost power or never answered).
    pub async fn is_initialized(&mut self) -> Result<bool, Error> {
        for (reg, val) in INIT_REGS {
            if self.read_reg(reg).await? != val {
                return Ok(false);
            }
        }
        Ok(true)
    }

    /// Soft reset; waits up to ~150 ms for the chip to wake up again.
    pub async fn reset(&mut self) -> Result<(), Error> {
        self.write_reg(PCDRegister::CommandReg, PCDCommand::SoftReset)
            .await?;

        for _ in 0..3 {
            if let Ok(cmd) = self.read_reg(PCDRegister::CommandReg).await
                && cmd & (1 << 4) == 0
            {
                break;
            }
            self.sleep_ms(50).await;
        }
        Ok(())
    }

    pub async fn antenna_on(&mut self) -> Result<(), Error> {
        let val = self.read_reg(PCDRegister::TxControlReg).await?;
        if val & 0x03 != 0x03 {
            self.write_reg(PCDRegister::TxControlReg, val | 0x03)
                .await?;
        }
        Ok(())
    }

    pub async fn antenna_off(&mut self) -> Result<(), Error> {
        self.clear_bits(PCDRegister::TxControlReg, 0x03).await
    }

    pub async fn antenna_gain(&mut self) -> Result<AntennaGain, Error> {
        let val = self.read_reg(PCDRegister::RFCfgReg).await?;
        Ok(AntennaGain::from_bits(val))
    }

    pub async fn set_antenna_gain(&mut self, gain: AntennaGain) -> Result<(), Error> {
        let val = self.read_reg(PCDRegister::RFCfgReg).await?;
        self.write_reg(PCDRegister::RFCfgReg, (val & !0x70) | gain as u8)
            .await
    }

    pub async fn version(&mut self) -> Result<Version, Error> {
        let byte = self.read_reg(PCDRegister::VersionReg).await?;
        Ok(Version::from_byte(byte))
    }

    /// Enters soft power-down (antenna and oscillator off).
    pub async fn power_down(&mut self) -> Result<(), Error> {
        self.set_bits(PCDRegister::CommandReg, 1 << 4).await
    }

    /// Leaves soft power-down; waits up to 500 ms for the oscillator.
    pub async fn power_up(&mut self) -> Result<(), Error> {
        self.clear_bits(PCDRegister::CommandReg, 1 << 4).await?;

        let start = (self.now_us)();
        while self.elapsed_us(start) < 500_000 {
            if self.read_reg(PCDRegister::CommandReg).await? & (1 << 4) == 0 {
                return Ok(());
            }
        }
        Err(Error::Timeout)
    }

    /// Ends the encrypted MIFARE Classic session started by [`Self::authenticate`].
    pub async fn stop_crypto1(&mut self) -> Result<(), Error> {
        self.clear_bits(PCDRegister::Status2Reg, 0x08).await
    }

    /// MIFARE Classic authentication of the sector containing `block`.
    pub async fn authenticate(
        &mut self,
        key_type: KeyType,
        block: u8,
        key: &Key,
        uid: &Uid,
    ) -> Result<(), Error> {
        let cmd = match key_type {
            KeyType::A => PICCCommand::PICC_CMD_MF_AUTH_KEY_A,
            KeyType::B => PICCCommand::PICC_CMD_MF_AUTH_KEY_B,
        };
        // the card checks the last 4 UID bytes
        let uid = uid.as_bytes();
        let uid_tail = uid
            .get(uid.len().saturating_sub(4)..)
            .filter(|t| t.len() == 4)
            .ok_or(Error::InvalidArgument)?;

        let mut send = [0; 12];
        send[0] = cmd;
        send[1] = block;
        send[2..8].copy_from_slice(&key.0);
        send[8..12].copy_from_slice(uid_tail);

        // wait for IdleIRq
        self.communicate(PCDCommand::MFAuthent, 0x10, &send, 0, None, 0, false)
            .await?;
        Ok(())
    }

    /// NTAG21x `PWD_AUTH`; returns the 2-byte PACK.
    pub async fn ntag_pwd_auth(&mut self, password: [u8; 4]) -> Result<[u8; 2], Error> {
        let mut send = [0; 7];
        send[0] = 0x1B;
        send[1..5].copy_from_slice(&password);
        let crc = self.calc_crc(&send[..5]).await?;
        send[5..].copy_from_slice(&crc);

        let mut back = [0; 5];
        let (len, _) = self.transceive(&send, 0, Some(&mut back), 0, false).await?;
        if len < 2 {
            return Err(Error::Communication);
        }
        Ok([back[0], back[1]])
    }

    /// Sends `send` (≤ 16 bytes) plus CRC and expects a 4-bit MIFARE ACK.
    pub(crate) async fn mifare_transceive(
        &mut self,
        send: &[u8],
        accept_timeout: bool,
    ) -> Result<(), Error> {
        let mut buf = [0; 18];
        let n = send.len();
        if n > 16 {
            return Err(Error::InvalidArgument);
        }
        buf[..n].copy_from_slice(send);
        let crc = self.calc_crc(send).await?;
        buf[n..n + 2].copy_from_slice(&crc);

        let mut back = [0; 18];
        let res = self
            .transceive(&buf[..n + 2], 0, Some(&mut back), 0, false)
            .await;
        let (len, valid_bits) = match res {
            Err(Error::Timeout) if accept_timeout => return Ok(()),
            res => res?,
        };

        if len != 1 || valid_bits != 4 {
            return Err(Error::Communication);
        }
        if back[0] != 0x0A {
            return Err(Error::Nack);
        }
        Ok(())
    }

    pub(crate) async fn transceive(
        &mut self,
        send: &[u8],
        tx_last_bits: u8,
        back: Option<&mut [u8]>,
        rx_align: u8,
        check_crc: bool,
    ) -> Result<(usize, u8), Error> {
        // wait for RxIRq or IdleIRq
        self.communicate(
            PCDCommand::Transceive,
            0x30,
            send,
            tx_last_bits,
            back,
            rx_align,
            check_crc,
        )
        .await
    }

    /// Runs `cmd` with `send` in the FIFO. Received bytes go to `back` (its length is the
    /// capacity). Returns `(received bytes, valid bits in the last byte)`.
    #[allow(clippy::too_many_arguments)]
    async fn communicate(
        &mut self,
        cmd: u8,
        wait_irq: u8,
        send: &[u8],
        tx_last_bits: u8,
        mut back: Option<&mut [u8]>,
        rx_align: u8,
        check_crc: bool,
    ) -> Result<(usize, u8), Error> {
        // FIFO is 64 bytes, bit counts are 0..=7
        if send.len() > 64 || rx_align > 7 || tx_last_bits > 7 {
            return Err(Error::InvalidArgument);
        }

        self.write_reg(PCDRegister::CommandReg, PCDCommand::Idle)
            .await?;
        self.write_reg(PCDRegister::ComIrqReg, 0x7F).await?;
        self.write_reg(PCDRegister::FIFOLevelReg, 0x80).await?;
        self.write_regs(PCDRegister::FIFODataReg, send).await?;
        self.write_reg(PCDRegister::BitFramingReg, (rx_align << 4) | tx_last_bits)
            .await?;
        self.write_reg(PCDRegister::CommandReg, cmd).await?;
        if cmd == PCDCommand::Transceive {
            self.set_bits(PCDRegister::BitFramingReg, 0x80).await?; // StartSend
        }

        let start = (self.now_us)();
        loop {
            let irq = self.read_reg(PCDRegister::ComIrqReg).await?;
            if irq & wait_irq != 0 {
                break;
            }
            // TimerIRq or 36 ms software deadline (the PCD timer is set to 25 ms in init)
            if irq & 0x01 != 0 || self.elapsed_us(start) >= 36_000 {
                return Err(Error::Timeout);
            }
        }

        let error = self.read_reg(PCDRegister::ErrorReg).await?;
        if error & 0x13 != 0 {
            // BufferOvfl, ParityErr, ProtocolErr
            return Err(Error::Communication);
        }

        let (len, valid_bits) = match back.as_deref_mut() {
            Some(back) => {
                let n = self.read_reg(PCDRegister::FIFOLevelReg).await? as usize;
                let dst = back.get_mut(..n).ok_or(Error::BufferTooSmall)?;
                self.read_regs(PCDRegister::FIFODataReg, dst, rx_align)
                    .await?;
                let valid_bits = self.read_reg(PCDRegister::ControlReg).await? & 0x07;
                (n, valid_bits)
            }
            None => (0, 0),
        };

        // the received bits are already in `back` (anticollision needs them)
        if error & 0x08 != 0 {
            return Err(Error::Collision);
        }

        if let Some(back) = back
            && check_crc
        {
            if len == 1 && valid_bits == 4 {
                return Err(Error::Nack);
            }
            if len < 2 || valid_bits != 0 {
                return Err(Error::Crc);
            }

            let (data, crc) = back[..len].split_at(len - 2);
            if self.calc_crc(data).await?[..] != *crc {
                return Err(Error::Crc);
            }
        }

        Ok((len, valid_bits))
    }

    /// CRC_A of `data` (≤ 64 bytes) computed by the chip's coprocessor.
    pub(crate) async fn calc_crc(&mut self, data: &[u8]) -> Result<[u8; 2], Error> {
        if data.len() > 64 {
            return Err(Error::InvalidArgument);
        }

        self.write_reg(PCDRegister::CommandReg, PCDCommand::Idle)
            .await?;
        self.write_reg(PCDRegister::DivIrqReg, 0x04).await?;
        self.write_reg(PCDRegister::FIFOLevelReg, 0x80).await?;
        self.write_regs(PCDRegister::FIFODataReg, data).await?;
        self.write_reg(PCDRegister::CommandReg, PCDCommand::CalcCRC)
            .await?;

        let start = (self.now_us)();
        while self.elapsed_us(start) < 89_000 {
            if self.read_reg(PCDRegister::DivIrqReg).await? & 0x04 != 0 {
                self.write_reg(PCDRegister::CommandReg, PCDCommand::Idle)
                    .await?;
                let lo = self.read_reg(PCDRegister::CRCResultRegL).await?;
                let hi = self.read_reg(PCDRegister::CRCResultRegH).await?;
                return Ok([lo, hi]);
            }
        }
        Err(Error::Timeout)
    }
}
