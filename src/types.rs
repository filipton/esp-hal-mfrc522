//! Public value types: card UID, keys, card/chip identification and errors.

use core::fmt;

/// UID of a selected card (4, 7 or 10 bytes) and its SAK byte.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Uid {
    pub(crate) bytes: [u8; 10],
    pub(crate) len: u8,
    pub(crate) sak: u8,
}

impl Uid {
    /// The UID bytes (4, 7 or 10 of them).
    pub fn as_bytes(&self) -> &[u8] {
        let len = (self.len as usize).min(self.bytes.len());
        &self.bytes[..len]
    }

    /// Select acknowledge byte returned by the card.
    pub fn sak(&self) -> u8 {
        self.sak
    }

    /// Card type derived from the SAK byte.
    pub fn card_type(&self) -> CardType {
        CardType::from_sak(self.sak)
    }

    /// UID bytes interpreted as a little-endian number (same value as 0.3's `Uid::get_number`).
    pub fn to_u128(&self) -> u128 {
        let mut bytes = [0; 16];
        let uid = self.as_bytes();
        bytes[..uid.len()].copy_from_slice(uid);
        u128::from_le_bytes(bytes)
    }
}

/// MIFARE Classic key (6 bytes).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Key(pub [u8; 6]);

impl Key {
    /// Factory default key (`FF FF FF FF FF FF`).
    pub const DEFAULT: Key = Key([0xFF; 6]);
}

/// Which sector key to authenticate with.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KeyType {
    A,
    B,
}

/// Card family, as reported by the SAK byte.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CardType {
    /// ISO/IEC 14443-4 compliant.
    Iso14443_4,
    /// ISO/IEC 18092 (NFC) compliant.
    Iso18092,
    /// MIFARE Classic Mini (320 bytes).
    MifareMini,
    /// MIFARE Classic 1K.
    Mifare1K,
    /// MIFARE Classic 4K.
    Mifare4K,
    /// MIFARE Ultralight / Ultralight C / NTAG.
    MifareUltralight,
    /// MIFARE Plus.
    MifarePlus,
    /// TNP3XXX (NXP AN 10833).
    Tnp3xxx,
    /// SAK says the UID is not complete.
    NotComplete,
    Unknown,
}

impl CardType {
    pub fn from_sak(sak: u8) -> Self {
        match sak & 0x7F {
            0x20 => Self::Iso14443_4,
            0x40 => Self::Iso18092,
            0x09 => Self::MifareMini,
            0x08 => Self::Mifare1K,
            0x18 => Self::Mifare4K,
            0x00 => Self::MifareUltralight,
            0x10 | 0x11 => Self::MifarePlus,
            0x01 => Self::Tnp3xxx,
            0x04 => Self::NotComplete,
            _ => Self::Unknown,
        }
    }
}

/// Chip version read from `VersionReg`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Version {
    /// Known clone that reports 0x12.
    Counterfeit,
    Fm17522,
    Fm17522_1,
    Fm17522E,
    V0_0,
    V1_0,
    V2_0,
    Unknown(u8),
}

impl Version {
    pub(crate) fn from_byte(byte: u8) -> Self {
        match byte {
            0x12 => Self::Counterfeit,
            0x88 => Self::Fm17522,
            0xB2 => Self::Fm17522_1,
            0x89 => Self::Fm17522E,
            0x90 => Self::V0_0,
            0x91 => Self::V1_0,
            0x92 => Self::V2_0,
            other => Self::Unknown(other),
        }
    }
}

/// Receiver gain (`RFCfgReg.RxGain`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AntennaGain {
    Db18 = 0x00,
    Db23 = 0x10,
    Db18b = 0x20,
    Db23b = 0x30,
    Db33 = 0x40,
    Db38 = 0x50,
    Db43 = 0x60,
    Db48 = 0x70,
}

impl AntennaGain {
    pub(crate) fn from_bits(bits: u8) -> Self {
        match bits & 0x70 {
            0x00 => Self::Db18,
            0x10 => Self::Db23,
            0x20 => Self::Db18b,
            0x30 => Self::Db23b,
            0x40 => Self::Db33,
            0x50 => Self::Db38,
            0x60 => Self::Db43,
            _ => Self::Db48,
        }
    }
}

/// Errors returned by the driver.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// The chip reported a protocol, parity or buffer error.
    Communication,
    /// More than one card answered (bit collision).
    Collision,
    /// No answer in time (usually: no card in the field).
    Timeout,
    /// A buffer is too small for the data.
    BufferTooSmall,
    /// An argument is out of range.
    InvalidArgument,
    /// CRC_A of the answer does not match.
    Crc,
    /// The card answered with NAK (e.g. wrong key or access bits).
    Nack,
    /// Unexpected state inside the anticollision algorithm.
    Internal,
    /// SPI bus error.
    Spi(embedded_hal_async::spi::ErrorKind),
    /// I2C bus error.
    I2c(embedded_hal_async::i2c::ErrorKind),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Communication => f.write_str("communication error"),
            Self::Collision => f.write_str("card collision"),
            Self::Timeout => f.write_str("timeout"),
            Self::BufferTooSmall => f.write_str("buffer too small"),
            Self::InvalidArgument => f.write_str("invalid argument"),
            Self::Crc => f.write_str("CRC mismatch"),
            Self::Nack => f.write_str("card answered NAK"),
            Self::Internal => f.write_str("internal error"),
            Self::Spi(kind) => write!(f, "SPI error: {kind:?}"),
            Self::I2c(kind) => write!(f, "I2C error: {kind:?}"),
        }
    }
}

impl core::error::Error for Error {}
