extern crate std;

use crate::{
    MFRC522, MfrcDriver,
    consts::{PCDErrorCode, PCDRegister, Uid},
};
use core::{
    future::Future,
    pin::pin,
    task::{Context, Poll, Waker},
};
use std::vec::Vec;

/// Register-map mock: reads return `regs[reg]`, FIFO reads return `fifo`, FIFO writes are recorded.
struct Mock {
    regs: [u8; 64],
    fifo: Vec<u8>,
    fifo_writes: Vec<Vec<u8>>,
}

impl Mock {
    /// Every command completes instantly, without errors, with `fifo` as the PICC answer.
    fn answering(fifo: &[u8], valid_bits: u8) -> Self {
        let mut regs = [0; 64];
        regs[PCDRegister::ComIrqReg as usize] = 0x30; // RxIRq | IdleIRq
        regs[PCDRegister::DivIrqReg as usize] = 0x04; // CRCIRq
        regs[PCDRegister::FIFOLevelReg as usize] = fifo.len() as u8;
        regs[PCDRegister::ControlReg as usize] = valid_bits;
        Self {
            regs,
            fifo: fifo.to_vec(),
            fifo_writes: Vec::new(),
        }
    }
}

impl MfrcDriver for Mock {
    async fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), PCDErrorCode> {
        if reg == PCDRegister::FIFODataReg {
            self.fifo_writes.push([val].to_vec());
        }
        Ok(())
    }

    async fn write_reg_buff(
        &mut self,
        reg: u8,
        count: usize,
        values: &[u8],
    ) -> Result<(), PCDErrorCode> {
        if reg == PCDRegister::FIFODataReg {
            self.fifo_writes.push(values[..count].to_vec());
        }
        Ok(())
    }

    async fn read_reg(&mut self, reg: u8) -> Result<u8, PCDErrorCode> {
        Ok(self.regs[reg as usize])
    }

    async fn read_reg_buff(
        &mut self,
        _reg: u8,
        count: usize,
        output_buff: &mut [u8],
        _rx_align: u8,
    ) -> Result<(), PCDErrorCode> {
        output_buff[..count].copy_from_slice(&self.fifo[..count]);
        Ok(())
    }
}

/// The mock never returns Pending, so a single poll drives any future to completion.
fn block_on<F: Future>(fut: F) -> F::Output {
    let mut fut = pin!(fut);
    match fut.as_mut().poll(&mut Context::from_waker(Waker::noop())) {
        Poll::Ready(out) => out,
        Poll::Pending => panic!("mock future pended"),
    }
}

fn pcd(mock: Mock) -> MFRC522<Mock> {
    MFRC522::new(mock, || 0)
}

#[test]
fn uid_number_never_panics() {
    let mut uid = Uid {
        size: 4,
        uid_bytes: [1, 2, 3, 4, 5, 6, 7, 8, 9, 10],
        sak: 0,
    };
    assert_eq!(uid.get_number(), 0x0403_0201);
    uid.size = 7;
    assert_eq!(uid.get_number(), 0x07_0605_0403_0201);
    uid.size = 255;
    assert_eq!(
        uid.get_number(),
        u128::from_le_bytes([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 0, 0, 0, 0, 0, 0])
    );
}

#[test]
fn fifo_larger_than_back_buffer_is_no_room() {
    let mut mfrc = pcd(Mock::answering(&[0; 10], 0));
    let mut back = [0; 2];
    let mut back_len = 18; // caller lies about its buffer size
    let res = block_on(mfrc.pcd_transceive_data(
        &[0x26],
        1,
        &mut back,
        Some(&mut back_len),
        None,
        0,
        false,
    ));
    assert_eq!(res, Err(PCDErrorCode::NoRoom));
}

#[test]
fn invalid_arguments_are_errors_not_panics() {
    let mut mfrc = pcd(Mock::answering(&[], 0));
    let uid = Uid {
        size: 2,
        uid_bytes: [0; 10],
        sak: 0,
    };
    assert_eq!(
        block_on(mfrc.pcd_authenticate(0x60, 0, &[0xFF; 6], &uid)),
        Err(PCDErrorCode::Invalid)
    );
    let uid = Uid { size: 4, ..uid };
    assert_eq!(
        block_on(mfrc.pcd_authenticate(0x60, 0, &[0xFF; 10], &uid)),
        Err(PCDErrorCode::Invalid)
    );
    // write command is ACKed, then the 3-byte data (claimed 16) must be rejected
    let mut acking = pcd(Mock::answering(&[0x0A], 4));
    assert_eq!(
        block_on(acking.mifare_write(4, &[0; 3], 16)),
        Err(PCDErrorCode::Invalid)
    );
    assert_eq!(
        block_on(mfrc.pcd_calc_crc(&[1], 5, &mut [0; 2])),
        Err(PCDErrorCode::NoRoom)
    );
    let mut short = [0; 4];
    let mut size = 18;
    assert_eq!(
        block_on(mfrc.mifare_read(0, &mut short, &mut size)),
        Err(PCDErrorCode::NoRoom)
    );
}

#[test]
fn set_value_writes_value_block_layout() {
    // every transceive gets a 4-bit ACK (0xA)
    let mut mfrc = pcd(Mock::answering(&[0x0A], 4));
    block_on(mfrc.mifare_set_value(5, 0x1234_5678)).unwrap();

    // FIFO writes: [crc(cmd), cmd+crc, crc(data), data+crc] -> data frame is the last one
    let data = mfrc.driver.fifo_writes.last().unwrap();
    assert_eq!(&data[0..4], &[0x78, 0x56, 0x34, 0x12]);
    assert_eq!(&data[4..8], &[!0x78, !0x56, !0x34, !0x12]);
    assert_eq!(&data[8..12], &[0x78, 0x56, 0x34, 0x12]);
    assert_eq!(&data[12..16], &[5, !5, 5, !5]);
}

#[test]
fn collision_on_reqa_counts_as_card_present() {
    let mut mock = Mock::answering(&[0x44, 0x00], 0);
    mock.regs[PCDRegister::ErrorReg as usize] = 0x08; // CollErr
    assert_eq!(block_on(pcd(mock).picc_is_new_card_present()), Ok(()));
}
