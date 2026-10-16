extern crate std;

use crate::{
    Error, Interface, Key, KeyType, Mfrc522, Uid, access_bits,
    consts::{PCDCommand, PCDRegister},
    mifare::value_block,
};
use core::{
    future::Future,
    pin::pin,
    task::{Context, Poll, Waker},
};
use std::{collections::VecDeque, vec::Vec};

/// CRC the mock "coprocessor" always reports.
const CRC: [u8; 2] = [0xAA, 0xBB];
/// `valid_bits` marker for an answer where the card stays silent (TimerIRq).
const SILENT: u8 = 0xFF;

/// Register-map mock: every command completes instantly. Each `Transceive` pops the next
/// `(fifo, valid_bits)` answer (the last one repeats); FIFO writes are recorded.
struct Mock {
    regs: [u8; 64],
    answers: VecDeque<(Vec<u8>, u8)>,
    fifo: Vec<u8>,
    fifo_writes: Vec<Vec<u8>>,
}

impl Mock {
    fn answering(answers: &[(&[u8], u8)]) -> Self {
        let mut regs = [0; 64];
        regs[PCDRegister::ComIrqReg as usize] = 0x30; // RxIRq | IdleIRq
        regs[PCDRegister::DivIrqReg as usize] = 0x04; // CRCIRq
        regs[PCDRegister::CRCResultRegL as usize] = CRC[0];
        regs[PCDRegister::CRCResultRegH as usize] = CRC[1];
        Self {
            regs,
            answers: answers.iter().map(|(f, v)| (f.to_vec(), *v)).collect(),
            fifo: Vec::new(),
            fifo_writes: Vec::new(),
        }
    }

    fn next_answer(&mut self) {
        let answer = match self.answers.len() {
            0 => return,
            1 => self.answers[0].clone(),
            _ => self.answers.pop_front().unwrap_or_default(),
        };
        self.regs[PCDRegister::ComIrqReg as usize] = if answer.1 == SILENT { 0x01 } else { 0x30 };
        self.regs[PCDRegister::FIFOLevelReg as usize] = answer.0.len() as u8;
        self.regs[PCDRegister::ControlReg as usize] = answer.1;
        self.fifo = answer.0;
    }
}

impl Interface for Mock {
    async fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), Error> {
        if reg == PCDRegister::CommandReg && val == PCDCommand::Transceive {
            self.next_answer();
        }
        Ok(())
    }

    async fn write_regs(&mut self, reg: u8, values: &[u8]) -> Result<(), Error> {
        if reg == PCDRegister::FIFODataReg {
            self.fifo_writes.push(values.to_vec());
        }
        Ok(())
    }

    async fn read_reg(&mut self, reg: u8) -> Result<u8, Error> {
        Ok(self.regs[reg as usize])
    }

    async fn read_regs(&mut self, _reg: u8, out: &mut [u8], _rx_align: u8) -> Result<(), Error> {
        out.copy_from_slice(&self.fifo[..out.len()]);
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

fn rfid(answers: &[(&[u8], u8)]) -> Mfrc522<Mock> {
    Mfrc522::with_clock(Mock::answering(answers), || 0)
}

const ACK: (&[u8], u8) = (&[0x0A], 4);

#[test]
fn uid_accessors() {
    let uid = Uid {
        bytes: [1, 2, 3, 4, 5, 6, 7, 8, 9, 10],
        len: 7,
        sak: 0x08,
    };
    assert_eq!(uid.as_bytes(), &[1, 2, 3, 4, 5, 6, 7]);
    assert_eq!(uid.to_u128(), 0x07_0605_0403_0201);
    assert_eq!(uid.card_type(), crate::CardType::Mifare1K);

    let broken = Uid { len: 255, ..uid };
    assert_eq!(broken.as_bytes().len(), 10);
}

#[test]
fn select_4_byte_uid() {
    let mut rfid = rfid(&[
        (&[0xDE, 0xAD, 0xBE, 0xEF, 0xDE ^ 0xAD ^ 0xBE ^ 0xEF], 0), // anticollision: UID + BCC
        (&[0x08, CRC[0], CRC[1]], 0),                              // select: SAK + CRC
    ]);
    let uid = block_on(rfid.select()).unwrap();
    assert_eq!(uid.as_bytes(), &[0xDE, 0xAD, 0xBE, 0xEF]);
    assert_eq!(uid.sak(), 0x08);
}

#[test]
fn select_rejects_bad_sak_crc() {
    let mut rfid = rfid(&[(&[1, 2, 3, 4, 4], 0), (&[0x08, 0x00, 0x00], 0)]);
    assert_eq!(block_on(rfid.select()), Err(Error::Crc));
}

#[test]
fn oversized_fifo_is_buffer_too_small() {
    // REQA expects a 2-byte ATQA, the chip claims 10 bytes
    let mut rfid = rfid(&[(&[0; 10], 0)]);
    assert_eq!(block_on(rfid.request_a()), Err(Error::BufferTooSmall));
}

#[test]
fn authenticate_rejects_short_uid() {
    let mut rfid = rfid(&[]);
    let uid = Uid {
        bytes: [0; 10],
        len: 2,
        sak: 0,
    };
    assert_eq!(
        block_on(rfid.authenticate(KeyType::A, 0, &Key::DEFAULT, &uid)),
        Err(Error::InvalidArgument)
    );
}

#[test]
fn read_block_returns_data_and_checks_length() {
    let mut answer = [7u8; 18];
    answer[16..].copy_from_slice(&CRC);
    let mut ok = rfid(&[(&answer, 0)]);
    assert_eq!(block_on(ok.read_block(4)), Ok([7; 16]));

    let mut short = rfid(&[(&[1, 2, CRC[0], CRC[1]], 0)]);
    assert_eq!(block_on(short.read_block(4)), Err(Error::Communication));

    let mut nack = rfid(&[(&[0x04], 4)]);
    assert_eq!(block_on(nack.read_block(4)), Err(Error::Nack));
}

#[test]
fn write_value_uses_value_block_layout() {
    let mut rfid = rfid(&[ACK]);
    block_on(rfid.write_value(5, 0x1234_5678)).unwrap();

    // FIFO writes: [crc(cmd), cmd+crc, crc(data), data+crc] -> data frame is the last one
    let frame = rfid.release().fifo_writes.pop().unwrap();
    assert_eq!(frame[..16], value_block(5, 0x1234_5678));
    assert_eq!(
        value_block(5, -1),
        [
            0xFF, 0xFF, 0xFF, 0xFF, 0, 0, 0, 0, 0xFF, 0xFF, 0xFF, 0xFF, 5, !5, 5, !5
        ]
    );
}

#[test]
fn value_ops_accept_silent_operand_step() {
    // command ACKed, operand answered with silence like a real card
    let mut acked = rfid(&[ACK, (&[], SILENT)]);
    assert_eq!(block_on(acked.increment(4, 1)), Ok(()));

    // a silent card on the command step is still a timeout
    let mut silent = rfid(&[(&[], SILENT)]);
    assert_eq!(block_on(silent.decrement(4, 1)), Err(Error::Timeout));
}

#[test]
fn presence_maps_timeout_and_collision() {
    let mut present = rfid(&[(&[0x44, 0x00], 0)]);
    assert_eq!(block_on(present.is_new_card_present()), Ok(true));

    let mut mock = Mock::answering(&[(&[0x44, 0x00], 0)]);
    mock.regs[PCDRegister::ErrorReg as usize] = 0x08; // CollErr
    assert_eq!(
        block_on(Mfrc522::with_clock(mock, || 0).is_new_card_present()),
        Ok(true)
    );

    let mut silent = rfid(&[(&[], SILENT)]);
    assert_eq!(block_on(silent.is_new_card_present()), Ok(false));
}

#[test]
fn access_bits_transport_configuration() {
    // factory trailer access bytes FF 07 80
    assert_eq!(access_bits(0, 0, 0, 1), [0xFF, 0x07, 0x80]);
}
