use crate::{MfrcDriver, consts::PCDErrorCode};

pub struct SpiDriver<S>
where
    S: embedded_hal_async::spi::SpiDevice,
{
    spi: S,
}

impl<S> SpiDriver<S>
where
    S: embedded_hal_async::spi::SpiDevice,
{
    pub fn new(spi: S) -> Self {
        Self { spi }
    }
}

impl<S> MfrcDriver for SpiDriver<S>
where
    S: embedded_hal_async::spi::SpiDevice,
{
    async fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), PCDErrorCode> {
        self.spi
            .transaction(&mut [embedded_hal_async::spi::Operation::Write(&[reg << 1, val])])
            .await?;

        Ok(())
    }

    async fn write_reg_buff(
        &mut self,
        reg: u8,
        count: usize,
        values: &[u8],
    ) -> Result<(), PCDErrorCode> {
        self.spi
            .transaction(&mut [
                embedded_hal_async::spi::Operation::Write(&[reg << 1]),
                embedded_hal_async::spi::Operation::Write(&values[..count]),
            ])
            .await?;

        Ok(())
    }

    async fn read_reg(&mut self, reg: u8) -> Result<u8, PCDErrorCode> {
        let mut read = [0; 1];
        self.spi
            .transaction(&mut [
                embedded_hal_async::spi::Operation::Write(&[(reg << 1) | 0x80]),
                embedded_hal_async::spi::Operation::Transfer(&mut read, &[0]),
            ])
            .await?;

        Ok(read[0])
    }

    async fn read_reg_buff(
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
        let first_out_byte = output_buff[0];
        output_buff[..count - 1].fill(addr);
        output_buff[count - 1] = 0;

        self.spi
            .transaction(&mut [
                embedded_hal_async::spi::Operation::Write(&[addr]),
                embedded_hal_async::spi::Operation::TransferInPlace(&mut output_buff[..count]),
            ])
            .await?;

        if rx_align > 0 {
            let mask = 0xFF << rx_align;
            output_buff[0] = (first_out_byte & !mask) | (output_buff[0] & mask);
        }

        Ok(())
    }
}
