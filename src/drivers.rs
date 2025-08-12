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
            .await
            .map_err(PCDErrorCode::from_spi_error)?;

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
            .await
            .map_err(PCDErrorCode::from_spi_error)?;

        Ok(())
    }

    async fn read_reg(&mut self, reg: u8) -> Result<u8, PCDErrorCode> {
        let mut read = [0; 1];
        self.spi
            .transaction(&mut [
                embedded_hal_async::spi::Operation::Write(&[(reg << 1) | 0x80]),
                embedded_hal_async::spi::Operation::Transfer(&mut read, &[0]),
            ])
            .await
            .map_err(PCDErrorCode::from_spi_error)?;

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
            .await
            .map_err(PCDErrorCode::from_spi_error)?;

        if rx_align > 0 {
            let mask = 0xFF << rx_align;
            output_buff[0] = (first_out_byte & !mask) | (output_buff[0] & mask);
        }

        Ok(())
    }
}

pub struct I2CDriver<I>
where
    I: embedded_hal_async::i2c::I2c,
{
    address: u8,
    i2c: I,
}

impl<I> I2CDriver<I>
where
    I: embedded_hal_async::i2c::I2c,
{
    pub fn new(i2c: I, addr: u8) -> Self {
        Self { address: addr, i2c }
    }
}

impl<I> MfrcDriver for I2CDriver<I>
where
    I: embedded_hal_async::i2c::I2c,
{
    async fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), PCDErrorCode> {
        self.i2c
            .transaction(
                self.address,
                &mut [embedded_hal_async::i2c::Operation::Write(&[reg, val])],
            )
            .await
            .map_err(PCDErrorCode::from_i2c_error)?;

        Ok(())
    }

    async fn write_reg_buff(
        &mut self,
        reg: u8,
        count: usize,
        values: &[u8],
    ) -> Result<(), PCDErrorCode> {
        self.i2c
            .transaction(
                self.address,
                &mut [
                    embedded_hal_async::i2c::Operation::Write(&[reg]),
                    embedded_hal_async::i2c::Operation::Write(&values[..count]),
                ],
            )
            .await
            .map_err(PCDErrorCode::from_i2c_error)?;

        Ok(())
    }

    async fn read_reg(&mut self, reg: u8) -> Result<u8, PCDErrorCode> {
        let mut read = [0; 1];
        self.i2c
            .transaction(
                self.address,
                &mut [
                    embedded_hal_async::i2c::Operation::Write(&[reg]),
                    embedded_hal_async::i2c::Operation::Read(&mut read),
                ],
            )
            .await
            .map_err(PCDErrorCode::from_i2c_error)?;

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

        self.i2c
            .transaction(
                self.address,
                &mut [
                    embedded_hal_async::i2c::Operation::Write(&[reg]),
                    embedded_hal_async::i2c::Operation::Read(&mut output_buff[..count]),
                ],
            )
            .await
            .map_err(PCDErrorCode::from_i2c_error)?;

        // TODO: impl this
        /*
               * while(_wire.available() && index < count) {
          if(index == 0 && rxAlign) {    // Only update bit positions rxAlign..7 in values[0]
            // Create bit mask for bit positions rxAlign..7
            byte mask = 0;

            for(byte i     = rxAlign; i <= 7; i++) {
              mask |= (1 << i);
            }
            // Read value and tell that we want to read the same address again.
            byte     value = (byte)_wire.read(); // returns int but only with uint8 content

            // Apply mask to both current value of values[0] and the new data in value.
            values[0] = (values[index] & ~mask) | (value & mask);
          } else { // Normal case
            values[index] = (byte)_wire.read(); // returns int but only with uint8 content
          }
          index++;
        }
              */

        /*
        if rx_align > 0 {
            let mask = 0xFF << rx_align;
            output_buff[0] = (first_out_byte & !mask) | (output_buff[0] & mask);
        }
        */

        Ok(())
    }
}
