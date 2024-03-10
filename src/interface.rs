use crate::{registers::RegisterSpace, DirectCommand};
use embedded_hal::spi::{Operation, SpiDevice};
pub trait Interface: Sized {
    type Error;
    /// Write one or more registers
    fn register_write(
        &mut self,
        addr: u8,
        space: RegisterSpace,
        buf: &[u8],
    ) -> Result<(), Self::Error>;
    /// Read one or more registers
    fn register_read(
        &mut self,
        addr: u8,
        space: RegisterSpace,
        buf: &mut [u8],
    ) -> Result<(), Self::Error>;
    /// Write to the FIFO
    fn fifo_write(&mut self, buf: &[u8]) -> Result<(), Self::Error>;
    /// Read from the FIFO
    fn fifo_read(&mut self, buf: &mut [u8]) -> Result<(), Self::Error>;
    /// Write to passive target memory, addresses 0..48
    fn pt_memory_load_a_config(&mut self, buf: &[u8]) -> Result<(), Self::Error>;
    /// Write to passive target memory, address 15..48
    fn pt_memory_load_f_config(&mut self, buf: &[u8]) -> Result<(), Self::Error>;
    /// Write to passive target memory, address 36..48
    ///
    /// Allows reloading the TSN random numbers without overwriting previous memory
    fn pt_memory_load_tsn_data(&mut self, buf: &[u8]) -> Result<(), Self::Error>;
    /// Read passive target memory
    fn pt_memory_read(&mut self, buf: &mut [u8]) -> Result<(), Self::Error>;
    fn direct_command(&mut self, cmd: DirectCommand) -> Result<(), Self::Error>;
}

pub mod spi_modes {
    /// Lower 6 bits set to the register address
    pub const REG_WRITE: u8 = 0b00 << 6;
    /// Lower 6 bits set to the register address
    pub const REG_READ: u8 = 0b01 << 6;
    pub const FIFO_LOAD: u8 = 0b10 << 6;
    pub const PT_MEM_LOAD_A_CFG: u8 = 0b1010_0000;
    pub const PT_MEM_LOAD_F_CFG: u8 = 0b1010_1000;
    pub const PT_MEM_LOAD_TSN_DATA: u8 = 0b1010_1100;
    pub const PT_MEM_READ: u8 = 0b1011_1111;
    pub const FIFO_READ: u8 = 0b1001_1111;
    pub const DIRECT_COMMAND: u8 = 0b11 << 6;
}

pub struct SpiInterface<S: SpiDevice> {
    dev: S,
}

impl<S: SpiDevice> Interface for SpiInterface<S> {
    type Error = S::Error;

    fn register_write(
        &mut self,
        addr: u8,
        space: RegisterSpace,
        buf: &[u8],
    ) -> Result<(), Self::Error> {
        defmt::trace!("Register {=u8:X}, write {=[u8]:b}", addr, buf);
        self.dev.transaction(&mut [
            Operation::Write(space.prefix()),
            Operation::Write(&[spi_modes::REG_WRITE | (addr & 0b111111)]),
            Operation::Write(buf),
        ])
    }

    fn register_read(
        &mut self,
        addr: u8,
        space: RegisterSpace,
        buf: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.dev.transaction(&mut [
            Operation::Write(space.prefix()),
            Operation::Write(&[spi_modes::REG_READ | (addr & 0b111111)]),
            Operation::Read(buf),
        ])?;
        defmt::trace!("Register {=u8:X}, read {=[u8]:b}", addr, buf);
        Ok(())
    }

    fn fifo_write(&mut self, buf: &[u8]) -> Result<(), Self::Error> {
        assert!(buf.len() > 0 && buf.len() <= 512);
        self.dev.transaction(&mut [
            Operation::Write(&[spi_modes::FIFO_LOAD]),
            Operation::Write(buf),
        ])
    }

    fn fifo_read(&mut self, buf: &mut [u8]) -> Result<(), Self::Error> {
        self.dev.transfer(buf, &[spi_modes::FIFO_READ])
    }

    fn direct_command(&mut self, cmd: DirectCommand) -> Result<(), Self::Error> {
        defmt::trace!("Direct command {} ({=u8:#X})", cmd, cmd as u8);
        self.dev
            .write(&[spi_modes::DIRECT_COMMAND | (cmd as u8 & 0b111111)])
    }

    fn pt_memory_load_a_config(&mut self, buf: &[u8]) -> Result<(), Self::Error> {
        self.dev.transaction(&mut [
            Operation::Write(&[spi_modes::PT_MEM_LOAD_A_CFG]),
            Operation::Write(buf.get(..48).unwrap_or(buf)),
        ])
    }
    fn pt_memory_load_f_config(&mut self, buf: &[u8]) -> Result<(), Self::Error> {
        self.dev.transaction(&mut [
            Operation::Write(&[spi_modes::PT_MEM_LOAD_F_CFG]),
            Operation::Write(buf.get(..48 - 15).unwrap_or(buf)),
        ])
    }
    fn pt_memory_load_tsn_data(&mut self, buf: &[u8]) -> Result<(), Self::Error> {
        self.dev.transaction(&mut [
            Operation::Write(&[spi_modes::PT_MEM_LOAD_TSN_DATA]),
            Operation::Write(buf.get(..48 - 36).unwrap_or(buf)),
        ])
    }
    fn pt_memory_read(&mut self, buf: &mut [u8]) -> Result<(), Self::Error> {
        let clipped_buf = if let Some(b) = buf.get_mut(..48) {
            b
        } else {
            buf
        };
        self.dev.transaction(&mut [
            Operation::Write(&[spi_modes::PT_MEM_LOAD_TSN_DATA]),
            Operation::Read(clipped_buf),
        ])
    }
}

impl<S: SpiDevice> SpiInterface<S> {
    pub fn new(dev: S) -> Self {
        Self { dev }
    }
}
