use crate::{
    registers::{self, io_configuration::I2cThd, Register, RegisterSpace},
    DirectCommand,
};
use embedded_hal::{
    i2c::{I2c, Operation as I2cOperation},
    spi::{Operation as SpiOperation, SpiDevice},
};
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
    fn init_peripheral(&mut self) -> Result<(), Self::Error>;
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

impl<S: SpiDevice> SpiInterface<S> {
    pub fn new(dev: S) -> Self {
        Self { dev }
    }
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
            SpiOperation::Write(space.prefix()),
            SpiOperation::Write(&[spi_modes::REG_WRITE | (addr & 0b111111)]),
            SpiOperation::Write(buf),
        ])
    }

    fn register_read(
        &mut self,
        addr: u8,
        space: RegisterSpace,
        buf: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.dev.transaction(&mut [
            SpiOperation::Write(space.prefix()),
            SpiOperation::Write(&[spi_modes::REG_READ | (addr & 0b111111)]),
            SpiOperation::Read(buf),
        ])?;
        defmt::trace!("Register {=u8:X}, read {=[u8]:b}", addr, buf);
        Ok(())
    }

    fn fifo_write(&mut self, buf: &[u8]) -> Result<(), Self::Error> {
        assert!(buf.len() > 0 && buf.len() <= 512);
        self.dev.transaction(&mut [
            SpiOperation::Write(&[spi_modes::FIFO_LOAD]),
            SpiOperation::Write(buf),
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
            SpiOperation::Write(&[spi_modes::PT_MEM_LOAD_A_CFG]),
            SpiOperation::Write(buf.get(..48).unwrap_or(buf)),
        ])
    }
    fn pt_memory_load_f_config(&mut self, buf: &[u8]) -> Result<(), Self::Error> {
        self.dev.transaction(&mut [
            SpiOperation::Write(&[spi_modes::PT_MEM_LOAD_F_CFG]),
            SpiOperation::Write(buf.get(..48 - 15).unwrap_or(buf)),
        ])
    }
    fn pt_memory_load_tsn_data(&mut self, buf: &[u8]) -> Result<(), Self::Error> {
        self.dev.transaction(&mut [
            SpiOperation::Write(&[spi_modes::PT_MEM_LOAD_TSN_DATA]),
            SpiOperation::Write(buf.get(..48 - 36).unwrap_or(buf)),
        ])
    }
    fn pt_memory_read(&mut self, buf: &mut [u8]) -> Result<(), Self::Error> {
        let clipped_buf = if let Some(b) = buf.get_mut(..48) {
            b
        } else {
            buf
        };
        self.dev.transaction(&mut [
            SpiOperation::Write(&[spi_modes::PT_MEM_LOAD_TSN_DATA]),
            SpiOperation::Read(clipped_buf),
        ])
    }
    fn init_peripheral(&mut self) -> Result<(), Self::Error> {
        // set default state
        self.direct_command(DirectCommand::SetDefault)?;
        let mut io_cfg = registers::IoConfiguration::default();
        // increase MISO driving level for higher SPI clock
        io_cfg.set_io_drv_lvl(registers::io_configuration::IoDriverLevel::Increased);
        // enable MISO pulldown
        io_cfg.set_miso_pd1(true);
        io_cfg.set_miso_pd2(true);
        io_cfg.set_out_cl(registers::io_configuration::OutClk::Disabled);
        io_cfg.write(self)
    }
}

pub struct I2CInterface<I>(I);

/// I2c interface for the ST25R3916
///
/// Standard (100kHz) mode is assumed during init,
/// set the correct hold time with `set_i2c_thd` before reclocking the bus
impl<I: I2c> I2CInterface<I> {
    const ADDR: u8 = 0x50;
    pub fn new(dev: I) -> Self {
        Self(dev)
    }
    pub fn set_i2c_thd(&mut self, hold: I2cThd) -> Result<(), I::Error> {
        registers::IoConfiguration::modify(self, |r| r.set_i2c_thd(hold))
    }
}

impl<I: I2c> Interface for I2CInterface<I> {
    type Error = I::Error;
    fn direct_command(&mut self, cmd: DirectCommand) -> Result<(), Self::Error> {
        self.0.write(Self::ADDR, &[cmd as u8])
    }
    fn register_write(
        &mut self,
        addr: u8,
        space: RegisterSpace,
        buf: &[u8],
    ) -> Result<(), Self::Error> {
        self.0.transaction(
            Self::ADDR,
            &mut [
                I2cOperation::Write(space.prefix()),
                I2cOperation::Write(&[spi_modes::REG_WRITE | (addr & 0b111111)]),
                I2cOperation::Write(buf),
            ],
        )
    }
    fn register_read(
        &mut self,
        addr: u8,
        space: RegisterSpace,
        buf: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.0.transaction(
            Self::ADDR,
            &mut [
                I2cOperation::Write(space.prefix()),
                I2cOperation::Write(&[spi_modes::REG_READ | (addr & 0b111111)]),
                I2cOperation::Read(buf),
            ],
        )
    }
    fn fifo_write(&mut self, buf: &[u8]) -> Result<(), Self::Error> {
        assert!(buf.len() > 0 && buf.len() <= 512);
        self.0.transaction(
            Self::ADDR,
            &mut [
                I2cOperation::Write(&[spi_modes::FIFO_LOAD]),
                I2cOperation::Write(buf),
            ],
        )
    }
    fn fifo_read(&mut self, buf: &mut [u8]) -> Result<(), Self::Error> {
        self.0.write_read(Self::ADDR, &[spi_modes::FIFO_READ], buf)
    }
    fn pt_memory_load_a_config(&mut self, buf: &[u8]) -> Result<(), Self::Error> {
        self.0.transaction(
            Self::ADDR,
            &mut [
                I2cOperation::Write(&[spi_modes::PT_MEM_LOAD_A_CFG]),
                I2cOperation::Write(buf.get(..48).unwrap_or(buf)),
            ],
        )
    }
    fn pt_memory_load_f_config(&mut self, buf: &[u8]) -> Result<(), Self::Error> {
        self.0.transaction(
            Self::ADDR,
            &mut [
                I2cOperation::Write(&[spi_modes::PT_MEM_LOAD_F_CFG]),
                I2cOperation::Write(buf.get(..48 - 15).unwrap_or(buf)),
            ],
        )
    }
    fn pt_memory_load_tsn_data(&mut self, buf: &[u8]) -> Result<(), Self::Error> {
        self.0.transaction(
            Self::ADDR,
            &mut [
                I2cOperation::Write(&[spi_modes::PT_MEM_LOAD_F_CFG]),
                I2cOperation::Write(buf.get(..48 - 36).unwrap_or(buf)),
            ],
        )
    }
    fn pt_memory_read(&mut self, buf: &mut [u8]) -> Result<(), Self::Error> {
        let clipped_buf = if let Some(b) = buf.get_mut(..48) {
            b
        } else {
            buf
        };
        self.0
            .write_read(Self::ADDR, &[spi_modes::PT_MEM_LOAD_TSN_DATA], clipped_buf)
    }
    fn init_peripheral(&mut self) -> Result<(), Self::Error> {
        self.direct_command(DirectCommand::SetDefault)?;
        let mut io_cfg = registers::IoConfiguration::default();
        io_cfg.set_out_cl(registers::io_configuration::OutClk::Disabled);
        io_cfg.set_io_drv_lvl(registers::io_configuration::IoDriverLevel::Increased);
        io_cfg.write(self)
    }
}
