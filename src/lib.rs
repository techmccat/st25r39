#![no_std]

use self::commands::DirectCommand;
use embedded_hal::{
    digital::InputPin,
    spi::{Operation, SpiDevice},
};
use registers::{Register, RegisterSpace};

pub mod commands;
pub mod registers;

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

pub struct ST25R3916<I: Interface, P: InputPin> {
    dev: I,
    irq: P,
}

impl<I: Interface, P: InputPin> ST25R3916<I, P> {
    /// Run a function with access to the interface
    ///
    /// Can be used to reclock after initialization
    pub fn with_interface(&mut self, f: impl FnOnce(&mut I)) {
        f(&mut self.dev);
    }

    /// Takes ownership of the interface and initializes the device
    pub fn init(dev: I, irq: P) -> Result<Self, I::Error> {
        let mut drv = Self { dev, irq };
        // set default state
        drv.dev.direct_command(DirectCommand::SetDefault)?;
        let mut io_cfg = registers::IoConfiguration::default();
        // TODO: decouple this into the interface, we'll want I2C support in the future
        // increase MISO driving level for higher SPI clock
        io_cfg.set_io_drv_lvl(registers::io_configuration::IoDriverLevel::Increased);
        // enable MISO pulldown
        io_cfg.set_miso_pd1(true);
        io_cfg.set_miso_pd2(true);
        io_cfg.set_out_cl(registers::io_configuration::OutClk::Disabled);
        io_cfg.write(&mut drv.dev)?;
        // let io_cfg = registers::IoConfiguration::read::<_, u16>(&mut drv.dev)?;
        // defmt::debug!("IO Configuration: {}", io_cfg);

        let identity: u8 = registers::IcIdentity::read(&mut drv.dev)?.into();
        defmt::info!("IC type {0=0..3}, revision {0=3..8}", identity);

        drv.enable_oscillator()?;

        // measure voltage
        let vdd = drv.measure_voltage(registers::regulator_control::MeasureSource::Vdd)?;
        defmt::info!("Measured voltage: {=u16}mV", vdd);
        // set power supply level
        registers::IoConfiguration::modify(&mut drv.dev, |r| r.set_sup3v(vdd < 3600))?;
        // disable tx/rx
        registers::OperationControl::modify(&mut drv.dev, |reg| {
            reg.set_tx_en(false);
            reg.set_rx_en(false);
        })?;
        // disable interrupts
        drv.disable_interrupts(u32::MAX.into())?;
        // clear interrupts
        registers::InterruptRegister::read(&mut drv.dev)?;
        Ok(drv)
    }

    /// Enables clock output for measuring or use in clocking the host MCU
    ///
    /// The pin is not connected on X-NUCLEO-06/08 boards
    pub fn set_clock_out(
        &mut self,
        setting: registers::io_configuration::OutClk,
    ) -> Result<(), I::Error> {
        registers::IoConfiguration::modify(&mut self.dev, |reg| reg.set_out_cl(setting))
    }

    /// Measures requested voltage in millivolts
    fn measure_voltage(
        &mut self,
        source: registers::regulator_control::MeasureSource,
    ) -> Result<u16, I::Error> {
        // enable direct command interrupt
        let mut irq = registers::Interrupt::default();
        irq.set_dct(true);
        self.enable_interrupts(irq)?;

        registers::RegulatorControl::modify(&mut self.dev, |r| r.set_mpsv(source))?;
        self.command_and_wait(DirectCommand::MeasurePowerSupply)?;

        let raw = registers::ADConverterOutput::read(&mut self.dev)?.value() as u16;
        // copied from the arduino library, each step is supposedly 23.4mV
        let mv = raw * 23 + ((raw * 4) + 5) / 10;

        Ok(mv)
    }

    fn enable_interrupts(&mut self, int: registers::Interrupt) -> Result<(), I::Error> {
        let mut mask: u32 = registers::InterruptMask::read(&mut self.dev)?.into();
        mask &= !(u32::from(int));
        registers::InterruptMask::from(mask).write(&mut self.dev)?;
        Ok(())
    }
    fn disable_interrupts(&mut self, int: registers::Interrupt) -> Result<(), I::Error> {
        let mut mask: u32 = registers::InterruptMask::read(&mut self.dev)?.into();
        mask |= u32::from(int);
        registers::InterruptMask::from(mask).write(&mut self.dev)?;
        Ok(())
    }
    pub fn wait_for_interrupt(&mut self, int: registers::Interrupt) -> Result<(), I::Error> {
        let int = u32::from(int);
        defmt::trace!(
            "Waiting for IRQs [{0=0..8:08b} {0=8..16:08b} {0=16..24:08b} {0=24..32:08b}]",
            int,
        );
        // todo: custom error type
        loop {
            while self.irq.is_low().unwrap() {
                core::hint::spin_loop()
            }
            let firing: u32 = registers::InterruptRegister::read(&mut self.dev)?.into();
            defmt::trace!("Got IRQ, {=u32:032b}", firing);
            if firing & u32::from(int) != 0 {
                break Ok(());
            }
        }
    }

    /// Enables the oscillator and waits until it's stable
    fn enable_oscillator(&mut self) -> Result<(), I::Error> {
        let op_ctr = registers::OperationControl::read(&mut self.dev)?;
        // not already enabled
        if !op_ctr.en() {
            defmt::debug!("Enabling oscillator");
            // clear interrupts
            let _ = registers::InterruptRegister::read(&mut self.dev)?;

            let mut int = registers::Interrupt::default();
            int.set_osc(true);

            self.enable_interrupts(int)?;
            registers::OperationControl::modify(&mut self.dev, |reg| reg.set_en(true))?;
            self.wait_for_interrupt(int)?;
            defmt::debug!("Oscillator running");
            self.disable_interrupts(int)?;
        }
        Ok(())
    }

    fn command_and_wait(&mut self, cmd: DirectCommand) -> Result<(), I::Error> {
        let mut irq = registers::Interrupt::default();
        irq.set_dct(true);
        self.enable_interrupts(irq)?;
        self.dev.direct_command(cmd)?;
        self.wait_for_interrupt(irq)?;
        self.disable_interrupts(irq)
    }

    /// Adjust regulators and returns the regulated voltage
    pub fn adjust_regulators(&mut self) -> Result<u16, I::Error> {
        registers::RegulatorControl::modify(&mut self.dev, |r| {
            r.set_reg_s(registers::regulator_control::RegulatedSettingSource::AdjustRegulators)
        })?;
        self.command_and_wait(DirectCommand::AdjustRegulators)?;

        let regulator_output = registers::RegulatorDisplay::read(&mut self.dev)?.reg();

        let base_voltage = if registers::IoConfiguration::read(&mut self.dev)?.sup3v() {
            2400
        } else {
            3600
        };

        Ok(base_voltage + 100 * regulator_output.value() as u16)
    }
}
