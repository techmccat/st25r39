#![no_std]

use self::commands::DirectCommand;
use defmt::Format;
use embedded_hal::{
    digital::InputPin,
    spi::{Operation, SpiDevice},
};
use registers::{Interrupt, Register, RegisterSpace};

pub type GPTDuration = fugit::Duration<u32, 59, 100_000_000>;

pub mod aat;
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
        registers::RegulatorControl::modify(&mut self.dev, |r| r.set_mpsv(source))?;
        self.command_and_wait(DirectCommand::MeasurePowerSupply)?;

        let raw = registers::ADConverterOutput::read(&mut self.dev)?.value() as u16;
        // copied from the arduino library, each step is supposedly 23.4mV
        let mv = raw * 23 + ((raw * 4) + 5) / 10;

        Ok(mv)
    }

    fn enable_interrupts(&mut self, int: Interrupt) -> Result<(), I::Error> {
        let mut mask: u32 = registers::InterruptMask::read(&mut self.dev)?.into();
        mask &= !(u32::from(int));
        registers::InterruptMask::from(mask).write(&mut self.dev)?;
        Ok(())
    }
    fn disable_interrupts(&mut self, int: Interrupt) -> Result<(), I::Error> {
        let mut mask: u32 = registers::InterruptMask::read(&mut self.dev)?.into();
        mask |= u32::from(int);
        registers::InterruptMask::from(mask).write(&mut self.dev)?;
        Ok(())
    }
    fn wait_for_interrupt(&mut self, int: Interrupt) -> Result<(), I::Error> {
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

            let mut int = Interrupt::default();
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
        let mut irq = Interrupt::default();
        irq.set_dct(true);
        self.enable_interrupts(irq)?;
        self.dev.direct_command(cmd)?;
        self.wait_for_interrupt(irq)?;
        self.disable_interrupts(irq)
    }

    /// Delay using the peripheral's general purpose timer
    pub fn delay(&mut self, d: GPTDuration) -> Result<(), I::Error> {
        let mut ticks = d.ticks();
        let mut irq = registers::Interrupt::default();
        irq.set_gpe(true);
        self.enable_interrupts(irq)?;

        while ticks > u16::MAX as u32 {
            registers::GPTimer::from_ticks(u16::MAX).write(&mut self.dev)?;
            self.dev.direct_command(DirectCommand::StartGPT)?;
            self.wait_for_interrupt(irq)?;

            ticks -= u16::MAX as u32;
        }
        registers::GPTimer::from_ticks(ticks as u16).write(&mut self.dev)?;
        self.dev.direct_command(DirectCommand::StartGPT)?;
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

    /// Measures the signal amplitude on the RFI pins
    ///
    /// The amplitude detector conversion gain is 0.6 VinPP / Vout referenced to the RF signal
    /// on a single RFI pin.  
    /// Thus, one LSB of the A/D converter output represents 13.02 mVPP on either of the RFI inputs.
    pub fn measure_amplitude_raw(&mut self) -> Result<u8, I::Error> {
        use registers::{mode_definition, operation_control, receiver_configuration};

        // save registers
        let op_bak = registers::OperationControl::read(&mut self.dev)?;
        let mode_bak = registers::ModeDefinition::read(&mut self.dev)?;
        let rx_conf_bak = registers::ReceiverConfiguration::read(&mut self.dev)?;
        let aux_bak = registers::AuxiliaryModulationSetting::read(&mut self.dev)?;

        // disable bits that influence the receiver chain
        let mut op_new = op_bak;
        op_new.set_rx_chn(operation_control::RxChanEnable::Both);
        let mut mode_new = registers::ModeDefinition::default();
        mode_new.set_om(mode_definition::OperationMode::InitiatorIso14443A);
        mode_new.set_tr_am(mode_definition::ModulationMode::OOK);
        mode_new.set_nfc_ar(mode_definition::NfcAutomaticResponse::Off);
        let mut rx_conf_new = rx_conf_bak;
        rx_conf_new.set_ch_sel(receiver_configuration::ChannelSelect::ChannelAM);
        rx_conf_new.set_demod_mode(receiver_configuration::DemodulationMode::AMPM);
        rx_conf_new.set_amd_sel(receiver_configuration::AMDemodulatorSelect::PeakDetector);
        // TODO: feature gate to st25r3916b (rfal has an ifdef)
        // disable active wave shaping for amplitude measurement
        let mut aux_new = aux_bak;
        aux_new.set_regulator_am(false);

        op_new.write(&mut self.dev)?;
        mode_new.write(&mut self.dev)?;
        rx_conf_new.write(&mut self.dev)?;
        aux_new.write(&mut self.dev)?;

        self.command_and_wait(DirectCommand::MeasureAmplitude)?;
        let amplitude = registers::ADConverterOutput::read(&mut self.dev)?.value();

        // restore registers
        op_bak.write(&mut self.dev)?;
        mode_bak.write(&mut self.dev)?;
        rx_conf_bak.write(&mut self.dev)?;
        aux_bak.write(&mut self.dev)?;

        Ok(amplitude)
    }

    fn measure_phase_raw(&mut self) -> Result<u8, I::Error> {
        self.command_and_wait(DirectCommand::MeasurePhase)?;
        registers::ADConverterOutput::read(&mut self.dev).map(|r| r.value())
    }

    /// Measures the phase difference between the RFO (output) and RFI (input) signals
    pub fn measure_phase(&mut self) -> Result<PhaseDifference, I::Error> {
        self.measure_phase_raw().map(PhaseDifference::from_raw)
    }

    /// Sets the AAT capacitors' DAC output
    pub fn set_aat_capacitance(
        &mut self,
        a: u8,
        b: u8,
    ) -> Result<(), I::Error> {
        let reg = registers::AntennaTuningControl::new(a, b);
        reg.write(&mut self.dev)?;
        // wait for variable capacitors to adjust
        self.delay(GPTDuration::millis(10))
    }

    /// Sets the AAT capacitors' DAC output and measures the new amplitude and phase
    pub fn set_capacitance_and_measure(
        &mut self,
        // TODO: use peripheral's timer for delay
        a: u8,
        b: u8,
    ) -> Result<(u8, u8), I::Error> {
        self.set_aat_capacitance(a, b)?;

        let amplitude = self.measure_amplitude_raw()?;
        let phase = self.measure_phase_raw()?;
        Ok((amplitude, phase))
    }

    pub fn tune_antennas(
        &mut self,
        config: aat::TunerSettings,
    ) -> Result<(), I::Error> {
        let mut state = aat::TunerState::new_from_settings(&config, self)?;
        while !state.is_done() {
            let best_dir =
                aat::find_best_step(self, &mut state, &config /*, prev_dir*/)?;

            if let Some(d) = best_dir {
                while aat::try_greedy_step(self, &mut state, &config, d)? {}
            }

            state.halve_steps();
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, Format)]
pub enum PhaseDifference {
    /// Between or equal to 0° and 17°
    Below17,
    Measured(u8),
    /// Between or equal to 163° and 180°
    Above163,
}

impl PhaseDifference {
    pub fn from_raw(raw: u8) -> Self {
        match raw {
            0 => Self::Above163,
            255 => Self::Below17,
            _ => Self::Measured(17 + ((1.0 - raw as f32 / 255.0) * 146.0) as u8),
        }
    }
}
