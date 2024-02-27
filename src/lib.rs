#![no_std]

use defmt::Format;
use commands::DirectCommand;
use embedded_hal::digital::InputPin;
use registers::{Interrupt, Register};
use interface::Interface;

pub type GPTDuration = fugit::Duration<u32, 59, 100_000_000>;

pub mod aat;
pub mod commands;
pub mod interface;
pub mod registers;

pub enum Error<I: Interface, P: InputPin> {
    Interface(I::Error),
    GPIO(P::Error),
}

pub type Result<T, I, P> = core::result::Result<T, Error<I, P>>;

pub struct ST25R3916<I: Interface, P: InputPin> {
    dev: I,
    irq: P,
}

impl<I: Interface, P: InputPin> ST25R3916<I, P> {
    /// Run a function with access to the interface
    ///
    /// Can be used to reclock after initialization
    pub fn borrow_interface(&mut self) -> &mut I {
        &mut self.dev
    }

    /// Takes ownership of the interface and initializes the device
    pub fn init(dev: I, irq: P) -> Result<Self, I, P> {
        let mut drv = Self { dev, irq };
        // set default state
        drv.dev.direct_command(DirectCommand::SetDefault).map_err(Error::Interface)?;
        let mut io_cfg = registers::IoConfiguration::default();
        // TODO: decouple this into the interface, we'll want I2C support in the future
        // increase MISO driving level for higher SPI clock
        io_cfg.set_io_drv_lvl(registers::io_configuration::IoDriverLevel::Increased);
        // enable MISO pulldown
        io_cfg.set_miso_pd1(true);
        io_cfg.set_miso_pd2(true);
        io_cfg.set_out_cl(registers::io_configuration::OutClk::Disabled);
        io_cfg.write(&mut drv.dev).map_err(Error::Interface)?;
        // let io_cfg = registers::IoConfiguration::read::<_, u16>(&mut drv.dev)?;
        // defmt::debug!("IO Configuration: {}", io_cfg);

        let identity: u8 = registers::IcIdentity::read(&mut drv.dev).map_err(Error::Interface)?.into();
        defmt::info!("IC type {0=0..3}, revision {0=3..8}", identity);

        drv.enable_oscillator()?;

        // measure voltage
        let vdd = drv.measure_voltage(registers::regulator_control::MeasureSource::Vdd)?;
        defmt::info!("Measured voltage: {=u16}mV", vdd);
        // set power supply level
        registers::IoConfiguration::modify(&mut drv.dev, |r| r.set_sup3v(vdd < 3600)).map_err(Error::Interface)?;
        // disable tx/rx
        registers::OperationControl::modify(&mut drv.dev, |reg| {
            reg.set_tx_en(false);
            reg.set_rx_en(false);
        }).map_err(Error::Interface)?;
        // disable interrupts
        drv.disable_interrupts(u32::MAX.into())?;
        // clear interrupts
        registers::InterruptRegister::read(&mut drv.dev).map_err(Error::Interface)?;
        Ok(drv)
    }

    /// Enables clock output for measuring or use in clocking the host MCU
    ///
    /// The pin is not connected on X-NUCLEO-06/08 boards
    pub fn set_clock_out(
        &mut self,
        setting: registers::io_configuration::OutClk,
    ) -> Result<(), I, P> {
        registers::IoConfiguration::modify(&mut self.dev, |reg| reg.set_out_cl(setting)).map_err(Error::Interface)
    }

    /// Measures requested voltage in millivolts
    fn measure_voltage(
        &mut self,
        source: registers::regulator_control::MeasureSource,
    ) -> Result<u16, I, P> {
        registers::RegulatorControl::modify(&mut self.dev, |r| r.set_mpsv(source)).map_err(Error::Interface)?;
        self.command_and_wait(DirectCommand::MeasurePowerSupply)?;

        let raw = registers::ADConverterOutput::read(&mut self.dev).map_err(Error::Interface)?.value() as u16;
        // copied from the arduino library, each step is supposedly 23.4mV
        let mv = raw * 23 + ((raw * 4) + 5) / 10;

        Ok(mv)
    }

    fn enable_interrupts(&mut self, int: Interrupt) -> Result<(), I, P> {
        let mut mask: u32 = registers::InterruptMask::read(&mut self.dev).map_err(Error::Interface)?.into();
        mask &= !(u32::from(int));
        registers::InterruptMask::from(mask).write(&mut self.dev).map_err(Error::Interface)?;
        Ok(())
    }
    fn disable_interrupts(&mut self, int: Interrupt) -> Result<(), I, P> {
        let mut mask: u32 = registers::InterruptMask::read(&mut self.dev).map_err(Error::Interface)?.into();
        mask |= u32::from(int);
        registers::InterruptMask::from(mask).write(&mut self.dev).map_err(Error::Interface)?;
        Ok(())
    }
    fn wait_for_interrupt(&mut self, int: Interrupt) -> Result<(), I, P> {
        let int = u32::from(int);
        defmt::trace!(
            "Waiting for IRQs [{0=0..8:08b} {0=8..16:08b} {0=16..24:08b} {0=24..32:08b}]",
            int,
        );
        loop {
            while self.irq.is_low().map_err(Error::GPIO)? {
                core::hint::spin_loop()
            }
            let firing: u32 = registers::InterruptRegister::read(&mut self.dev).map_err(Error::Interface)?.into();
            defmt::trace!("Got IRQ, {=u32:032b}", firing);
            if firing & u32::from(int) != 0 {
                break Ok(());
            }
        }
    }

    /// Enables the oscillator and waits until it's stable
    fn enable_oscillator(&mut self) -> Result<(), I, P> {
        let op_ctr = registers::OperationControl::read(&mut self.dev).map_err(Error::Interface)?;
        // not already enabled
        if !op_ctr.en() {
            defmt::debug!("Enabling oscillator");
            // clear interrupts
            let _ = registers::InterruptRegister::read(&mut self.dev).map_err(Error::Interface)?;

            let mut int = Interrupt::default();
            int.set_osc(true);

            self.enable_interrupts(int)?;
            registers::OperationControl::modify(&mut self.dev, |reg| reg.set_en(true)).map_err(Error::Interface)?;
            self.wait_for_interrupt(int)?;
            defmt::debug!("Oscillator running");
            self.disable_interrupts(int)?;
        }
        Ok(())
    }

    fn command_and_wait(&mut self, cmd: DirectCommand) -> Result<(), I, P> {
        let mut irq = Interrupt::default();
        irq.set_dct(true);
        self.enable_interrupts(irq)?;
        self.dev.direct_command(cmd).map_err(Error::Interface)?;
        self.wait_for_interrupt(irq)?;
        self.disable_interrupts(irq)
    }

    /// Delay using the peripheral's general purpose timer
    pub fn delay(&mut self, d: GPTDuration) -> Result<(), I, P> {
        let mut ticks = d.ticks();
        let mut irq = registers::Interrupt::default();
        irq.set_gpe(true);
        self.enable_interrupts(irq)?;

        while ticks > u16::MAX as u32 {
            registers::GPTimer::from_ticks(u16::MAX).write(&mut self.dev).map_err(Error::Interface)?;
            self.dev.direct_command(DirectCommand::StartGPT).map_err(Error::Interface)?;
            self.wait_for_interrupt(irq)?;

            ticks -= u16::MAX as u32;
        }
        registers::GPTimer::from_ticks(ticks as u16).write(&mut self.dev).map_err(Error::Interface)?;
        self.dev.direct_command(DirectCommand::StartGPT).map_err(Error::Interface)?;
        self.wait_for_interrupt(irq)?;

        self.disable_interrupts(irq)
    }

    /// Adjust regulators and returns the regulated voltage
    pub fn adjust_regulators(&mut self) -> Result<u16, I, P> {
        registers::RegulatorControl::modify(&mut self.dev, |r| {
            r.set_reg_s(registers::regulator_control::RegulatedSettingSource::AdjustRegulators)
        }).map_err(Error::Interface)?;
        self.command_and_wait(DirectCommand::AdjustRegulators)?;

        let regulator_output = registers::RegulatorDisplay::read(&mut self.dev).map_err(Error::Interface)?.reg();

        let base_voltage = if registers::IoConfiguration::read(&mut self.dev).map_err(Error::Interface)?.sup3v() {
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
    pub fn measure_amplitude_raw(&mut self) -> Result<u8, I, P> {
        use registers::{mode_definition, operation_control, receiver_configuration};

        // save registers
        let op_bak = registers::OperationControl::read(&mut self.dev).map_err(Error::Interface)?;
        let mode_bak = registers::ModeDefinition::read(&mut self.dev).map_err(Error::Interface)?;
        let rx_conf_bak = registers::ReceiverConfiguration::read(&mut self.dev).map_err(Error::Interface)?;
        let aux_bak = registers::AuxiliaryModulationSetting::read(&mut self.dev).map_err(Error::Interface)?;

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

        op_new.write(&mut self.dev).map_err(Error::Interface)?;
        mode_new.write(&mut self.dev).map_err(Error::Interface)?;
        rx_conf_new.write(&mut self.dev).map_err(Error::Interface)?;
        aux_new.write(&mut self.dev).map_err(Error::Interface)?;

        self.command_and_wait(DirectCommand::MeasureAmplitude)?;
        let amplitude = registers::ADConverterOutput::read(&mut self.dev).map_err(Error::Interface)?.value();

        // restore registers
        op_bak.write(&mut self.dev).map_err(Error::Interface)?;
        mode_bak.write(&mut self.dev).map_err(Error::Interface)?;
        rx_conf_bak.write(&mut self.dev).map_err(Error::Interface)?;
        aux_bak.write(&mut self.dev).map_err(Error::Interface)?;

        Ok(amplitude)
    }

    pub fn measure_phase_raw(&mut self) -> Result<u8, I, P> {
        self.command_and_wait(DirectCommand::MeasurePhase)?;
        registers::ADConverterOutput::read(&mut self.dev).map(|r| r.value()).map_err(Error::Interface)
    }

    /// Measures the phase difference between the RFO (output) and RFI (input) signals
    pub fn measure_phase(&mut self) -> Result<PhaseDifference, I, P> {
        self.measure_phase_raw().map(PhaseDifference::from_raw)
    }

    /// Sets the AAT capacitors' DAC output
    pub fn set_aat_capacitance(
        &mut self,
        a: u8,
        b: u8,
    ) -> Result<(), I, P> {
        let reg = registers::AntennaTuningControl::new(a, b);
        reg.write(&mut self.dev).map_err(Error::Interface)?;
        // wait for variable capacitors to adjust
        self.delay(GPTDuration::millis(10))
    }

    /// Sets the AAT capacitors' DAC output and measures the new amplitude and phase
    pub fn set_capacitance_and_measure(
        &mut self,
        a: u8,
        b: u8,
    ) -> Result<(u8, u8), I, P> {
        self.set_aat_capacitance(a, b)?;

        let amplitude = self.measure_amplitude_raw()?;
        let phase = self.measure_phase_raw()?;
        Ok((amplitude, phase))
    }

    pub fn tune_antennas(
        &mut self,
        config: aat::TunerSettings,
    ) -> Result<(), I, P> {
        registers::IoConfiguration::modify(&mut self.dev, |r| r.set_aat_en(true)).map_err(Error::Interface)?;
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
            // for some reason f32.round() is not compiling so we add 0.5 and truncate
            _ => Self::Measured(17 + ((1.0 - raw as f32 / 255.0) * 146.0 + 0.5) as u8),
        }
    }
}
