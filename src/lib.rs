#![no_std]

use bilge::arbitrary_int::u4;
use commands::DirectCommand;
use core::fmt::{Debug, Formatter};
use defmt::Format;
use embedded_hal::digital::InputPin;
use interface::Interface;
use registers::{
    operation_control::FieldDetectorControl, timer_emv_control::GptStart, Interrupt, Register,
};

pub type GPTDuration = fugit::Duration<u32, 59, 100_000_000>;

pub mod aat;
pub mod commands;
pub mod interface;
pub mod nfc_a;
pub mod registers;

pub use interface::SpiInterface;

#[derive(Clone, PartialEq)]
pub enum Error<I: Interface, P: InputPin> {
    Interface(I::Error),
    GPIO(P::Error),
    ExternalField,
    CollisionDetected,
    Timeout,
    LinkLost,
    Framing,
    Parity,
    Crc,
    IncompleteByte,
    NoMemory,
}

impl<I: Interface, P: InputPin> Debug for Error<I, P>
where
    I::Error: Debug,
    P::Error: Debug,
{
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Interface(i) => f.debug_tuple("Interface").field(i).finish(),
            Self::GPIO(i) => f.debug_tuple("GPIO").field(i).finish(),
            Self::ExternalField => f.write_str("ExternalField"),
            Self::CollisionDetected => f.write_str("CollisionDetected"),
            Self::Timeout => f.write_str("Timeout"),
            Self::LinkLost => f.write_str("LinkLost"),
            Self::Framing => f.write_str("Framing"),
            Self::Parity => f.write_str("Parity"),
            Self::Crc => f.write_str("Crc"),
            Self::IncompleteByte => f.write_str("IncompleteByte"),
            Self::NoMemory => f.write_str("NoMemory"),
        }
    }
}

impl<I: Interface, P: InputPin> Format for Error<I, P> {
    fn format(&self, f: defmt::Formatter) {
        match self {
            Self::Interface(_) => defmt::write!(f, "Interface error"),
            Self::GPIO(_) => defmt::write!(f, "GPIO error"),
            Self::ExternalField => defmt::write!(f, "ExternalField"),
            Self::CollisionDetected => defmt::write!(f, "CollisionDetected"),
            Self::Timeout => defmt::write!(f, "Timeout"),
            Self::LinkLost => defmt::write!(f, "LinkLost"),
            Self::Framing => defmt::write!(f, "Framing"),
            Self::Parity => defmt::write!(f, "Parity"),
            Self::Crc => defmt::write!(f, "Crc"),
            Self::IncompleteByte => defmt::write!(f, "IncompleteByte"),
            Self::NoMemory => defmt::write!(f, "NoMemory"),
        }
    }
}

pub type Result<T, I, P> = core::result::Result<T, Error<I, P>>;

#[derive(Debug)]
pub struct ST25R3916<I, P> {
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
        drv.dev.init_peripheral().map_err(Error::Interface)?;

        let identity: u8 = registers::IcIdentity::read(&mut drv.dev)
            .map_err(Error::Interface)?
            .into();
        defmt::info!("IC type {0=0..3}, revision {0=3..8}", identity);

        drv.enable_oscillator()?;

        // measure voltage
        let vdd = drv.measure_voltage(registers::regulator_control::MeasureSource::Vdd)?;
        defmt::debug!("Measured voltage: {=u16}mV", vdd);
        // set power supply level
        registers::IoConfiguration::modify(&mut drv.dev, |r| r.set_sup3v(vdd < 3600))
            .map_err(Error::Interface)?;
        // disable tx/rx
        registers::OperationControl::modify(&mut drv.dev, |reg| {
            reg.set_tx_en(false);
            reg.set_rx_en(false);
        })
        .map_err(Error::Interface)?;
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
        registers::IoConfiguration::modify(&mut self.dev, |reg| reg.set_out_cl(setting))
            .map_err(Error::Interface)
    }

    /// Measures requested voltage in millivolts
    fn measure_voltage(
        &mut self,
        source: registers::regulator_control::MeasureSource,
    ) -> Result<u16, I, P> {
        registers::RegulatorControl::modify(&mut self.dev, |r| r.set_mpsv(source))
            .map_err(Error::Interface)?;
        self.command_and_wait(DirectCommand::MeasurePowerSupply)?;

        let raw = registers::ADConverterOutput::read(&mut self.dev)
            .map_err(Error::Interface)?
            .value() as u16;
        // copied from the arduino library, each step is supposedly 23.4mV
        let mv = raw * 23 + ((raw * 4) + 5) / 10;

        Ok(mv)
    }

    fn clear_interrupts(&mut self) -> Result<(), I, P> {
        registers::InterruptRegister::read(&mut self.dev)
            .map_err(Error::Interface)
            .map(|_| ())
    }

    fn enable_interrupts(&mut self, int: Interrupt) -> Result<(), I, P> {
        let mask: u32 = registers::InterruptMask::read(&mut self.dev)
            .map_err(Error::Interface)?
            .into();
        let new_mask = mask & !u32::from(int);
        if mask != new_mask {
            registers::InterruptMask::from(new_mask)
                .write(&mut self.dev)
                .map_err(Error::Interface)
        } else {
            Ok(())
        }
    }
    fn disable_interrupts(&mut self, int: Interrupt) -> Result<(), I, P> {
        let mut mask: u32 = registers::InterruptMask::read(&mut self.dev)
            .map_err(Error::Interface)?
            .into();
        mask |= u32::from(int);
        registers::InterruptMask::from(mask)
            .write(&mut self.dev)
            .map_err(Error::Interface)?;
        Ok(())
    }
    fn wait_for_interrupt(&mut self, int: Interrupt) -> Result<(), I, P> {
        let int = u32::from(int);
        defmt::trace!(
            "Waiting for IRQs [{0=0..8:08b} {0=8..16:08b} {0=16..24:08b} {0=24..32:08b}]",
            int,
        );
        loop {
            let firing: u32 = self.wait_for_any_interrupt()?.into();
            defmt::trace!("Got IRQ, {=u32:032b}", firing);
            if firing & u32::from(int) != 0 {
                break Ok(());
            }
        }
    }
    fn wait_for_any_interrupt(&mut self) -> Result<Interrupt, I, P> {
        while self.irq.is_low().map_err(Error::GPIO)? {
            core::hint::spin_loop()
        }
        registers::InterruptRegister::read(&mut self.dev)
            .map(|r| r.val_0())
            .map_err(Error::Interface)
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
            registers::OperationControl::modify(&mut self.dev, |reg| reg.set_en(true))
                .map_err(Error::Interface)?;
            self.wait_for_interrupt(int)?;
            // defmt::debug!("Oscillator running");
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
            self.set_gpt(u16::MAX)?;
            self.start_gpt()?;
            self.wait_for_interrupt(irq)?;

            ticks -= u16::MAX as u32;
        }
        self.set_gpt(ticks as u16)?;
        self.start_gpt()?;
        self.wait_for_interrupt(irq)?;

        self.disable_interrupts(irq)
    }

    fn set_gpt(&mut self, ticks: u16) -> Result<(), I, P> {
        registers::GPTimer::from_ticks(ticks)
            .write(&mut self.dev)
            .map_err(Error::Interface)
    }

    fn start_gpt(&mut self) -> Result<(), I, P> {
        self.dev
            .direct_command(DirectCommand::StartGPT)
            .map_err(Error::Interface)
    }

    fn set_nrt(&mut self, ticks: u16) -> Result<(), I, P> {
        registers::NRTimer::from_ticks(ticks)
            .write(&mut self.dev)
            .map_err(Error::Interface)
    }

    fn set_guard_time(&mut self, micros: u32) -> Result<(), I, P> {
        // 151 us step, minimum of 75us
        let ticks: u8 = (micros - 75).div_ceil(151) as u8;
        registers::NFCFieldOnGuardTimer::new(ticks)
            .write(&mut self.dev)
            .map_err(Error::Interface)
    }

    fn set_mrt(&mut self, ticks: u8) -> Result<(), I, P> {
        registers::MRTimer::new(ticks)
            .write(&mut self.dev)
            .map_err(Error::Interface)
    }

    /// Adjust regulators and returns the regulated voltage
    pub fn adjust_regulators(&mut self) -> Result<u16, I, P> {
        registers::RegulatorControl::modify(&mut self.dev, |r| {
            r.set_reg_s(registers::regulator_control::RegulatedSettingSource::AdjustRegulators)
        })
        .map_err(Error::Interface)?;
        self.command_and_wait(DirectCommand::AdjustRegulators)?;

        let regulator_output = registers::RegulatorDisplay::read(&mut self.dev)
            .map_err(Error::Interface)?
            .reg();

        let base_voltage = if registers::IoConfiguration::read(&mut self.dev)
            .map_err(Error::Interface)?
            .sup3v()
        {
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
        let rx_conf_bak =
            registers::ReceiverConfiguration::read(&mut self.dev).map_err(Error::Interface)?;
        let aux_bak =
            registers::AuxiliaryModulationSetting::read(&mut self.dev).map_err(Error::Interface)?;

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
        let amplitude = registers::ADConverterOutput::read(&mut self.dev)
            .map_err(Error::Interface)?
            .value();

        // restore registers
        op_bak.write(&mut self.dev).map_err(Error::Interface)?;
        mode_bak.write(&mut self.dev).map_err(Error::Interface)?;
        rx_conf_bak.write(&mut self.dev).map_err(Error::Interface)?;
        aux_bak.write(&mut self.dev).map_err(Error::Interface)?;

        Ok(amplitude)
    }

    pub fn measure_phase_raw(&mut self) -> Result<u8, I, P> {
        self.command_and_wait(DirectCommand::MeasurePhase)?;
        registers::ADConverterOutput::read(&mut self.dev)
            .map(|r| r.value())
            .map_err(Error::Interface)
    }

    /// Measures the phase difference between the RFO (output) and RFI (input) signals
    pub fn measure_phase(&mut self) -> Result<PhaseDifference, I, P> {
        self.measure_phase_raw().map(PhaseDifference::from_raw)
    }

    /// Sets the AAT capacitors' DAC output
    pub fn set_aat_capacitance(&mut self, a: u8, b: u8) -> Result<(), I, P> {
        let reg = registers::AntennaTuningControl::new(a, b);
        reg.write(&mut self.dev).map_err(Error::Interface)?;
        // wait for variable capacitors to adjust
        self.delay(GPTDuration::millis(10))
    }

    /// Sets the AAT capacitors' DAC output and measures the new amplitude and phase
    pub fn set_capacitance_and_measure(&mut self, a: u8, b: u8) -> Result<(u8, u8), I, P> {
        self.set_aat_capacitance(a, b)?;

        let amplitude = self.measure_amplitude_raw()?;
        let phase = self.measure_phase_raw()?;
        Ok((amplitude, phase))
    }

    pub fn tune_antennas(&mut self, config: aat::TunerSettings) -> Result<(), I, P> {
        registers::IoConfiguration::modify(&mut self.dev, |r| r.set_aat_en(true))
            .map_err(Error::Interface)?;
        let mut state = aat::TunerState::new_from_settings(&config, self)?;
        while !state.is_done() {
            let best_dir = aat::find_best_step(self, &mut state, &config /*, prev_dir*/)?;

            if let Some(d) = best_dir {
                while aat::try_greedy_step(self, &mut state, &config, d)? {}
            }

            state.halve_steps();
        }
        Ok(())
    }

    pub fn into_iso14443a_initiator(mut self) -> Result<nfc_a::Iso14443aInitiator<I, P>, I, P> {
        // no need to disable wakeup mode if it's not enabled
        registers::ModeDefinition::modify(&mut self.dev, |r| {
            r.set_om(registers::mode_definition::OperationMode::InitiatorIso14443A);
            r.set_tr_am(registers::mode_definition::ModulationMode::OOK);
        })
        .map_err(Error::Interface)?;
        registers::TxDriver::modify(&mut self.dev, |r| {
            r.set_am_modulation(registers::tx_driver::AmModulation::Percent82)
        })
        .map_err(Error::Interface)?;
        registers::AWSConfig::modify(&mut self.dev, |r| {
            r.set_am_sym(false);
            r.set_en_modsink(true);
            r.set_am_filter(u4::new(8));
        })
        .map_err(Error::Interface)?;

        registers::Auxiliary::modify(&mut self.dev, |r| r.set_dis_corr(false))
            .map_err(Error::Interface)?;

        // 5ms guard time
        self.set_guard_time(nfc_a::GUARD_TIME_US)?;

        // configure gpt as fdt
        let gpt_ticks = nfc_a::FDT_POLL_POLLER - nfc_a::FDT_POLL_ADJUSTMENT;
        self.set_gpt(gpt_ticks as u16)?;
        registers::TimerEMVControl::modify(&mut self.dev, |r| r.set_gpt_start(GptStart::RxEnd))
            .map_err(Error::Interface)?;

        Ok(nfc_a::Iso14443aInitiator(self))
    }

    fn field_on_and_wait_gt(&mut self) -> Result<(), I, P> {
        // skip if field is already on
        let mut op_ctr =
            registers::OperationControl::read(&mut self.dev).map_err(Error::Interface)?;
        if op_ctr.tx_en() {
            return Ok(());
        }

        op_ctr.set_en_fd_c(FieldDetectorControl::ManualCollisionAvoidance);
        op_ctr.write(&mut self.dev).map_err(Error::Interface)?;
        // TODO: wrapping auto-incrmenting RF wait time to avoid aliasing or something
        let mut irqs_enabled = Interrupt::default();
        irqs_enabled.set_cac(true);
        irqs_enabled.set_cat(true);
        irqs_enabled.set_apon(true);

        self.clear_interrupts()?;
        self.enable_interrupts(irqs_enabled)?;
        self.dev
            .direct_command(DirectCommand::InitialFieldOn)
            .map_err(Error::Interface)?;
        let firing = self.wait_for_any_interrupt()?;

        if firing.cac() {
            Err(Error::ExternalField)
        } else if firing.apon() {
            let mut cat = Interrupt::default();
            cat.set_cat(true);
            // TODO: sanity timeouts, somehow (no ehal-1 support yet)
            self.wait_for_interrupt(cat)
        } else {
            // no more enabled interrupts (cat can't happend before apon)
            unreachable!()
        }?;

        self.disable_interrupts(irqs_enabled)?;

        registers::OperationControl::modify(&mut self.dev, |r| {
            defmt::assert!(r.tx_en());
            r.set_rx_en(true);
            r.set_en_fd_c(FieldDetectorControl::AutoEnable);
        })
        .map_err(Error::Interface)
    }
    pub fn field_off(&mut self) -> Result<(), I, P> {
        registers::OperationControl::modify(&mut self.dev, |r| {
            r.set_tx_en(false);
            r.set_rx_en(false);
        })
        .map_err(Error::Interface)
    }
    fn set_tx_bits(&mut self, bits: u16) -> Result<(), I, P> {
        registers::NumTxBytes::from_bits(bits)
            .write(&mut self.dev)
            .map_err(Error::Interface)
    }
    fn read_fifo(&mut self, buf: &mut [u8]) -> Result<(u16, u8), I, P> {
        let fifo_status = registers::FifoStatus::read(&mut self.dev).map_err(Error::Interface)?;
        let (bytes, bits) = fifo_status.capacity();

        let len = bytes as usize + if bits != 0 { 1 } else { 0 };
        if buf.len() < len {
            return Err(Error::NoMemory);
        }
        self.dev
            .fifo_read(&mut buf[..len])
            .map_err(Error::Interface)?;

        Ok((bytes, bits))
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
