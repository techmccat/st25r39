// use crate::st25r31916::Interface;
use bilge::prelude::*;
use defmt::Format;

use helper_traits::*;

#[derive(Debug, PartialEq, Format)]
pub enum RegisterSpace {
    A,
    B,
}

impl RegisterSpace {
    pub const fn prefix(&self) -> &[u8] {
        match self {
            RegisterSpace::A => &[],
            RegisterSpace::B => &[0xFB],
        }
    }
}

pub mod helper_traits {
    pub trait MemStorage {
        type ByteArray: AsRef<[u8]> + AsMut<[u8]> + Default;
    }
    pub trait ToLeBytes: MemStorage {
        fn to_le_bytes(self) -> Self::ByteArray;
    }
    pub trait FromLeBytes: MemStorage {
        fn from_le_bytes(val: Self::ByteArray) -> Self;
    }
}

// i don't like having to specify the assocaited type on every call but i haven't found better ways
// to semi-automatically implement the trait for registers which are Into<Integer>
pub trait Register: Sized + FromLeBytes + ToLeBytes {
    const ADDRESS: u8;
    const SPACE: RegisterSpace;
    fn read<I: super::Interface>(iface: &mut I) -> Result<Self, I::Error> {
        let mut buf = Self::ByteArray::default();
        iface.register_read(Self::ADDRESS, Self::SPACE, &mut buf.as_mut())?;
        Ok(Self::from_le_bytes(buf))
    }
    fn write<I: super::Interface>(self, iface: &mut I) -> Result<(), I::Error> {
        iface.register_write(Self::ADDRESS, Self::SPACE, self.to_le_bytes().as_ref())
    }
    fn modify<I: super::Interface>(
        iface: &mut I,
        mut f: impl FnMut(&mut Self),
    ) -> Result<(), I::Error> {
        let mut reg = Self::read::<I>(iface)?;
        f(&mut reg);
        reg.write::<I>(iface)?;
        Ok(())
    }
}

macro_rules! register_impl {
    ($type:ty, $inner:ty, $addr:literal, $space:ident) => {
        impl MemStorage for $type {
            type ByteArray = [u8; core::mem::size_of::<Self>()];
        }
        impl FromLeBytes for $type {
            fn from_le_bytes(bytes: Self::ByteArray) -> Self {
                Self {
                    value: <$inner>::from_le_bytes(bytes),
                }
            }
        }
        impl ToLeBytes for $type {
            fn to_le_bytes(self) -> Self::ByteArray {
                self.value.to_le_bytes()
            }
        }
        impl Register for $type {
            const ADDRESS: u8 = $addr;
            const SPACE: RegisterSpace = RegisterSpace::$space;
        }
    };
}

pub mod io_configuration {
    use bilge::prelude::*;
    use defmt::Format;

    #[bitsize(2)]
    #[derive(FromBits, Debug, Format, Clone, Copy, Default)]
    pub enum OutClk {
        #[default]
        Mhz3_39 = 0b00,
        Mhz6_78 = 0b01,
        Mhz13_56 = 0b10,
        Disabled = 0b11,
    }

    #[bitsize(2)]
    #[derive(FromBits, Debug, Format, Clone, Copy, Default)]
    pub enum I2cThd {
        #[default]
        T380ns160ns = 0b00,
        T180ns160ns = 0b01,
        T180ns70ns = 0b10,
        T100ns70ns = 0b11,
    }

    #[bitsize(1)]
    #[derive(FromBits, Debug, Format, Clone, Copy, Default)]
    pub enum AmRef {
        #[default]
        VddDr = 0,
        VddRf = 1,
    }

    #[bitsize(1)]
    #[derive(FromBits, Debug, Format, Clone, Copy, Default)]
    pub enum IoDriverLevel {
        #[default]
        Normal = 0,
        Increased = 1,
    }
}

register_impl!(IoConfiguration, u16, 0x00, A);
/// Controls I/O parameters
#[bitsize(16)]
#[derive(FromBits, DebugBits, Format, Clone, Copy, Default)]
pub struct IoConfiguration {
    /// LF clock on MCU_CLK
    ///
    /// By default the 32 kHz LF clock is present on MCU_CLK output when
    /// Xtal oscillator is not running and the MCU_CLK output is not disabled.
    pub lf_clk_off: bool,
    /// Selection of clock frequency on MCU_CLK
    pub out_cl: io_configuration::OutClk,
    reserved: u1,
    /// Non hs-modes / hs-modes
    pub i2c_thd: io_configuration::I2cThd,
    /// Chooses the antenna driver in case of single driving
    ///
    /// 0: RF01, RFI1  
    /// 1: RF02, RFI2
    pub rfo: u1,
    /// Disable differential antenna driving
    pub single: bool,
    /// Active sink enabled
    ///
    /// Set to 1 when Regulator modulation is intended (reg_am=1)  
    /// and big capacitor is connected to VDD_AM (2.2 uF)
    pub act_amsink: bool,
    /// Selects non modulated RF voltage level reference of the VDD_AM voltage regulator.
    pub am_ref: io_configuration::AmRef,
    /// Increases IO driver strength of MISO, MCU_CLK an IRQ.
    ///
    /// Recommended to set to 1 for all I2C operation, and for SPI operation if VDD_IO < 3.3 V.
    pub io_drv_lvl: io_configuration::IoDriverLevel,
    /// Pull-down on MISO when BSS is high
    pub miso_pd1: bool,
    /// Pull-down on MISO, when BSS is low and MISO is not driven by the ST25R39xxB
    pub miso_pd2: bool,
    /// Enable AAT D/A converters if both `aat_en` and `en` are set
    ///
    /// Is `en` is not set AAT outputs are set to a fixed value
    pub aat_en: bool,
    /// Disable VDD_D regulator
    ///
    /// When this bit is set, VDD_D and VDD_A must be shorted externally
    pub vspd_off: bool,
    /// Power supply voltage
    ///
    /// false: 3.6V < VDD <= 5.5V  
    /// true: 2.4V < VDD <= 3.6V
    pub sup3v: bool,
}

pub mod operation_control {
    use bilge::prelude::*;
    use defmt::Format;

    #[bitsize(2)]
    #[derive(FromBits, Debug, Format, Clone, Copy, Default)]
    pub enum FieldDetectorControl {
        /// External field detector off
        #[default]
        FieldOff = 0b00,
        /// Manually enable with collision avoidance detection threshold
        ManualCollisionAvoidance = 0b01,
        /// Manually enable with peer detection threshold
        ManualPeerDetection = 0b10,
        /// Enable external field detector automatically
        AutoEnable = 0b11,
    }

    #[bitsize(1)]
    #[derive(FromBits, Debug, Format, Clone, Copy, Default)]
    pub enum RxChanEnable {
        /// Both AM and PM channels are enabled
        #[default]
        Both = 0,
        /// Only one channel is enabled
        One = 1,
    }
}

register_impl!(OperationControl, u8, 0x02, A);
/// Operation control register
#[bitsize(8)]
#[derive(FromBits, DebugBits, Format, Clone, Copy, Default)]
pub struct OperationControl {
    /// External field detector control
    pub en_fd_c: operation_control::FieldDetectorControl,
    #[doc(alias = "wu")]
    pub wakeup_enable: bool,
    /// Automatically set by NFC field on commands
    pub tx_en: bool,
    /// Manual RX channel selection if both channels are enabled
    pub rx_man: bool,
    pub rx_chn: operation_control::RxChanEnable,
    /// Enables RX operation
    pub rx_en: bool,
    /// Enables oscillator and regulator
    pub en: bool,
}

pub mod mode_definition {
    use bilge::prelude::*;
    use defmt::Format;

    #[bitsize(2)]
    #[derive(FromBits, Debug, Format, Clone, Copy, Default)]
    pub enum NfcAutomaticResponse {
        #[default]
        Off = 0b00,
        AutoAfterReception = 0b01,
        AlwaysAfterPeerOff = 0b10,
        Reserved = 0b11,
    }

    #[bitsize(1)]
    #[derive(FromBits, Debug, Format, Clone, Copy, Default)]
    pub enum ModulationMode {
        #[default]
        OOK = 0,
        AAM = 1,
    }

    #[bitsize(5)]
    #[repr(u8)]
    #[derive(FromBits, Debug, Clone, Copy, Default)]
    pub enum OperationMode {
        InitiatorActiveNfcIP1 = 0b00000,
        #[default]
        InitiatorIso14443A = 0b00001,
        InitiatorIso14443B = 0b00010,
        InitiatorFelica = 0b00011,
        /// NFC Forum Type 1 tag
        InitiatorTopaz = 0b00100,
        InitiatorSubcarrierStream = 0b01110,
        InitiatorBPSKStream = 0b01111,
        TargetPassiveIso14443A = 0b10001,
        TargetPassiveFelica = 0b10100,
        TargetActiveNfcIP1 = 0b10111,
        #[fallback]
        Other(u5),
    }

    impl Format for OperationMode {
        fn format(&self, fmt: defmt::Formatter) {
            defmt::write!(
                fmt,
                "om is a mess: {{ {=u8:05b} }}",
                u5::from(*self).value()
            )
        }
    }

    impl OperationMode {
        pub fn is_bitrate_detect(&self) -> bool {
            if let OperationMode::Other(val) = self {
                *val & u5::new(0b11000) != u5::new(0)
            } else {
                false
            }
        }
    }

    impl From<BitrateDetect> for OperationMode {
        fn from(value: BitrateDetect) -> Self {
            let repr: u3 = value.into();
            let val: u5 = repr.widen::<5>() | u5::new(0b11000);
            OperationMode::Other(val)
        }
    }

    impl TryInto<BitrateDetect> for OperationMode {
        type Error = ();
        fn try_into(self) -> Result<BitrateDetect, Self::Error> {
            match self {
                Self::Other(val) => Ok(BitrateDetect::from(u3::new(val.value() & 0b111))),
                _ => Err(()),
            }
        }
    }

    #[bitsize(3)]
    #[derive(FromBits, DebugBits, Clone, Copy)]
    pub struct BitrateDetect {
        pub iso14443a: bool,
        reserved: u1,
        pub felica: bool,
    }
}

register_impl!(ModeDefinition, u8, 0x03, A);
/// Controls NFC mode, moduleation and automatic response
#[bitsize(8)]
#[derive(FromBits, DebugBits, Format, Clone, Copy, Default)]
pub struct ModeDefinition {
    /// Automatically starts the Response RF collision
    // Refer to datasheet section 4.4.5 for handling
    pub nfc_ar: mode_definition::NfcAutomaticResponse,
    /// NFC modulation mode
    pub tr_am: mode_definition::ModulationMode,
    /// Selection of target and operation modes
    pub om: mode_definition::OperationMode,
}

pub mod bitrate_definition {
    use bilge::prelude::*;
    use defmt::Format;

    #[bitsize(2)]
    #[derive(FromBits, Debug, Format, Clone, Copy, Default)]
    pub enum Bitrate {
        #[default]
        Kbps106 = 0b00,
        Kbps212 = 0b01,
        Kbps424 = 0b10,
        Kbps848 = 0b11,
    }
}

register_impl!(BitrateDefinition, u8, 0x04, A);
#[bitsize(8)]
#[derive(FromBits, DebugBits, Format, Clone, Copy, Default)]
pub struct BitrateDefinition {
    pub rx_rate: bitrate_definition::Bitrate,
    reserved: u2,
    pub tx_rate: bitrate_definition::Bitrate,
    reserved: u2,
}

register_impl!(Iso14443ASettings, u8, 0x05, A);
/// ISO14443A and NFC 106kb/s settings register
#[bitsize(8)]
#[derive(FromBits, DebugBits, Format, Clone, Copy, Default)]
pub struct Iso14443ASettings {
    /// Standard or ISO14443A anticollision frame
    #[doc(alias = "antcl")]
    pub anticollision: bool,
    /// Modulation pulse width
    ///
    /// Width is defined in number of 13.39Mhz clock cycles
    #[doc(alias = "p_len")]
    pulse_width: u4,
    /// Support of NFCIP-1 Transport Frame format
    ///
    /// Must not be set in bitrate detection mode
    nfc_f0: bool,
    /// Only supported for 106kb/s data rate
    no_rx_parity: bool,
    /// Transmission is to be done with the transmit without CRC command
    no_tx_parity: bool,
}

pub mod iso14443b_settings {
    use bilge::prelude::*;
    use defmt::Format;

    /// Encoded as 48 + 16*len
    ///
    /// Also valid for NFCIP-1 active communication bit rates 212 and 424 kb/s
    #[bitsize(2)]
    #[derive(Debug, FromBits, Format, Clone, Copy, Default)]
    pub enum FelicaPreambleLength {
        #[default]
        B48 = 0b00,
        B64 = 0b01,
        B80 = 0b10,
        B96 = 0b11,
    }
}

register_impl!(Iso14443BSettings, u16, 0x06, A);
/// ISO14443B settings register
#[bitsize(16)]
#[derive(FromBits, DebugBits, Format, Clone, Copy, Default)]
pub struct Iso14443BSettings {
    reserved: u1,
    /// Sets SOF and EOF settings in middle of specification
    ///
    /// false: SOF and EOF defined by sof_0, sof_1, and eof bit  
    /// true: SOF 10.5 etu logic 0, 2.5 etu logic 1, EOF: 10.5 etu logic 0
    pub half: bool,
    /// EOF, number of etu with logic 0
    pub eof: u1,
    /// SOF, number of etu with logic 1
    #[doc(alias = "sof_1")]
    pub sof_logic_1: u1,
    /// SOF, number of etu with logic 0
    #[doc(alias = "sof_0")]
    pub sof_logic_0: u1,
    /// EGT defined in number of etu
    pub egt: u3,
    /// FeliCa preamble length
    #[doc(alias = "f_p")]
    pub felica_preamble_len: iso14443b_settings::FelicaPreambleLength,
    reserved: u2,
    /// Support PICC (card) not sending EOF bit to PCD (reader)
    pub no_eof: bool,
    /// Support PICC (card) not sending SOF bit to PCD (reader)
    pub no_sof: bool,
    /// Minimum TR1 codings
    ///
    /// | tr1 | fc/128 | >fc/128 |
    /// | --- | ------ | ------- |
    /// | 0 | 80/fs | 80/fs |
    /// | 1 | 64/fs | 32/fs |
    pub tr1: u1,
    reserved: u1,
}

register_impl!(NfcIp1Passive, u8, 0x08, A);
/// NFCIP-1 passive target definition register
///
/// Disabling automatic responses (anti-collison and SENSF_RES)
/// in passive target mode makes the reader operate completely via the FIFO
#[bitsize(8)]
#[derive(FromBits, DebugBits, Format, Clone, Copy, Default)]
pub struct NfcIp1Passive {
    /// Disable automatic anti-collision detection
    #[doc(alias = "d_106_ac_a")]
    pub nfca_auto_anticollision_disable: bool,
    reserved: u1,
    /// Enable automatic SENSF_RES
    #[doc(alias = "d_106_ac_a")]
    pub auto_sensf_res: bool,
    /// Enable AP2P frame recognition
    #[doc(alias = "d_ac_ap2p")]
    pub ap2p_frame_recognition_disable: bool,
    /// PCD to PICC (reader to card) FDT compensation
    ///
    /// Encoded as `fdel`/fc
    ///
    /// In NFC-A mode `fdel` > 0 shortens FDT provided by logic  
    /// `fdel = 2` is expected to be a good setting
    #[doc(alias = "fdel")]
    pub fdt_compensation: u4,
}

pub mod stream_mode {
    use bilge::prelude::*;
    use defmt::Format;

    #[bitsize(3)]
    #[repr(u8)]
    #[derive(FromBits, Debug, Clone, Copy, Default)]
    pub enum TxModulatorTimePeriod {
        #[default]
        Fc128 = 0b00,
        Fc64 = 0b01,
        Fc32 = 0b10,
        Fc16 = 0b11,
        #[fallback]
        Reserved(u3),
    }

    impl Format for TxModulatorTimePeriod {
        fn format(&self, fmt: defmt::Formatter) {
            let val: u3 = (*self).into();
            defmt::write!(fmt, "{=u8:03b}", val.value());
        }
    }

    /// Number of subcarrier pulses per report period for subcarrier stream mode
    #[bitsize(2)]
    #[repr(u8)]
    #[derive(FromBits, Debug, Clone, Copy, Default)]
    pub enum SubcarrierPulseCount {
        #[default]
        N4 = 0b10,
        N8 = 0b11,
        #[fallback]
        Reserved(u2),
    }

    impl Format for SubcarrierPulseCount {
        fn format(&self, fmt: defmt::Formatter) {
            let val: u2 = (*self).into();
            defmt::write!(fmt, "{=u8:02b}", val.value());
        }
    }

    #[bitsize(2)]
    #[repr(u8)]
    #[derive(FromBits, Debug, Clone, Copy, Default)]
    pub enum SubcarrierFrequency {
        /// fc/16, only valid for BPSK mode
        #[default]
        Bpsk848khz = 0b00,
        /// fc/32, only valid for transparent mode
        Subcarrier424khz = 0b01,
        #[fallback]
        Reserved(u2),
    }

    impl Format for SubcarrierFrequency {
        fn format(&self, fmt: defmt::Formatter) {
            let val: u2 = (*self).into();
            defmt::write!(fmt, "{=u8:02b}", val.value());
        }
    }
}

register_impl!(StreamMode, u8, 0x09, A);
/// Stream mode definition register
///
/// Configuration of parameters for subcarrier stream (transparent) mode
#[bitsize(8)]
#[derive(FromBits, DebugBits, Format, Clone, Copy, Default)]
pub struct StreamMode {
    /// Time period for Tx modulator control
    pub stx: stream_mode::TxModulatorTimePeriod,
    /// Number of subcarrier pulses per report period
    pub scp: stream_mode::SubcarrierPulseCount,
    /// Subcarrier frequency definiton
    pub scf: stream_mode::SubcarrierFrequency,
    reserved: u1,
}

pub mod auxiliary {
    use bilge::prelude::*;
    use defmt::Format;

    /// NFCID1 Size
    #[bitsize(2)]
    #[repr(u8)]
    #[derive(FromBits, Debug, Clone, Copy, Format, Default)]
    pub enum NfcIdSize {
        /// 4 bytes
        #[default]
        B4 = 0b00,
        /// 7 bytes
        B7 = 0b01,
        #[fallback]
        Reserved,
    }
}

register_impl!(Auxiliary, u8, 0x0A, A);
/// Auxiliary definition register
#[bitsize(8)]
#[derive(FromBits, DebugBits, Format, Clone, Copy, Default)]
pub struct Auxiliary {
    /// Value for direct commands NFC Initial Field On and NFC Response Field On
    pub nfc_n: u2,
    /// RW receiver operation
    ///
    /// false: correlation operation  
    /// true: reserved
    pub dis_corr: bool,
    /// Shift clock 90° for phase measurement
    #[doc(alias = "mfaz_90")]
    pub measure_phase_shift_90: bool,
    pub nfc_id_size: auxiliary::NfcIdSize,
    reserved: u1,
    /// Receive without CRC check
    ///
    /// Done automatically for ISO14443A REQA, WUPA
    pub no_crc_rx: bool,
}

register_impl!(EmdSuppressionConfig, u8, 0x05, B);
/// EMD Suppression Control register
#[bitsize(8)]
#[derive(FromBits, DebugBits, Format, Clone, Copy, Default)]
pub struct EmdSuppressionConfig {
    /// Minimum received frame length
    ///
    /// Must be set to 4 for EMVCo compliance.
    #[doc(alias = "emd_thld")]
    pub emv_thld: u4,
    reserved: u2,
    /// Reception enabled even if the first 4 bits of the frame contain an error
    ///
    /// Applies to ISO-A 106k only.  
    /// Must be set to 1 for EMVCo compliance.
    pub rx_start_emv: bool,
    /// Enable EMD suppression according to EMVCo
    ///
    /// RX parity anc CRC myst be turned on, bit nrt_emv must be set
    pub emd_emv: bool,
}

register_impl!(SubcarrierStartTimer, u8, 0x06, B);
/// Subcarrier start timer register
#[bitsize(8)]
#[derive(FromBits, DebugBits, Format, Clone, Copy, Default)]
pub struct SubcarrierStartTimer {
    /// Subcarrier start time
    ///
    /// If the time from the end of the MRT timer to the detection of a subcarrier
    /// is shorter than sst<4:0>, then a soft error interrupt is generated.  
    /// If emd_emv = 1 the frame will be suppressed as EMD
    /// and a restart interrupt will be generated.  
    /// Note that corr_s3 defines the length of subcarrier start detection
    /// and affects the correct sst<4:0> setting.
    ///
    /// Step: 0.25 etu, range: 0 etu to 7.75 etu  
    /// Applies to ISO14443B 106kb/s
    pub sst: u5,
    reserved: u3,
}

pub mod receiver_configuration {
    use bilge::prelude::*;
    use defmt::Format;

    /// First and third stage high pass filtering settings
    #[bitsize(4)]
    #[repr(u8)]
    #[derive(FromBits, Debug, Clone, Copy, Default)]
    pub enum FirstThirdZero {
        /// First stage zero: 60kHz, third stage zero: 400kHz
        #[default]
        First60Third400 = 0b0000,
        /// First stage zero: 60kHz, third stage zero: 200kHz
        First60Third200 = 0b0100,
        /// First stage zero: 40kHz, third stage zero: 80kHz
        First40Third80 = 0b0010,
        /// First stage zero: 12kHz, third stage zero: 200kHz
        First12Third200 = 0b0101,
        /// First stage zero: 12kHz, third stage zero: 80kHz
        First12Third80 = 0b0011,
        /// First stage zero: 600kHz, third stage zero: 400kHz
        First600Third400 = 0b1000,
        /// First stage zero: 600kHz, third stage zero: 400kHz
        First600Third200 = 0b1100,
        #[fallback]
        NotUsed(u4),
    }
    impl Format for FirstThirdZero {
        fn format(&self, fmt: defmt::Formatter) {
            let val: u4 = (*self).into();
            defmt::write!(fmt, "{=u8:04b}", val.value());
        }
    }

    /// -1dB point for the low-pass filter
    #[bitsize(3)]
    #[derive(FromBits, Debug, Clone, Copy, Default, Format)]
    pub enum LowPassControl {
        #[default]
        KHz1200 = 0b000,
        KHz600 = 0b001,
        KHz300 = 0b010,
        MHz2 = 0b100,
        MHz7 = 0b101,
        #[fallback]
        NotUsed,
    }

    #[bitsize(1)]
    #[derive(Format, FromBits, Debug, Clone, Copy, Default)]
    pub enum ChannelSelect {
        #[default]
        ChannelAM = 0,
        ChannelPM = 1,
    }

    #[bitsize(1)]
    #[derive(Format, FromBits, Debug, Clone, Copy, Default)]
    pub enum AGCRatio {
        #[default]
        Ratio3 = 0,
        Ratio6 = 1,
    }

    #[bitsize(1)]
    #[derive(Format, FromBits, Debug, Clone, Copy, Default)]
    pub enum AGCAlgorithm {
        #[default]
        Preset = 0,
        Reset = 1,
    }

    #[bitsize(1)]
    #[derive(Format, FromBits, Debug, Clone, Copy, Default)]
    pub enum AGCDuration {
        /// AGC operates on first 8 subcarrier pulses
        First8 = 0,
        /// AGC operates during complete receive period
        #[default]
        CompleteRx = 1,
    }

    #[bitsize(1)]
    #[derive(Format, FromBits, Debug, Clone, Copy, Default)]
    pub enum SquelchRatio {
        /// Recommended for ISO-A 106k correlator, ISO-A HBR/ISO-B pulse decoder,
        /// ISO-15693, and FeliCa
        #[default]
        Ratio1 = 0,
        /// Recommended for ISO-A HBR/ISO-B correlator
        Ratio6to3 = 1,
    }

    #[bitsize(1)]
    #[derive(Format, FromBits, Debug, Clone, Copy, Default)]
    pub enum AMDemodulatorSelect {
        #[default]
        PeakDetector = 0,
        Mixer = 1,
    }

    #[bitsize(1)]
    #[derive(Format, FromBits, Debug, Clone, Copy, Default)]
    pub enum DemodulationMode {
        /// AM/PM demodulation
        #[default]
        AMPM = 0,
        /// I/Q demodulation
        ///
        /// Requires amd_sel = Mixer (1)
        IQ = 1,
    }

    #[bitsize(1)]
    #[derive(Format, FromBits, Debug, Clone, Copy, Default)]
    pub enum LFOperation {
        #[default]
        Differential = 0,
        /// LF input split (RFI1 to AM channel, RFI2 to PM channel)
        Split = 1,
    }
}

register_impl!(ReceiverConfiguration, u32, 0x0B, A);
/// Receiver configuration register
#[bitsize(32)]
#[derive(FromBits, DebugBits, Format, Clone, Copy)]
pub struct ReceiverConfiguration {
    /// Bits z12k, z600k, h80 and h200 according to table 6 of the datasheet
    ///
    /// 12k/200k encoding was duplicated, the library uses the one with the h200 bit set
    pub high_pass_config: receiver_configuration::FirstThirdZero,
    #[doc(alias = "lp")]
    pub low_pass_control: receiver_configuration::LowPassControl,
    /// Select channel
    ///
    /// Depending on [`OperationControl`].rx_chn the channel is either the only one enabled or the
    /// one selected between the two enabled
    pub ch_sel: receiver_configuration::ChannelSelect,
    /// Select Automatic Gain Control trigger level
    ///
    /// AGC triggers on signals 3 or 6 times above the minimum detectable signal level.
    pub agc6_3: receiver_configuration::AGCRatio,
    /// Select algorithm with preset or reset
    ///
    /// Algorithm with preset is recommended for protocols with short SOF
    /// (like ISO14443A fc / 128).
    pub agc_alg: receiver_configuration::AGCAlgorithm,
    /// Duration of AGC operation
    pub agc_m: receiver_configuration::AGCDuration,
    /// Enable Automatic Gain Correction
    pub agc_en: bool,
    /// Select squelch trigger level (signal to digitizer threshold ratio)
    #[doc(alias = "pulz_61")]
    pub squelch_ratio: receiver_configuration::SquelchRatio,
    /// Enable dynamic squelch activation after end of transmission
    ///
    /// false disables squelch entirely
    ///
    /// Squelch is activated 18.88 μs after end of TX,
    /// and stops when the Mask receive timer reaches the sqt<7:0> setting.
    #[doc(alias = "sqm_dyn")]
    pub squelch_enable: bool,
    /// Selects AM demodulator
    pub amd_sel: receiver_configuration::AMDemodulatorSelect,
    /// Selects demodulator operation mode
    pub demod_mode: receiver_configuration::DemodulationMode,
    /// LF Operation selection
    pub lf_op: receiver_configuration::LFOperation,
    /// Enable Low Frequency mode (default mode is high frequency)
    pub lf_en: bool,
    /// Control gain in the first gain stage of the PM channel
    ///
    /// 0..=6 reduce gain in steps of 2.5dB, 7 boosts gain by 5.5dB
    pub rg1_pm: u3,
    /// Control gain in the first gain stage of the AM channel
    ///
    /// 0..=6 reduce gain in steps of 2.5dB, 7 boosts gain by 5.5dB
    pub rg1_am: u3,
    /// PM channel gain reduction in second and third stages of the digitizer
    ///
    /// Valid values are from 0 to 10  
    /// - Values from 1 to 4 reduce gain by increasing the digitizer window in 3 dB steps
    /// - Values from 5 to 10 additionally reduce the gain in second and third gain stage, in 3dB
    /// steps
    pub rg2_pm: u4,
    /// AM channel gain reduction in second and third stages of the digitizer
    ///
    /// Valid values are from 0 to 10  
    /// - Values from 1 to 4 reduce gain by increasing the digitizer window in 3 dB steps
    /// - Values from 5 to 10 additionally reduce the gain in second and third gain stage, in 3dB
    /// steps
    pub rg2_am: u4,
}

impl Default for ReceiverConfiguration {
    fn default() -> Self {
        let mut ret = Self::from(0);
        ret.set_agc_en(true);
        ret.set_agc_m(receiver_configuration::AGCDuration::CompleteRx);
        ret.set_squelch_enable(true);
        ret.set_rg1_pm(u3::new(6));
        ret.set_rg1_am(u3::new(6));
        ret
    }
}

pub mod p2p_receiver_config {
    use bilge::prelude::*;
    use defmt::Format;

    #[bitsize(1)]
    #[derive(Format, FromBits, Debug, Clone, Copy, Default)]
    pub enum AskThreshold {
        #[default]
        Percent97 = 0,
        Percent95 = 1,
    }

    #[bitsize(2)]
    #[derive(Format, FromBits, Debug, Clone, Copy, Default)]
    pub enum AskRCConstant {
        Us8_4 = 0b00,
        Us6_8 = 0b01,
        #[default]
        Us4_4 = 0b10,
        Us2_4 = 0b11,
    }

    #[bitsize(2)]
    #[derive(Format, FromBits, Debug, Clone, Copy, Default)]
    pub enum OokRCConstant {
        #[default]
        Us1_4 = 0b00,
        Us1_0 = 0b01,
        Us0_6 = 0b10,
        Us0_2 = 0b11,
    }
}

register_impl!(P2PRxConfig, u8, 0x0B, B);
/// Peer to peer receiver configuration
#[bitsize(8)]
#[derive(FromBits, DebugBits, Format, Clone, Copy)]
pub struct P2PRxConfig {
    /// ASK threshold level
    pub ask_thd: p2p_receiver_config::AskThreshold,
    /// ASK RC time constant
    pub ask_rc: p2p_receiver_config::AskRCConstant,
    /// OOK threshold, depending on ook_rc
    ///
    /// | ook\_thd | ook\_rc = 0 | ook_rc > 0 |
    /// | --- | --- | --- |
    /// | 0b00 | 55% | 80% |
    /// | 0b01 | 45% | 75% |
    /// | 0b10 | 35% | 70% |
    /// | 0b11 | 25% | 65% |
    pub ook_thd: u2,
    /// OOK RC time constant
    pub ook_rc: p2p_receiver_config::OokRCConstant,
    /// OOK fast decay
    pub ook_fd: bool,
}

impl Default for P2PRxConfig {
    fn default() -> Self {
        let mut ret = Self::from(0);
        // defaults to 75%
        ret.set_ook_thd(u2::new(0b01));
        ret
    }
}

register_impl!(GPTimer, u16, 0x0B, B);
/// General purpose timer ticks
#[bitsize(16)]
#[derive(FromBits, DebugBits, Format, Clone, Copy)]
pub struct GPTimer {
    pub msb: u8,
    pub lsb: u8
}
impl GPTimer {
    pub fn from_ticks(ticks: u16) -> Self {
        let msb = ticks >> 8;
        let lsb = ticks & 0xFF;
        Self::new(msb as u8, lsb as u8)
    }
    pub fn to_ticks(self) -> u16 {
        ((self.msb() as u16) << 8) + self.lsb() as u16
    }
}

register_impl!(InterruptRegister, u32, 0x1A, A);
/// Interrupt registers
#[bitsize(32)]
#[derive(FromBits, DebugBits, Format, Clone, Copy, Default)]
pub struct InterruptRegister(Interrupt);

register_impl!(InterruptMask, u32, 0x16, A);
/// Interrupt mask registers
#[bitsize(32)]
#[derive(FromBits, DebugBits, Format, Clone, Copy, Default)]
pub struct InterruptMask(Interrupt);

#[bitsize(32)]
#[derive(FromBits, DebugBits, Format, Clone, Copy, Default)]
pub struct Interrupt {
    reserved: u1,
    /// Automatic reception restart
    ///
    /// Set when a frame is suppressed as EMD
    pub rx_rest: bool,
    /// Bit collision
    pub col: bool,
    /// End of transmission
    pub txe: bool,
    /// End of receive
    pub rxe: bool,
    /// Start of receive
    pub rxs: bool,
    /// FIFO water level
    ///
    /// Fires at 300B during RX, 200B during TX
    pub wl: bool,
    /// Fires when oscillator frequency is stable
    pub osc: bool,
    /// Initiator bitrate recognized while in target mode
    pub nfct: bool,
    /// Minimum guard time expire
    ///
    /// An external field not detected during collision avoidance,
    /// collision avoidance, field was switched on,
    /// IRQ sent after minimum guard time according to NFCIP-1
    pub cat: bool,
    /// Detection of collision
    ///
    /// Must be cleared before collision detection is performed
    pub cac: bool,
    /// Detection of external field drop below Target activation level
    pub eof: bool,
    /// Detection of external field higher than Target activation level
    pub eon: bool,
    /// General purpose timer expire
    pub gpe: bool,
    /// No-response timer expire
    pub nre: bool,
    /// Termination of direct command
    pub dct: bool,
    reserved: u1,
    /// Wakeup due to phase measurement
    pub wph: bool,
    /// Wakeup due to amplitude measurement
    pub wam: bool,
    /// Wakeup timer
    pub wt: bool,
    /// Hard framng error
    ///
    /// Results in corrupted RX data
    #[doc(alias = "err1")]
    pub err_framing_hard: bool,
    /// Soft framng error
    ///
    /// Doesn't result in corrupted RX data
    #[doc(alias = "err2")]
    pub err_framing_soft: bool,
    #[doc(alias = "par")]
    pub err_parity: bool,
    #[doc(alias = "crc")]
    pub err_crc: bool,
    /// Passive target Active interrupt
    #[doc(alias = "wu_a")]
    pub wakeup_active: bool,
    /// Passive target Active\* interrupt
    #[doc(alias = "wu_a*")]
    pub wakeup_active_x: bool,
    reserved: u1,
    /// NFC 212/424kb/s Passive target ‘Active’ interrupt
    pub wu_f: bool,
    /// End of receive, 3916 is handling the response
    ///
    /// Sent in passive target mode when NFC-A anti-collision
    /// or NFC-F SENSF_RES is automatically sent (MCU action required).
    pub rxe_pta: bool,
    /// Active P2P field on event
    ///
    /// Sent after RF collision avoidance,
    /// if there was no collision and field was turned on.
    pub apon: bool,
    /// Passive target slot number water level
    ///
    /// Sent if four unused slot numbers (TSN) remain in PT_memory.
    #[doc(alias = "sl_wl")]
    pub slot_water_level: bool,
    /// PPON2 field on waiting timer interrupt
    pub ppon2: bool,
}

pub mod regulator_control {
    use bilge::prelude::*;
    use defmt::Format;

    /// Voltage to be measured
    #[bitsize(3)]
    #[derive(Format, FromBits, Debug, Clone, Copy, Default)]
    pub enum MeasureSource {
        #[default]
        Vdd = 0b000,
        VddA = 0b001,
        VddD = 0b010,
        VddRF = 0b011,
        VddAM = 0b100,
        VddTX = 0b101,
        #[fallback]
        Reserved,
    }

    #[bitsize(1)]
    #[derive(Format, FromBits, Debug, Clone, Copy, Default)]
    pub enum RegulatedSettingSource {
        /// Regulated voltages come from the Adjust Regulators command
        #[default]
        AdjustRegulators = 0,
        /// Regulated voltages come from [`RegulatorControl`].rege
        ExternalRegister = 1,
    }
}

register_impl!(RegulatorControl, u8, 0x2C, A);
/// Regulator voltage control
#[bitsize(8)]
#[derive(FromBits, DebugBits, Format, Clone, Copy, Default)]
pub struct RegulatorControl {
    /// Defines source of direct command Measure power supply.
    pub mpsv: regulator_control::MeasureSource,
    /// External definition of regulated voltage
    ///
    /// See datasheet table 92
    pub rege: u4,
    /// Controls the source for regulated voltage definiton
    pub reg_s: regulator_control::RegulatedSettingSource,
}

register_impl!(RegulatorDisplay, u8, 0x2C, B);
#[bitsize(8)]
#[derive(FromBits, DebugBits, Format, Clone, Copy, Default)]
pub struct RegulatorDisplay {
    /// VDD_RF regulator in current limit mode
    pub i_lim: bool,
    reserved: u3,
    /// Voltage regulator setting after adjust regulators command
    ///
    /// Refer to datasheet table 92
    pub reg: u4,
}

register_impl!(ADConverterOutput, u8, 0x25, A);
#[bitsize(8)]
#[derive(FromBits, DebugBits, Format, Clone, Copy, Default)]
pub struct ADConverterOutput {
    pub value: u8,
}

register_impl!(AntennaTuningControl, u16, 0x26, A);
#[bitsize(16)]
#[derive(FromBits, DebugBits, Format, Clone, Copy)]
pub struct AntennaTuningControl {
    pub a: u8,
    pub b: u8,
}
impl Default for AntennaTuningControl {
    fn default() -> Self {
        Self::new(1 << 7, 1 << 7)
    }
}

register_impl!(AuxiliaryModulationSetting, u8, 0x28, B);
#[bitsize(8)]
#[derive(FromBits, DebugBits, Format, Clone, Copy, Default)]
pub struct AuxiliaryModulationSetting {
    reserved: u2,
    /// Regulator shaped AM modulation enable
    ///
    /// Set to
    /// - true: When it is used with act_amsink=0 and small VDD_AM capacitor (10-50 nF)
    /// - false: In card mode, wake-up mode and measure amplitude/phase from tx_en=0
    pub regulator_am: bool,
    /// Resistive AM modulation enable
    ///
    /// Uses md_res<6:0> to configure resistive AM modulated driver resistance
    pub resistive_am: bool,
    /// Driver load modulation enable
    ///
    /// Uses Passive target modulation register to set driver load modulation resistance.
    pub lm_driver: bool,
    /// External load modulation enable
    ///
    /// Enables output of load modulation signal on LM_EXT pin.
    pub lm_external: bool,
    /// true for inverted polarity (LM_EXT active low)
    pub lm_external_polarity: bool,
    /// Disables AM regulator
    ///
    /// Set to:
    /// - true: with act_amsink=0 and small VDD_AM capacitor (10-50nF, depending on RF load)
    /// - false: act_amsink=1 and big VDD_AM capacitor (2.2uF)
    pub disable_am_regulator: bool,
}

register_impl!(IcIdentity, u8, 0x3F, A);
#[bitsize(8)]
#[derive(FromBits, DebugBits, Format, Clone, Copy, Default)]
pub struct IcIdentity {
    pub rev_code: u3,
    pub type_code: u5,
}
