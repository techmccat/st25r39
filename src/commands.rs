/// Direct commands
///
/// Copied and pasted from the
/// [datasheet](https://www.st.com/resource/en/datasheet/st25r3916b.pdf), section 4.4
#[derive(Debug, Clone, Copy, defmt::Format)]
#[repr(u8)]
pub enum DirectCommand {
    /// Puts the ST25R3916B into power-up state
    SetDefault = 0xC1,
    /// Stops all activities: transmission, reception, direct command execution, timers
    Stop = 0xC2,
    /// Starts a transmit sequence with automatic CRC generation
    TransmitWithCRC = 0xC4,
    /// Starts a transmit sequence without automatic CRC generation
    TransmitWithoutCRC = 0xC5,
    /// Transmits REQA command (ISO14443A mode only)
    TransmitReqA = 0xC6,
    /// Transmits WUPA command (ISO14443A mode only)
    TransmitWupA = 0xC7,
    /// Performs Initial RF Collision avoidance and switches on the field
    InitialFieldOn = 0xC8,
    /// Performs Response RF Collision avoidance and switches on the field
    ResponseFieldOn = 0xC9,
    /// Puts the passive target logic into Sense (Idle) state
    GoToSense = 0xCD,
    /// Puts the passive target logic into Sleep (Halt) state
    GoToSleep = 0xCE,
    /// Stops receivers and RX decoders
    MaskRx = 0xD0,
    /// Starts receivers and RX decoders
    UnmaskRx = 0xD1,
    /// Changes AM modulation state
    ChangeAmModulationState = 0xD2,
    /// Measures the amplitude of the signal present on RFI inputs and stores the result
    /// in the [A/D converter output register]
    MeasureAmplitude = 0xD3,
    /// Resets receiver gain to the value in the [Receiver configuration register 4]
    ResetRxGain = 0xD5,
    /// Adjusts supply regulators according to the current supply voltage level
    AdjustRegulators = 0xD6,
    /// Starts the driver timing calibration according to the setting in the
    /// [TX driver timing display register]
    CalibrateDriverTiming = 0xD8,
    /// Measures the phase difference between the signal on RFO and RFI
    MeasurePhase = 0xD9,
    /// Clears the RSSI bits in the [RSSI display register] and restarts the measurement
    ClearRSSI = 0xDA,
    ClearFIFO = 0xDB,
    /// Enters transparent mode (digitized subcarrier is passed through to MOSI)
    EnterTransparentMode = 0xDC,
    MeasurePowerSupply = 0xDF,
    /// Starts the General Purpose Timer
    StartGPT = 0xE0,
    StartWakeupTimer = 0xE1,
    /// Starts the mask-receive timer and squelch operation
    StartMaskRxTimer = 0xE2,
    StartNoResponseTimer = 0xE3,
    StartPPOn2Timer = 0xE4,
    StopNoResponseTimer = 0xE8,
    TriggerRcCalibration = 0xEA,
    RegisterSpaceBAccess = 0xFB,
    TestAccess = 0xFC,
}
