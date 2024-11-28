use defmt::Format;
use embedded_hal::digital::InputPin;

const MF_CLASSIC_FWT: u32 = 60000;

use crate::{
    interface::Interface, nfc_a::{self, compute_crc, Iso14443aInitiator, TransceiveConfig}, Error, Result
};

#[derive(Debug, Clone, Copy, Format)]
pub enum Command {
    AuthKeyA = 0x60,
    AuthKeyB = 0x61,
    ReadBlock = 0x30,
    WriteBlock = 0xA0,
    ValueDec = 0xC0,
    ValueInc = 0xC1,
    ValueRestore = 0xC2,
    ValueTransfer = 0xB0,
    Ack = 0x0A,
    Nack = 0x00,
    NackTransferInvalid = 0x04,
    NackTransferCrcError = 0x01,
}

pub struct MfClassicPoller<'a, I, P> {
    pub drv: &'a mut nfc_a::Iso14443aInitiator<I, P>,
}

impl<'a, I: Interface, P: InputPin> MfClassicPoller<'a, I, P> {
    pub fn new(drv: &'a mut Iso14443aInitiator<I, P>) -> Self {
        Self { drv }
    }

    /// Tries detecting a card by authenticating to sector 0
    pub fn detect(&mut self) -> Result<bool, I, P> {
        let auth_command = [Command::AuthKeyA as u8, 0x30, 0x76, 0x4a];
        todo!("Fix your fucking crc");
        // let crc = compute_crc(&auth_command[0..2]);
        // auth_command[2] = crc as u8;
        // auth_command[3] = (crc >> 8) as u8;
        defmt::info!("Sending command {=[u8]}", auth_command);

        let mut rx = [0u8; 4];
        let res = self.drv.transceive(
            &auth_command,
            8 * auth_command.len(),
            TransceiveConfig {
                no_append_crc: true,
                no_rx_crc: true,
                ..Default::default()
            },
            &mut rx,
            MF_CLASSIC_FWT,
        );

        let out = match res {
            // todo: disable crc checks in mifare mode
            Ok((4, 0)) => true,
            Ok((_, _)) | Err(Error::NoMemory) => { defmt::info!("Got a weird nonce"); true }
            Err(Error::Parity) => { defmt::warn!("Parity error in nonce"); true }
            Err(Error::Timeout) => false,
            Err(e) => Err(e)?
        };
        if out {
            defmt::info!("Got tag nonce: {=[u8]}", rx)
        }
        Ok(out)
    }
}
