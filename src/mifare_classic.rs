use bitvec::view::{AsMutBits, BitView};
use crypto1::Crypto1;
use defmt::Format;
use embedded_hal::digital::InputPin;

mod crypto1;

const MF_CLASSIC_FWT: u32 = 60000;

use crate::{
    interface::Interface,
    nfc_a::{self, compute_crc, Iso14443aInitiator, TransceiveConfig},
    Error, Result,
};

pub const fn bytes_plus_parity(len: usize) -> usize {
    (len * 9).div_ceil(8)
}

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
    Halt = 0x50,
}

#[derive(Debug, Clone, Copy, Format, PartialEq)]
pub enum MfcKeyKind {
    KeyA,
    KeyB,
}
impl MfcKeyKind {
    fn auth_command(&self) -> Command {
        if let Self::KeyA = self {
            Command::AuthKeyA
        } else {
            Command::AuthKeyB
        }
    }
}

#[derive(Debug, Clone, Copy, Format)]
pub struct MfClassicKey {
    pub key: [u8; 6],
    pub kind: MfcKeyKind,
}

impl MfClassicKey {
    pub fn new(key: u64, kind: MfcKeyKind) -> Self {
        Self {
            key: key.to_be_bytes()[..6].try_into().unwrap(),
            kind,
        }
    }
    pub fn key_a(key: u64) -> Self {
        Self::new(key, MfcKeyKind::KeyA)
    }
    pub fn key_b(key: u64) -> Self {
        Self::new(key, MfcKeyKind::KeyB)
    }
}

pub struct MfClassicPoller<'a, I, P> {
    pub drv: &'a mut nfc_a::Iso14443aInitiator<I, P>,
}

impl<'a, I: Interface, P: InputPin> MfClassicPoller<'a, I, P> {
    pub fn new(drv: &'a mut Iso14443aInitiator<I, P>) -> Self {
        Self { drv }
    }

    /// Mifare classic does encrypted parity and no CRC
    const ENC_TRX_CONFIG: TransceiveConfig = TransceiveConfig {
        no_append_crc: true,
        no_tx_parity: true,
        no_rx_crc: true,
        no_rx_parity: true,
    };

    /// Tries detecting a card by authenticating to sector 0
    ///
    /// Will return true to any response, even if the length is wrong
    // Might want to change this?
    pub fn detect(&mut self) -> Result<bool, I, P> {
        Ok(match self.auth_get_nonce(0, MfcKeyKind::KeyA) {
            // TODO: disable crc checks in mifare mode
            Ok(_nonce) => true,
            Err(Error::Framing) => {
                defmt::info!("Got a weird nonce");
                true
            }
            Err(Error::Parity) => {
                defmt::warn!("Parity error in nonce");
                true
            }
            Err(Error::Timeout) => false,
            Err(e) => return Err(e),
        })
    }
    // TODO: maybe return a sector reader proxy or something
    pub fn authenticate_block(
        &mut self,
        sector: u8,
        key: MfClassicKey,
        uid: [u8; 4],
    ) -> Result<Crypto1, I, P> {
        let tag_nonce = self.auth_get_nonce(sector, key.kind)?;
        let mut crypto1 = Crypto1::init(key);

        // TODO: maybe do RNG?
        let reader_nonce = 0xf00ff00fu32;
        let nt_xor_uid = (u32::from_ne_bytes(tag_nonce) ^ u32::from_ne_bytes(uid)).to_ne_bytes();

        let mut reply_enc = [0u8; 9];
        let reply_bits = reply_enc.as_mut_bits();
        crypto1
            .encrypt_with_parity(
                &reader_nonce.to_be_bytes(),
                &mut reply_bits[..36],
                Some(&nt_xor_uid),
            )
            .unwrap();
        let reader_answer = crypto1::prng_successor(u32::from_be_bytes(tag_nonce), 32);

        crypto1
            .encrypt_with_parity(&reader_answer.to_be_bytes(), &mut reply_bits[36..], None)
            .unwrap();

        defmt::debug!("Sending NR, AR {=[u8; 9]:02X}", reply_enc);
        let mut tag_reply = [0u8; 5];
        let (bytes, bits) = self.drv.transceive(
            &reply_enc,
            72,
            Self::ENC_TRX_CONFIG,
            &mut tag_reply,
            MF_CLASSIC_FWT,
        )?;

        let bits = bytes as usize * 8 + bits as usize;
        // todo: should i condier this a protocol error?
        if bits != 32 + 4 {
            return Err(Error::Framing);
        }

        let mut dec_tag_resp = [0u8; 4];
        crypto1
            .decrypt_with_parity(tag_reply.view_bits(), &mut dec_tag_resp)
            .unwrap();
        defmt::debug!("Got AT {=[u8;4]:02X}", dec_tag_resp);

        let ks3: [u8; 4] = core::array::from_fn(|_| crypto1.next_byte(0, false));
        let suc3_nt = crypto1::prng_successor(reader_answer, 32).to_be_bytes();

        let expected: [u8; 4] = core::array::from_fn(|i| ks3[i] ^ suc3_nt[i]);

        if dec_tag_resp != expected {
            defmt::info!("Expected {=[u8;4]:02X}", expected);
            // tag auth failed
            // or i screwed up something here
            Err(Error::Authentication)
        } else {
            Ok(crypto1)
        }
    }
    /// Starts auth to provided sector
    ///
    /// Returns framing error if nonce is too short
    fn auth_get_nonce(&mut self, sector: u8, key_type: MfcKeyKind) -> Result<[u8; 4], I, P> {
        let mut auth_command = [key_type.auth_command() as u8, sector, 0x00, 0x00];
        let crc = compute_crc(&auth_command[0..2]);
        auth_command[2] = crc as u8;
        auth_command[3] = (crc >> 8) as u8;
        defmt::debug!("Sending command {=[u8;4]:02X}", auth_command);

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

        match res {
            Ok((4, 0)) => {
                defmt::debug!("Got nonce {=[u8; 4]:02X}", rx);
                Ok(rx)
            }
            Ok((_, _)) | Err(Error::NoMemory) => Err(Error::Framing),
            Err(e) => Err(e),
        }
    }
    /// Reads provided block after auth for sector has been done
    pub fn read_block(
        &mut self,
        crypto: &mut Crypto1,
        block: u8,
        buf: &mut [u8; 16],
    ) -> Result<(), I, P> {
        let cmd = [Command::ReadBlock as u8, block];
        let mut tx_buf = [0u8; 3];
        let len_bits = crypto
            .encrypt_with_parity(&cmd, tx_buf.as_mut_bits(), None)
            .unwrap();

        const RX_LEN: usize = bytes_plus_parity(16);
        let mut rx_buf = [0u8; RX_LEN];
        match self.drv.transceive(
            &tx_buf,
            len_bits,
            Self::ENC_TRX_CONFIG,
            &mut rx_buf,
            MF_CLASSIC_FWT,
        ) {
            Ok((bytes, 0)) if bytes == RX_LEN as u16 => Ok(()),
            Ok((_, _)) | Err(Error::NoMemory) => Err(Error::Framing),
            Err(e) => Err(e),
        }?;
        crypto
            .decrypt_with_parity(rx_buf.as_mut_bits(), buf)
            .unwrap();

        Ok(())
    }
}
