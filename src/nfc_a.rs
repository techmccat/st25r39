use bilge::prelude::*;
use bitvec::{order::Lsb0, view::AsMutBits};
use defmt::Format;
use embedded_hal::digital::InputPin;

use crate::{
    commands::DirectCommand,
    interface::Interface,
    registers::{self, Interrupt, Register},
    Error, Result,
};

/// Minimum PCD to PICC delay, in 1/fc
///
/// Per spec it's 1172, adding 3\*128 as margin
pub const FDT_A_LISTEN_RELAXED: u32 = 1620;
pub const FDT_A_LISTEN_MIN: u32 = 1172;
/// Minimum PCD to PICC delay, in fc
pub const FDT_A_POLL: u32 = 6780;

/// Margin for execution delay
pub const FDT_A_ADJUSTMENT: i32 = 512;
pub const MRT_ADJUSTMENT: i32 = -256;

pub const GUARD_TIME_US: u32 = 5_000;

pub const MAX_ATS_LEN: usize = 20;
pub const MAX_HISTORICAL_BYTES: usize = MAX_ATS_LEN - 5;

pub enum ShortFrame {
    /// SENS_REQ for NFC Forum
    ReqA,
    /// ALL_REQ for NFC Forum
    WupA,
}

#[derive(Debug, Format, Clone, Copy)]
pub enum CascadeLevel {
    One = 0b10010011,
    Two = 0b10010101,
    Three = 0b10010111,
}

#[derive(Debug, Format, Clone, Copy)]
pub enum NfcId {
    Single([u8; 4]),
    Double([u8; 7]),
    Triple([u8; 10]),
}

#[derive(Debug, Format, Clone, Copy)]
pub struct AntiCollisionHeader {
    pub sel_cmd: CascadeLevel,
    pub bit_count: u8,
}

impl AntiCollisionHeader {
    pub fn new(sel_cmd: CascadeLevel, bit_count: u8) -> Self {
        Self { sel_cmd, bit_count }
    }
    pub fn bytes(&self) -> [u8; 2] {
        let bits = self.bit_count & 0b111;
        let bytes = self.bit_count >> 3;
        let bit_count = bits | bytes << 4;

        [self.sel_cmd as u8, bit_count]
    }
}

#[bitsize(8)]
#[derive(DebugBits, Format, Clone, Copy, FromBits)]
pub struct SelectResponse {
    reserved: u2,
    pub cascade: bool,
    reserved: u2,
    pub kind: TagKind,
    reserved: u1,
}

#[bitsize(2)]
#[derive(Debug, Format, Clone, Copy, FromBits)]
pub enum TagKind {
    Type2 = 0b00,
    Type4A = 0b01,
    NfcDep = 0b10,
    NfcDepType4A = 0b11,
}

impl TagKind {
    pub fn supports_rats(&self) -> bool {
        *self as u8 & 1 == 1
    }
}

#[derive(Debug, Format, Clone)]
pub struct AnswerToSelect {
    tl: u8,
    format: T0,
    ta_1: Option<TA1>,
    tb_1: Option<TB1>,
    tc_1: Option<TC1>,
    historical_bytes: heapless::Vec<u8, MAX_HISTORICAL_BYTES>,
}

#[derive(Clone, Copy, Debug, Format)]
pub enum ParseError {
    InvalidLength,
    InvalidData,
}

impl TryFrom<&[u8]> for AnswerToSelect {
    type Error = ParseError;
    fn try_from(value: &[u8]) -> core::prelude::v1::Result<Self, Self::Error> {
        let mut bytes = value.into_iter().cloned();
        if let Some(tl) = bytes.next() {
            if tl as usize != value.len() || tl < 2 {
                return Err(ParseError::InvalidLength);
            };
            let t0 = T0::from(bytes.next().unwrap());
            let mut ret = AnswerToSelect {
                tl,
                format: t0,
                ta_1: None,
                tb_1: None,
                tc_1: None,
                historical_bytes: heapless::Vec::new(),
            };

            if t0.ta_transmitted() {
                ret.ta_1 = Some(
                    bytes
                        .next()
                        .map(|b| TA1::from(b))
                        .ok_or(ParseError::InvalidData)?,
                );
            }
            if t0.tb_transmitted() {
                ret.tb_1 = Some(
                    bytes
                        .next()
                        .map(|b| TB1::from(b))
                        .ok_or(ParseError::InvalidData)?,
                );
            }
            if t0.tc_transmitted() {
                ret.tc_1 = Some(
                    bytes
                        .next()
                        .map(|b| TC1::from(b))
                        .ok_or(ParseError::InvalidData)?,
                );
            }
            ret.historical_bytes.extend(bytes);

            Ok(ret)
        } else {
            Err(ParseError::InvalidLength)
        }
    }
}

#[bitsize(8)]
#[derive(DebugBits, Clone, Copy, FromBits)]
pub struct T0 {
    pub fsci: u4,
    pub ta_transmitted: bool,
    pub tb_transmitted: bool,
    pub tc_transmitted: bool,
    reserved: u1,
}

impl Format for T0 {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "T0 {{ FSCI: {0=0..4}, TA(1) transmitted: {0=4..5}, TB(1) transmitted: {0=5..6}, TC(1) transmitted: {0=6..7} }}", u8::from(*self))
    }
}

#[bitsize(8)]
#[derive(DebugBits, Clone, Copy, FromBits)]
pub struct TA1 {
    pub d_poll_listen_2_support: bool,
    pub d_poll_listen_4_support: bool,
    pub d_poll_listen_8_support: bool,
    reserved: u1,
    pub d_listen_poll_2_support: bool,
    pub d_listen_poll_4_support: bool,
    pub d_listen_poll_8_support: bool,
    pub symmetric_bitrate_only: bool,
}

impl Format for TA1 {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "TA(1) {{ Poll->Listen: {0=0..3:03b}, Listen->Poll: {0=4..7:03b}, Symmetric bitrate: {0=7..8} }}", u8::from(*self))
    }
}

#[bitsize(8)]
#[derive(DebugBits, Clone, Copy, FromBits)]
pub struct TB1 {
    pub sfgi: u4,
    pub fwi: u4,
}

impl Format for TB1 {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(
            fmt,
            "TB(1) {{ SFGI: {0=0..4}, FWI: {0=4..8} }}",
            u8::from(*self)
        )
    }
}

#[bitsize(8)]
#[derive(DebugBits, Clone, Copy, FromBits)]
pub struct TC1 {
    nad_supported: bool,
    did_supported: bool,
    reserved: u2,
    advanced_features_supported: bool,
    reserved: u3,
}

impl Format for TC1 {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "TC(1) {{ NAD supported: {0=0..1}, DID supported: {0=1..2}, Advanced feature support: {0=4..5} }}", u8::from(*self))
    }
}

#[derive(Debug)]
pub struct Iso14443aInitiator<I, P>(pub(crate) crate::ST25R3916<I, P>);

impl<I: Interface, P: InputPin> Iso14443aInitiator<I, P> {
    pub fn detect_presence(&mut self, rx_buf: &mut [u8]) -> Result<bool, I, P> {
        self.0.field_on_and_wait_gt()?;
        let res = self.transceive_short_frame(rx_buf, ShortFrame::ReqA, FDT_A_LISTEN_RELAXED);
        Ok(match res {
            Ok(_len) => true,
            Err(Error::Collision) => true,
            Err(Error::Crc) => true,
            Err(Error::NoMemory) => true,
            Err(Error::Framing) => true,
            Err(Error::Parity) => true,
            Err(Error::Timeout) => false,
            Err(e) => return Err(e),
        })
    }
    /// Timeout is in 1/fc
    pub fn transceive_short_frame(
        &mut self,
        buf: &mut [u8],
        frame: ShortFrame,
        fdt_listen: u32,
    ) -> Result<(u16, u8), I, P> {
        // wait for fdt_poll (using gpt)
        self.0.wait_for_gpt()?;
        self.0.stop_activities()?;
        self.0.set_nrt(fdt_listen + FDT_A_ADJUSTMENT as u32)?;

        self.prepare_transceive(TransceiveConfig {
            // not really needed since it's set automatically for short frames or anticollision
            no_rx_crc: true,
            mode: TransceiveMode::Poller,
        })?;

        self.0.set_tx_bits(0)?;

        let command = match frame {
            ShortFrame::ReqA => DirectCommand::TransmitReqA,
            ShortFrame::WupA => DirectCommand::TransmitWupA,
        };
        self.0
            .dev
            .direct_command(command)
            .map_err(Error::Interface)?;

        // TODO: 1ms timeout
        self.0.wait_for_interrupt(Interrupt::new_txe())?;

        let res = self.receive_data(buf);

        // TODO: transceive cleanup function
        registers::Iso14443ASettings::modify(&mut self.0.dev, |r| {
            r.set_no_tx_parity(false);
            r.set_no_rx_parity(false);
            r.set_nfc_f0(false);
        })
        .map_err(Error::Interface)?;
        registers::Auxiliary::modify(&mut self.0.dev, |r| r.set_no_crc_rx(false))
            .map_err(Error::Interface)?;
        // registers::ReceiverConfiguration::modify(&mut self.0.dev, |r| r.set_agc_en(true))
        // .map_err(Error::Interface)?;

        res
    }

    pub fn transceive(
        &mut self,
        tx: &[u8],
        bits: usize,
        append_crc: bool,
        rx: &mut [u8],
        fdt_listen: u32,
    ) -> Result<(u16, u8), I, P> {
        self.0.set_nrt(fdt_listen + FDT_A_ADJUSTMENT as u32)?;
        self.transmit(tx, bits, append_crc)?;
        self.0.wait_for_interrupt(Interrupt::new_txe())?;
        self.receive_data(rx)
    }

    /// Transmits data
    pub fn transmit(&mut self, tx: &[u8], bits: usize, append_crc: bool) -> Result<(), I, P> {
        self.0.wait_for_gpt()?;
        self.prepare_transceive(TransceiveConfig {
            no_rx_crc: false,
            mode: TransceiveMode::Poller,
        })?;

        self.0.load_fifo(tx, bits)?;
        self.0
            .dev
            .direct_command(if append_crc {
                DirectCommand::TransmitWithCRC
            } else {
                DirectCommand::TransmitWithoutCRC
            })
            .map_err(Error::Interface)
    }

    fn prepare_transceive(&mut self, config: TransceiveConfig) -> Result<(), I, P> {
        // for now we assume initiator mode
        self.0
            .dev
            .direct_command(DirectCommand::Stop)
            .map_err(Error::Interface)?;
        self.0
            .dev
            .direct_command(DirectCommand::ResetRxGain)
            .map_err(Error::Interface)?;

        let mut irq = Interrupt::default();
        irq.set_col(true);
        irq.set_wl(true);
        irq.set_txe(true);
        irq.set_rxs(true);
        irq.set_rxe(true);
        irq.set_err_parity(true);
        irq.set_err_crc(true);
        irq.set_err_framing_hard(true);
        irq.set_err_framing_soft(true);
        irq.set_nre(true);

        registers::Auxiliary::modify(&mut self.0.dev, |r| {
            r.set_no_crc_rx(config.no_rx_crc);
            // if mode == active
            // r.set_nfc_n(u2::new(0u8));
        })
        .map_err(Error::Interface)?;
        // TODO: NRT EMV mode if we're in EMV eh mode
        // TODO: sanity timer (no ehal-1 timer traits yet)

        match config.mode {
            TransceiveMode::Listener => {
                self.0.clear_fifo()?;
                irq.set_eof(true);
                irq.set_wu_f(true);
            }
            TransceiveMode::ActiveP2P => {
                irq.set_eon(true);
                irq.set_ppon2(true);
                irq.set_cat(true);
                irq.set_cac(true);
            }
            TransceiveMode::Poller => (),
        }

        self.0.get_interrupts()?;
        self.0.enable_interrupts(irq)
    }
    // assumes relevant interrupts (eon, ppon2, nre)
    fn receive_data(&mut self, buf: &mut [u8]) -> Result<(u16, u8), I, P> {
        // wait for eon if in active (p2p) or passive listen mode
        // loop {
        //     let firing = self.0.wait_for_any_interrupt()?;
        //     // TODO: active wave shaping analog config on st25r3916b (not sure why, we're receiving)
        //     if firing.nre() {
        //         return Err(Error::Timeout)
        //     } else if firing.ppon2() { // TODO: start the timer on st25r3916?
        //         return Err(Error::LinkLost)
        //     } else if firing.eon() {
        //         break
        //     }
        // }
        // wait for rx start
        loop {
            let firing = self.0.wait_for_any_interrupt()?;
            if firing.nre() {
                return Err(Error::Timeout);
            } else if firing.eof() {
                // timeout if in active p2p because field on has already occurred
                return Err(Error::LinkLost);
            } else if firing.rxs() {
                break;
            }
        }
        // wait for rx end
        loop {
            let firing = self.0.wait_for_any_interrupt()?;
            // TODO: software timer to check for rxe taking too long (framing error)
            // if firing.rxs() {
            //     defmt::debug!("RX started");
            // }
            if firing.rx_rest() {
                // rx reset, should go back to waiting for rxs
                defmt::warn!("Not handling RX restart due to EMD");
                if firing.nre() {
                    return Err(Error::Timeout);
                }
                // check br detect register, timeout if nrt is on?
                // todo: check for rx activity
                // if present, keep waiting for rxe
                // else go back to waiting for rxs
            }
            if firing.wl() {
                // read fifo, then what?
                defmt::warn!("FIFO water level reached, not handled yet");
                // } else if firing.wu_f() && !firing.eof() {
                // SENSF_REQ automatic response (what?), keep waiting
                // TODO: separate error checking function maybe
            }
            check_irq_err(firing)?;
            if firing.rxe() {
                let fs = registers::FifoStatus::read(&mut self.0.dev).map_err(Error::Interface)?;
                if fs.no_parity() {
                    return Err(Error::Parity);
                }
                break self.0.read_fifo(buf);
            }
        }
    }

    /// length of the frame (including the header) is provided in the header
    pub fn transceive_anticollision_frame(
        &mut self,
        header: AntiCollisionHeader,
        buf: &mut [u8; 5],
        fdt_listen: u32,
    ) -> Result<usize, I, P> {
        // defmt::debug!("{}", header);
        assert!(header.bit_count >= 16 && header.bit_count <= 48);
        self.0.wait_for_gpt()?;
        self.0.set_nrt(fdt_listen + FDT_A_ADJUSTMENT as u32)?;

        // stmicro driver disable agc for better collision detection if receiver correlator is
        // disabled, but in the st25r3916b it's always enabled
        self.prepare_transceive(TransceiveConfig {
            no_rx_crc: true,
            mode: TransceiveMode::Poller,
        })?;
        self.0.clear_fifo()?;
        self.0
            .dev
            .fifo_write(&header.bytes())
            .map_err(Error::Interface)?;
        if header.bit_count > 16 {
            self.0
                .dev
                .fifo_write(&buf[..header.bit_count.div_ceil(8) as usize - 2])
                .map_err(Error::Interface)?;
        }
        self.0.set_tx_bits(header.bit_count as u16)?;
        registers::Iso14443ASettings::modify(&mut self.0.dev, |r| r.set_anticollision(true))
            .map_err(Error::Interface)?;

        self.0
            .dev
            .direct_command(DirectCommand::TransmitWithoutCRC)
            .map_err(Error::Interface)?;

        let mut rx_buf = [0u8; 5];
        let res = self.receive_data(rx_buf.as_mut());

        registers::Iso14443ASettings::modify(&mut self.0.dev, |r| r.set_anticollision(false))
            .map_err(Error::Interface)?;

        let bits = match res {
            Ok((bytes, bits)) => {
                let bits = bytes as usize * 8 + bits as usize;
                let total = header.bit_count as usize + bits;
                Ok(total)
            }
            Err(Error::Collision) => registers::CollisionDisplay::read(&mut self.0.dev)
                .map_err(Error::Interface)
                .map(|r| r.bits() as usize),
            Err(e) => Err(e),
        }?;
        // let col_dis =
        //     registers::CollisionDisplay::read(&mut self.0.dev).map_err(Error::Interface)?;
        // defmt::debug!("{:08b}", col_dis);

        let net_bits = (header.bit_count - 16) as usize;
        let start_idx = net_bits / 8;
        let shamt = net_bits % 8;
        // merge the two bit buffers. overlap byte (bits) are ORed, the rest (if present) is copied
        rx_buf.as_mut_bits::<Lsb0>().rotate_right(shamt);
        buf[start_idx] &= 0xff << (8 - shamt);
        buf[start_idx] |= rx_buf[0];
        if let Some(s) = buf.get_mut(start_idx + 1..) {
            s.copy_from_slice(&rx_buf[1..s.len() + 1])
        }

        if bits == 56 {
            let bcc = buf[0] ^ buf[1] ^ buf[2] ^ buf[3];
            if bcc != buf[4] {
                return Err(Error::Crc);
            }
        }

        Ok(bits)
    }

    pub fn transceive_select(
        &mut self,
        sel_cmd: CascadeLevel,
        id: [u8; 4],
    ) -> Result<SelectResponse, I, P> {
        let mut packet = [0u8; 7];
        packet[0] = sel_cmd as u8;
        packet[1] = 0x70;
        packet[2..=5].copy_from_slice(&id);
        let bcc = id[0] ^ id[1] ^ id[2] ^ id[3];
        packet[6] = bcc;
        let mut rx_buf = [0u8; 3];
        let bit_count = self.transceive(
            &packet,
            packet.len() * 8,
            true,
            &mut rx_buf,
            FDT_A_LISTEN_RELAXED,
        )?;
        // if i read the datasheet right the reader ic checks the crc so we don't need to
        // check it
        if bit_count.0 != 1 && bit_count.1 != 0 {
            Err(Error::Framing)
        } else {
            Ok(SelectResponse::from(rx_buf[0]))
        }
    }

    pub fn perform_anticollision(&mut self) -> Result<(NfcId, SelectResponse), I, P> {
        let mut complete_id = heapless::Vec::<u8, 10>::new();
        let mut final_data = None;

        for cascade in [CascadeLevel::One, CascadeLevel::Two, CascadeLevel::Three] {
            let mut bits = 16;
            let mut id_buf = [0u8; 5];

            loop {
                bits = self.transceive_anticollision_frame(
                    AntiCollisionHeader::new(cascade, bits),
                    &mut id_buf,
                    FDT_A_LISTEN_RELAXED,
                )? as u8;
                if bits == 56 {
                    break;
                }

                let idx = bits / 8;
                let sft = bits % 8;
                // set the first bit with a collision to 1
                id_buf[idx as usize] |= 1 << sft;
            }
            let id: [u8; 4] = id_buf[..4].try_into().unwrap();
            defmt::debug!("Cascade level {}, got ID {=[u8]:02X}", cascade, id);

            let sel_res = self.transceive_select(cascade, id)?;
            if sel_res.cascade() {
                // there should be no way for the id buffer to overlow but i'm keeping this just in
                // case
                complete_id
                    .extend_from_slice(&id_buf[1..4])
                    .map_err(|_| Error::IncorrectResponse)?;
            } else {
                complete_id
                    .extend_from_slice(&id_buf[..4])
                    .map_err(|_| Error::IncorrectResponse)?;
                final_data = Some((cascade, sel_res));
                break;
            }
        }

        if let Some((cascade, res)) = final_data {
            Ok((
                match cascade {
                    CascadeLevel::One => NfcId::Single(
                        complete_id
                            .as_slice()
                            .try_into()
                            .map_err(|_| Error::IncorrectResponse)?,
                    ),
                    CascadeLevel::Two => NfcId::Double(
                        complete_id
                            .as_slice()
                            .try_into()
                            .map_err(|_| Error::IncorrectResponse)?,
                    ),
                    CascadeLevel::Three => NfcId::Triple(
                        complete_id
                            .as_slice()
                            .try_into()
                            .map_err(|_| Error::IncorrectResponse)?,
                    ),
                },
                res,
            ))
        } else {
            Err(Error::IncorrectResponse)
        }
    }

    pub fn transmit_sleep(&mut self) -> Result<(), I, P> {
        self.transmit(&[0x50, 0], 16, true)
    }

    pub fn transceive_rats(&mut self) -> Result<AnswerToSelect, I, P> {
        let fsdi = 0x8;
        // todo: maybe support cid?
        let param = fsdi << 4;
        let frame = [0xe0, param];

        let mut rx_buf = [0; MAX_ATS_LEN + 2];
        let len = match self.transceive(&frame, 16, true, &mut rx_buf, u16::MAX as u32) {
            Ok((bytes, bits)) => {
                if bits == 0 && bytes - 2 > 2 && bytes - 2 < MAX_ATS_LEN as u16 {
                    Ok(bytes as usize - 2)
                } else {
                    Err(Error::Framing)
                }
            }
            Err(Error::NoMemory) => Err(Error::Framing),
            Err(e) => Err(e),
        }?;

        match AnswerToSelect::try_from(&rx_buf[..len]) {
            Ok(ats) => Ok(ats),
            Err(_) => Err(Error::IncorrectResponse),
        }
    }
}

fn check_irq_err<I: Interface, P: InputPin>(firing: Interrupt) -> Result<(), I, P> {
    // defmt::info!("{=[u8;4]:08b}", u32::from(firing).to_le_bytes());
    if firing.col() {
        Err(Error::Collision)
    } else if firing.err_framing_hard() || firing.err_framing_soft() {
        // TODO: discard soft framing errors in ap2p and ce
        Err(Error::Framing)
    } else if firing.err_parity() {
        Err(Error::Parity)
    } else if firing.err_crc() {
        Err(Error::Crc)
    } else if firing.eof() {
        // should only happen in passive listen
        Err(Error::LinkLost)
    } else if firing.nre() {
        Err(Error::Timeout)
    } else {
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransceiveMode {
    Poller,
    Listener,
    ActiveP2P,
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TransceiveConfig {
    pub no_rx_crc: bool,
    pub mode: TransceiveMode,
}
