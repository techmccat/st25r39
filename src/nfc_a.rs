use bitvec::{order::Lsb0, view::AsMutBits};
use defmt::Format;
use embedded_hal::digital::InputPin;

use crate::{
    commands::DirectCommand,
    interface::Interface,
    registers::{self, Interrupt, Register},
    Error, Result,
};

/// Minimum PCD to PICC delay, in fc
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

#[derive(Debug)]
pub struct Iso14443aInitiator<I, P>(pub(crate) crate::ST25R3916<I, P>);

impl<I: Interface, P: InputPin> Iso14443aInitiator<I, P> {
    pub fn detect_presence(&mut self, rx_buf: &mut [u8]) -> Result<bool, I, P> {
        self.0.field_on_and_wait_gt()?;
        let res = self.transceive_short_frame(rx_buf, ShortFrame::ReqA, FDT_A_LISTEN_RELAXED);
        Ok(match res {
            Ok(_len) => true,
            Err(Error::CollisionDetected) => true,
            Err(Error::Crc) => true,
            Err(Error::NoMemory) => true,
            Err(Error::Framing) => true,
            Err(Error::Parity) => true,
            Err(Error::IncompleteByte) => true,
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
            Err(Error::CollisionDetected) => registers::CollisionDisplay::read(&mut self.0.dev)
                .map_err(Error::Interface)
                .map(|r| r.bits() as usize),
            Err(e) => Err(e),
        }?;
        {
            let col_dis = registers::CollisionDisplay::read(&mut self.0.dev)
                .map_err(Error::Interface)?;
            defmt::warn!("{:08b}", col_dis);
        }

        // defmt::info!("{=[u8;5]:02x}", rx_buf);
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
}

fn check_irq_err<I: Interface, P: InputPin>(firing: Interrupt) -> Result<(), I, P> {
    // defmt::info!("{=[u8;4]:08b}", u32::from(firing).to_le_bytes());
    if firing.col() {
        Err(Error::CollisionDetected)
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
