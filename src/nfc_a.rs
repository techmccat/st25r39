use core::{cmp::min, u8};

use bilge::{arbitrary_int::{u2, u5}, bitsize};
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
    ReqA,
    WupA,
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
        fwt: u32,
    ) -> Result<usize, I, P> {
        // wait for frame delay timer (using gpt)
        if registers::BitrateDetection::read(&mut self.0.dev).map_err(Error::Interface)?.gpt_on() {
            self.0.enable_interrupts(Interrupt::new_gpe())?;
            self.0.wait_for_interrupt(Interrupt::new_gpe())?;
        }
        let nrt_ticks = min(u16::MAX as i32, (fwt as i32 + FDT_A_ADJUSTMENT) / 64);
        self.0.set_nrt(nrt_ticks as u16)?;

        self.prepare_transceive(TransceiveConfig {
            // doesn't really need to be set for short frames or anticollision
            no_rx_crc: true,
            mode: TransceiveMode::Poller,
        })?;
        let mut col = Interrupt::default();
        col.set_col(true);
        self.0.enable_interrupts(col)?;

        self.0.set_tx_bits(0)?;

        let command = match frame {
            ShortFrame::ReqA => DirectCommand::TransmitReqA,
            ShortFrame::WupA => DirectCommand::TransmitWupA,
        };
        self.0
            .dev
            .direct_command(command)
            .map_err(Error::Interface)?;

        let mut txe = Interrupt::default();
        txe.set_txe(true);
        // TODO: 1ms timeout
        self.0.wait_for_interrupt(txe)?;

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
        registers::ReceiverConfiguration::modify(&mut self.0.dev, |r| r.set_agc_en(true))
            .map_err(Error::Interface)?;

        self.0.disable_interrupts(col)?;
        res
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
            r.set_nfc_n(u2::new(0u8));
        })
        .map_err(Error::Interface)?;
        // TODO: NRT EMV mode if we're in EMV eh mode
        // TODO: sanity timer (no ehal-1 timer traits yet)

        match config.mode {
            TransceiveMode::Listener => {
                self.0
                    .dev
                    .direct_command(DirectCommand::ClearFIFO)
                    .map_err(Error::Interface)?;
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

        self.0.clear_interrupts()?;
        self.0.enable_interrupts(irq)
    }
    // assumes relevant interrupts (eon, ppon2, nre)
    fn receive_data(&mut self, buf: &mut [u8]) -> Result<usize, I, P> {
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
            if firing.rx_rest() {
                // rx reset, should go back to waiting for rxs
                defmt::warn!("Not handling RX restart due to EMD");
                if firing.nre() {
                    return Err(Error::Timeout);
                }
                // check br detect register, timeout if nrt is on?
                // check for rx activity
                // if present, keep waiting for rxe
                // else go back to waiting for rxs
            } else if firing.wl() {
                defmt::warn!("FIFO water level reached, not handled yet");
                // read fifo, then what?
            } else if firing.wu_f() && !firing.eof() {
                // SENSF_REQ automatic response (what?), keep waiting
                // TODO: separate error checking function maybe
            } else if firing.err_framing_hard() {
                return Err(Error::Framing);
            // TODO: discard soft framing errors in ap2p and ce
            } else if firing.err_framing_soft() {
                return Err(Error::Framing);
            } else if firing.err_parity() {
                return Err(Error::Parity);
            } else if firing.err_crc() {
                return Err(Error::Crc);
            } else if firing.col() {
                return Err(Error::CollisionDetected);
            // should only be in passive listen
            } else if firing.eof() {
                return Err(Error::LinkLost);
            } else if firing.nre() {
                return Err(Error::Timeout);
            } else if firing.rxe() {
                let fs = registers::FifoStatus::read(&mut self.0.dev).map_err(Error::Interface)?;
                if fs.bits().value() != 0 {
                    return Err(Error::IncompleteByte);
                } else if fs.no_parity() {
                    return Err(Error::Framing);
                }
                let (len, _) = self.0.read_fifo(buf)?;
                break Ok(len as usize);
            }
        }
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
