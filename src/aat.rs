use core::ops::RangeInclusive;

use embedded_hal::digital::InputPin;

use crate::{
    registers::{self, Register},
    Error, Interface,
};

#[derive(Clone, Debug, defmt::Format)]
pub struct TunerSettings {
    pub amplitude_target: u8,
    pub amplitude_weight: u8,
    pub phase_target: u8,
    pub phase_weight: u8,
    pub a_start: Option<u8>,
    pub a_range: RangeInclusive<u8>,
    pub a_step: u8,
    pub b_start: Option<u8>,
    pub b_range: RangeInclusive<u8>,
    pub b_step: u8,
}

impl Default for TunerSettings {
    fn default() -> Self {
        Self {
            amplitude_target: 196,
            amplitude_weight: 1,
            phase_target: 128,
            phase_weight: 2,
            a_start: None,
            a_range: 0..=255,
            a_step: 32,
            b_start: None,
            b_range: 0..=255,
            b_step: 32,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, defmt::Format)]
pub(crate) enum Direction {
    UpA,
    DownA,
    UpB,
    DownB,
}

// impl Direction {
//     fn opposite(&self) -> Self {
//         match self {
//             Self::UpB => Self::DownB,
//             Self::UpA => Self::DownA,
//             Self::DownB => Self::UpB,
//             Self::DownA => Self::UpA,
//         }
//     }
// }

const DIRECTIONS: [Direction; 4] = [
    Direction::UpA,
    Direction::UpB,
    Direction::DownA,
    Direction::DownB,
];

#[derive(Clone, Debug, defmt::Format)]
pub(crate) struct TunerState {
    pub a: u8,
    pub b: u8,
    step_a: u8,
    step_b: u8,
    pub diff: u16,
}

impl TunerState {
    pub fn new_from_settings<I: Interface, P: InputPin>(
        conf: &TunerSettings,
        driver: &mut crate::ST25R3916<I, P>,
    ) -> crate::Result<Self, I, P> {
        let (a, b) = if conf.a_start.is_none() || conf.b_start.is_none() {
            let reg =
                registers::AntennaTuningControl::read(&mut driver.dev).map_err(Error::Interface)?;
            (Some(reg.a()), Some(reg.b()))
        } else {
            (None, None)
        };
        let amp = driver.measure_amplitude_raw()?;
        let phase = driver.measure_phase_raw()?;
        let ret = Self {
            a: conf.a_start.unwrap_or(a.unwrap()),
            b: conf.b_start.unwrap_or(b.unwrap()),
            step_a: conf.a_step,
            step_b: conf.b_step,
            diff: compute_diff(conf, amp, phase),
        };
        defmt::debug!("New state: {:?}, amp={=u8}, phase={=u8}", ret, amp, phase);
        Ok(ret)
    }
    pub fn is_done(&self) -> bool {
        self.step_a == 0 || self.step_b == 0
    }
    pub fn halve_steps(&mut self) {
        self.step_a /= 2;
        self.step_b /= 2;
    }
}

fn compute_diff(settings: &TunerSettings, amplitude: u8, phase: u8) -> u16 {
    let amp_diff = settings.amplitude_target.abs_diff(amplitude) as u16;
    let phase_diff = settings.phase_target.abs_diff(phase) as u16;
    amp_diff * settings.amplitude_weight as u16 + phase_diff * settings.phase_weight as u16
}

fn step_values(state: &TunerState, conf: &TunerSettings, dir: Direction) -> (u8, u8) {
    match dir {
        Direction::UpA => (
            state
                .a
                .saturating_add(state.step_a)
                .min(*conf.a_range.end()),
            state.b,
        ),
        Direction::UpB => (
            state.a,
            state
                .b
                .saturating_add(state.step_b)
                .min(*conf.b_range.end()),
        ),
        Direction::DownA => (
            state
                .a
                .saturating_sub(state.step_a)
                .max(*conf.a_range.start()),
            state.b,
        ),
        Direction::DownB => (
            state.a,
            state
                .b
                .saturating_sub(state.step_b)
                .max(*conf.b_range.start()),
        ),
    }
}

pub(crate) fn find_best_step<I: Interface, P: InputPin>(
    driver: &mut crate::ST25R3916<I, P>,
    state: &mut TunerState,
    settings: &TunerSettings,
    /*previous: Option<Direction>,*/
) -> crate::Result<Option<Direction>, I, P> {
    let mut lastdir = None;
    let mut skip_apply = false;
    for d in DIRECTIONS {
        skip_apply = false;
        // if Some(d) == previous.map(|d| d.opposite()) {
        //     continue;
        // };

        let (new_a, new_b) = step_values(state, settings, d);
        let (amp, phase) = driver.set_capacitance_and_measure(new_a, new_b)?;

        let diff = compute_diff(settings, amp, phase);
        if diff < state.diff {
            // gets overwritten if the loop runs again
            // true only if the last run got the best result
            skip_apply = true;
            defmt::debug!(
                "Best yet: {}, a={=u8} b={=u8}, amp={=u8} phase={=u8}, diff={=u16}",
                d,
                new_a,
                new_b,
                amp,
                phase,
                diff
            );
            lastdir = Some(d);
            state.a = new_a;
            state.b = new_b;
            state.diff = diff;
        }
    }

    // saves a little over 10ms per step on best case
    if !skip_apply {
        driver.set_aat_capacitance(state.a, state.b)?;
    }
    Ok(lastdir)
}

pub(crate) fn try_greedy_step<I: Interface, P: InputPin>(
    driver: &mut crate::ST25R3916<I, P>,
    state: &mut TunerState,
    settings: &TunerSettings,
    dir: Direction,
) -> crate::Result<bool, I, P> {
    let (new_a, new_b) = step_values(state, settings, dir);
    let (amp, phase) = driver.set_capacitance_and_measure(new_a, new_b)?;
    let diff = compute_diff(settings, amp, phase);
    Ok(if diff < state.diff {
        defmt::debug!(
            "Kept dir: {}, a={=u8} b={=u8}, amp={=u8} phase={=u8}, diff={=u16}",
            dir,
            new_a,
            new_b,
            amp,
            phase,
            diff
        );
        state.a = new_a;
        state.b = new_b;
        state.diff = diff;
        true
    } else {
        driver.set_aat_capacitance(state.a, state.b)?;
        false
    })
}
