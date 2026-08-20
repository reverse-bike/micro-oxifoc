//! Fixed-point OxiFOC Hall-center sweep.

use crate::calibration::types::{Actuation, Failure};
use crate::config::{HALL_CALIBRATION_CURRENT_COUNTS, PWM_HZ};

const RAMP_CYCLES: u32 = PWM_HZ;
const SETTLE_CYCLES: u32 = PWM_HZ / 5;
const STEP_CYCLES: u32 = PWM_HZ / 200;
const SWEEP_COUNT: u8 = 6;
const DEGREES_PER_SWEEP: u16 = 360;
const MIN_SAMPLES_PER_STATE: u16 = 30;

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[repr(u8)]
pub enum State {
    #[default]
    Idle = 0,
    RampUp = 1,
    Settle = 2,
    Sweep = 3,
    RampDown = 4,
    Complete = 5,
    Failed = 6,
    Finalize = 7,
}

impl State {
    pub const fn active(self) -> bool {
        matches!(
            self,
            Self::RampUp | Self::Settle | Self::Sweep | Self::RampDown | Self::Finalize
        )
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Result {
    pub forward_centers_q16: [u16; 8],
    pub reverse_centers_q16: [u16; 8],
    pub valid_mask: u8,
    pub forward_minimum_samples: u8,
    pub reverse_minimum_samples: u8,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct HallCalibration {
    state: State,
    failure: Failure,
    cycle_in_state: u32,
    angle: u32,
    sweep: u8,
    degree: u16,
    anchor: [[u32; 8]; 2],
    offset_sum: [[i64; 8]; 2],
    counts: [[u16; 8]; 2],
    finalize_raw: u8,
    directional_centers: [[u16; 8]; 2],
    directional_minimum: [u16; 2],
    result: Result,
}

impl HallCalibration {
    pub const fn new() -> Self {
        Self {
            state: State::Idle,
            failure: Failure::None,
            cycle_in_state: 0,
            angle: 0,
            sweep: 0,
            degree: 0,
            anchor: [[0; 8]; 2],
            offset_sum: [[0; 8]; 2],
            counts: [[0; 8]; 2],
            finalize_raw: 0,
            directional_centers: [[0; 8]; 2],
            directional_minimum: [u16::MAX; 2],
            result: Result {
                forward_centers_q16: [0; 8],
                reverse_centers_q16: [0; 8],
                valid_mask: 0,
                forward_minimum_samples: 0,
                reverse_minimum_samples: 0,
            },
        }
    }

    pub fn start(&mut self) {
        *self = Self::new();
        self.state = State::RampUp;
    }

    pub fn fail(&mut self, failure: Failure) {
        self.state = State::Failed;
        self.failure = failure;
        self.cycle_in_state = 0;
    }

    pub const fn state(&self) -> State {
        self.state
    }

    pub const fn failure(&self) -> Failure {
        self.failure
    }

    pub const fn active(&self) -> bool {
        self.state.active()
    }

    pub const fn result(&self) -> Result {
        self.result
    }

    pub const fn progress(&self) -> u16 {
        (u16::from_le_bytes([self.sweep, 0]) << 9) | self.degree
    }

    pub fn actuation(&self) -> Actuation {
        let current = match self.state {
            State::RampUp => ramp_current(self.cycle_in_state),
            State::Settle | State::Sweep => HALL_CALIBRATION_CURRENT_COUNTS,
            State::RampDown => HALL_CALIBRATION_CURRENT_COUNTS - ramp_current(self.cycle_in_state),
            State::Idle | State::Complete | State::Failed | State::Finalize => {
                return Actuation::Off;
            }
        };
        Actuation::Current {
            angle: self.angle,
            direct_counts: current,
            quadrature_counts: 0,
        }
    }

    pub fn observe(&mut self, raw_hall: u8) {
        self.cycle_in_state = self.cycle_in_state.saturating_add(1);
        match self.state {
            State::RampUp if self.cycle_in_state >= RAMP_CYCLES => self.enter(State::Settle),
            State::Settle if self.cycle_in_state >= SETTLE_CYCLES => self.enter(State::Sweep),
            State::Sweep if self.cycle_in_state >= STEP_CYCLES => self.record_step(raw_hall),
            State::RampDown if self.cycle_in_state >= RAMP_CYCLES => self.begin_finalization(),
            State::Finalize => self.finalize_next_raw(),
            _ => {}
        }
    }

    fn record_step(&mut self, raw_hall: u8) {
        self.cycle_in_state = 0;
        let index = usize::from(raw_hall & 7);
        let direction = usize::from(self.sweep & 1);
        if self.counts[direction][index] == 0 {
            self.anchor[direction][index] = self.angle;
        }
        self.offset_sum[direction][index] +=
            i64::from(self.angle.wrapping_sub(self.anchor[direction][index]) as i32);
        self.counts[direction][index] = self.counts[direction][index].saturating_add(1);
        self.degree = self.degree.saturating_add(1);
        if self.degree >= DEGREES_PER_SWEEP {
            self.sweep = self.sweep.saturating_add(1);
            self.degree = 0;
            if self.sweep >= SWEEP_COUNT {
                self.angle = 0;
                self.enter(State::RampDown);
                return;
            }
        }
        let degree = if self.sweep & 1 == 0 {
            self.degree
        } else {
            DEGREES_PER_SWEEP - 1 - self.degree
        };
        self.angle = ((u64::from(degree) << 32) / u64::from(DEGREES_PER_SWEEP)) as u32;
    }

    fn begin_finalization(&mut self) {
        if self.counts[0][0] != 0
            || self.counts[0][7] != 0
            || self.counts[1][0] != 0
            || self.counts[1][7] != 0
        {
            self.fail(Failure::HallStates);
            return;
        }
        self.finalize_raw = 1;
        self.directional_centers = [[0; 8]; 2];
        self.directional_minimum = [u16::MAX; 2];
        self.enter(State::Finalize);
    }

    fn finalize_next_raw(&mut self) {
        let raw = usize::from(self.finalize_raw);
        for direction in 0..2 {
            let count = self.counts[direction][raw];
            if count < MIN_SAMPLES_PER_STATE {
                self.fail(Failure::HallStates);
                return;
            }
            self.directional_minimum[direction] = self.directional_minimum[direction].min(count);
            let mean_offset = self.offset_sum[direction][raw] / i64::from(count);
            self.directional_centers[direction][raw] =
                (self.anchor[direction][raw].wrapping_add(mean_offset as i32 as u32) >> 16) as u16;
        }
        if self.finalize_raw < 6 {
            self.finalize_raw += 1;
            return;
        }
        self.result = Result {
            forward_centers_q16: self.directional_centers[0],
            reverse_centers_q16: self.directional_centers[1],
            valid_mask: 0x7e,
            forward_minimum_samples: self.directional_minimum[0].min(u16::from(u8::MAX)) as u8,
            reverse_minimum_samples: self.directional_minimum[1].min(u16::from(u8::MAX)) as u8,
        };
        self.enter(State::Complete);
    }

    fn enter(&mut self, state: State) {
        self.state = state;
        self.cycle_in_state = 0;
    }
}

fn ramp_current(cycle: u32) -> i16 {
    (i64::from(HALL_CALIBRATION_CURRENT_COUNTS) * i64::from(cycle.min(RAMP_CYCLES))
        / i64::from(RAMP_CYCLES)) as i16
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fixed_circular_average_handles_the_turn_boundary() {
        let angles = [u32::MAX - (1 << 24), 0, 1 << 24];
        let anchor = angles[0];
        let mut offset_sum = 0_i64;
        for angle in &angles {
            offset_sum += i64::from(angle.wrapping_sub(anchor) as i32);
        }
        let center = anchor.wrapping_add((offset_sum / 3) as i32 as u32);
        assert!(center < 1 << 20 || center > u32::MAX - (1 << 20));
    }

    #[test]
    fn six_bidirectional_sweeps_recover_all_raw_state_centers() {
        let raw_by_sector = [1_u8, 3, 2, 6, 4, 5];
        let mut calibration = HallCalibration::new();
        calibration.state = State::Sweep;
        for _ in 0..u32::from(SWEEP_COUNT) * u32::from(DEGREES_PER_SWEEP) {
            let sector = ((u64::from(calibration.angle) * 6) >> 32) as usize;
            calibration.record_step(raw_by_sector[sector]);
        }
        assert_eq!(calibration.state, State::RampDown);
        calibration.begin_finalization();
        assert_eq!(calibration.state, State::Finalize);
        assert_eq!(calibration.actuation(), Actuation::Off);
        for raw in 1..=6 {
            assert_eq!(calibration.finalize_raw, raw);
            calibration.observe(0);
        }
        assert_eq!(calibration.state, State::Complete);
        let result = calibration.result();
        assert_eq!(result.valid_mask, 0x7e);
        assert!(result.forward_minimum_samples >= 30);
        assert!(result.reverse_minimum_samples >= 30);
        for (sector, raw) in raw_by_sector.into_iter().enumerate() {
            let expected = ((sector as u32 * 60 + 30) * 65_536 / 360) as u16;
            assert!(result.forward_centers_q16[usize::from(raw)].abs_diff(expected) < 200);
            assert!(result.reverse_centers_q16[usize::from(raw)].abs_diff(expected) < 200);
        }
    }
}
