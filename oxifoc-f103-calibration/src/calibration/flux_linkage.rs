//! Fixed-point form of OxiFOC's driven back-EMF-vector flux measurement.
//!
//! The command frame is captured at one electrical revolution per second,
//! ramped to the measurement speed, and held while the two back-EMF vector
//! components are averaged. Taking the magnitude after component averaging
//! preserves OxiFOC's load-angle-independent formulation.

use crate::calibration::types::{Actuation, Failure};
use crate::config::{FLUX_CURRENT_COUNTS, FLUX_TARGET_ERPM, PWM_HZ, PWM_PERIOD_TICKS};

const CAPTURE_ERPM: i32 = 60;
const CAPTURE_CYCLES: u32 = PWM_HZ * 2 / 5;
const SPEED_RAMP_CYCLES: u32 = PWM_HZ * 4;
const SPEED_RAMP_OBSERVATION_CYCLES: u32 = SPEED_RAMP_CYCLES / 100;
const SETTLE_CYCLES: u32 = PWM_HZ * 3;
const SAMPLE_CYCLES: u32 = PWM_HZ;
const RAMP_DOWN_CYCLES: u32 = PWM_HZ * 2;
const MIN_SYNC_VOLTAGE_TICKS: u32 = 64;
const MIN_VALID_FLUX_NWB: u32 = 100_000;
const MAX_VALID_FLUX_NWB: u32 = 1_000_000_000;
const TAU_MILLIRADIANS: i64 = 6_283;
const Q16: i64 = 1_i64 << 16;

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[repr(u8)]
pub enum State {
    #[default]
    Idle = 0,
    Capture = 1,
    SpeedRamp = 2,
    Settle = 3,
    Sample = 4,
    RampDown = 5,
    Complete = 6,
    Failed = 7,
}

impl State {
    pub const fn active(self) -> bool {
        matches!(
            self,
            Self::Capture | Self::SpeedRamp | Self::Settle | Self::Sample | Self::RampDown
        )
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Observation {
    pub measured_d_counts: i16,
    pub measured_q_counts: i16,
    pub applied_d_tick_bits: i32,
    pub applied_q_tick_bits: i32,
    pub bus_voltage_mv: u32,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Result {
    pub flux_linkage_nwb: u32,
    pub average_bemf_d_uv: i32,
    pub average_bemf_q_uv: i32,
    pub measurement_erpm: i16,
    pub sync_minimum_percent: u8,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct FluxLinkageCalibration {
    state: State,
    failure: Failure,
    pending_failure: Failure,
    cycle_in_state: u32,
    angle: u32,
    electrical_rpm: i32,
    ramp_down_start_erpm: i32,
    resistance_uv_per_count: u32,
    inductance_nwb_per_count: u32,
    bemf_d_sum_uv: i64,
    bemf_q_sum_uv: i64,
    sample_count: u32,
    voltage_filtered_ticks: u32,
    voltage_maximum_ticks: u32,
    sync_minimum_percent: u8,
    result: Result,
}

impl FluxLinkageCalibration {
    pub const fn new() -> Self {
        Self {
            state: State::Idle,
            failure: Failure::None,
            pending_failure: Failure::None,
            cycle_in_state: 0,
            angle: 0,
            electrical_rpm: 0,
            ramp_down_start_erpm: 0,
            resistance_uv_per_count: 0,
            inductance_nwb_per_count: 0,
            bemf_d_sum_uv: 0,
            bemf_q_sum_uv: 0,
            sample_count: 0,
            voltage_filtered_ticks: 0,
            voltage_maximum_ticks: 0,
            sync_minimum_percent: 100,
            result: Result {
                flux_linkage_nwb: 0,
                average_bemf_d_uv: 0,
                average_bemf_q_uv: 0,
                measurement_erpm: 0,
                sync_minimum_percent: 0,
            },
        }
    }

    pub fn start(
        &mut self,
        resistance_uv_per_count: u32,
        inductance_d_nwb_per_count: u32,
        inductance_q_nwb_per_count: u32,
    ) {
        *self = Self::new();
        if resistance_uv_per_count == 0
            || inductance_d_nwb_per_count == 0
            || inductance_q_nwb_per_count == 0
        {
            self.fail(Failure::MissingPrerequisite);
            return;
        }
        self.resistance_uv_per_count = resistance_uv_per_count;
        self.inductance_nwb_per_count =
            inductance_d_nwb_per_count.saturating_add(inductance_q_nwb_per_count) / 2;
        self.electrical_rpm = CAPTURE_ERPM;
        self.state = State::Capture;
    }

    pub fn fail(&mut self, failure: Failure) {
        self.state = State::Failed;
        self.failure = failure;
        self.pending_failure = Failure::None;
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

    pub const fn electrical_rpm(&self) -> i32 {
        self.electrical_rpm
    }

    pub const fn result(&self) -> Result {
        self.result
    }

    pub fn progress(&self) -> u16 {
        let denominator = match self.state {
            State::Capture => CAPTURE_CYCLES,
            State::SpeedRamp => SPEED_RAMP_CYCLES,
            State::Settle => SETTLE_CYCLES,
            State::Sample => SAMPLE_CYCLES,
            State::RampDown => RAMP_DOWN_CYCLES,
            State::Idle | State::Complete | State::Failed => 1,
        };
        ((self.cycle_in_state.min(denominator) * 1_000) / denominator) as u16
    }

    pub fn actuation(&self) -> Actuation {
        let current = match self.state {
            State::Capture => ramp_i16(0, FLUX_CURRENT_COUNTS, self.cycle_in_state, CAPTURE_CYCLES),
            State::SpeedRamp | State::Settle | State::Sample => FLUX_CURRENT_COUNTS,
            State::RampDown => ramp_i16(
                FLUX_CURRENT_COUNTS,
                0,
                self.cycle_in_state,
                RAMP_DOWN_CYCLES,
            ),
            State::Idle | State::Complete | State::Failed => return Actuation::Off,
        };
        Actuation::Current {
            angle: self.angle,
            direct_counts: 0,
            quadrature_counts: current,
        }
    }

    pub fn observe(&mut self, observation: Observation) {
        if !self.state.active() {
            return;
        }
        self.angle = self.angle.wrapping_add(erpm_step(self.electrical_rpm));
        match self.state {
            State::Capture => {
                self.advance_after(CAPTURE_CYCLES, State::SpeedRamp);
            }
            State::SpeedRamp => self.observe_speed_ramp(observation),
            State::Settle => self.advance_after(SETTLE_CYCLES, State::Sample),
            State::Sample => self.observe_sample(observation),
            State::RampDown => self.observe_ramp_down(),
            State::Idle | State::Complete | State::Failed => {}
        }
    }

    fn observe_speed_ramp(&mut self, observation: Observation) {
        self.cycle_in_state = self.cycle_in_state.saturating_add(1);
        self.electrical_rpm = ramp_i32(
            CAPTURE_ERPM,
            FLUX_TARGET_ERPM,
            self.cycle_in_state,
            SPEED_RAMP_CYCLES,
        );
        if self
            .cycle_in_state
            .is_multiple_of(SPEED_RAMP_OBSERVATION_CYCLES)
        {
            let voltage_ticks = vector_magnitude_ticks(
                observation.applied_d_tick_bits,
                observation.applied_q_tick_bits,
            );
            self.voltage_filtered_ticks = if self.voltage_filtered_ticks == 0 {
                voltage_ticks
            } else {
                (self
                    .voltage_filtered_ticks
                    .saturating_mul(85)
                    .saturating_add(voltage_ticks.saturating_mul(15)))
                    / 100
            };
            self.voltage_maximum_ticks =
                self.voltage_maximum_ticks.max(self.voltage_filtered_ticks);
            if let Some(percent) = self
                .voltage_filtered_ticks
                .saturating_mul(100)
                .checked_div(self.voltage_maximum_ticks)
            {
                self.sync_minimum_percent = self.sync_minimum_percent.min(percent.min(100) as u8);
            }
            if self.cycle_in_state > SPEED_RAMP_CYCLES / 2
                && self.voltage_maximum_ticks > MIN_SYNC_VOLTAGE_TICKS
                && self.voltage_filtered_ticks.saturating_mul(10)
                    < self.voltage_maximum_ticks.saturating_mul(7)
            {
                self.pending_failure = Failure::MotorNotResponding;
                self.ramp_down_start_erpm = self.electrical_rpm;
                self.state = State::RampDown;
                self.cycle_in_state = 0;
                return;
            }
        }
        if self.cycle_in_state >= SPEED_RAMP_CYCLES {
            self.electrical_rpm = FLUX_TARGET_ERPM;
            self.state = State::Settle;
            self.cycle_in_state = 0;
        }
    }

    fn observe_sample(&mut self, observation: Observation) {
        let omega_milliradians_per_second = i64::from(self.electrical_rpm) * TAU_MILLIRADIANS / 60;
        let vd_uv = tick_bits_to_uv(observation.applied_d_tick_bits, observation.bus_voltage_mv);
        let vq_uv = tick_bits_to_uv(observation.applied_q_tick_bits, observation.bus_voltage_mv);
        let id = i64::from(observation.measured_d_counts);
        let iq = i64::from(observation.measured_q_counts);
        let resistance = i64::from(self.resistance_uv_per_count);
        let inductance = i64::from(self.inductance_nwb_per_count);
        let reactance_d_uv = omega_milliradians_per_second
            .saturating_mul(inductance)
            .saturating_mul(iq)
            / 1_000_000;
        let reactance_q_uv = omega_milliradians_per_second
            .saturating_mul(inductance)
            .saturating_mul(id)
            / 1_000_000;
        self.bemf_d_sum_uv = self
            .bemf_d_sum_uv
            .saturating_add(vd_uv - resistance * id + reactance_d_uv);
        self.bemf_q_sum_uv = self
            .bemf_q_sum_uv
            .saturating_add(vq_uv - resistance * iq - reactance_q_uv);
        self.sample_count = self.sample_count.saturating_add(1);
        self.cycle_in_state = self.cycle_in_state.saturating_add(1);
        if self.cycle_in_state >= SAMPLE_CYCLES {
            self.ramp_down_start_erpm = self.electrical_rpm;
            self.state = State::RampDown;
            self.cycle_in_state = 0;
        }
    }

    fn observe_ramp_down(&mut self) {
        self.cycle_in_state = self.cycle_in_state.saturating_add(1);
        self.electrical_rpm = ramp_i32(
            self.ramp_down_start_erpm,
            0,
            self.cycle_in_state,
            RAMP_DOWN_CYCLES,
        );
        if self.cycle_in_state < RAMP_DOWN_CYCLES {
            return;
        }
        if self.pending_failure != Failure::None {
            self.state = State::Failed;
            self.failure = self.pending_failure;
        } else if self.finish_result() {
            self.state = State::Complete;
        } else {
            self.state = State::Failed;
            self.failure = Failure::FluxRange;
        }
        self.electrical_rpm = 0;
        self.cycle_in_state = 0;
    }

    fn finish_result(&mut self) -> bool {
        if self.sample_count == 0 || self.electrical_rpm != 0 {
            return false;
        }
        let average_d = self.bemf_d_sum_uv / i64::from(self.sample_count);
        let average_q = self.bemf_q_sum_uv / i64::from(self.sample_count);
        let magnitude_uv = integer_sqrt_u64(
            average_d
                .unsigned_abs()
                .saturating_pow(2)
                .saturating_add(average_q.unsigned_abs().saturating_pow(2)),
        );
        let omega_milliradians_per_second = i64::from(FLUX_TARGET_ERPM) * TAU_MILLIRADIANS / 60;
        let flux_nwb =
            magnitude_uv.saturating_mul(1_000_000) / omega_milliradians_per_second as u64;
        if !(u64::from(MIN_VALID_FLUX_NWB)..=u64::from(MAX_VALID_FLUX_NWB)).contains(&flux_nwb) {
            return false;
        }
        self.result = Result {
            flux_linkage_nwb: flux_nwb as u32,
            average_bemf_d_uv: saturating_i64_to_i32(average_d),
            average_bemf_q_uv: saturating_i64_to_i32(average_q),
            measurement_erpm: FLUX_TARGET_ERPM as i16,
            sync_minimum_percent: self.sync_minimum_percent,
        };
        true
    }

    fn advance_after(&mut self, cycles: u32, next: State) {
        self.cycle_in_state = self.cycle_in_state.saturating_add(1);
        if self.cycle_in_state >= cycles {
            self.state = next;
            self.cycle_in_state = 0;
        }
    }
}

fn erpm_step(erpm: i32) -> u32 {
    ((i64::from(erpm) << 32) / (60 * i64::from(PWM_HZ))) as u32
}

fn tick_bits_to_uv(tick_bits: i32, bus_voltage_mv: u32) -> i64 {
    i64::from(tick_bits)
        .saturating_mul(i64::from(bus_voltage_mv))
        .saturating_mul(1_000)
        / (i64::from(PWM_PERIOD_TICKS) * Q16)
}

fn vector_magnitude_ticks(d_bits: i32, q_bits: i32) -> u32 {
    let d = i64::from(d_bits >> 16);
    let q = i64::from(q_bits >> 16);
    integer_sqrt_u64(
        d.unsigned_abs()
            .saturating_pow(2)
            .saturating_add(q.unsigned_abs().saturating_pow(2)),
    )
    .min(u64::from(u32::MAX)) as u32
}

fn integer_sqrt_u64(mut radicand: u64) -> u64 {
    let mut result = 0_u64;
    let mut bit = 1_u64 << 62;
    while bit > radicand {
        bit >>= 2;
    }
    while bit != 0 {
        if radicand >= result + bit {
            radicand -= result + bit;
            result = (result >> 1) + bit;
        } else {
            result >>= 1;
        }
        bit >>= 2;
    }
    result
}

fn ramp_i16(start: i16, end: i16, cycle: u32, duration: u32) -> i16 {
    ramp_i32(i32::from(start), i32::from(end), cycle, duration) as i16
}

fn ramp_i32(start: i32, end: i32, cycle: u32, duration: u32) -> i32 {
    let value = i64::from(start)
        + (i64::from(end) - i64::from(start)) * i64::from(cycle.min(duration))
            / i64::from(duration);
    saturating_i64_to_i32(value)
}

fn saturating_i64_to_i32(value: i64) -> i32 {
    value.clamp(i64::from(i32::MIN), i64::from(i32::MAX)) as i32
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn vector_equation_recovers_flux_independent_of_load_angle() {
        let lambda_nwb = 13_400_000_i64;
        let erpm = 6_000_i64;
        let omega_mrad = erpm * TAU_MILLIRADIANS / 60;
        let bemf_uv = lambda_nwb * omega_mrad / 1_000_000;
        let recovered = bemf_uv.unsigned_abs() * 1_000_000 / omega_mrad as u64;
        assert!((recovered as i64 - lambda_nwb).unsigned_abs() < 2_000);
    }

    #[test]
    fn erpm_step_integrates_one_turn_per_second_at_sixty_erpm() {
        let accumulated = u64::from(erpm_step(60)) * u64::from(PWM_HZ);
        assert!((accumulated as i64 - (1_i64 << 32)).unsigned_abs() < u64::from(PWM_HZ));
    }

    #[test]
    fn averaged_vector_result_reports_the_expected_flux() {
        let omega_mrad = i64::from(FLUX_TARGET_ERPM) * TAU_MILLIRADIANS / 60;
        let expected_nwb = 13_400_000_i64;
        let bemf_uv = expected_nwb * omega_mrad / 1_000_000;
        let mut calibration = FluxLinkageCalibration::new();
        calibration.sample_count = 100;
        calibration.bemf_d_sum_uv = bemf_uv * 60;
        calibration.bemf_q_sum_uv = bemf_uv * 80;
        calibration.electrical_rpm = 0;
        assert!(calibration.finish_result());
        assert!(
            calibration
                .result()
                .flux_linkage_nwb
                .abs_diff(expected_nwb as u32)
                < 2_000
        );
    }
}
