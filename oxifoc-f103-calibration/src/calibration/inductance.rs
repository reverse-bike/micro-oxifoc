//! OxiFOC discharge-anchored voltage-pulse inductance measurement.
//!
//! The rotor is PI-locked, the settled holding voltage is measured, and a
//! current-budgeted direct-voltage step is applied for the latency scan. The
//! largest one-period current rise is used for each pulse. Reporting remains
//! in native units: `L * current_scale` as nanowebers per ADC count.

use crate::calibration::types::{Actuation, Failure};
use crate::config::{
    CURRENT_LOOP_TUNING_BANDWIDTH_RAD_S, INDUCTANCE_HOLD_CURRENT_COUNTS,
    INDUCTANCE_PULSES_PER_AXIS, INDUCTANCE_TARGET_DI_COUNTS, PWM_HZ, PWM_PERIOD_TICKS,
};

const QUARTER_TURN: u32 = 0x4000_0000;
const RAMP_CYCLES: u32 = PWM_HZ / 2;
const SETTLE_CYCLES: u32 = PWM_HZ / 5;
const HOLD_SAMPLE_CYCLES: u32 = PWM_HZ / 5;
const AXIS_PAUSE_CYCLES: u32 = PWM_HZ / 5;
const DISCHARGE_MAX_CYCLES: u32 = PWM_HZ / 5;
const PIPELINE_SCAN_CYCLES: u8 = 5;
const MAX_PULSE_ATTEMPTS: u8 = INDUCTANCE_PULSES_PER_AXIS * 2;
const MIN_VALID_INDUCTANCE_NWB_PER_COUNT: u32 = 100;
const MAX_VALID_INDUCTANCE_NWB_PER_COUNT: u32 = 1_000_000;
const Q16: i64 = 1_i64 << 16;

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[repr(u8)]
pub enum State {
    #[default]
    Idle = 0,
    RampHold = 1,
    SettleHold = 2,
    SampleHold = 3,
    Discharge = 4,
    PulseCommand = 5,
    PulseScan = 6,
    RampDown = 7,
    AxisPause = 8,
    Complete = 9,
    Failed = 10,
}

impl State {
    pub const fn active(self) -> bool {
        matches!(
            self,
            Self::RampHold
                | Self::SettleHold
                | Self::SampleHold
                | Self::Discharge
                | Self::PulseCommand
                | Self::PulseScan
                | Self::RampDown
                | Self::AxisPause
        )
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Observation {
    pub measured_d_counts: i16,
    pub applied_d_tick_bits: i32,
    pub bus_voltage_mv: u32,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Result {
    pub inductance_d_nwb_per_count: u32,
    pub inductance_q_nwb_per_count: u32,
    pub residual_dead_time_uv: u32,
    pub pulse_step_tick_bits: i32,
    pub last_pulse_di_counts: i16,
    pub gain_bus_voltage_mv: u16,
    pub proportional_d_q16: i32,
    pub proportional_q_q16: i32,
    pub integral_per_cycle_q16: i32,
    pub bandwidth_rad_s: u16,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct InductanceCalibration {
    state: State,
    failure: Failure,
    pending_failure: Failure,
    cycle_in_state: u32,
    axis: u8,
    resistance_uv_per_count: u32,
    hold_current_sum: i64,
    hold_voltage_tick_bits_sum: i64,
    hold_voltage_tick_mv_q16_sum: i64,
    hold_sample_count: u32,
    hold_current_counts: i16,
    hold_voltage_tick_bits: i32,
    pulse_ceiling_tick_bits: i32,
    pulse_step_tick_bits: i32,
    probing: bool,
    previous_current_counts: i16,
    best_before_counts: i16,
    best_after_counts: i16,
    best_di_counts: i16,
    scan_remaining: u8,
    pulse_attempts: u8,
    valid_pulses: u8,
    inductance_sum_nwb_per_count: u64,
    result: Result,
}

impl InductanceCalibration {
    pub const fn new() -> Self {
        Self {
            state: State::Idle,
            failure: Failure::None,
            pending_failure: Failure::None,
            cycle_in_state: 0,
            axis: 0,
            resistance_uv_per_count: 0,
            hold_current_sum: 0,
            hold_voltage_tick_bits_sum: 0,
            hold_voltage_tick_mv_q16_sum: 0,
            hold_sample_count: 0,
            hold_current_counts: 0,
            hold_voltage_tick_bits: 0,
            pulse_ceiling_tick_bits: 0,
            pulse_step_tick_bits: 0,
            probing: true,
            previous_current_counts: 0,
            best_before_counts: 0,
            best_after_counts: 0,
            best_di_counts: i16::MIN,
            scan_remaining: 0,
            pulse_attempts: 0,
            valid_pulses: 0,
            inductance_sum_nwb_per_count: 0,
            result: Result {
                inductance_d_nwb_per_count: 0,
                inductance_q_nwb_per_count: 0,
                residual_dead_time_uv: 0,
                pulse_step_tick_bits: 0,
                last_pulse_di_counts: 0,
                gain_bus_voltage_mv: 0,
                proportional_d_q16: 0,
                proportional_q_q16: 0,
                integral_per_cycle_q16: 0,
                bandwidth_rad_s: 0,
            },
        }
    }

    pub fn start(&mut self, resistance_uv_per_count: u32) {
        *self = Self::new();
        if resistance_uv_per_count == 0 {
            self.fail(Failure::MissingPrerequisite);
            return;
        }
        self.resistance_uv_per_count = resistance_uv_per_count;
        self.state = State::RampHold;
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

    pub const fn result(&self) -> Result {
        self.result
    }

    pub const fn pulse_progress(&self) -> u16 {
        u16::from_le_bytes([self.valid_pulses, self.axis])
    }

    pub fn actuation(&self) -> Actuation {
        let angle = u32::from(self.axis) * QUARTER_TURN;
        match self.state {
            State::RampHold => Actuation::Current {
                angle,
                direct_counts: ramp_i16(
                    0,
                    INDUCTANCE_HOLD_CURRENT_COUNTS,
                    self.cycle_in_state,
                    RAMP_CYCLES,
                ),
                quadrature_counts: 0,
            },
            State::SettleHold | State::SampleHold => Actuation::Current {
                angle,
                direct_counts: INDUCTANCE_HOLD_CURRENT_COUNTS,
                quadrature_counts: 0,
            },
            State::Discharge => Actuation::DirectVoltage {
                angle,
                direct_tick_bits: self.hold_voltage_tick_bits,
            },
            State::PulseCommand | State::PulseScan => Actuation::DirectVoltage {
                angle,
                direct_tick_bits: self
                    .hold_voltage_tick_bits
                    .saturating_add(self.pulse_step_tick_bits),
            },
            State::RampDown => Actuation::DirectVoltage {
                angle,
                direct_tick_bits: ramp_i32(
                    self.hold_voltage_tick_bits,
                    0,
                    self.cycle_in_state,
                    RAMP_CYCLES,
                ),
            },
            State::Idle | State::AxisPause | State::Complete | State::Failed => Actuation::Off,
        }
    }

    pub fn observe(&mut self, observation: Observation) {
        match self.state {
            State::Idle | State::Complete | State::Failed => {}
            State::RampHold => self.advance_after(RAMP_CYCLES, State::SettleHold),
            State::SettleHold => self.advance_after(SETTLE_CYCLES, State::SampleHold),
            State::SampleHold => self.observe_hold(observation),
            State::Discharge => self.observe_discharge(observation.measured_d_counts),
            State::PulseCommand => self.begin_scan(observation.measured_d_counts),
            State::PulseScan => self.observe_scan(observation),
            State::RampDown => self.observe_ramp_down(),
            State::AxisPause => {
                self.cycle_in_state = self.cycle_in_state.saturating_add(1);
                if self.cycle_in_state >= AXIS_PAUSE_CYCLES {
                    self.axis = 1;
                    self.reset_axis();
                    self.state = State::RampHold;
                }
            }
        }
    }

    fn observe_hold(&mut self, observation: Observation) {
        self.hold_current_sum += i64::from(observation.measured_d_counts);
        self.hold_voltage_tick_bits_sum += i64::from(observation.applied_d_tick_bits);
        self.hold_voltage_tick_mv_q16_sum +=
            i64::from(observation.applied_d_tick_bits) * i64::from(observation.bus_voltage_mv);
        self.hold_sample_count = self.hold_sample_count.saturating_add(1);
        self.cycle_in_state = self.cycle_in_state.saturating_add(1);
        if self.cycle_in_state < HOLD_SAMPLE_CYCLES {
            return;
        }

        let count = i64::from(self.hold_sample_count);
        self.hold_current_counts = saturating_i64_to_i16(div_round(self.hold_current_sum, count));
        self.hold_voltage_tick_bits =
            saturating_i64_to_i32(div_round(self.hold_voltage_tick_bits_sum, count));
        if !settled(self.hold_current_counts, INDUCTANCE_HOLD_CURRENT_COUNTS) {
            self.pending_failure = Failure::CurrentDidNotSettle;
            self.state = State::RampDown;
            self.cycle_in_state = 0;
            return;
        }
        let hold_voltage_uv = tick_mv_q16_to_uv(
            div_round(self.hold_voltage_tick_mv_q16_sum, count),
            PWM_PERIOD_TICKS,
        );
        let residual = hold_voltage_uv.saturating_sub(
            i64::from(self.resistance_uv_per_count) * i64::from(self.hold_current_counts),
        );
        self.result.residual_dead_time_uv = self
            .result
            .residual_dead_time_uv
            .max(residual.max(0).min(i64::from(u32::MAX)) as u32);
        let vector_limit_bits = oxifoc_f103::config::FOC_VECTOR_LIMIT_TICKS
            .to_bits()
            .saturating_abs();
        self.pulse_ceiling_tick_bits = vector_limit_bits
            .saturating_sub(self.hold_voltage_tick_bits.saturating_abs())
            .max(1 << 16);
        self.pulse_step_tick_bits = (self.pulse_ceiling_tick_bits / 50).max(1 << 16);
        self.result.gain_bus_voltage_mv =
            observation.bus_voltage_mv.min(u32::from(u16::MAX)) as u16;
        self.state = State::Discharge;
        self.cycle_in_state = 0;
    }

    fn observe_discharge(&mut self, measured_d_counts: i16) {
        self.cycle_in_state = self.cycle_in_state.saturating_add(1);
        let settled = (i32::from(measured_d_counts) - i32::from(self.hold_current_counts))
            .unsigned_abs()
            <= (i32::from(INDUCTANCE_HOLD_CURRENT_COUNTS).unsigned_abs() / 10).max(2);
        if settled || self.cycle_in_state >= DISCHARGE_MAX_CYCLES {
            self.state = State::PulseCommand;
            self.cycle_in_state = 0;
        }
    }

    fn begin_scan(&mut self, measured_d_counts: i16) {
        self.previous_current_counts = measured_d_counts;
        self.best_before_counts = measured_d_counts;
        self.best_after_counts = measured_d_counts;
        self.best_di_counts = i16::MIN;
        self.scan_remaining = PIPELINE_SCAN_CYCLES;
        self.state = State::PulseScan;
    }

    fn observe_scan(&mut self, observation: Observation) {
        let di = observation
            .measured_d_counts
            .saturating_sub(self.previous_current_counts);
        if di > self.best_di_counts {
            self.best_di_counts = di;
            self.best_before_counts = self.previous_current_counts;
            self.best_after_counts = observation.measured_d_counts;
        }
        self.previous_current_counts = observation.measured_d_counts;
        self.scan_remaining = self.scan_remaining.saturating_sub(1);
        if self.scan_remaining == 0 {
            self.finish_pulse(observation.bus_voltage_mv);
        }
    }

    fn finish_pulse(&mut self, bus_voltage_mv: u32) {
        self.result.last_pulse_di_counts = self.best_di_counts;
        if self.probing {
            if self.best_di_counts >= INDUCTANCE_TARGET_DI_COUNTS {
                self.pulse_step_tick_bits = scale_pulse_to_target(
                    self.pulse_step_tick_bits,
                    self.best_di_counts,
                    self.pulse_ceiling_tick_bits,
                );
                self.probing = false;
            } else if self.pulse_step_tick_bits >= self.pulse_ceiling_tick_bits {
                self.probing = false;
            } else {
                self.pulse_step_tick_bits = (i64::from(self.pulse_step_tick_bits) * 3 / 2)
                    .min(i64::from(self.pulse_ceiling_tick_bits))
                    as i32;
            }
            self.state = State::Discharge;
            self.cycle_in_state = 0;
            return;
        }

        self.pulse_attempts = self.pulse_attempts.saturating_add(1);
        if self.best_di_counts >= 2
            && let Some(inductance) = pulse_inductance_nwb_per_count(
                self.resistance_uv_per_count,
                self.pulse_step_tick_bits,
                bus_voltage_mv,
                self.hold_current_counts,
                self.best_before_counts,
                self.best_after_counts,
            )
        {
            self.inductance_sum_nwb_per_count = self
                .inductance_sum_nwb_per_count
                .saturating_add(u64::from(inductance));
            self.valid_pulses = self.valid_pulses.saturating_add(1);
        }
        if self.valid_pulses >= INDUCTANCE_PULSES_PER_AXIS {
            let average = (self.inductance_sum_nwb_per_count / u64::from(self.valid_pulses))
                .min(u64::from(u32::MAX)) as u32;
            if self.axis == 0 {
                self.result.inductance_d_nwb_per_count = average;
            } else {
                self.result.inductance_q_nwb_per_count = average;
            }
            self.result.pulse_step_tick_bits = self.pulse_step_tick_bits;
            self.state = State::RampDown;
            self.cycle_in_state = 0;
        } else if self.pulse_attempts >= MAX_PULSE_ATTEMPTS {
            self.pending_failure = Failure::PulseResponse;
            self.state = State::RampDown;
            self.cycle_in_state = 0;
        } else {
            self.state = State::Discharge;
            self.cycle_in_state = 0;
        }
    }

    fn observe_ramp_down(&mut self) {
        self.cycle_in_state = self.cycle_in_state.saturating_add(1);
        if self.cycle_in_state < RAMP_CYCLES {
            return;
        }
        if self.pending_failure != Failure::None {
            self.state = State::Failed;
            self.failure = self.pending_failure;
            self.pending_failure = Failure::None;
        } else if self.axis == 0 {
            self.state = State::AxisPause;
        } else if self.finish_result() {
            self.state = State::Complete;
        } else {
            self.state = State::Failed;
            self.failure = Failure::InductanceRange;
        }
        self.cycle_in_state = 0;
    }

    fn finish_result(&mut self) -> bool {
        let ld = self.result.inductance_d_nwb_per_count;
        let lq = self.result.inductance_q_nwb_per_count;
        if !(MIN_VALID_INDUCTANCE_NWB_PER_COUNT..=MAX_VALID_INDUCTANCE_NWB_PER_COUNT).contains(&ld)
            || !(MIN_VALID_INDUCTANCE_NWB_PER_COUNT..=MAX_VALID_INDUCTANCE_NWB_PER_COUNT)
                .contains(&lq)
        {
            return false;
        }
        let bus_mv = u32::from(self.result.gain_bus_voltage_mv);
        let volts_per_tick_uv = bus_mv.saturating_mul(1_000) / u32::from(PWM_PERIOD_TICKS);
        if volts_per_tick_uv == 0 {
            return false;
        }
        self.result.proportional_d_q16 = proportional_gain_q16(ld, volts_per_tick_uv);
        self.result.proportional_q_q16 = proportional_gain_q16(lq, volts_per_tick_uv);
        self.result.integral_per_cycle_q16 =
            integral_gain_q16(self.resistance_uv_per_count, volts_per_tick_uv);
        self.result.bandwidth_rad_s = CURRENT_LOOP_TUNING_BANDWIDTH_RAD_S as u16;
        true
    }

    fn reset_axis(&mut self) {
        self.cycle_in_state = 0;
        self.hold_current_sum = 0;
        self.hold_voltage_tick_bits_sum = 0;
        self.hold_voltage_tick_mv_q16_sum = 0;
        self.hold_sample_count = 0;
        self.hold_current_counts = 0;
        self.hold_voltage_tick_bits = 0;
        self.pulse_ceiling_tick_bits = 0;
        self.pulse_step_tick_bits = 0;
        self.probing = true;
        self.pulse_attempts = 0;
        self.valid_pulses = 0;
        self.inductance_sum_nwb_per_count = 0;
    }

    fn advance_after(&mut self, cycles: u32, next: State) {
        self.cycle_in_state = self.cycle_in_state.saturating_add(1);
        if self.cycle_in_state >= cycles {
            self.state = next;
            self.cycle_in_state = 0;
        }
    }
}

fn pulse_inductance_nwb_per_count(
    resistance_uv_per_count: u32,
    pulse_step_tick_bits: i32,
    bus_voltage_mv: u32,
    hold_current_counts: i16,
    before_counts: i16,
    after_counts: i16,
) -> Option<u32> {
    let di = i64::from(after_counts) - i64::from(before_counts);
    if di <= 0 {
        return None;
    }
    let pulse_uv = i64::from(pulse_step_tick_bits)
        .saturating_mul(i64::from(bus_voltage_mv))
        .saturating_mul(1_000)
        / (i64::from(PWM_PERIOD_TICKS) * Q16);
    let twice_delta_from_hold =
        i64::from(before_counts) + i64::from(after_counts) - 2 * i64::from(hold_current_counts);
    let resistive_uv = i64::from(resistance_uv_per_count) * twice_delta_from_hold / 2;
    let inductive_uv = pulse_uv - resistive_uv;
    if inductive_uv <= 0 {
        return None;
    }
    let inductance = inductive_uv.saturating_mul(1_000) / (i64::from(PWM_HZ) * di);
    let inductance = u32::try_from(inductance).ok()?;
    (MIN_VALID_INDUCTANCE_NWB_PER_COUNT..=MAX_VALID_INDUCTANCE_NWB_PER_COUNT)
        .contains(&inductance)
        .then_some(inductance)
}

fn proportional_gain_q16(inductance_nwb_per_count: u32, volts_per_tick_uv: u32) -> i32 {
    let numerator = u64::from(inductance_nwb_per_count)
        .saturating_mul(u64::from(CURRENT_LOOP_TUNING_BANDWIDTH_RAD_S))
        .saturating_mul(1 << 16);
    let denominator = u64::from(volts_per_tick_uv).saturating_mul(1_000);
    (numerator / denominator).min(i32::MAX as u64) as i32
}

fn integral_gain_q16(resistance_uv_per_count: u32, volts_per_tick_uv: u32) -> i32 {
    let numerator = u64::from(resistance_uv_per_count)
        .saturating_mul(u64::from(CURRENT_LOOP_TUNING_BANDWIDTH_RAD_S))
        .saturating_mul(1 << 16);
    let denominator = u64::from(PWM_HZ).saturating_mul(u64::from(volts_per_tick_uv));
    (numerator / denominator).min(i32::MAX as u64) as i32
}

fn scale_pulse_to_target(pulse_bits: i32, measured_di: i16, ceiling_bits: i32) -> i32 {
    (i64::from(pulse_bits) * i64::from(INDUCTANCE_TARGET_DI_COUNTS) / i64::from(measured_di))
        .clamp(1_i64 << 16, i64::from(ceiling_bits)) as i32
}

fn tick_mv_q16_to_uv(tick_mv_q16: i64, period_ticks: u16) -> i64 {
    tick_mv_q16.saturating_mul(1_000) / (i64::from(period_ticks) * Q16)
}

fn ramp_i16(start: i16, end: i16, cycle: u32, duration: u32) -> i16 {
    let value = i64::from(start)
        + (i64::from(end) - i64::from(start)) * i64::from(cycle.min(duration))
            / i64::from(duration);
    saturating_i64_to_i16(value)
}

fn ramp_i32(start: i32, end: i32, cycle: u32, duration: u32) -> i32 {
    let value = i64::from(start)
        + (i64::from(end) - i64::from(start)) * i64::from(cycle.min(duration))
            / i64::from(duration);
    saturating_i64_to_i32(value)
}

fn settled(measured: i16, target: i16) -> bool {
    (i32::from(measured) - i32::from(target)).unsigned_abs()
        <= (i32::from(target).unsigned_abs() * 3 / 10).max(2)
}

fn div_round(value: i64, divisor: i64) -> i64 {
    if value < 0 {
        (value - divisor / 2) / divisor
    } else {
        (value + divisor / 2) / divisor
    }
}

fn saturating_i64_to_i16(value: i64) -> i16 {
    value.clamp(i64::from(i16::MIN), i64::from(i16::MAX)) as i16
}

fn saturating_i64_to_i32(value: i64) -> i32 {
    value.clamp(i64::from(i32::MIN), i64::from(i32::MAX)) as i32
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn native_pulse_equation_recovers_effective_inductance() {
        // 52.3 V bus, 100-tick pulse, 43 mV/count resistance coefficient,
        // 20-count rise around a 100-count hold.
        let pulse_bits = 100 << 16;
        let pulse_uv = 100_i64 * 52_300 * 1_000 / 2_250;
        let expected_di = 20_i64;
        let resistance_uv = 4_300_i64 * expected_di / 2;
        let expected = (pulse_uv - resistance_uv) * 1_000 / (16_000 * expected_di);
        assert_eq!(
            pulse_inductance_nwb_per_count(4_300, pulse_bits, 52_300, 100, 100, 120),
            Some(expected as u32)
        );
    }

    #[test]
    fn pi_gain_conversion_matches_the_fixed_tick_domain() {
        let volts_per_tick_uv = 52_300 * 1_000 / 2_250;
        let kp = proportional_gain_q16(7_500, volts_per_tick_uv);
        let ki = integral_gain_q16(4_300, volts_per_tick_uv);
        assert!((21_000..=21_300).contains(&kp));
        assert!((755..=760).contains(&ki));
    }

    #[test]
    fn pulse_probe_scales_back_to_the_current_target() {
        assert_eq!(scale_pulse_to_target(120 << 16, 30, 500 << 16), 80 << 16);
    }
}
