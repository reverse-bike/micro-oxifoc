//! OxiFOC two-point phase-resistance measurement in native controller units.
//!
//! The sequence preserves the original sweep: ramp to 20% current, settle and
//! sample, ramp to full test current, settle and sample, then ramp down. The
//! differential slope removes constant current offsets and bridge voltage
//! offsets. Conversion to physical resistance remains explicitly dependent on
//! the nominal current-sensor scale.

use super::types::Failure;
use crate::config::{
    NOMINAL_CURRENT_MA_PER_COUNT, PWM_HZ, PWM_PERIOD_TICKS, RESISTANCE_CURRENT_HIGH_COUNTS,
    RESISTANCE_CURRENT_LOW_COUNTS,
};

const RAMP_STEPS: u32 = 50;
const RAMP_STEP_CYCLES: u32 = PWM_HZ * 4 / 1_000;
const RAMP_CYCLES: u32 = RAMP_STEPS * RAMP_STEP_CYCLES;
const SETTLE_CYCLES: u32 = PWM_HZ;
const SAMPLE_CYCLES: u32 = PWM_HZ / 10;
const MIN_EFFECTIVE_UV_PER_COUNT: u32 = 500;
const MAX_EFFECTIVE_UV_PER_COUNT: u32 = 20_000;

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[repr(u8)]
pub enum State {
    #[default]
    Idle = 0,
    RampLow = 1,
    SettleLow = 2,
    SampleLow = 3,
    RampHigh = 4,
    SettleHigh = 5,
    SampleHigh = 6,
    RampDown = 7,
    Complete = 8,
    Failed = 9,
}

impl State {
    pub const fn active(self) -> bool {
        matches!(
            self,
            Self::RampLow
                | Self::SettleLow
                | Self::SampleLow
                | Self::RampHigh
                | Self::SettleHigh
                | Self::SampleHigh
                | Self::RampDown
        )
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Sample {
    pub measured_d_counts: i16,
    /// Q16.16 commanded d-axis phase-voltage ticks.
    pub applied_d_tick_bits: i32,
    pub bus_voltage_mv: u32,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Point {
    pub current_counts: i16,
    pub voltage_ticks: i16,
    pub bus_voltage_mv: u16,
    /// Average of `applied_d_tick_bits * bus_voltage_mv` in Q16.16 tick-mV.
    pub voltage_tick_mv_q16: i64,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Result {
    pub low: Point,
    pub high: Point,
    /// Effective terminal slope before assuming a physical current scale.
    pub effective_uv_per_count: u32,
    /// Physical phase resistance using `NOMINAL_CURRENT_MA_PER_COUNT` only.
    pub nominal_resistance_uohm: u32,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
struct Accumulator {
    current_sum: i64,
    voltage_tick_bits_sum: i64,
    bus_voltage_sum: u64,
    voltage_tick_mv_q16_sum: i64,
    samples: u32,
}

impl Accumulator {
    fn record(&mut self, sample: Sample) {
        self.current_sum += i64::from(sample.measured_d_counts);
        self.voltage_tick_bits_sum += i64::from(sample.applied_d_tick_bits);
        self.bus_voltage_sum = self
            .bus_voltage_sum
            .saturating_add(u64::from(sample.bus_voltage_mv));
        self.voltage_tick_mv_q16_sum +=
            i64::from(sample.applied_d_tick_bits) * i64::from(sample.bus_voltage_mv);
        self.samples = self.samples.saturating_add(1);
    }

    fn point(self) -> Point {
        if self.samples == 0 {
            return Point::default();
        }
        let divisor = i64::from(self.samples);
        Point {
            current_counts: saturating_i64_to_i16(div_round(self.current_sum, divisor)),
            voltage_ticks: saturating_i64_to_i16(div_round(
                div_round(self.voltage_tick_bits_sum, divisor),
                1_i64 << 16,
            )),
            bus_voltage_mv: (self.bus_voltage_sum / u64::from(self.samples))
                .min(u64::from(u16::MAX)) as u16,
            voltage_tick_mv_q16: div_round(self.voltage_tick_mv_q16_sum, divisor),
        }
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct ResistanceCalibration {
    state: State,
    failure: Failure,
    cycle_in_state: u32,
    low_accumulator: Accumulator,
    high_accumulator: Accumulator,
    result: Result,
    pending_failure: Failure,
}

impl ResistanceCalibration {
    pub const fn new() -> Self {
        Self {
            state: State::Idle,
            failure: Failure::None,
            cycle_in_state: 0,
            low_accumulator: Accumulator {
                current_sum: 0,
                voltage_tick_bits_sum: 0,
                bus_voltage_sum: 0,
                voltage_tick_mv_q16_sum: 0,
                samples: 0,
            },
            high_accumulator: Accumulator {
                current_sum: 0,
                voltage_tick_bits_sum: 0,
                bus_voltage_sum: 0,
                voltage_tick_mv_q16_sum: 0,
                samples: 0,
            },
            result: Result {
                low: Point {
                    current_counts: 0,
                    voltage_ticks: 0,
                    bus_voltage_mv: 0,
                    voltage_tick_mv_q16: 0,
                },
                high: Point {
                    current_counts: 0,
                    voltage_ticks: 0,
                    bus_voltage_mv: 0,
                    voltage_tick_mv_q16: 0,
                },
                effective_uv_per_count: 0,
                nominal_resistance_uohm: 0,
            },
            pending_failure: Failure::None,
        }
    }

    pub fn start(&mut self) {
        *self = Self::new();
        self.state = State::RampLow;
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

    pub fn sample_progress(&self) -> u16 {
        let samples = match self.state {
            State::SampleLow => self.low_accumulator.samples,
            State::SampleHigh => self.high_accumulator.samples,
            _ => 0,
        };
        samples.min(u32::from(u16::MAX)) as u16
    }

    pub fn target_counts(&self) -> i16 {
        match self.state {
            State::RampLow => ramp(0, RESISTANCE_CURRENT_LOW_COUNTS, self.cycle_in_state),
            State::SettleLow | State::SampleLow => RESISTANCE_CURRENT_LOW_COUNTS,
            State::RampHigh => ramp(
                RESISTANCE_CURRENT_LOW_COUNTS,
                RESISTANCE_CURRENT_HIGH_COUNTS,
                self.cycle_in_state,
            ),
            State::SettleHigh | State::SampleHigh => RESISTANCE_CURRENT_HIGH_COUNTS,
            State::RampDown => ramp(RESISTANCE_CURRENT_HIGH_COUNTS, 0, self.cycle_in_state),
            State::Idle | State::Complete | State::Failed => 0,
        }
    }

    /// Advances the sweep by one 16 kHz control sample.
    pub fn tick(&mut self, sample: Sample) {
        match self.state {
            State::Idle | State::Complete | State::Failed => {}
            State::RampLow => self.advance_after(RAMP_CYCLES, State::SettleLow),
            State::SettleLow => self.advance_after(SETTLE_CYCLES, State::SampleLow),
            State::SampleLow => {
                self.low_accumulator.record(sample);
                self.advance_after(SAMPLE_CYCLES, State::RampHigh);
            }
            State::RampHigh => self.advance_after(RAMP_CYCLES, State::SettleHigh),
            State::SettleHigh => self.advance_after(SETTLE_CYCLES, State::SampleHigh),
            State::SampleHigh => {
                self.high_accumulator.record(sample);
                self.cycle_in_state = self.cycle_in_state.saturating_add(1);
                if self.cycle_in_state >= SAMPLE_CYCLES {
                    self.result.low = self.low_accumulator.point();
                    self.result.high = self.high_accumulator.point();
                    self.pending_failure = self.calculate_result();
                    self.state = State::RampDown;
                    self.cycle_in_state = 0;
                }
            }
            State::RampDown => {
                self.cycle_in_state = self.cycle_in_state.saturating_add(1);
                if self.cycle_in_state >= RAMP_CYCLES {
                    if self.pending_failure == Failure::None {
                        self.state = State::Complete;
                    } else {
                        self.state = State::Failed;
                        self.failure = self.pending_failure;
                    }
                    self.pending_failure = Failure::None;
                    self.cycle_in_state = 0;
                }
            }
        }
    }

    fn advance_after(&mut self, cycles: u32, next: State) {
        self.cycle_in_state = self.cycle_in_state.saturating_add(1);
        if self.cycle_in_state >= cycles {
            self.state = next;
            self.cycle_in_state = 0;
        }
    }

    fn calculate_result(&mut self) -> Failure {
        let low = self.result.low;
        let high = self.result.high;
        if !settled(low.current_counts, RESISTANCE_CURRENT_LOW_COUNTS)
            || !settled(high.current_counts, RESISTANCE_CURRENT_HIGH_COUNTS)
        {
            return Failure::CurrentDidNotSettle;
        }
        let delta_current = i64::from(high.current_counts) - i64::from(low.current_counts);
        if delta_current.abs()
            < i64::from(RESISTANCE_CURRENT_HIGH_COUNTS - RESISTANCE_CURRENT_LOW_COUNTS) / 2
        {
            return Failure::CurrentDidNotSettle;
        }
        let delta_voltage_tick_mv_q16 = high.voltage_tick_mv_q16 - low.voltage_tick_mv_q16;
        let denominator = i64::from(PWM_PERIOD_TICKS) * delta_current * (1_i64 << 16);
        if denominator == 0 {
            return Failure::InvalidSlope;
        }
        let effective_uv_per_count =
            (delta_voltage_tick_mv_q16.saturating_mul(1_000) / denominator).unsigned_abs();
        let effective_uv_per_count = effective_uv_per_count.min(u64::from(u32::MAX)) as u32;
        if !(MIN_EFFECTIVE_UV_PER_COUNT..=MAX_EFFECTIVE_UV_PER_COUNT)
            .contains(&effective_uv_per_count)
        {
            return Failure::InvalidSlope;
        }
        self.result.effective_uv_per_count = effective_uv_per_count;
        self.result.nominal_resistance_uohm =
            effective_uv_per_count.saturating_mul(1_000) / NOMINAL_CURRENT_MA_PER_COUNT;
        Failure::None
    }
}

fn ramp(start: i16, end: i16, cycle: u32) -> i16 {
    let step = (cycle / RAMP_STEP_CYCLES + 1).min(RAMP_STEPS);
    let start = i32::from(start);
    let span = i32::from(end) - start;
    (start + span * step as i32 / RAMP_STEPS as i32) as i16
}

fn settled(measured: i16, target: i16) -> bool {
    let tolerance = i32::from(target).unsigned_abs().saturating_mul(3) / 10;
    (i32::from(measured) - i32::from(target)).unsigned_abs() <= tolerance.max(2)
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

#[cfg(test)]
mod tests {
    use super::*;

    fn run_to_completion(calibration: &mut ResistanceCalibration) {
        for _ in 0..50_000 {
            let current = calibration.target_counts();
            // At 45 V and ARR=2250, one tick is 20 mV. The simulated winding
            // has a 4 mV/count slope plus a constant 340 mV bridge offset.
            let voltage = 17 + i32::from(current) / 5;
            calibration.tick(Sample {
                measured_d_counts: current,
                applied_d_tick_bits: voltage << 16,
                bus_voltage_mv: 45_000,
            });
            if !calibration.active() {
                return;
            }
        }
        panic!("calibration did not finish");
    }

    #[test]
    fn two_point_slope_cancels_constant_bridge_voltage() {
        let mut calibration = ResistanceCalibration::new();
        calibration.start();
        run_to_completion(&mut calibration);

        assert_eq!(calibration.state(), State::Complete);
        assert_eq!(calibration.failure(), Failure::None);
        assert_eq!(calibration.result().effective_uv_per_count, 4_000);
        assert_eq!(calibration.result().nominal_resistance_uohm, 40_000);
        assert_eq!(calibration.result().low.current_counts, 50);
        assert_eq!(calibration.result().high.current_counts, 250);
    }

    #[test]
    fn fractional_voltage_ticks_preserve_the_loaded_motor_slope() {
        let mut calibration = ResistanceCalibration::new();
        calibration.start();
        for _ in 0..50_000 {
            let current = calibration.target_counts();
            let slope_bits =
                i64::from(current) * 4_300 * i64::from(PWM_PERIOD_TICKS) * (1_i64 << 16)
                    / (52_300 * 1_000);
            calibration.tick(Sample {
                measured_d_counts: current,
                applied_d_tick_bits: (12_i64 * (1_i64 << 16) + slope_bits) as i32,
                bus_voltage_mv: 52_300,
            });
            if !calibration.active() {
                break;
            }
        }

        assert_eq!(calibration.state(), State::Complete);
        assert!(calibration.result().effective_uv_per_count.abs_diff(4_300) <= 1);
    }

    #[test]
    fn a_current_loop_that_does_not_land_is_rejected_after_ramp_down() {
        let mut calibration = ResistanceCalibration::new();
        calibration.start();
        for _ in 0..50_000 {
            calibration.tick(Sample {
                measured_d_counts: 0,
                applied_d_tick_bits: 17 << 16,
                bus_voltage_mv: 45_000,
            });
            if !calibration.active() {
                break;
            }
        }
        assert_eq!(calibration.state(), State::Failed);
        assert_eq!(calibration.failure(), Failure::CurrentDidNotSettle);
    }

    #[test]
    fn explicit_failure_stops_the_sequence_immediately() {
        let mut calibration = ResistanceCalibration::new();
        calibration.start();
        calibration.fail(Failure::Stopped);
        assert_eq!(calibration.target_counts(), 0);
        assert_eq!(calibration.state(), State::Failed);
        assert_eq!(calibration.failure(), Failure::Stopped);
    }
}
