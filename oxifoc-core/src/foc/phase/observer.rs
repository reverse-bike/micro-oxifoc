//! Sensorless phase observers for the synchronous FOC controller.
//!
//! [`BackEmfObserver`] is OxiFOC's MXLEMMING-style flux integrator and PLL,
//! expressed in the fixed-point units used by the F103 control loop. Voltage
//! and current inputs remain physical volts and amps; flux is represented in
//! milliwebers so sub-0.1 mH motor inductance retains useful Q16.16 precision.
//! The observer runs at the current-loop rate and is selected by
//! [`super::PhaseManager`], including Hall-to-observer crossover policy.

use crate::foc::numeric::Fixed;

use super::PhaseInput;
use crate::foc::trig::{CordicSinCos, SinCos, Turns};

const READY_MIN_CONFIDENCE: Fixed = Fixed::ratio(1, 2);
const READY_MIN_ERPM: i32 = 287;
const READY_ACQUIRE_MAX_ERROR_Q32: u32 = 136_713_055; // 0.2 rad
const READY_HOLD_MAX_ERROR_Q32: u32 = 410_139_165; // 0.6 rad
const READY_VALID_TRAVEL_Q32: u64 = 2_u64 << 32;
const VALID_BEMF_RATIO_MIN_NUMERATOR: i64 = 1;
const VALID_BEMF_RATIO_MIN_DENOMINATOR: i64 = 4;
const VALID_BEMF_RATIO_MAX_NUMERATOR: i64 = 5;
const VALID_BEMF_RATIO_MAX_DENOMINATOR: i64 = 2;
const TAU_Q16: i64 = 411_775;

/// Fixed-point MXLEMMING flux observer with a phase-locked loop.
///
/// Constructor units deliberately preserve the physical motor model:
/// resistance is ohms, inductance is millihenries, and flux linkage is
/// milliwebers. The milliscale flux units avoid quantizing a traction motor's
/// inductive `L * delta_i` term out of Q16.16.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct BackEmfObserver {
    flux_alpha_mwb: Fixed,
    flux_beta_mwb: Fixed,
    current_alpha_last_a: Fixed,
    current_beta_last_a: Fixed,
    phase_raw: Turns,
    phase_pll: Turns,
    velocity_step_q32: i32,
    phase_error_filtered_q32: u32,
    flux_magnitude_mwb: Fixed,
    confidence: Fixed,
    bemf_q_filtered_v: Fixed,
    valid_travel_q32: u64,
    invalid_cycles: u32,
    ready_latched: bool,
    resistance_ohms: Fixed,
    inductance_millihenries: Fixed,
    flux_linkage_mwb: Fixed,
    inverse_flux_linkage: Fixed,
    bemf_volts_per_erpm_q24: i32,
    control_frequency_hz: u32,
    flux_voltage_step: Fixed,
    centering_step: Fixed,
    bemf_filter_step: Fixed,
    pll_kp_step_q32: u32,
    pll_ki_step_q32: u32,
    phase_error_filter_step_q32: u32,
    valid_revoke_cycles: u32,
}

impl BackEmfObserver {
    /// Build the round-rotor observer used by the Hall crossover.
    pub const fn new(
        resistance_ohms: Fixed,
        inductance_millihenries: Fixed,
        flux_linkage_mwb: Fixed,
        control_frequency_hz: u32,
    ) -> Self {
        assert!(control_frequency_hz > 0);
        assert!(flux_linkage_mwb.to_bits() > 0);
        let inverse_flux = (1_i64 << 32) / flux_linkage_mwb.to_bits() as i64;
        let inverse_flux_bits = if inverse_flux < 0 {
            0
        } else if inverse_flux > i32::MAX as i64 {
            i32::MAX
        } else {
            inverse_flux as i32
        };
        let frequency = control_frequency_hz as u64;
        let bemf_coefficient_denominator = (1_i64 << 16) * 60_000;
        let bemf_volts_per_erpm_q24 = (flux_linkage_mwb.to_bits() as i64 * TAU_Q16 * 256
            + bemf_coefficient_denominator / 2)
            / bemf_coefficient_denominator;
        let flux_voltage_step = (1_000_u64 << 16) / frequency;
        let centering_step = (500_u64 << 16) / frequency;
        let bemf_filter_step = (40_u64 << 16) / frequency;
        Self {
            flux_alpha_mwb: Fixed::ZERO,
            flux_beta_mwb: Fixed::ZERO,
            current_alpha_last_a: Fixed::ZERO,
            current_beta_last_a: Fixed::ZERO,
            phase_raw: 0,
            phase_pll: 0,
            velocity_step_q32: 0,
            phase_error_filtered_q32: u32::MAX / 2,
            flux_magnitude_mwb: Fixed::ZERO,
            confidence: Fixed::ZERO,
            bemf_q_filtered_v: Fixed::ZERO,
            valid_travel_q32: 0,
            invalid_cycles: 0,
            ready_latched: false,
            resistance_ohms,
            inductance_millihenries,
            flux_linkage_mwb,
            inverse_flux_linkage: Fixed::from_bits(inverse_flux_bits),
            bemf_volts_per_erpm_q24: if bemf_volts_per_erpm_q24 > i32::MAX as i64 {
                i32::MAX
            } else {
                bemf_volts_per_erpm_q24 as i32
            },
            control_frequency_hz,
            // V * 1000 / Hz is mWb accumulated per control cycle.
            flux_voltage_step: Fixed::from_bits(if flux_voltage_step > i32::MAX as u64 {
                i32::MAX
            } else {
                flux_voltage_step as i32
            }),
            // OxiFOC defaults: nonlinear centering gain = 500 / s.
            centering_step: Fixed::from_bits(if centering_step > i32::MAX as u64 {
                i32::MAX
            } else {
                centering_step as i32
            }),
            // Back-EMF validity proxy time constant = 25 ms.
            bemf_filter_step: Fixed::from_bits(if bemf_filter_step > i32::MAX as u64 {
                i32::MAX
            } else {
                bemf_filter_step as i32
            }),
            // OxiFOC PLL defaults: kp = 1000 / s, ki = 20000 / s^2.
            pll_kp_step_q32: ratio_q32(1_000, frequency),
            pll_ki_step_q32: ratio_q32(20_000, frequency.saturating_mul(frequency)),
            // Ten-millisecond phase-error readiness filter.
            phase_error_filter_step_q32: ratio_q32(100, frequency),
            // A granted validity state tolerates 400 ms of disagreement.
            valid_revoke_cycles: control_frequency_hz.saturating_mul(2) / 5,
        }
    }

    pub const fn phase(&self) -> Turns {
        self.phase_pll
    }

    pub const fn phase_raw(&self) -> Turns {
        self.phase_raw
    }

    pub fn electrical_rpm(&self) -> i32 {
        let cycles_per_minute = u64::from(self.control_frequency_hz).saturating_mul(60);
        ((i64::from(self.velocity_step_q32) * cycles_per_minute as i64) >> 32)
            .clamp(i64::from(i32::MIN), i64::from(i32::MAX)) as i32
    }

    pub const fn confidence(&self) -> Fixed {
        self.confidence
    }

    pub const fn flux_magnitude_mwb(&self) -> Fixed {
        self.flux_magnitude_mwb
    }

    pub const fn bemf_q_filtered_v(&self) -> Fixed {
        self.bemf_q_filtered_v
    }

    pub const fn phase_error_filtered_q32(&self) -> u32 {
        self.phase_error_filtered_q32
    }

    /// External-validity progress, where 255 represents the two-revolution
    /// corroboration threshold.
    pub const fn validity_progress(&self) -> u8 {
        let progress = self.valid_travel_q32 >> 25;
        if progress > u8::MAX as u64 {
            u8::MAX
        } else {
            progress as u8
        }
    }

    pub const fn is_ready(&self) -> bool {
        self.ready_latched
    }

    /// Seed phase and velocity from a trustworthy physical sensor.
    ///
    /// As in OxiFOC's floating-point observer, a trusted seed also grants the
    /// two-revolution external-validity credit. Live BEMF disagreement still
    /// revokes that credit after the normal 400 ms hold interval.
    pub fn seed(&mut self, phase: Turns, electrical_rpm: i32) {
        let (sin, cos) = CordicSinCos::sin_cos(phase);
        self.flux_alpha_mwb = self.flux_linkage_mwb * cos;
        self.flux_beta_mwb = self.flux_linkage_mwb * sin;
        self.current_alpha_last_a = Fixed::ZERO;
        self.current_beta_last_a = Fixed::ZERO;
        self.phase_raw = phase;
        self.phase_pll = phase;
        self.velocity_step_q32 = erpm_to_step(electrical_rpm, self.control_frequency_hz);
        self.phase_error_filtered_q32 = 0;
        self.flux_magnitude_mwb = self.flux_linkage_mwb;
        self.confidence = Fixed::ONE;
        self.bemf_q_filtered_v =
            expected_bemf_voltage(self.bemf_volts_per_erpm_q24, electrical_rpm);
        self.valid_travel_q32 = READY_VALID_TRAVEL_Q32;
        self.invalid_cycles = 0;
        self.ready_latched = self.readiness_met(false, electrical_rpm);
    }

    pub fn reset(&mut self) {
        self.flux_alpha_mwb = Fixed::ZERO;
        self.flux_beta_mwb = Fixed::ZERO;
        self.current_alpha_last_a = Fixed::ZERO;
        self.current_beta_last_a = Fixed::ZERO;
        self.phase_raw = 0;
        self.phase_pll = 0;
        self.velocity_step_q32 = 0;
        self.phase_error_filtered_q32 = u32::MAX / 2;
        self.flux_magnitude_mwb = Fixed::ZERO;
        self.confidence = Fixed::ZERO;
        self.bemf_q_filtered_v = Fixed::ZERO;
        self.valid_travel_q32 = 0;
        self.invalid_cycles = 0;
        self.ready_latched = false;
    }

    /// Integrate one causally paired voltage/current sample.
    #[inline(never)]
    pub fn update(&mut self, input: &PhaseInput) {
        // MXLEMMING active-flux form:
        //   psi += (v - R*i) * dt - L * delta_i
        // These physical inputs are bounded by the ADC and PWM envelopes.
        // Operating on their raw Q16.16 words avoids emitting a saturation
        // tree around every multiply on the M3; the integrator state and
        // model outputs retain explicit clamps at their physical bounds.
        let current_alpha = input.measured_current.alpha.to_bits();
        let current_beta = input.measured_current.beta.to_bits();
        let resistance = self.resistance_ohms.to_bits();
        let resistive_alpha_v = multiply_q16(resistance, current_alpha);
        let resistive_beta_v = multiply_q16(resistance, current_beta);
        let current_delta_alpha = current_alpha.wrapping_sub(self.current_alpha_last_a.to_bits());
        let current_delta_beta = current_beta.wrapping_sub(self.current_beta_last_a.to_bits());
        let e_alpha = input
            .applied_voltage
            .alpha
            .to_bits()
            .wrapping_sub(resistive_alpha_v);
        let e_beta = input
            .applied_voltage
            .beta
            .to_bits()
            .wrapping_sub(resistive_beta_v);
        let voltage_step = self.flux_voltage_step.to_bits();
        let inductance = self.inductance_millihenries.to_bits();
        let flux_alpha = i64::from(self.flux_alpha_mwb.to_bits())
            + i64::from(multiply_q16(e_alpha, voltage_step))
            - i64::from(multiply_q16(inductance, current_delta_alpha));
        let flux_beta = i64::from(self.flux_beta_mwb.to_bits())
            + i64::from(multiply_q16(e_beta, voltage_step))
            - i64::from(multiply_q16(inductance, current_delta_beta));
        self.flux_alpha_mwb = Fixed::from_bits(saturating_i64_to_i32(flux_alpha));
        self.flux_beta_mwb = Fixed::from_bits(saturating_i64_to_i32(flux_beta));
        self.current_alpha_last_a = Fixed::from_bits(current_alpha);
        self.current_beta_last_a = Fixed::from_bits(current_beta);

        // The readiness magnitude does not need an exact square root. This
        // max-plus-3/8-min norm is bounded, monotonic, and cheap enough for a
        // Cortex-M3; phase itself still comes from both exact vector axes.
        let raw_magnitude =
            approximate_magnitude_bits(self.flux_alpha_mwb.to_bits(), self.flux_beta_mwb.to_bits());
        self.flux_magnitude_mwb = Fixed::from_bits(raw_magnitude);
        self.confidence = Fixed::from_bits(
            multiply_q16(raw_magnitude, self.inverse_flux_linkage.to_bits())
                .clamp(0, Fixed::ONE.to_bits()),
        );

        // OxiFOC's one-sided radial centering drains only flux outside the
        // configured lambda circle. Component clamps remain the hardstop.
        let flux_limit = self.flux_linkage_mwb.to_bits();
        if raw_magnitude > flux_limit {
            let normalized = multiply_q16(raw_magnitude, self.inverse_flux_linkage.to_bits());
            let error = Fixed::ONE
                .to_bits()
                .wrapping_sub(multiply_q16(normalized, normalized));
            let pull = multiply_q16(error, self.centering_step.to_bits())
                .clamp(Fixed::ratio(-1, 2).to_bits(), 0);
            self.flux_alpha_mwb = Fixed::from_bits(
                self.flux_alpha_mwb
                    .to_bits()
                    .wrapping_add(multiply_q16(self.flux_alpha_mwb.to_bits(), pull)),
            );
            self.flux_beta_mwb = Fixed::from_bits(
                self.flux_beta_mwb
                    .to_bits()
                    .wrapping_add(multiply_q16(self.flux_beta_mwb.to_bits(), pull)),
            );
        }
        self.flux_alpha_mwb =
            Fixed::from_bits(clamp_symmetric(self.flux_alpha_mwb.to_bits(), flux_limit));
        self.flux_beta_mwb =
            Fixed::from_bits(clamp_symmetric(self.flux_beta_mwb.to_bits(), flux_limit));

        self.phase_raw = atan2_turns(self.flux_beta_mwb.to_bits(), self.flux_alpha_mwb.to_bits());
        let phase_error = signed_angle_difference(self.phase_raw, self.phase_pll);
        self.velocity_step_q32 = self
            .velocity_step_q32
            .saturating_add(multiply_q32(phase_error, self.pll_ki_step_q32));
        let proportional_step = multiply_q32(phase_error, self.pll_kp_step_q32);
        self.phase_pll = self
            .phase_pll
            .wrapping_add(self.velocity_step_q32.saturating_add(proportional_step) as u32);
        self.phase_error_filtered_q32 = ema_u32(
            self.phase_error_filtered_q32,
            phase_error.unsigned_abs().min(i32::MAX as u32),
            self.phase_error_filter_step_q32,
        );

        // External validity: project e = v - R*i onto the estimated q axis.
        // Normalize by the measured pre-correction magnitude, as OxiFOC does,
        // so a flux-linkage configuration error is not counted once in the
        // flux vector and a second time in this projection. The lambda/10
        // floor bounds amplification while the integrator is building.
        let cross = multiply_q16(e_beta, self.flux_alpha_mwb.to_bits())
            .wrapping_sub(multiply_q16(e_alpha, self.flux_beta_mwb.to_bits()));
        let bemf_q = divide_q16(cross, raw_magnitude.max(flux_limit / 10));
        let bemf_step = multiply_q16(
            bemf_q.wrapping_sub(self.bemf_q_filtered_v.to_bits()),
            self.bemf_filter_step.to_bits(),
        );
        self.bemf_q_filtered_v =
            Fixed::from_bits(self.bemf_q_filtered_v.to_bits().wrapping_add(bemf_step));
        let electrical_rpm = self.electrical_rpm();
        self.update_validity(electrical_rpm);

        let was_ready = self.ready_latched;
        self.ready_latched = self.readiness_met(was_ready, electrical_rpm);
    }

    fn update_validity(&mut self, erpm: i32) {
        let expected = expected_bemf_voltage(self.bemf_volts_per_erpm_q24, erpm);
        let measured_bits = i64::from(self.bemf_q_filtered_v.to_bits());
        let expected_bits = i64::from(expected.to_bits());
        let measured_abs = measured_bits.unsigned_abs() as i64;
        let expected_abs = expected_bits.unsigned_abs() as i64;
        let same_direction =
            measured_bits != 0 && expected_bits != 0 && (measured_bits > 0) == (expected_bits > 0);
        let above_minimum = measured_abs * VALID_BEMF_RATIO_MIN_DENOMINATOR
            >= expected_abs * VALID_BEMF_RATIO_MIN_NUMERATOR;
        let below_maximum = measured_abs * VALID_BEMF_RATIO_MAX_DENOMINATOR
            <= expected_abs * VALID_BEMF_RATIO_MAX_NUMERATOR;
        let corroborated = erpm.unsigned_abs() >= READY_MIN_ERPM as u32
            && same_direction
            && above_minimum
            && below_maximum;
        let validity_granted = self.valid_travel_q32 >= READY_VALID_TRAVEL_Q32;
        if corroborated {
            self.invalid_cycles = 0;
            self.valid_travel_q32 = self
                .valid_travel_q32
                .saturating_add(u64::from(self.velocity_step_q32.unsigned_abs()))
                .min(READY_VALID_TRAVEL_Q32);
        } else if validity_granted {
            self.invalid_cycles = self.invalid_cycles.saturating_add(1);
            if self.invalid_cycles >= self.valid_revoke_cycles {
                self.valid_travel_q32 = 0;
                self.invalid_cycles = 0;
            }
        } else {
            self.valid_travel_q32 = 0;
            self.invalid_cycles = 0;
        }
    }

    fn readiness_met(&self, holding: bool, electrical_rpm: i32) -> bool {
        let error_limit = if holding {
            READY_HOLD_MAX_ERROR_Q32
        } else {
            READY_ACQUIRE_MAX_ERROR_Q32
        };
        self.confidence >= READY_MIN_CONFIDENCE
            && self.phase_error_filtered_q32 < error_limit
            && electrical_rpm.unsigned_abs() >= READY_MIN_ERPM as u32
            && self.valid_travel_q32 >= READY_VALID_TRAVEL_Q32
    }
}

pub const fn signed_angle_difference(angle: Turns, reference: Turns) -> i32 {
    angle.wrapping_sub(reference) as i32
}

fn erpm_to_step(electrical_rpm: i32, control_frequency_hz: u32) -> i32 {
    let cycles_per_minute = control_frequency_hz.saturating_mul(60).max(1);
    let step_per_erpm_q32 = reciprocal_q32_rounded(cycles_per_minute);
    (i64::from(electrical_rpm) * i64::from(step_per_erpm_q32))
        .clamp(i64::from(i32::MIN), i64::from(i32::MAX)) as i32
}

/// Approximate `2^32 / denominator` with a rounded integer reciprocal.
/// Expressing the seed conversion as a reciprocal multiply keeps its error
/// bounded while using only the Cortex-M3's native 32-bit division.
fn reciprocal_q32_rounded(denominator: u32) -> u32 {
    if denominator <= 1 {
        return u32::MAX;
    }
    let quotient = u32::MAX / denominator;
    let remainder = (u32::MAX % denominator) + 1;
    quotient.saturating_add(u32::from(remainder >= denominator - remainder))
}

fn expected_bemf_voltage(volts_per_erpm_q24: i32, electrical_rpm: i32) -> Fixed {
    Fixed::from_bits(saturating_i64_to_i32(
        (i64::from(volts_per_erpm_q24) * i64::from(electrical_rpm)) >> 8,
    ))
}

fn approximate_magnitude_bits(alpha: i32, beta: i32) -> i32 {
    let alpha = alpha.unsigned_abs();
    let beta = beta.unsigned_abs();
    let maximum = alpha.max(beta);
    let minimum = alpha.min(beta);
    maximum
        .saturating_add(minimum.saturating_mul(3) >> 3)
        .min(i32::MAX as u32) as i32
}

#[inline(always)]
fn multiply_q16(left: i32, right: i32) -> i32 {
    ((i64::from(left) * i64::from(right)) >> 16) as i32
}

fn multiply_q32(value: i32, coefficient_q32: u32) -> i32 {
    ((i64::from(value) * i64::from(coefficient_q32)) >> 32) as i32
}

/// Divide two Q16.16 values and return their Q16.16 ratio without a 64-bit
/// divide. Scaling both operands to at most 15 significant bits keeps the
/// shifted numerator in `i32` and maps to the Cortex-M3's bounded `sdiv`.
fn divide_q16(numerator: i32, denominator: i32) -> i32 {
    if denominator <= 0 {
        return 0;
    }
    let largest = numerator.unsigned_abs().max(denominator as u32);
    let bit_length = 32 - largest.leading_zeros();
    let shift = bit_length.saturating_sub(15);
    let scaled_numerator = numerator >> shift;
    let scaled_denominator = (denominator >> shift).max(1);
    scaled_numerator.saturating_mul(1 << 16) / scaled_denominator
}

fn ema_u32(previous: u32, sample: u32, coefficient_q32: u32) -> u32 {
    let difference = i64::from(sample) - i64::from(previous);
    (i64::from(previous) + ((difference * i64::from(coefficient_q32)) >> 32)) as u32
}

fn saturating_i64_to_i32(value: i64) -> i32 {
    if value > i64::from(i32::MAX) {
        i32::MAX
    } else if value < i64::from(i32::MIN) {
        i32::MIN
    } else {
        value as i32
    }
}

fn clamp_symmetric(value: i32, limit: i32) -> i32 {
    if value > limit {
        limit
    } else if value < limit.saturating_neg() {
        limit.saturating_neg()
    } else {
        value
    }
}

const fn ratio_q32(numerator: u64, denominator: u64) -> u32 {
    if denominator == 0 {
        return u32::MAX;
    }
    let value = ((numerator as u128) << 32) / denominator as u128;
    if value > u32::MAX as u128 {
        u32::MAX
    } else {
        value as u32
    }
}

/// VESC-style polynomial atan2, returning Q0.32 electrical turns.
fn atan2_turns(y: i32, x: i32) -> Turns {
    if x == 0 && y == 0 {
        return 0;
    }
    const A_Q32: i64 = 134_183_864;
    const B_Q32: i64 = -671_056_031;
    const EIGHTH_TURN_Q32: i64 = 1_i64 << 29;
    let absolute_y = y.saturating_abs();
    let (numerator, denominator, base) = if x >= 0 {
        (
            x.saturating_sub(absolute_y),
            x.saturating_add(absolute_y),
            EIGHTH_TURN_Q32,
        )
    } else {
        (
            x.saturating_add(absolute_y),
            absolute_y.saturating_sub(x),
            3 * EIGHTH_TURN_Q32,
        )
    };
    let ratio = ratio_q15(numerator, denominator);
    let ratio_squared = (i64::from(ratio) * i64::from(ratio)) >> 15;
    let polynomial = ((A_Q32 * ratio_squared) >> 15) + B_Q32;
    let angle = base + ((polynomial * i64::from(ratio)) >> 15);
    if y < 0 {
        (0_u32).wrapping_sub(angle as u32)
    } else {
        angle as u32
    }
}

fn ratio_q15(numerator: i32, denominator: i32) -> i32 {
    if denominator <= 0 {
        return 0;
    }
    let largest = numerator.unsigned_abs().max(denominator as u32);
    let bit_length = 32 - largest.leading_zeros();
    let shift = bit_length.saturating_sub(15);
    let scaled_numerator = numerator >> shift;
    let scaled_denominator = (denominator >> shift).max(1);
    scaled_numerator.saturating_mul(1 << 15) / scaled_denominator
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::foc::AlphaBeta;

    fn fixed(value: f32) -> Fixed {
        Fixed::from_bits((value * 65_536.0) as i32)
    }

    fn turns(value: f32) -> Turns {
        (f64::from(value.rem_euclid(1.0)) * 4_294_967_296.0_f64) as u32
    }

    fn turn_error(left: Turns, right: Turns) -> f32 {
        (f64::from(signed_angle_difference(left, right)) / 4_294_967_296.0_f64) as f32
    }

    #[test]
    fn polynomial_atan2_tracks_the_unit_circle() {
        let mut maximum_error = 0.0_f32;
        for index in 0..4096 {
            let angle = index as f32 / 4096.0 * core::f32::consts::TAU;
            let actual = atan2_turns(fixed(angle.sin()).to_bits(), fixed(angle.cos()).to_bits());
            maximum_error =
                maximum_error.max(turn_error(actual, turns(angle / core::f32::consts::TAU)).abs());
        }
        assert!(
            maximum_error < 0.002,
            "atan2 error was {maximum_error} turns"
        );
    }

    fn run_ideal_observer(electrical_rpm: i32) -> BackEmfObserver {
        const FREQUENCY: u32 = 16_000;
        let resistance = fixed(0.0884);
        let inductance_millihenries = fixed(0.039);
        let flux_mwb = fixed(12.2);
        let mut observer =
            BackEmfObserver::new(resistance, inductance_millihenries, flux_mwb, FREQUENCY);
        let mut phase_turns = 0.1_f32;
        observer.seed(turns(phase_turns), electrical_rpm);
        let turns_per_cycle = electrical_rpm as f32 / (60.0 * FREQUENCY as f32);
        let omega = electrical_rpm as f32 * core::f32::consts::TAU / 60.0;
        let lambda_wb = 0.0122_f32;
        for _ in 0..32_000 {
            let angle = phase_turns * core::f32::consts::TAU;
            observer.update(&PhaseInput::new(
                AlphaBeta {
                    alpha: fixed(-omega * lambda_wb * angle.sin()),
                    beta: fixed(omega * lambda_wb * angle.cos()),
                },
                AlphaBeta::default(),
                62_500,
            ));
            phase_turns = (phase_turns + turns_per_cycle).rem_euclid(1.0);
        }
        assert!(
            turn_error(observer.phase(), turns(phase_turns)).abs() < 0.04,
            "phase error={} turns",
            turn_error(observer.phase(), turns(phase_turns)),
        );
        observer
    }

    #[test]
    fn seeded_observer_tracks_forward_rotation() {
        for electrical_rpm in [3_000, 6_000, 20_000, 40_000] {
            let observer = run_ideal_observer(electrical_rpm);
            assert!((observer.electrical_rpm() - electrical_rpm).unsigned_abs() < 150);
            assert!(observer.is_ready());
        }
    }

    #[test]
    fn seeded_observer_tracks_reverse_rotation() {
        for electrical_rpm in [-3_000, -6_000, -20_000, -40_000] {
            let observer = run_ideal_observer(electrical_rpm);
            assert!((observer.electrical_rpm() - electrical_rpm).unsigned_abs() < 150);
            assert!(observer.is_ready());
        }
    }

    #[test]
    fn unexcited_observer_never_becomes_ready() {
        let mut observer = BackEmfObserver::new(fixed(0.0884), fixed(0.039), fixed(12.2), 16_000);
        for _ in 0..10_000 {
            observer.update(&PhaseInput::default());
        }
        assert!(!observer.is_ready());
        assert_eq!(observer.confidence(), Fixed::ZERO);
    }

    fn run_loaded_observer(electrical_rpm: i32, final_q_current_a: f32) -> BackEmfObserver {
        const FREQUENCY: u32 = 16_000;
        const DT: f32 = 1.0 / FREQUENCY as f32;
        const RESISTANCE_OHMS: f32 = 0.0884;
        const INDUCTANCE_HENRIES: f32 = 0.000_039;
        const FLUX_LINKAGE_WEBERS: f32 = 0.0122;

        let mut observer = BackEmfObserver::new(
            fixed(RESISTANCE_OHMS),
            fixed(INDUCTANCE_HENRIES * 1_000.0),
            fixed(FLUX_LINKAGE_WEBERS * 1_000.0),
            FREQUENCY,
        );
        let mut phase_turns = 0.1_f32;
        observer.seed(turns(phase_turns), electrical_rpm);
        let turns_per_cycle = electrical_rpm as f32 / (60.0 * FREQUENCY as f32);
        let mut current_alpha = 0.0_f32;
        let mut current_beta = 0.0_f32;
        let angle = phase_turns * core::f32::consts::TAU;
        let mut flux_alpha = FLUX_LINKAGE_WEBERS * angle.cos();
        let mut flux_beta = FLUX_LINKAGE_WEBERS * angle.sin();

        for cycle in 0..32_000 {
            phase_turns = (phase_turns + turns_per_cycle).rem_euclid(1.0);
            let angle = phase_turns * core::f32::consts::TAU;
            let current_ramp = (cycle as f32 / 100.0).min(1.0);
            let q_current = final_q_current_a * current_ramp;
            let next_current_alpha = -q_current * angle.sin();
            let next_current_beta = q_current * angle.cos();
            let next_flux_alpha = FLUX_LINKAGE_WEBERS * angle.cos();
            let next_flux_beta = FLUX_LINKAGE_WEBERS * angle.sin();
            let voltage_alpha = RESISTANCE_OHMS * next_current_alpha
                + (next_flux_alpha - flux_alpha) / DT
                + INDUCTANCE_HENRIES * (next_current_alpha - current_alpha) / DT;
            let voltage_beta = RESISTANCE_OHMS * next_current_beta
                + (next_flux_beta - flux_beta) / DT
                + INDUCTANCE_HENRIES * (next_current_beta - current_beta) / DT;

            observer.update(&PhaseInput::new(
                AlphaBeta {
                    alpha: fixed(voltage_alpha),
                    beta: fixed(voltage_beta),
                },
                AlphaBeta {
                    alpha: fixed(next_current_alpha),
                    beta: fixed(next_current_beta),
                },
                62_500,
            ));
            current_alpha = next_current_alpha;
            current_beta = next_current_beta;
            flux_alpha = next_flux_alpha;
            flux_beta = next_flux_beta;
        }

        assert!(
            turn_error(observer.phase(), turns(phase_turns)).abs() < 0.04,
            "loaded phase error={} turns",
            turn_error(observer.phase(), turns(phase_turns)),
        );
        observer
    }

    #[test]
    fn motor_model_cancels_resistive_and_inductive_drop_under_load() {
        for electrical_rpm in [3_000, -3_000] {
            let observer = run_loaded_observer(electrical_rpm, 60.0);
            assert!(
                (observer.electrical_rpm() - electrical_rpm).unsigned_abs() < 150,
                "observer speed={} expected={electrical_rpm}",
                observer.electrical_rpm(),
            );
            assert!(observer.is_ready());
        }
    }

    #[test]
    fn q16_division_preserves_sign_and_fraction() {
        for (numerator, denominator) in [(3.8, 12.2), (-3.8, 12.2), (48.0, 12.2)] {
            let actual = divide_q16(fixed(numerator).to_bits(), fixed(denominator).to_bits());
            let expected = fixed(numerator / denominator).to_bits();
            assert!((actual - expected).unsigned_abs() < 256);
        }
        assert_eq!(divide_q16(Fixed::ONE.to_bits(), 0), 0);
    }

    #[test]
    fn rounded_erpm_reciprocal_preserves_seed_speed() {
        const FREQUENCY: u32 = 16_000;
        assert_eq!(reciprocal_q32_rounded(FREQUENCY * 60), 4_474);

        for electrical_rpm in -76_000..=76_000 {
            let velocity_step = erpm_to_step(electrical_rpm, FREQUENCY);
            let recovered = ((i64::from(velocity_step) * i64::from(FREQUENCY * 60)) >> 32) as i32;
            assert!(
                (recovered - electrical_rpm).unsigned_abs() <= 2,
                "requested={electrical_rpm} recovered={recovered}",
            );
        }
    }
}
