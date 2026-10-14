//! Synchronous fixed-point field-oriented current controller.

use core::marker::PhantomData;

use super::control_types::{AlphaBeta, Dq, PwmDuty};
use super::numeric::{Fixed, saturating_i64_to_i32};
use super::pi_controller::PIController;
use super::svpwm::space_vector_pwm_ticks;
use super::transforms::{clarke, inverse_park, park};
use super::trig::{Turns, sin_cos};

/// Compile-time motor model used for reference-current dq decoupling and
/// permanent-magnet back-EMF feedforward.
pub trait DecouplingModel {
    fn feedforward(electrical_rpm: i32, target: Dq, volts_per_pwm_tick: Fixed) -> Dq;
}

#[derive(Clone, Copy, Debug, Default)]
pub struct NoDecoupling;

impl DecouplingModel for NoDecoupling {
    #[inline]
    fn feedforward(_electrical_rpm: i32, _target: Dq, _volts_per_pwm_tick: Fixed) -> Dq {
        Dq::default()
    }
}

/// Fixed-point motor model. Inductance parameters are pre-combined with the
/// current-sense scale as Q16.16 mWb per ADC count; flux is Q16.16 mWb.
pub struct FixedDecoupling<
    const LD_FLUX_PER_COUNT_BITS: i32,
    const LQ_FLUX_PER_COUNT_BITS: i32,
    const FLUX_LINKAGE_MWB_BITS: i32,
>;

impl<
    const LD_FLUX_PER_COUNT_BITS: i32,
    const LQ_FLUX_PER_COUNT_BITS: i32,
    const FLUX_LINKAGE_MWB_BITS: i32,
> DecouplingModel
    for FixedDecoupling<LD_FLUX_PER_COUNT_BITS, LQ_FLUX_PER_COUNT_BITS, FLUX_LINKAGE_MWB_BITS>
{
    #[inline]
    fn feedforward(electrical_rpm: i32, target: Dq, volts_per_pwm_tick: Fixed) -> Dq {
        let volts_per_tick_bits = volts_per_pwm_tick.to_bits();
        // 512 Q16.16 V/tick corresponds to 17.6 V across a 2,250-tick PWM
        // period, well below this controller's undervoltage cutoff. It also
        // bounds every whole-tick feedforward result inside Q16.16.
        if electrical_rpm == 0 || volts_per_tick_bits < 512 {
            return Dq::default();
        }

        // The motor driver constrains current references before this point;
        // the phase source likewise supplies a plausibility-limited speed.
        let direct_counts = target.d.to_bits() >> 16;
        let quadrature_counts = target.q.to_bits() >> 16;

        // 6863 is 2*pi/60 in Q16.16. Combining it at compile time with each
        // Q16.16 mWb motor parameter yields a Q8 tick numerator. Feedforward
        // is resolved to whole timer ticks because that is the actuator's
        // finest output; the current references used by this loop are whole
        // ADC counts as well.
        let ld_scale_q8 =
            ((6_863_i64 * i64::from(LD_FLUX_PER_COUNT_BITS) + 128_000) / 256_000) as i32;
        let lq_scale_q8 =
            ((6_863_i64 * i64::from(LQ_FLUX_PER_COUNT_BITS) + 128_000) / 256_000) as i32;
        let flux_scale_q8 =
            ((6_863_i64 * i64::from(FLUX_LINKAGE_MWB_BITS) + 128_000) / 256_000) as i32;
        let speed = i64::from(electrical_rpm);
        let inductive_ticks = |counts: i32, scale_q8: i32| {
            feedforward_ticks(
                (speed * i64::from(counts)).saturating_mul(i64::from(scale_q8)),
                volts_per_tick_bits,
            )
        };
        let direct_ticks = -inductive_ticks(quadrature_counts, lq_scale_q8);
        let quadrature_ticks = inductive_ticks(direct_counts, ld_scale_q8)
            + feedforward_ticks(speed * i64::from(flux_scale_q8), volts_per_tick_bits);
        Dq::new(
            Fixed::from_integer(saturating_i64_to_i32(direct_ticks)),
            Fixed::from_integer(saturating_i64_to_i32(quadrature_ticks)),
        )
    }
}

/// Truncates a Q8 speed-flux product to whole PWM ticks, matching
/// `product / volts_per_tick_bits / 256` without a 64-bit divide.
///
/// Truncating the Q8 shift on the magnitude first is exact for integer
/// division. A shifted magnitude beyond `u32` is reported as `u32::MAX`
/// ticks; for every scale up to 2 V per tick (2^17 Q16.16 bits) the true
/// result already exceeds the Q16.16 whole-tick range, so the caller's
/// saturation is unchanged.
#[inline]
fn feedforward_ticks(product_q8: i64, volts_per_tick_bits: i32) -> i64 {
    let magnitude = product_q8.unsigned_abs() >> 8;
    let ticks = match u32::try_from(magnitude) {
        Ok(magnitude) => magnitude / volts_per_tick_bits as u32,
        Err(_) => u32::MAX,
    };
    if product_q8 < 0 {
        -i64::from(ticks)
    } else {
        i64::from(ticks)
    }
}

/// Synchronous dq current controller.
///
/// `DEAD_TIME_NUMERATOR / DEAD_TIME_DENOMINATOR` is the dead-time
/// compensation in phase-voltage ticks. An enabled ratio with a zero
/// denominator is rejected when the controller is built:
///
/// ```compile_fail
/// use oxifoc_core::foc::{Fixed, FocController, PIController};
///
/// let pi = PIController::new(Fixed::ZERO, Fixed::ZERO);
/// let _ = FocController::<9, 0>::new(pi, pi, Fixed::from_integer(1_250), 1_085);
/// ```
pub struct FocController<
    const DEAD_TIME_NUMERATOR: i32 = 0,
    const DEAD_TIME_DENOMINATOR: i32 = 1,
    D: DecouplingModel = NoDecoupling,
> {
    direct: PIController,
    quadrature: PIController,
    vector_limit_ticks: Fixed,
    phase_limit_ticks: u16,
    requested_voltage: Dq,
    feedforward_voltage: Dq,
    applied_voltage: Dq,
    applied_stationary: AlphaBeta,
    measured_stationary: AlphaBeta,
    voltage_limited: bool,
    actuation_advance: Fixed,
    decoupling: PhantomData<D>,
}

impl<const DEAD_TIME_NUMERATOR: i32, const DEAD_TIME_DENOMINATOR: i32, D: DecouplingModel>
    FocController<DEAD_TIME_NUMERATOR, DEAD_TIME_DENOMINATOR, D>
{
    const DEAD_TIME_RATIO_VALID: () = assert!(
        DEAD_TIME_NUMERATOR == 0 || DEAD_TIME_DENOMINATOR != 0,
        "enabled dead-time compensation requires a nonzero denominator"
    );

    pub const fn new(
        direct: PIController,
        quadrature: PIController,
        vector_limit_ticks: Fixed,
        phase_limit_ticks: u16,
    ) -> Self {
        let () = Self::DEAD_TIME_RATIO_VALID;
        Self {
            direct,
            quadrature,
            vector_limit_ticks,
            phase_limit_ticks,
            requested_voltage: Dq::new(Fixed::ZERO, Fixed::ZERO),
            feedforward_voltage: Dq::new(Fixed::ZERO, Fixed::ZERO),
            applied_voltage: Dq::new(Fixed::ZERO, Fixed::ZERO),
            applied_stationary: AlphaBeta {
                alpha: Fixed::ZERO,
                beta: Fixed::ZERO,
            },
            measured_stationary: AlphaBeta {
                alpha: Fixed::ZERO,
                beta: Fixed::ZERO,
            },
            voltage_limited: false,
            actuation_advance: Fixed::ZERO,
            decoupling: PhantomData,
        }
    }

    pub fn reset(&mut self) {
        self.direct.reset();
        self.quadrature.reset();
        self.requested_voltage = Dq::new(Fixed::ZERO, Fixed::ZERO);
        self.feedforward_voltage = Dq::new(Fixed::ZERO, Fixed::ZERO);
        self.applied_voltage = Dq::new(Fixed::ZERO, Fixed::ZERO);
        self.applied_stationary = AlphaBeta {
            alpha: Fixed::ZERO,
            beta: Fixed::ZERO,
        };
        self.measured_stationary = AlphaBeta {
            alpha: Fixed::ZERO,
            beta: Fixed::ZERO,
        };
        self.voltage_limited = false;
        self.actuation_advance = Fixed::ZERO;
    }

    pub const fn applied_voltage(&self) -> Dq {
        self.applied_voltage
    }

    pub const fn requested_voltage(&self) -> Dq {
        self.requested_voltage
    }

    pub const fn feedforward_voltage(&self) -> Dq {
        self.feedforward_voltage
    }

    pub const fn applied_stationary(&self) -> AlphaBeta {
        self.applied_stationary
    }

    pub const fn measured_stationary(&self) -> AlphaBeta {
        self.measured_stationary
    }

    pub const fn voltage_limited(&self) -> bool {
        self.voltage_limited
    }

    /// OxiFOC's `t_dead * f_pwm` factor after conversion to the modulator's
    /// phase-voltage tick domain. A zero numerator disables compensation.
    /// Both operands are compile-time constants and the denominator is
    /// checked when the controller is built, so this folds to a constant.
    pub fn dead_time_comp_ticks(&self) -> Fixed {
        if DEAD_TIME_NUMERATOR == 0 {
            Fixed::ZERO
        } else {
            Fixed::ratio(DEAD_TIME_NUMERATOR, DEAD_TIME_DENOMINATOR)
        }
    }

    /// Set the electrical angle traversed between the current sample and the
    /// PWM command taking effect. The correction is applied only to the
    /// output voltage frame; the sampled current remains transformed at the
    /// measurement angle.
    pub fn set_actuation_advance(&mut self, advance: Fixed) {
        self.actuation_advance = advance.clamp(-Fixed::HALF, Fixed::HALF);
    }

    pub const fn actuation_advance(&self) -> Fixed {
        self.actuation_advance
    }

    pub fn step(
        &mut self,
        phase_a: Fixed,
        phase_b: Fixed,
        electrical_angle: Turns,
        target: Dq,
        pwm_neutral: u16,
    ) -> (Dq, PwmDuty) {
        self.step_with_velocity(
            phase_a,
            phase_b,
            electrical_angle,
            0,
            target,
            Fixed::ONE,
            pwm_neutral,
        )
    }

    /// Apply a dq voltage command directly while retaining the controller's
    /// transforms, circular voltage limit, dead-time compensation, and SVPWM.
    /// Detection sequences use this after a PI-regulated rotor lock to create
    /// precisely timed voltage pulses without advancing either PI integrator.
    pub fn step_direct_voltage(
        &mut self,
        phase_a: Fixed,
        phase_b: Fixed,
        electrical_angle: Turns,
        voltage: Dq,
        pwm_neutral: u16,
    ) -> (Dq, PwmDuty) {
        let (sin, cos) = sin_cos(electrical_angle);
        let (alpha, beta) = clarke(phase_a, phase_b);
        self.measured_stationary = AlphaBeta { alpha, beta };
        let (measured_d, measured_q) = park(alpha, beta, sin, cos);
        let measured = Dq::new(measured_d, measured_q);
        self.requested_voltage = voltage;
        self.feedforward_voltage = Dq::new(Fixed::ZERO, Fixed::ZERO);
        let (applied_d, applied_q, voltage_scale) =
            Fixed::limit_vector(voltage.d, voltage.q, self.vector_limit_ticks);
        let applied = Dq::new(applied_d, applied_q);
        self.applied_voltage = applied;
        self.voltage_limited = voltage_scale != Fixed::ONE;
        let (voltage_alpha, voltage_beta) = inverse_park(applied.d, applied.q, sin, cos);
        let (voltage_alpha, voltage_beta) =
            self.apply_actuation_advance(voltage_alpha, voltage_beta);
        self.applied_stationary = AlphaBeta {
            alpha: voltage_alpha,
            beta: voltage_beta,
        };
        let (modulated_alpha, modulated_beta) =
            self.apply_dead_time_comp(voltage_alpha, voltage_beta, phase_a, phase_b);
        let duties = space_vector_pwm_ticks(
            AlphaBeta {
                alpha: modulated_alpha,
                beta: modulated_beta,
            },
            pwm_neutral,
            self.phase_limit_ticks,
        );
        (measured, duties)
    }

    #[allow(clippy::too_many_arguments)]
    pub fn step_with_velocity(
        &mut self,
        phase_a: Fixed,
        phase_b: Fixed,
        electrical_angle: Turns,
        electrical_rpm: i32,
        target: Dq,
        volts_per_pwm_tick: Fixed,
        pwm_neutral: u16,
    ) -> (Dq, PwmDuty) {
        let (sin, cos) = sin_cos(electrical_angle);
        let (alpha, beta) = clarke(phase_a, phase_b);
        self.measured_stationary = AlphaBeta { alpha, beta };
        let (measured_d, measured_q) = park(alpha, beta, sin, cos);
        let measured = Dq::new(measured_d, measured_q);
        let direct_update = self.direct.prepare_update(target.d, measured.d);
        let quadrature_update = self.quadrature.prepare_update(target.q, measured.q);
        let feedforward = D::feedforward(electrical_rpm, target, volts_per_pwm_tick);
        let requested = Dq::new(
            direct_update.raw_output + feedforward.d,
            quadrature_update.raw_output + feedforward.q,
        );
        self.requested_voltage = requested;
        self.feedforward_voltage = feedforward;
        let (applied_d, applied_q, voltage_scale) =
            Fixed::limit_vector(requested.d, requested.q, self.vector_limit_ticks);
        let applied = Dq::new(applied_d, applied_q);
        self.applied_voltage = applied;
        self.voltage_limited = voltage_scale != Fixed::ONE;
        self.direct.apply_back_calculation(
            direct_update,
            direct_update.raw_output * (voltage_scale - Fixed::ONE),
        );
        self.quadrature.apply_back_calculation(
            quadrature_update,
            quadrature_update.raw_output * (voltage_scale - Fixed::ONE),
        );
        let (voltage_alpha, voltage_beta) = inverse_park(applied.d, applied.q, sin, cos);
        let (voltage_alpha, voltage_beta) =
            self.apply_actuation_advance(voltage_alpha, voltage_beta);
        self.applied_stationary = AlphaBeta {
            alpha: voltage_alpha,
            beta: voltage_beta,
        };
        let (modulated_alpha, modulated_beta) =
            self.apply_dead_time_comp(voltage_alpha, voltage_beta, phase_a, phase_b);
        let duties = space_vector_pwm_ticks(
            AlphaBeta {
                alpha: modulated_alpha,
                beta: modulated_beta,
            },
            pwm_neutral,
            self.phase_limit_ticks,
        );
        (measured, duties)
    }

    #[inline]
    fn apply_actuation_advance(&self, alpha: Fixed, beta: Fixed) -> (Fixed, Fixed) {
        let advance = self.actuation_advance;
        if advance == Fixed::ZERO {
            return (alpha, beta);
        }
        let advance_squared = advance * advance;
        let sin = advance - advance_squared * advance * Fixed::ratio(1, 6);
        let cos = Fixed::ONE - advance_squared * Fixed::HALF;
        (alpha * cos - beta * sin, alpha * sin + beta * cos)
    }

    #[inline]
    fn apply_dead_time_comp(
        &self,
        alpha: Fixed,
        beta: Fixed,
        phase_a: Fixed,
        phase_b: Fixed,
    ) -> (Fixed, Fixed) {
        let factor = self.dead_time_comp_ticks();
        if factor == Fixed::ZERO {
            return (alpha, beta);
        }

        // These are the original OxiFOC inverse-Clarke phase-current signs.
        // Using the two measured phases directly avoids reconstructing values
        // the Clarke transform received earlier in this same control step.
        let phase_c = -(phase_a + phase_b);
        let signs = (u8::from(phase_a >= Fixed::ZERO) << 2)
            | (u8::from(phase_b >= Fixed::ZERO) << 1)
            | u8::from(phase_c >= Fixed::ZERO);
        let alpha_step = factor * Fixed::TWO_THIRDS;
        let beta_step = factor * Fixed::TWO_INV_SQRT_3;
        let (comp_alpha, comp_beta) = match signs {
            4 => (alpha_step + alpha_step, Fixed::ZERO),
            6 => (alpha_step, beta_step),
            2 => (-alpha_step, beta_step),
            3 => (-(alpha_step + alpha_step), Fixed::ZERO),
            1 => (-alpha_step, -beta_step),
            5 => (alpha_step, -beta_step),
            _ => (Fixed::ZERO, Fixed::ZERO),
        };
        (alpha + comp_alpha, beta + comp_beta)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::foc::trig::Turns;

    fn fixed_controller() -> FocController {
        let pi = PIController::new(Fixed::ratio(512, 1_024), Fixed::ratio(605, 16_384));
        FocController::new(pi, pi, Fixed::from_integer(1_250), 1_085)
    }

    fn limit_voltage(voltage: Dq, limit_ticks: Fixed) -> Dq {
        let (direct, quadrature, _) = Fixed::limit_vector(voltage.d, voltage.q, limit_ticks);
        Dq::new(direct, quadrature)
    }

    #[test]
    fn vector_limit_is_circular_and_preserves_direction() {
        let diagonal = limit_voltage(
            Dq::new(Fixed::from_integer(1_000), Fixed::from_integer(1_000)),
            Fixed::from_integer(1_250),
        );
        assert!((diagonal.d - diagonal.q).abs_ceil_u32() <= 1);
        assert!((882..=884).contains(&diagonal.d.integer()));
        assert!(!Fixed::magnitude_exceeds(
            diagonal.d,
            diagonal.q,
            Fixed::from_integer(1_250),
        ));

        let shallow = limit_voltage(
            Dq::new(Fixed::from_integer(2_000), Fixed::from_integer(50)),
            Fixed::from_integer(1_250),
        );
        let cross_product_error = (i64::from(shallow.d.to_bits()) * 50
            - i64::from(shallow.q.to_bits()) * 2_000)
            .unsigned_abs();
        assert!(cross_product_error <= 2_000);
        assert!((1_248..=1_250).contains(&shallow.d.integer()));
        assert!(!Fixed::magnitude_exceeds(
            shallow.d,
            shallow.q,
            Fixed::from_integer(1_250),
        ));
    }

    #[test]
    fn controller_reports_and_resets_voltage_limiting() {
        let mut controller = fixed_controller();
        let _ = controller.step(
            Fixed::ZERO,
            Fixed::ZERO,
            0,
            Dq::new(Fixed::from_integer(10_000), Fixed::ZERO),
            1_125,
        );
        assert!(controller.voltage_limited());
        controller.reset();
        assert!(!controller.voltage_limited());
        assert_eq!(
            controller.applied_voltage(),
            Dq::new(Fixed::ZERO, Fixed::ZERO)
        );
    }

    #[test]
    fn direct_voltage_uses_the_shared_limit_and_modulator_without_pi_output() {
        let mut controller = fixed_controller();
        let (measured, duties) = controller.step_direct_voltage(
            Fixed::from_integer(12),
            Fixed::from_integer(-4),
            0,
            Dq::new(Fixed::from_integer(2_000), Fixed::ZERO),
            1_125,
        );

        assert!((11..=12).contains(&measured.d.integer()));
        assert!(controller.voltage_limited());
        assert!((1_249..=1_250).contains(&controller.applied_voltage().d.integer()));
        assert_eq!(controller.feedforward_voltage(), Dq::default());
        assert!(duties.as_array().into_iter().all(|duty| duty <= 2_210));
    }

    #[test]
    fn fixed_controller_back_calculation_tracks_the_applied_vector() {
        let mut controller = fixed_controller();
        for _ in 0..100 {
            let (_, duty) = controller.step(
                Fixed::ZERO,
                Fixed::ZERO,
                0 as Turns,
                Dq::new(Fixed::from_integer(10_000), Fixed::ZERO),
                1_125,
            );
            assert!(duty.a.abs_diff(1_125) <= 1_085);
            assert!(duty.b.abs_diff(1_125) <= 1_085);
            assert!(duty.c.abs_diff(1_125) <= 1_085);
        }
        let (_, duty) = controller.step(
            Fixed::ZERO,
            Fixed::ZERO,
            0,
            Dq::new(Fixed::from_integer(-100), Fixed::ZERO),
            1_125,
        );
        assert!(duty.a < 1_125);
    }

    #[test]
    fn actuation_advance_rotates_only_the_output_voltage_frame() {
        let mut unadvanced = fixed_controller();
        let mut advanced = fixed_controller();
        advanced.set_actuation_advance(Fixed::ratio(1, 10));

        let input = (
            Fixed::from_integer(240),
            Fixed::from_integer(-173),
            0x2000_0000,
            Dq::new(Fixed::ZERO, Fixed::from_integer(-320)),
            1_125,
        );
        let (plain_current, plain_duty) =
            unadvanced.step(input.0, input.1, input.2, input.3, input.4);
        let (advanced_current, advanced_duty) =
            advanced.step(input.0, input.1, input.2, input.3, input.4);

        assert_eq!(advanced_current, plain_current);
        assert_ne!(advanced_duty, plain_duty);
        assert_eq!(advanced.actuation_advance(), Fixed::ratio(1, 10));
    }

    #[test]
    fn actuation_advance_is_bounded_to_the_small_angle_model() {
        let mut controller = fixed_controller();
        controller.set_actuation_advance(Fixed::from_integer(2));
        assert_eq!(controller.actuation_advance(), Fixed::ratio(1, 2));
        controller.set_actuation_advance(Fixed::from_integer(-2));
        assert_eq!(controller.actuation_advance(), Fixed::ratio(-1, 2));
    }

    #[test]
    fn reset_clears_the_actuation_advance() {
        let mut controller = fixed_controller();
        controller.set_actuation_advance(Fixed::ratio(1, 10));
        controller.reset();
        assert_eq!(controller.actuation_advance(), Fixed::ZERO);
    }

    #[test]
    fn dead_time_compensation_follows_all_six_phase_current_sign_regions() {
        let pi = PIController::new(Fixed::ZERO, Fixed::ZERO);
        let controller = FocController::<9, 1>::new(pi, pi, Fixed::from_integer(1_250), 1_085);
        let alpha_step = Fixed::from_integer(9) * Fixed::TWO_THIRDS;
        let beta_step = Fixed::from_integer(9) * Fixed::TWO_INV_SQRT_3;
        for (phase_a, phase_b, expected_alpha, expected_beta) in [
            (2, -1, alpha_step + alpha_step, Fixed::ZERO),
            (1, 1, alpha_step, beta_step),
            (-1, 2, -alpha_step, beta_step),
            (-2, 1, -(alpha_step + alpha_step), Fixed::ZERO),
            (-1, -1, -alpha_step, -beta_step),
            (1, -2, alpha_step, -beta_step),
        ] {
            let compensated = controller.apply_dead_time_comp(
                Fixed::ZERO,
                Fixed::ZERO,
                Fixed::from_integer(phase_a),
                Fixed::from_integer(phase_b),
            );
            assert_eq!(compensated, (expected_alpha, expected_beta));
        }
    }

    #[test]
    fn dead_time_compensation_changes_only_the_modulator_command() {
        let mut plain = fixed_controller();
        let pi = PIController::new(Fixed::ratio(512, 1_024), Fixed::ratio(605, 16_384));
        let mut compensated = FocController::<9, 1>::new(pi, pi, Fixed::from_integer(1_250), 1_085);
        let input = (
            Fixed::from_integer(100),
            Fixed::from_integer(-60),
            0,
            Dq::new(Fixed::ZERO, Fixed::ZERO),
            1_125,
        );
        let (plain_current, plain_duty) = plain.step(input.0, input.1, input.2, input.3, input.4);
        let (compensated_current, compensated_duty) =
            compensated.step(input.0, input.1, input.2, input.3, input.4);

        assert_eq!(compensated_current, plain_current);
        assert_eq!(compensated.applied_voltage(), plain.applied_voltage());
        assert_eq!(compensated.applied_stationary(), plain.applied_stationary());
        assert_ne!(compensated_duty, plain_duty);
        assert_eq!(compensated.dead_time_comp_ticks(), Fixed::from_integer(9));
    }

    #[test]
    fn fixed_decoupling_matches_the_reference_current_speed_voltage_terms() {
        type Motor = FixedDecoupling<409, 409, 799_539>;
        let volts_per_tick = Fixed::ratio(523, 22_500);
        let forward = Motor::feedforward(
            6_000,
            Dq::new(Fixed::ZERO, Fixed::from_integer(838)),
            volts_per_tick,
        );
        assert!((-143..=-140).contains(&forward.d.integer()));
        assert!((328..=331).contains(&forward.q.integer()));

        let reverse = Motor::feedforward(
            -6_000,
            Dq::new(Fixed::ZERO, Fixed::from_integer(-838)),
            volts_per_tick,
        );
        assert!((-143..=-140).contains(&reverse.d.integer()));
        assert!((-331..=-328).contains(&reverse.q.integer()));
    }

    #[test]
    fn fixed_decoupling_matches_a_wide_reference_beyond_i32_products() {
        const LD: i32 = 409;
        const LQ: i32 = 409;
        const FLUX: i32 = 799_539;
        type Motor = FixedDecoupling<LD, LQ, FLUX>;
        let scale_q8 = |bits: i32| (6_863_i128 * i128::from(bits) + 128_000) / 256_000;
        let ticks = |product: i128, volts_per_tick_bits: i32| {
            product / i128::from(volts_per_tick_bits) / 256
        };
        let volts_per_tick = Fixed::ratio(523, 22_500);
        let volts_bits = volts_per_tick.to_bits();
        for electrical_rpm in [-120_000, -101_000, 101_000, 120_000] {
            let target = Dq::new(Fixed::from_integer(-200), Fixed::from_integer(838));
            let output = Motor::feedforward(electrical_rpm, target, volts_per_tick);
            let speed = i128::from(electrical_rpm);
            let expected_d = -ticks(speed * 838 * scale_q8(LQ), volts_bits);
            let expected_q = ticks(speed * -200 * scale_q8(LD), volts_bits)
                + ticks(speed * scale_q8(FLUX), volts_bits);
            assert_eq!(i128::from(output.d.integer()), expected_d);
            assert_eq!(i128::from(output.q.integer()), expected_q);
        }
    }

    #[test]
    fn fixed_decoupling_saturates_at_extreme_speed() {
        type Motor = FixedDecoupling<409, 409, 799_539>;
        let output = Motor::feedforward(
            i32::MAX,
            Dq::new(Fixed::ZERO, Fixed::from_integer(30_000)),
            Fixed::from_bits(512),
        );
        assert_eq!(output.d, Fixed::from_bits(i32::MIN));
        assert_eq!(output.q, Fixed::from_bits(i32::MAX));

        let reverse = Motor::feedforward(
            i32::MIN,
            Dq::new(Fixed::ZERO, Fixed::from_integer(30_000)),
            Fixed::from_bits(512),
        );
        assert_eq!(reverse.d, Fixed::from_bits(i32::MAX));
        assert_eq!(reverse.q, Fixed::from_bits(i32::MIN));
    }

    #[test]
    fn no_decoupling_contributes_zero_voltage_at_speed() {
        let proportional = PIController::new(Fixed::ONE, Fixed::ZERO);
        let mut controller: FocController = FocController::new(
            proportional,
            proportional,
            Fixed::from_integer(1_273),
            1_103,
        );
        let _ = controller.step_with_velocity(
            Fixed::ZERO,
            Fixed::ZERO,
            0,
            -31_000,
            Dq::new(Fixed::from_integer(11), Fixed::from_integer(-17)),
            Fixed::ratio(523, 22_500),
            1_125,
        );

        assert_eq!(controller.feedforward_voltage(), Dq::default());
        assert_eq!(
            controller.requested_voltage(),
            Dq::new(Fixed::from_integer(11), Fixed::from_integer(-17))
        );
    }

    struct SaturatingFeedforward;

    impl DecouplingModel for SaturatingFeedforward {
        fn feedforward(_electrical_rpm: i32, _target: Dq, _volts_per_tick: Fixed) -> Dq {
            Dq::new(Fixed::from_integer(100), Fixed::ZERO)
        }
    }

    #[test]
    fn feedforward_is_inside_the_circle_but_outside_pi_anti_windup() {
        type Controller = FocController<0, 1, SaturatingFeedforward>;
        let pi = PIController::new(Fixed::ZERO, Fixed::ZERO);
        let mut controller = Controller::new(pi, pi, Fixed::from_integer(50), 100);
        let _ = controller.step_with_velocity(
            Fixed::ZERO,
            Fixed::ZERO,
            0,
            1_000,
            Dq::default(),
            Fixed::ONE,
            100,
        );

        assert!(controller.voltage_limited());
        assert_eq!(
            controller.requested_voltage(),
            Dq::new(Fixed::from_integer(100), Fixed::ZERO)
        );
        assert_eq!(
            controller.feedforward_voltage(),
            Dq::new(Fixed::from_integer(100), Fixed::ZERO)
        );
        assert_eq!(controller.applied_voltage().d.integer(), 50);
        assert_eq!(controller.direct.integral(), Fixed::ZERO);

        controller.reset();
        assert_eq!(controller.requested_voltage(), Dq::default());
        assert_eq!(controller.feedforward_voltage(), Dq::default());
    }
}
