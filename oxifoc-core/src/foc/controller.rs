//! Synchronous fixed-point field-oriented current controller.

use core::marker::PhantomData;

use super::control_types::{AlphaBeta, Dq, PwmDuty};
use super::numeric::{Fixed, Scalar};
use super::pi_controller::PIController;
use super::svpwm::{SvpwmTickModulator, TickModulator};
use super::transforms::{clarke, inverse_park, park};
use super::trig::{CordicSinCos, SinCos};

pub struct FocController<
    N: Scalar = Fixed,
    T: SinCos<N> = CordicSinCos,
    M: TickModulator<N> = SvpwmTickModulator,
> {
    direct: PIController<N>,
    quadrature: PIController<N>,
    vector_limit_ticks: N,
    phase_limit_ticks: u16,
    applied_voltage: Dq<N>,
    voltage_limited: bool,
    actuation_advance: N,
    backend: PhantomData<(T, M)>,
}

impl<N, T, M> FocController<N, T, M>
where
    N: Scalar,
    T: SinCos<N>,
    M: TickModulator<N>,
{
    pub const fn new(
        direct: PIController<N>,
        quadrature: PIController<N>,
        vector_limit_ticks: N,
        phase_limit_ticks: u16,
    ) -> Self {
        Self {
            direct,
            quadrature,
            vector_limit_ticks,
            phase_limit_ticks,
            applied_voltage: Dq::new(N::ZERO, N::ZERO),
            voltage_limited: false,
            actuation_advance: N::ZERO,
            backend: PhantomData,
        }
    }

    pub fn reset(&mut self) {
        self.direct.reset();
        self.quadrature.reset();
        self.applied_voltage = Dq::new(N::ZERO, N::ZERO);
        self.voltage_limited = false;
    }

    pub const fn applied_voltage(&self) -> Dq<N> {
        self.applied_voltage
    }

    pub const fn voltage_limited(&self) -> bool {
        self.voltage_limited
    }

    /// Set the electrical angle traversed between the current sample and the
    /// PWM command taking effect. The correction is applied only to the
    /// output voltage frame; the sampled current remains transformed at the
    /// measurement angle.
    pub fn set_actuation_advance(&mut self, advance: N) {
        self.actuation_advance = advance.clamp(-N::HALF, N::HALF);
    }

    pub const fn actuation_advance(&self) -> N {
        self.actuation_advance
    }

    pub fn step(
        &mut self,
        phase_a: N,
        phase_b: N,
        electrical_angle: T::Angle,
        target: Dq<N>,
        pwm_neutral: u16,
    ) -> (Dq<N>, PwmDuty) {
        self.step_with_injection(
            phase_a,
            phase_b,
            electrical_angle,
            target,
            Dq::new(N::ZERO, N::ZERO),
            pwm_neutral,
        )
    }

    pub fn step_with_injection(
        &mut self,
        phase_a: N,
        phase_b: N,
        electrical_angle: T::Angle,
        target: Dq<N>,
        voltage_injection: Dq<N>,
        pwm_neutral: u16,
    ) -> (Dq<N>, PwmDuty) {
        let (sin, cos) = T::sin_cos(electrical_angle);
        let (alpha, beta) = clarke(phase_a, phase_b);
        let (measured_d, measured_q) = park(alpha, beta, sin, cos);
        let measured = Dq::new(measured_d, measured_q);
        let direct_update = self.direct.prepare_update(target.d, measured.d);
        let quadrature_update = self.quadrature.prepare_update(target.q, measured.q);
        let requested = Dq::new(
            direct_update.raw_output + voltage_injection.d,
            quadrature_update.raw_output + voltage_injection.q,
        );
        let (applied_d, applied_q, voltage_scale) =
            N::limit_vector(requested.d, requested.q, self.vector_limit_ticks);
        let applied = Dq::new(applied_d, applied_q);
        self.applied_voltage = applied;
        self.voltage_limited = voltage_scale != N::ONE;
        self.direct.apply_back_calculation(
            direct_update,
            direct_update.raw_output * (voltage_scale - N::ONE),
        );
        self.quadrature.apply_back_calculation(
            quadrature_update,
            quadrature_update.raw_output * (voltage_scale - N::ONE),
        );
        let (voltage_alpha, voltage_beta) = inverse_park(applied.d, applied.q, sin, cos);
        let (voltage_alpha, voltage_beta) =
            self.apply_actuation_advance(voltage_alpha, voltage_beta);
        let duties = M::to_duties(
            AlphaBeta {
                alpha: voltage_alpha,
                beta: voltage_beta,
            },
            pwm_neutral,
            self.phase_limit_ticks,
        );
        (measured, duties)
    }

    #[inline]
    fn apply_actuation_advance(&self, alpha: N, beta: N) -> (N, N) {
        let advance = self.actuation_advance;
        if advance == N::ZERO {
            return (alpha, beta);
        }
        let advance_squared = advance * advance;
        let sin = advance - advance_squared * advance * N::from_ratio(1, 6);
        let cos = N::ONE - advance_squared * N::HALF;
        (alpha * cos - beta * sin, alpha * sin + beta * cos)
    }
}

pub fn limit_voltage<N: Scalar>(voltage: Dq<N>, limit_ticks: N) -> Dq<N> {
    let (direct, quadrature, _) = N::limit_vector(voltage.d, voltage.q, limit_ticks);
    Dq::new(direct, quadrature)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::foc::trig::Turns;

    fn fixed_controller() -> FocController {
        let pi = PIController::new(Fixed::ratio(512, 1_024), Fixed::ratio(605, 16_384));
        FocController::new(pi, pi, Fixed::from_integer(1_250), 1_085)
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

    #[cfg(feature = "algorithms")]
    #[test]
    fn fixed_and_float_execute_the_same_complete_control_path() {
        use crate::foc::trig::LibmSinCos;

        let mut fixed = fixed_controller();
        let float_pi = PIController::new(0.5_f32, 605.0 / 16_384.0);
        let mut floating =
            FocController::<f32, LibmSinCos>::new(float_pi, float_pi, 1_250.0, 1_085);
        let (fixed_current, fixed_duty) = fixed.step(
            Fixed::from_integer(240),
            Fixed::from_integer(-173),
            0x2000_0000,
            Dq::new(Fixed::ZERO, Fixed::from_integer(-320)),
            1_125,
        );
        let (float_current, float_duty) = floating.step(
            240.0,
            -173.0,
            core::f32::consts::FRAC_PI_4,
            Dq::new(0.0, -320.0),
            1_125,
        );
        assert!((fixed_current.d.integer() as f32 - float_current.d).abs() <= 2.0);
        assert!((fixed_current.q.integer() as f32 - float_current.q).abs() <= 2.0);
        for (fixed_compare, float_compare) in
            fixed_duty.as_array().into_iter().zip(float_duty.as_array())
        {
            assert!(fixed_compare.abs_diff(float_compare) <= 3);
        }
    }
}
