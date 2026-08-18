//! Compact current-loop controller shared by fixed and floating-point builds.

use core::marker::PhantomData;

use super::angle::{AngleSinCos, CordicTurns};
use super::control_types::{AlphaBeta, Dq, PwmDuty};
use super::discrete_pi::Pi;
use super::numeric::{Fixed, Scalar};
use super::svpwm::{SvpwmTickModulator, TickModulator};
use super::transforms::{clarke, inverse_park, park};

pub struct FocKernel<
    N: Scalar = Fixed,
    T: AngleSinCos<N> = CordicTurns,
    M: TickModulator<N> = SvpwmTickModulator,
> {
    direct: Pi<N>,
    quadrature: Pi<N>,
    vector_limit_ticks: N,
    phase_limit_ticks: u16,
    applied_voltage: Dq<N>,
    voltage_limited: bool,
    backend: PhantomData<(T, M)>,
}

impl<N, T, M> FocKernel<N, T, M>
where
    N: Scalar,
    T: AngleSinCos<N>,
    M: TickModulator<N>,
{
    pub const fn new(
        direct: Pi<N>,
        quadrature: Pi<N>,
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

    pub fn phase_current_limit_from_dc(
        &self,
        dc_current_limit_counts: u16,
        maximum_phase_counts: u16,
        pwm_period_ticks: u16,
    ) -> u16 {
        let direct = self.applied_voltage.d.abs_ceil_u32();
        let quadrature = self.applied_voltage.q.abs_ceil_u32();
        let maximum = direct.max(quadrature);
        let minimum = direct.min(quadrature);
        let magnitude = maximum.saturating_add(minimum.saturating_mul(3) >> 3);
        if magnitude == 0 {
            return maximum_phase_counts;
        }
        u32::from(dc_current_limit_counts)
            .saturating_mul(u32::from(pwm_period_ticks))
            .saturating_mul(2)
            .checked_div(magnitude.saturating_mul(3))
            .unwrap_or(u32::MAX)
            .clamp(1, u32::from(maximum_phase_counts)) as u16
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
        let applied = limit_voltage_direct_priority(requested, self.vector_limit_ticks);
        self.applied_voltage = applied;
        self.voltage_limited = applied != requested;
        self.direct
            .apply_back_calculation(direct_update, applied.d - requested.d);
        self.quadrature
            .apply_back_calculation(quadrature_update, applied.q - requested.q);
        let (voltage_alpha, voltage_beta) = inverse_park(applied.d, applied.q, sin, cos);
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
}

pub fn limit_voltage_direct_priority<N: Scalar>(voltage: Dq<N>, limit_ticks: N) -> Dq<N> {
    let limit = limit_ticks.max(N::ZERO);
    let direct = voltage.d.clamp(-limit, limit);
    let quadrature_limit = N::circular_remaining(limit, direct);
    Dq::new(direct, voltage.q.clamp(-quadrature_limit, quadrature_limit))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::foc::angle::Turns;

    fn fixed_kernel() -> FocKernel {
        let pi = Pi::new(Fixed::ratio(512, 1_024), Fixed::ratio(605, 16_384));
        FocKernel::new(pi, pi, Fixed::from_integer(1_250), 1_085)
    }

    #[test]
    fn vector_limit_is_direct_priority_and_circular() {
        assert_eq!(
            limit_voltage_direct_priority(
                Dq::new(Fixed::from_integer(1_000), Fixed::from_integer(1_000)),
                Fixed::from_integer(1_250),
            ),
            Dq::new(Fixed::from_integer(1_000), Fixed::from_integer(750)),
        );
        assert_eq!(
            limit_voltage_direct_priority(
                Dq::new(Fixed::from_integer(2_000), Fixed::from_integer(50)),
                Fixed::from_integer(1_250),
            ),
            Dq::new(Fixed::from_integer(1_250), Fixed::ZERO),
        );
    }

    #[test]
    fn kernel_reports_and_resets_voltage_limiting() {
        let mut kernel = fixed_kernel();
        let _ = kernel.step(
            Fixed::ZERO,
            Fixed::ZERO,
            0,
            Dq::new(Fixed::from_integer(10_000), Fixed::ZERO),
            1_125,
        );
        assert!(kernel.voltage_limited());
        kernel.reset();
        assert!(!kernel.voltage_limited());
        assert_eq!(kernel.applied_voltage(), Dq::new(Fixed::ZERO, Fixed::ZERO));
    }

    #[test]
    fn fixed_kernel_back_calculation_tracks_the_applied_vector() {
        let mut kernel = fixed_kernel();
        for _ in 0..100 {
            let (_, duty) = kernel.step(
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
        let (_, duty) = kernel.step(
            Fixed::ZERO,
            Fixed::ZERO,
            0,
            Dq::new(Fixed::from_integer(-100), Fixed::ZERO),
            1_125,
        );
        assert!(duty.a < 1_125);
    }

    #[test]
    fn dc_projection_is_bounded_by_the_phase_envelope() {
        let mut kernel = fixed_kernel();
        assert_eq!(kernel.phase_current_limit_from_dc(400, 838, 2_250), 838);
        let _ = kernel.step(
            Fixed::ZERO,
            Fixed::ZERO,
            0,
            Dq::new(Fixed::ZERO, Fixed::from_integer(-10_000)),
            1_125,
        );
        assert_eq!(kernel.phase_current_limit_from_dc(250, 838, 2_250), 300);
        assert_eq!(kernel.phase_current_limit_from_dc(400, 838, 2_250), 480);
    }

    #[cfg(feature = "algorithms")]
    #[test]
    fn fixed_and_float_execute_the_same_complete_control_path() {
        use crate::foc::angle::LibmRadians;

        let mut fixed = fixed_kernel();
        let float_pi = Pi::new(0.5_f32, 605.0 / 16_384.0);
        let mut floating = FocKernel::<f32, LibmRadians>::new(float_pi, float_pi, 1_250.0, 1_085);
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
