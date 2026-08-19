//! Current-command, supply-current, and measured-overcurrent limits.

use super::{Dq, Fixed, Scalar};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CurrentLimitResult<N: Scalar = Fixed> {
    pub target: Dq<N>,
    pub quadrature_limit: N,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CurrentLimits<N: Scalar = Fixed> {
    pub max_current: N,
    pub overcurrent_threshold: N,
    pub bus_in_max: Option<N>,
    pub bus_regen_max: Option<N>,
}

impl<N: Scalar> CurrentLimits<N> {
    pub const fn new(
        max_current: N,
        overcurrent_threshold: N,
        bus_in_max: Option<N>,
        bus_regen_max: Option<N>,
    ) -> Self {
        Self {
            max_current,
            overcurrent_threshold,
            bus_in_max,
            bus_regen_max,
        }
    }

    pub fn clamp_targets(&self, target: Dq<N>) -> Dq<N> {
        if self.max_current <= N::ZERO {
            return target;
        }
        let direct = target.d.clamp(-self.max_current, self.max_current);
        let quadrature_limit = if direct == N::ZERO {
            self.max_current
        } else {
            N::circular_remaining(self.max_current, direct)
        };
        Dq::new(direct, target.q.clamp(-quadrature_limit, quadrature_limit))
    }

    pub fn is_overcurrent(&self, measured: Dq<N>) -> bool {
        if self.overcurrent_threshold <= N::ZERO {
            return false;
        }
        N::magnitude_exceeds(measured.d, measured.q, self.overcurrent_threshold)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CurrentLimiter {
    limits: CurrentLimits<Fixed>,
    filtered_q_voltage_ticks: Fixed,
    pwm_period_ticks: u16,
    modulation_filter_shift: u8,
}

impl CurrentLimiter {
    pub const fn new(
        limits: CurrentLimits<Fixed>,
        pwm_period_ticks: u16,
        modulation_filter_shift: u8,
    ) -> Self {
        Self {
            limits,
            filtered_q_voltage_ticks: Fixed::ZERO,
            pwm_period_ticks,
            modulation_filter_shift,
        }
    }

    pub const fn limits(&self) -> &CurrentLimits<Fixed> {
        &self.limits
    }

    pub fn set_bus_limits(&mut self, input: Option<Fixed>, regen: Option<Fixed>) {
        self.limits.bus_in_max = input;
        self.limits.bus_regen_max = regen;
    }

    pub fn reset(&mut self) {
        self.filtered_q_voltage_ticks = Fixed::ZERO;
    }

    pub fn note_applied_voltage(&mut self, applied: Dq<Fixed>) {
        let previous = self.filtered_q_voltage_ticks.to_bits();
        let sample = applied.q.to_bits();
        let shift = self.modulation_filter_shift.min(30);
        let difference = sample.saturating_sub(previous);
        let adjustment = if difference < 0 {
            -difference.saturating_neg().wrapping_shr(u32::from(shift))
        } else {
            difference.wrapping_shr(u32::from(shift))
        };
        let next = previous.saturating_add(adjustment);
        self.filtered_q_voltage_ticks = Fixed::from_bits(next);
    }

    pub fn filtered_q_modulation(&self) -> Fixed {
        if self.pwm_period_ticks == 0 {
            return Fixed::ZERO;
        }
        let numerator = i64::from(self.filtered_q_voltage_ticks.to_bits()).saturating_mul(3);
        let denominator = i64::from(self.pwm_period_ticks).saturating_mul(2);
        Fixed::from_bits(
            (numerator / denominator).clamp(i64::from(i32::MIN), i64::from(i32::MAX)) as i32,
        )
    }

    pub fn clamp_targets(&self, target: Dq<Fixed>) -> Dq<Fixed> {
        self.clamp_targets_with_limit(target).target
    }

    pub fn clamp_targets_with_limit(&self, target: Dq<Fixed>) -> CurrentLimitResult<Fixed> {
        let target = self.limits.clamp_targets(target);
        let static_quadrature_limit = if target.d == Fixed::ZERO {
            self.limits.max_current
        } else {
            Fixed::circular_remaining(self.limits.max_current, target.d)
        };
        let filtered_voltage_bits = self.filtered_q_voltage_ticks.to_bits();
        let deadband_bits = u32::from(self.pwm_period_ticks)
            .saturating_mul(2)
            .saturating_mul(1_u32 << 16)
            / 1_000;
        if filtered_voltage_bits.unsigned_abs().saturating_mul(3) < deadband_bits
            || target.q == Fixed::ZERO
        {
            return CurrentLimitResult {
                target,
                quadrature_limit: static_quadrature_limit,
            };
        }

        let motoring = (target.q > Fixed::ZERO) == (filtered_voltage_bits > 0);
        let bus_limit = if motoring {
            self.limits.bus_in_max
        } else {
            self.limits.bus_regen_max
        };
        let Some(bus_limit) = bus_limit else {
            return CurrentLimitResult {
                target,
                quadrature_limit: static_quadrature_limit,
            };
        };
        let quadrature_limit = core::cmp::min(
            phase_current_limit(
                bus_limit,
                self.filtered_q_voltage_ticks.abs_ceil_u32(),
                self.pwm_period_ticks,
            ),
            static_quadrature_limit,
        );
        CurrentLimitResult {
            target: Dq::new(
                target.d,
                Scalar::clamp(target.q, -quadrature_limit, quadrature_limit),
            ),
            quadrature_limit,
        }
    }

    pub fn quadrature_limit_counts(&self, direction: i8) -> u16 {
        let requested = if direction < 0 {
            -self.limits.max_current
        } else {
            self.limits.max_current
        };
        self.clamp_targets_with_limit(Dq::new(Fixed::ZERO, requested))
            .quadrature_limit
            .abs_ceil_u32()
            .min(u32::from(u16::MAX)) as u16
    }

    pub fn is_overcurrent(&self, measured: Dq<Fixed>) -> bool {
        self.limits.is_overcurrent(measured)
    }
}

fn phase_current_limit(bus_limit: Fixed, q_voltage_ticks: u32, pwm_period_ticks: u16) -> Fixed {
    let bus_counts = bus_limit.trunc_to_i32().max(0) as u32;
    if q_voltage_ticks == 0 {
        return Fixed::from_integer(i32::MAX);
    }
    let phase_counts = bus_counts
        .saturating_mul(u32::from(pwm_period_ticks))
        .saturating_mul(2)
        .checked_div(q_voltage_ticks.saturating_mul(3))
        .unwrap_or(u32::MAX)
        .min(i32::MAX as u32);
    Fixed::from_integer(phase_counts as i32)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn limits() -> CurrentLimits<Fixed> {
        CurrentLimits::new(
            Fixed::from_integer(838),
            Fixed::from_integer(1_344),
            Some(Fixed::from_integer(480)),
            Some(Fixed::ZERO),
        )
    }

    #[test]
    fn current_targets_use_a_direct_priority_circle() {
        let limits = CurrentLimits::new(
            Fixed::from_integer(100),
            Fixed::from_integer(130),
            None,
            None,
        );
        assert_eq!(
            limits.clamp_targets(Dq::new(Fixed::from_integer(60), Fixed::from_integer(100),)),
            Dq::new(Fixed::from_integer(60), Fixed::from_integer(80)),
        );
    }

    #[test]
    fn measured_overcurrent_uses_dq_magnitude() {
        let limits = CurrentLimits::new(
            Fixed::from_integer(100),
            Fixed::from_integer(130),
            None,
            None,
        );
        assert!(
            !limits.is_overcurrent(Dq::new(Fixed::from_integer(50), Fixed::from_integer(120),))
        );
        assert!(limits.is_overcurrent(Dq::new(Fixed::from_integer(50), Fixed::from_integer(121),)));
    }

    #[test]
    fn bus_limit_scales_with_q_axis_modulation_without_speed_input() {
        let mut limiter = CurrentLimiter::new(limits(), 2_250, 0);
        limiter.note_applied_voltage(Dq::new(Fixed::ZERO, Fixed::from_integer(-1_125)));
        assert_eq!(limiter.filtered_q_modulation(), Fixed::ratio(-3, 4));
        assert_eq!(limiter.quadrature_limit_counts(-1), 640);
        assert_eq!(
            limiter.clamp_targets(Dq::new(Fixed::ZERO, Fixed::from_integer(-838),)),
            Dq::new(Fixed::ZERO, Fixed::from_integer(-640)),
        );
    }

    #[test]
    fn zero_regen_limit_rejects_opposing_torque() {
        let mut limiter = CurrentLimiter::new(limits(), 2_250, 0);
        limiter.note_applied_voltage(Dq::new(Fixed::ZERO, Fixed::from_integer(-1_125)));
        assert_eq!(
            limiter.clamp_targets(Dq::new(Fixed::ZERO, Fixed::from_integer(100),)),
            Dq::new(Fixed::ZERO, Fixed::ZERO),
        );
    }

    #[test]
    fn bus_limits_are_symmetric_across_rotation_direction() {
        let limits = CurrentLimits::new(
            Fixed::from_integer(100),
            Fixed::from_integer(130),
            Some(Fixed::from_integer(10)),
            Some(Fixed::ZERO),
        );
        let mut limiter = CurrentLimiter::new(limits, 2_250, 0);
        limiter.note_applied_voltage(Dq::new(Fixed::ZERO, Fixed::from_integer(750)));
        assert_eq!(
            limiter
                .clamp_targets(Dq::new(Fixed::ZERO, Fixed::from_integer(30)))
                .q,
            Fixed::from_integer(20),
        );
        assert_eq!(
            limiter
                .clamp_targets(Dq::new(Fixed::ZERO, Fixed::from_integer(-5)))
                .q,
            Fixed::ZERO,
        );

        limiter.note_applied_voltage(Dq::new(Fixed::ZERO, Fixed::from_integer(-750)));
        assert_eq!(
            limiter
                .clamp_targets(Dq::new(Fixed::ZERO, Fixed::from_integer(-30)))
                .q,
            Fixed::from_integer(-20),
        );
        assert_eq!(
            limiter
                .clamp_targets(Dq::new(Fixed::ZERO, Fixed::from_integer(5)))
                .q,
            Fixed::ZERO,
        );
    }

    #[test]
    fn modulation_filter_uses_the_configured_control_period_ratio() {
        let mut limiter = CurrentLimiter::new(limits(), 2_250, 5);
        for _ in 0..32 {
            limiter.note_applied_voltage(Dq::new(Fixed::ZERO, Fixed::from_integer(-1_125)));
        }
        let modulation = limiter.filtered_q_modulation();
        assert!(modulation < Fixed::ratio(-47, 100));
        assert!(modulation > Fixed::ratio(-49, 100));
    }
}
