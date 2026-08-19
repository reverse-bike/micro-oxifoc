//! Synchronous fixed-point FOC motor driver.
//!
//! The driver owns the current controller, selected phase provider, circular
//! current-command envelope, supply-current projection, and measured dq trip.
//! Platform code retains raw ADC sampling, PWM register writes, and immediate
//! hardware shutdown so those operations remain explicit at the ISR boundary.

use crate::foc::phase::{PhaseEstimate, PhaseInput, PhaseProvider};
use crate::foc::trig::Turns;
use crate::foc::{Dq, Fixed, FocController, PwmDuty, Scalar};

/// Current-command, supply-current, and measured-overcurrent limits.
///
/// The command is first constrained to the d-priority current circle. The
/// driver then constrains q current from filtered q-axis modulation so the
/// projected DC-side current remains inside the selected motoring or regen
/// bound. Finally, measured dq magnitude is checked against the hard trip.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CurrentLimits {
    /// Maximum commanded dq magnitude in current-sensor counts.
    pub max_current: Fixed,
    /// Hard measured dq trip magnitude in current-sensor counts.
    pub overcurrent_threshold: Fixed,
    /// Maximum projected battery discharge current; `None` is unlimited.
    pub bus_in_max: Option<Fixed>,
    /// Maximum projected battery charge current; `None` is unlimited.
    pub bus_regen_max: Option<Fixed>,
}

impl CurrentLimits {
    pub const fn new(
        max_current: Fixed,
        overcurrent_threshold: Fixed,
        bus_in_max: Option<Fixed>,
        bus_regen_max: Option<Fixed>,
    ) -> Self {
        Self {
            max_current,
            overcurrent_threshold,
            bus_in_max,
            bus_regen_max,
        }
    }

    pub fn clamp_targets(&self, target: Dq) -> Dq {
        if self.max_current <= Fixed::ZERO {
            return target;
        }
        let direct = Scalar::clamp(target.d, -self.max_current, self.max_current);
        let quadrature_limit = if direct == Fixed::ZERO {
            self.max_current
        } else {
            Fixed::circular_remaining(self.max_current, direct)
        };
        Dq::new(
            direct,
            Scalar::clamp(target.q, -quadrature_limit, quadrature_limit),
        )
    }

    pub fn is_overcurrent(&self, measured: Dq) -> bool {
        if self.overcurrent_threshold <= Fixed::ZERO {
            return false;
        }
        Fixed::magnitude_exceeds(measured.d, measured.q, self.overcurrent_threshold)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum StepError {
    /// Measured dq magnitude crossed the configured hard trip.
    Overcurrent,
}

/// Complete result of one current-control step.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct FocOutput {
    /// Final target after the current circle and supply-current clamp.
    pub target: Dq,
    /// Positive q-axis authority remaining after both clamps.
    pub quadrature_limit: Fixed,
    pub measured_current: Dq,
    pub applied_voltage: Dq,
    pub duties: PwmDuty,
    pub voltage_limited: bool,
}

/// Owns current regulation, current limits, and the selected phase provider.
pub struct FocDriver<Phase> {
    controller: FocController,
    phase: Phase,
    current_limits: CurrentLimits,
    bus_mod_q_filt_ticks: Fixed,
    pwm_period_ticks: u16,
    modulation_filter_shift: u8,
}

impl<Phase> FocDriver<Phase> {
    pub const fn new(
        controller: FocController,
        phase: Phase,
        current_limits: CurrentLimits,
        pwm_period_ticks: u16,
        modulation_filter_shift: u8,
    ) -> Self {
        Self {
            controller,
            phase,
            current_limits,
            bus_mod_q_filt_ticks: Fixed::ZERO,
            pwm_period_ticks,
            modulation_filter_shift,
        }
    }

    pub const fn phase(&self) -> &Phase {
        &self.phase
    }

    pub fn phase_mut(&mut self) -> &mut Phase {
        &mut self.phase
    }

    pub const fn current_limits(&self) -> &CurrentLimits {
        &self.current_limits
    }

    pub fn set_current_limits(&mut self, limits: CurrentLimits) {
        self.current_limits = limits;
    }

    pub fn set_bus_limits(&mut self, input: Option<Fixed>, regen: Option<Fixed>) {
        self.current_limits.bus_in_max = input;
        self.current_limits.bus_regen_max = regen;
    }

    pub fn set_actuation_advance(&mut self, advance: Fixed) {
        self.controller.set_actuation_advance(advance);
    }

    pub fn reset(&mut self) {
        self.controller.reset();
        self.bus_mod_q_filt_ticks = Fixed::ZERO;
    }

    pub fn filtered_q_modulation(&self) -> Fixed {
        if self.pwm_period_ticks == 0 {
            return Fixed::ZERO;
        }
        let numerator = i64::from(self.bus_mod_q_filt_ticks.to_bits()).saturating_mul(3);
        let denominator = i64::from(self.pwm_period_ticks).saturating_mul(2);
        Fixed::from_bits(
            (numerator / denominator).clamp(i64::from(i32::MIN), i64::from(i32::MAX)) as i32,
        )
    }

    fn note_applied_voltage(&mut self, applied: Dq) {
        let previous = self.bus_mod_q_filt_ticks.to_bits();
        let sample = applied.q.to_bits();
        let shift = self.modulation_filter_shift.min(30);
        let difference = sample.saturating_sub(previous);
        let adjustment = if difference < 0 {
            -difference.saturating_neg().wrapping_shr(u32::from(shift))
        } else {
            difference.wrapping_shr(u32::from(shift))
        };
        self.bus_mod_q_filt_ticks = Fixed::from_bits(previous.saturating_add(adjustment));
    }

    fn clamp_targets_with_limit(&self, target: Dq) -> (Dq, Fixed) {
        let target = self.current_limits.clamp_targets(target);
        let static_quadrature_limit = if target.d == Fixed::ZERO {
            self.current_limits.max_current
        } else {
            Fixed::circular_remaining(self.current_limits.max_current, target.d)
        };
        let filtered_voltage_bits = self.bus_mod_q_filt_ticks.to_bits();
        let deadband_bits = u32::from(self.pwm_period_ticks)
            .saturating_mul(2)
            .saturating_mul(1_u32 << 16)
            / 1_000;
        if filtered_voltage_bits.unsigned_abs().saturating_mul(3) < deadband_bits
            || target.q == Fixed::ZERO
        {
            return (target, static_quadrature_limit);
        }

        let motoring = (target.q > Fixed::ZERO) == (filtered_voltage_bits > 0);
        let bus_limit = if motoring {
            self.current_limits.bus_in_max
        } else {
            self.current_limits.bus_regen_max
        };
        let Some(bus_limit) = bus_limit else {
            return (target, static_quadrature_limit);
        };
        let quadrature_limit = core::cmp::min(
            phase_current_limit(
                bus_limit,
                self.bus_mod_q_filt_ticks.abs_ceil_u32(),
                self.pwm_period_ticks,
            ),
            static_quadrature_limit,
        );
        (
            Dq::new(
                target.d,
                Scalar::clamp(target.q, -quadrature_limit, quadrature_limit),
            ),
            quadrature_limit,
        )
    }
}

impl<Phase> FocDriver<Phase>
where
    Phase: PhaseProvider<Fixed, Angle = Turns>,
{
    pub fn estimate_for_control(
        &mut self,
        elapsed_since_observation_us: u32,
        control_period_ns: u32,
    ) -> Option<PhaseEstimate<Turns>> {
        self.phase
            .estimate_for_control(elapsed_since_observation_us, control_period_ns)
    }

    pub fn step_current_control(
        &mut self,
        phase_a: Fixed,
        phase_b: Fixed,
        electrical_angle: Turns,
        target: Dq,
        pwm_neutral: u16,
        control_period_ns: u32,
    ) -> Result<FocOutput, StepError> {
        let (target, quadrature_limit) = self.clamp_targets_with_limit(target);
        let injection = self.phase.injection();
        let (measured_current, duties) = self.controller.step_with_injection(
            phase_a,
            phase_b,
            electrical_angle,
            target,
            injection,
            pwm_neutral,
        );
        if self.current_limits.is_overcurrent(measured_current) {
            self.controller.reset();
            return Err(StepError::Overcurrent);
        }
        let applied_voltage = self.controller.applied_voltage();
        self.note_applied_voltage(applied_voltage);
        let voltage_limited = self.controller.voltage_limited();
        self.phase.update(&PhaseInput {
            applied_voltage,
            measured_current,
            electrical_angle,
            control_period_ns,
        });
        Ok(FocOutput {
            target,
            quadrature_limit,
            measured_current,
            applied_voltage,
            duties,
            voltage_limited,
        })
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
    use crate::foc::PIController;
    use crate::foc::phase::PhaseSource;

    #[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
    struct TestPhase;

    impl PhaseProvider for TestPhase {
        type Angle = Turns;

        fn source(&self) -> PhaseSource {
            PhaseSource::Hall
        }

        fn estimate(&self, _elapsed_since_observation_us: u32) -> Option<PhaseEstimate<Turns>> {
            Some(PhaseEstimate {
                angle: 0,
                electrical_rpm: 0,
                trustworthy: true,
            })
        }
    }

    fn limits() -> CurrentLimits {
        CurrentLimits::new(
            Fixed::from_integer(838),
            Fixed::from_integer(1_344),
            Some(Fixed::from_integer(480)),
            Some(Fixed::ZERO),
        )
    }

    fn driver(filter_shift: u8) -> FocDriver<TestPhase> {
        let pi = PIController::new(Fixed::ZERO, Fixed::ZERO);
        FocDriver::new(
            FocController::new(pi, pi, Fixed::from_integer(1_273), 1_103),
            TestPhase,
            limits(),
            2_250,
            filter_shift,
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
        let mut driver = driver(0);
        driver.note_applied_voltage(Dq::new(Fixed::ZERO, Fixed::from_integer(-1_125)));
        assert_eq!(driver.filtered_q_modulation(), Fixed::ratio(-3, 4));
        let (target, quadrature_limit) =
            driver.clamp_targets_with_limit(Dq::new(Fixed::ZERO, Fixed::from_integer(-838)));
        assert_eq!(quadrature_limit, Fixed::from_integer(640));
        assert_eq!(target.q, Fixed::from_integer(-640));
    }

    #[test]
    fn zero_regen_limit_rejects_opposing_torque() {
        let mut driver = driver(0);
        driver.note_applied_voltage(Dq::new(Fixed::ZERO, Fixed::from_integer(-1_125)));
        let (target, _) =
            driver.clamp_targets_with_limit(Dq::new(Fixed::ZERO, Fixed::from_integer(100)));
        assert_eq!(target.q, Fixed::ZERO);
    }

    #[test]
    fn bus_limits_are_symmetric_across_rotation_direction() {
        let limits = CurrentLimits::new(
            Fixed::from_integer(100),
            Fixed::from_integer(130),
            Some(Fixed::from_integer(10)),
            Some(Fixed::ZERO),
        );
        let pi = PIController::new(Fixed::ZERO, Fixed::ZERO);
        let mut driver = FocDriver::new(
            FocController::new(pi, pi, Fixed::from_integer(1_273), 1_103),
            TestPhase,
            limits,
            2_250,
            0,
        );
        driver.note_applied_voltage(Dq::new(Fixed::ZERO, Fixed::from_integer(750)));
        let (forward, _) =
            driver.clamp_targets_with_limit(Dq::new(Fixed::ZERO, Fixed::from_integer(30)));
        let (forward_regen, _) =
            driver.clamp_targets_with_limit(Dq::new(Fixed::ZERO, Fixed::from_integer(-5)));
        assert_eq!(forward.q, Fixed::from_integer(20));
        assert_eq!(forward_regen.q, Fixed::ZERO);

        driver.note_applied_voltage(Dq::new(Fixed::ZERO, Fixed::from_integer(-750)));
        let (reverse, _) =
            driver.clamp_targets_with_limit(Dq::new(Fixed::ZERO, Fixed::from_integer(-30)));
        let (reverse_regen, _) =
            driver.clamp_targets_with_limit(Dq::new(Fixed::ZERO, Fixed::from_integer(5)));
        assert_eq!(reverse.q, Fixed::from_integer(-20));
        assert_eq!(reverse_regen.q, Fixed::ZERO);
    }

    #[test]
    fn modulation_filter_uses_the_configured_control_period_ratio() {
        let mut driver = driver(5);
        for _ in 0..32 {
            driver.note_applied_voltage(Dq::new(Fixed::ZERO, Fixed::from_integer(-1_125)));
        }
        let modulation = driver.filtered_q_modulation();
        assert!(modulation < Fixed::ratio(-47, 100));
        assert!(modulation > Fixed::ratio(-49, 100));
    }
}
