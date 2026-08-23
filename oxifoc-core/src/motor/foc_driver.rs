//! Synchronous fixed-point FOC motor driver.
//!
//! The driver owns the current controller, selected phase provider, circular
//! current-command envelope, supply-current projection, and measured dq trip.
//! Platform code retains raw ADC sampling, PWM register writes, and immediate
//! hardware shutdown so those operations remain explicit at the ISR boundary.

use crate::foc::phase::{PhaseEstimate, PhaseInput, PhaseProvider};
use crate::foc::trig::Turns;
use crate::foc::{
    AlphaBeta, DecouplingModel, Dq, Fixed, FixedFocController, NoDecoupling, PwmDuty, Scalar,
};

/// Current-command, supply-current, and measured-overcurrent limits.
///
/// The command is first constrained to the d-priority current circle. The
/// driver then constrains q current from the complete filtered dq terminal-
/// power projection so DC-side current remains inside the selected motoring
/// or regen bound. Finally, measured dq magnitude is checked against the hard
/// trip.
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
        self.clamp_targets_and_quadrature_limit(target).0
    }

    fn clamp_targets_and_quadrature_limit(&self, target: Dq) -> (Dq, Fixed) {
        if self.max_current <= Fixed::ZERO {
            return (target, self.max_current);
        }
        let direct = Scalar::clamp(target.d, -self.max_current, self.max_current);
        let quadrature_limit = if direct == Fixed::ZERO {
            self.max_current
        } else {
            Fixed::circular_remaining(self.max_current, direct)
        };
        (
            Dq::new(
                direct,
                Scalar::clamp(target.q, -quadrature_limit, quadrature_limit),
            ),
            quadrature_limit,
        )
    }

    pub fn is_overcurrent(&self, measured: Dq) -> bool {
        if self.overcurrent_threshold <= Fixed::ZERO {
            return false;
        }
        Fixed::magnitude_exceeds(measured.d, measured.q, self.overcurrent_threshold)
    }
}

/// Negative d-current generated from voltage-circle pressure.
///
/// The controller follows the MESC FW V2 policy recorded in OxiFOC's design
/// notes: voltage demand above a fixed headroom threshold moves the target
/// toward the configured weakening current, while restored headroom moves it
/// back toward zero. Motor parameters are not part of the feedback law.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct FieldWeakening {
    current: Fixed,
    maximum_current: Fixed,
    minimum_erpm: u32,
    voltage_threshold: Fixed,
    ramp_in_per_cycle: Fixed,
    ramp_out_per_cycle: Fixed,
}

impl FieldWeakening {
    pub const fn disabled() -> Self {
        Self {
            current: Fixed::ZERO,
            maximum_current: Fixed::ZERO,
            minimum_erpm: u32::MAX,
            voltage_threshold: Fixed::ZERO,
            ramp_in_per_cycle: Fixed::ZERO,
            ramp_out_per_cycle: Fixed::ZERO,
        }
    }

    pub const fn new(
        maximum_current: Fixed,
        minimum_erpm: u32,
        voltage_threshold: Fixed,
        ramp_in_per_cycle: Fixed,
        ramp_out_per_cycle: Fixed,
    ) -> Self {
        Self {
            current: Fixed::ZERO,
            maximum_current,
            minimum_erpm,
            voltage_threshold,
            ramp_in_per_cycle,
            ramp_out_per_cycle,
        }
    }

    pub const fn current(&self) -> Fixed {
        self.current
    }

    fn direct_target(&self, requested: Fixed) -> Fixed {
        if self.maximum_current > Fixed::ZERO {
            core::cmp::min(requested, self.current)
        } else {
            requested
        }
    }

    fn update(&mut self, requested_voltage: Dq, electrical_rpm: i32, requested_q: Fixed) {
        let motoring = (requested_q > Fixed::ZERO && electrical_rpm > 0)
            || (requested_q < Fixed::ZERO && electrical_rpm < 0);
        let may_weaken = electrical_rpm.unsigned_abs() >= self.minimum_erpm
            && (motoring || self.current < Fixed::ZERO);
        let voltage_full = may_weaken
            && self.maximum_current > Fixed::ZERO
            && whole_unit_magnitude_exceeds(requested_voltage, self.voltage_threshold);
        if voltage_full {
            self.current = Scalar::clamp(
                self.current - self.ramp_in_per_cycle,
                -self.maximum_current,
                Fixed::ZERO,
            );
        } else {
            self.current = Scalar::clamp(
                self.current + self.ramp_out_per_cycle,
                -self.maximum_current,
                Fixed::ZERO,
            );
        }
    }

    fn reset(&mut self) {
        self.current = Fixed::ZERO;
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
    /// Complete controller voltage request before the circular voltage limit.
    pub requested_voltage: Dq,
    /// Motor-model contribution included in `requested_voltage`.
    pub feedforward_voltage: Dq,
    pub applied_voltage: Dq,
    pub duties: PwmDuty,
    pub voltage_limited: bool,
}

/// Owns current regulation, current limits, and the selected phase provider.
pub struct FocDriver<
    Phase,
    const DEAD_TIME_NUMERATOR: i32 = 0,
    const DEAD_TIME_DENOMINATOR: i32 = 1,
    Decoupling: DecouplingModel<Fixed> = NoDecoupling,
> {
    controller: FixedFocController<DEAD_TIME_NUMERATOR, DEAD_TIME_DENOMINATOR, Decoupling>,
    phase: Phase,
    current_limits: CurrentLimits,
    field_weakening: FieldWeakening,
    bus_mod_filt_ticks: Dq,
    pwm_period_ticks: u16,
    modulation_filter_shift: u8,
    volts_per_pwm_tick: Fixed,
    amps_per_current_count: Fixed,
    previous_applied_voltage: AlphaBeta,
}

impl<
    Phase,
    const DEAD_TIME_NUMERATOR: i32,
    const DEAD_TIME_DENOMINATOR: i32,
    Decoupling: DecouplingModel<Fixed>,
> FocDriver<Phase, DEAD_TIME_NUMERATOR, DEAD_TIME_DENOMINATOR, Decoupling>
{
    pub const fn new(
        controller: FixedFocController<DEAD_TIME_NUMERATOR, DEAD_TIME_DENOMINATOR, Decoupling>,
        phase: Phase,
        current_limits: CurrentLimits,
        pwm_period_ticks: u16,
        modulation_filter_shift: u8,
    ) -> Self {
        Self {
            controller,
            phase,
            current_limits,
            field_weakening: FieldWeakening::disabled(),
            bus_mod_filt_ticks: Dq::new(Fixed::ZERO, Fixed::ZERO),
            pwm_period_ticks,
            modulation_filter_shift,
            volts_per_pwm_tick: Fixed::ONE,
            amps_per_current_count: Fixed::ONE,
            previous_applied_voltage: AlphaBeta {
                alpha: Fixed::ZERO,
                beta: Fixed::ZERO,
            },
        }
    }

    /// Configure the physical scales supplied to phase observers.
    pub const fn with_observer_scales(
        mut self,
        volts_per_pwm_tick: Fixed,
        amps_per_current_count: Fixed,
    ) -> Self {
        self.volts_per_pwm_tick = volts_per_pwm_tick;
        self.amps_per_current_count = amps_per_current_count;
        self
    }

    pub const fn with_field_weakening(mut self, field_weakening: FieldWeakening) -> Self {
        self.field_weakening = field_weakening;
        self
    }

    /// Refresh the bus-dependent voltage scale without disturbing the
    /// previous-cycle voltage/current pairing.
    pub fn set_volts_per_pwm_tick(&mut self, volts_per_pwm_tick: Fixed) {
        self.volts_per_pwm_tick = volts_per_pwm_tick;
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
        self.field_weakening.reset();
        self.bus_mod_filt_ticks = Dq::new(Fixed::ZERO, Fixed::ZERO);
        self.previous_applied_voltage = AlphaBeta::default();
    }

    pub const fn field_weakening_current(&self) -> Fixed {
        self.field_weakening.current()
    }

    pub fn filtered_q_modulation(&self) -> Fixed {
        if self.pwm_period_ticks == 0 {
            return Fixed::ZERO;
        }
        let numerator = i64::from(self.bus_mod_filt_ticks.q.to_bits()).saturating_mul(3);
        let denominator = i64::from(self.pwm_period_ticks).saturating_mul(2);
        Fixed::from_bits(
            (numerator / denominator).clamp(i64::from(i32::MIN), i64::from(i32::MAX)) as i32,
        )
    }

    fn note_applied_voltage(&mut self, applied: Dq) {
        let shift = self.modulation_filter_shift.min(30);
        self.bus_mod_filt_ticks.d = filter_step(self.bus_mod_filt_ticks.d, applied.d, shift);
        self.bus_mod_filt_ticks.q = filter_step(self.bus_mod_filt_ticks.q, applied.q, shift);
    }

    fn clamp_targets_with_limit(&self, target: Dq) -> (Dq, Fixed) {
        let (target, static_quadrature_limit) = self
            .current_limits
            .clamp_targets_and_quadrature_limit(target);
        let filtered_voltage_bits = self.bus_mod_filt_ticks.q.to_bits();
        let deadband_bits = u32::from(self.pwm_period_ticks)
            .saturating_mul(2)
            .saturating_mul(1_u32 << 16)
            / 1_000;
        if filtered_voltage_bits.unsigned_abs().saturating_mul(3) < deadband_bits
            || target.q == Fixed::ZERO
        {
            return (target, static_quadrature_limit);
        }

        if target.d == Fixed::ZERO {
            let q_voltage_ticks = self.bus_mod_filt_ticks.q.abs_ceil_u32();
            let motoring = (target.q > Fixed::ZERO) == (filtered_voltage_bits > 0);
            let bus_limit = if motoring {
                self.current_limits.bus_in_max
            } else {
                self.current_limits.bus_regen_max
            };
            let quadrature_limit = bus_limit.map_or(static_quadrature_limit, |limit| {
                core::cmp::min(
                    phase_current_limit(
                        bus_power_bound(limit, self.pwm_period_ticks),
                        q_voltage_ticks,
                    ),
                    static_quadrature_limit,
                )
            });
            return (
                Dq::new(
                    target.d,
                    Scalar::clamp(target.q, -quadrature_limit, quadrature_limit),
                ),
                quadrature_limit,
            );
        }

        let q_voltage_ticks = signed_ceil_abs(self.bus_mod_filt_ticks.q);
        let d_voltage_ticks = signed_ceil_abs(self.bus_mod_filt_ticks.d);
        let d_current_counts = signed_ceil_abs(target.d);
        let direct_power = d_voltage_ticks.saturating_mul(d_current_counts);
        let q_adds_power = (q_voltage_ticks > 0) == (target.q > Fixed::ZERO);
        let power_room = if q_adds_power {
            self.current_limits.bus_in_max.map(|limit| {
                bus_power_bound(limit, self.pwm_period_ticks).saturating_sub(direct_power)
            })
        } else {
            self.current_limits.bus_regen_max.map(|limit| {
                bus_power_bound(limit, self.pwm_period_ticks).saturating_add(direct_power)
            })
        };
        let quadrature_limit = power_room.map_or(static_quadrature_limit, |room| {
            core::cmp::min(
                phase_current_limit(room, q_voltage_ticks.unsigned_abs()),
                static_quadrature_limit,
            )
        });
        (
            Dq::new(
                target.d,
                Scalar::clamp(target.q, -quadrature_limit, quadrature_limit),
            ),
            quadrature_limit,
        )
    }
}

impl<
    Phase,
    const DEAD_TIME_NUMERATOR: i32,
    const DEAD_TIME_DENOMINATOR: i32,
    Decoupling: DecouplingModel<Fixed>,
> FocDriver<Phase, DEAD_TIME_NUMERATOR, DEAD_TIME_DENOMINATOR, Decoupling>
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
        estimate: PhaseEstimate<Turns>,
        target: Dq,
        pwm_neutral: u16,
        control_period_ns: u32,
    ) -> Result<FocOutput, StepError> {
        let target = Dq::new(self.field_weakening.direct_target(target.d), target.q);
        let (target, quadrature_limit) = self.clamp_targets_with_limit(target);
        let injection = self.phase.injection();
        let (measured_current, duties) = self.controller.step_with_velocity_and_injection(
            phase_a,
            phase_b,
            estimate.angle,
            estimate.electrical_rpm,
            target,
            injection,
            self.volts_per_pwm_tick,
            pwm_neutral,
        );
        if self.current_limits.is_overcurrent(measured_current) {
            self.controller.reset();
            return Err(StepError::Overcurrent);
        }
        let requested_voltage = self.controller.requested_voltage();
        let feedforward_voltage = self.controller.feedforward_voltage();
        let applied_voltage = self.controller.applied_voltage();
        self.field_weakening
            .update(requested_voltage, estimate.electrical_rpm, target.q);
        self.note_applied_voltage(applied_voltage);
        let voltage_limited = self.controller.voltage_limited();
        let measured_stationary = self.controller.measured_stationary();
        let measured_stationary = AlphaBeta {
            alpha: scale_observer_input(measured_stationary.alpha, self.amps_per_current_count),
            beta: scale_observer_input(measured_stationary.beta, self.amps_per_current_count),
        };
        let applied_stationary = self.controller.applied_stationary();
        let applied_stationary = AlphaBeta {
            alpha: scale_observer_input(applied_stationary.alpha, self.volts_per_pwm_tick),
            beta: scale_observer_input(applied_stationary.beta, self.volts_per_pwm_tick),
        };
        self.phase.update(&PhaseInput::new(
            self.previous_applied_voltage,
            measured_stationary,
            control_period_ns,
        ));
        self.previous_applied_voltage = applied_stationary;
        Ok(FocOutput {
            target,
            quadrature_limit,
            measured_current,
            requested_voltage,
            feedforward_voltage,
            applied_voltage,
            duties,
            voltage_limited,
        })
    }
}

#[inline(always)]
fn scale_observer_input(value: Fixed, scale: Fixed) -> Fixed {
    // Controller current and voltage are already bounded by the ADC and PWM
    // envelopes. Their physical scales cannot approach Q16.16 overflow, so
    // the observer bridge needs only the fixed-point product, not another
    // saturation tree around each of its four axes.
    Fixed::from_bits(((i64::from(value.to_bits()) * i64::from(scale.to_bits())) >> 16) as i32)
}

#[inline]
fn filter_step(previous: Fixed, sample: Fixed, shift: u8) -> Fixed {
    let previous = previous.to_bits();
    let difference = sample.to_bits().saturating_sub(previous);
    let adjustment = if difference < 0 {
        -difference.saturating_neg().wrapping_shr(u32::from(shift))
    } else {
        difference.wrapping_shr(u32::from(shift))
    };
    Fixed::from_bits(previous.saturating_add(adjustment))
}

fn signed_ceil_abs(value: Fixed) -> i32 {
    let magnitude = value.abs_ceil_u32().min(i32::MAX as u32) as i32;
    if value < Fixed::ZERO {
        -magnitude
    } else {
        magnitude
    }
}

#[inline]
fn whole_unit_magnitude_exceeds(value: Dq, limit: Fixed) -> bool {
    let direct = value.d.abs_ceil_u32();
    let quadrature = value.q.abs_ceil_u32();
    let limit = limit.integer().max(0) as u32;
    direct
        .saturating_mul(direct)
        .saturating_add(quadrature.saturating_mul(quadrature))
        > limit.saturating_mul(limit)
}

fn bus_power_bound(bus_limit: Fixed, pwm_period_ticks: u16) -> i32 {
    bus_limit
        .integer()
        .max(0)
        .saturating_mul(i32::from(pwm_period_ticks))
        .saturating_mul(2)
        / 3
}

fn phase_current_limit(power_room: i32, q_voltage_ticks: u32) -> Fixed {
    if power_room <= 0 || q_voltage_ticks == 0 {
        return Fixed::ZERO;
    }
    Fixed::from_integer((power_room as u32 / q_voltage_ticks).min(i32::MAX as u32) as i32)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::foc::phase::PhaseSource;
    use crate::foc::{FocController, PIController};

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

    #[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
    struct CapturingPhase {
        input: PhaseInput,
    }

    impl PhaseProvider for CapturingPhase {
        type Angle = Turns;

        fn source(&self) -> PhaseSource {
            PhaseSource::Observer
        }

        fn estimate(&self, _elapsed_since_observation_us: u32) -> Option<PhaseEstimate<Turns>> {
            Some(PhaseEstimate {
                angle: 0,
                electrical_rpm: 0,
                trustworthy: true,
            })
        }

        fn update(&mut self, input: &PhaseInput) {
            self.input = *input;
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
        driver_with_limits(filter_shift, limits())
    }

    fn driver_with_limits(filter_shift: u8, limits: CurrentLimits) -> FocDriver<TestPhase> {
        let pi = PIController::new(Fixed::ZERO, Fixed::ZERO);
        FocDriver::new(
            FocController::new(pi, pi, Fixed::from_integer(1_273), 1_103),
            TestPhase,
            limits,
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
    fn bus_limit_accounts_for_direct_axis_terminal_power() {
        let limits = CurrentLimits::new(
            Fixed::from_integer(880),
            Fixed::from_integer(1_344),
            Some(Fixed::from_integer(400)),
            Some(Fixed::from_integer(40)),
        );
        let mut driver = driver_with_limits(0, limits);
        driver.note_applied_voltage(Dq::new(
            Fixed::from_integer(-300),
            Fixed::from_integer(-900),
        ));

        let (target, quadrature_limit) = driver.clamp_targets_with_limit(Dq::new(
            Fixed::from_integer(-200),
            Fixed::from_integer(-838),
        ));

        assert_eq!(quadrature_limit, Fixed::from_integer(600));
        assert_eq!(
            target,
            Dq::new(Fixed::from_integer(-200), Fixed::from_integer(-600))
        );
    }

    #[test]
    fn zero_direct_target_keeps_the_q_only_bus_limit_with_filtered_direct_voltage() {
        let limits = CurrentLimits::new(
            Fixed::from_integer(880),
            Fixed::from_integer(1_344),
            Some(Fixed::from_integer(400)),
            Some(Fixed::from_integer(40)),
        );
        let mut driver = driver_with_limits(0, limits);
        driver.note_applied_voltage(Dq::new(
            Fixed::from_integer(-600),
            Fixed::from_integer(-900),
        ));

        let (target, quadrature_limit) =
            driver.clamp_targets_with_limit(Dq::new(Fixed::ZERO, Fixed::from_integer(-880)));

        assert_eq!(quadrature_limit, Fixed::from_integer(666));
        assert_eq!(target, Dq::new(Fixed::ZERO, Fixed::from_integer(-666)));
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
    fn hill_start_regen_budget_allows_full_torque_at_creep_speed() {
        let limits = CurrentLimits::new(
            Fixed::from_integer(838),
            Fixed::from_integer(1_344),
            Some(Fixed::from_integer(400)),
            Some(Fixed::from_integer(40)),
        );
        let mut driver = driver_with_limits(0, limits);
        driver.note_applied_voltage(Dq::new(Fixed::ZERO, Fixed::from_integer(66)));

        let (target, quadrature_limit) =
            driver.clamp_targets_with_limit(Dq::new(Fixed::ZERO, Fixed::from_integer(-838)));

        assert_eq!(quadrature_limit, Fixed::from_integer(838));
        assert_eq!(target.q, Fixed::from_integer(-838));
    }

    #[test]
    fn hill_start_regen_budget_progressively_limits_faster_rollback() {
        let limits = CurrentLimits::new(
            Fixed::from_integer(838),
            Fixed::from_integer(1_344),
            Some(Fixed::from_integer(400)),
            Some(Fixed::from_integer(40)),
        );
        let mut driver = driver_with_limits(0, limits);
        driver.note_applied_voltage(Dq::new(Fixed::ZERO, Fixed::from_integer(88)));

        let (target, quadrature_limit) =
            driver.clamp_targets_with_limit(Dq::new(Fixed::ZERO, Fixed::from_integer(-838)));

        assert_eq!(quadrature_limit, Fixed::from_integer(681));
        assert_eq!(target.q, Fixed::from_integer(-681));
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
        let mut driver: FocDriver<TestPhase> = FocDriver::new(
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

    #[test]
    fn field_weakening_uses_saturation_feedback_with_a_one_cycle_delay() {
        let proportional = PIController::new(Fixed::ONE, Fixed::ZERO);
        let weakening = FieldWeakening::new(
            Fixed::from_integer(200),
            6_000,
            Fixed::from_integer(50),
            Fixed::from_integer(10),
            Fixed::from_integer(5),
        );
        let mut driver: FocDriver<TestPhase> = FocDriver::new(
            FocController::new(proportional, proportional, Fixed::from_integer(100), 100),
            TestPhase,
            CurrentLimits::new(
                Fixed::from_integer(880),
                Fixed::from_integer(1_344),
                None,
                None,
            ),
            200,
            0,
        )
        .with_field_weakening(weakening);
        let estimate = PhaseEstimate {
            angle: 0,
            electrical_rpm: -10_000,
            trustworthy: true,
        };

        let first = driver
            .step_current_control(
                Fixed::ZERO,
                Fixed::ZERO,
                estimate,
                Dq::new(Fixed::ZERO, Fixed::from_integer(-100)),
                100,
                62_500,
            )
            .unwrap();
        assert_eq!(first.target.d, Fixed::ZERO);
        assert_eq!(driver.field_weakening_current(), Fixed::from_integer(-10));

        let second = driver
            .step_current_control(
                Fixed::ZERO,
                Fixed::ZERO,
                estimate,
                Dq::new(Fixed::ZERO, Fixed::from_integer(-100)),
                100,
                62_500,
            )
            .unwrap();
        assert_eq!(second.target.d, Fixed::from_integer(-10));
        assert_eq!(driver.field_weakening_current(), Fixed::from_integer(-20));
    }

    #[test]
    fn field_weakening_voltage_gate_uses_conservative_whole_ticks() {
        let threshold = Fixed::ratio(120_935, 100);
        assert!(!whole_unit_magnitude_exceeds(
            Dq::new(Fixed::from_integer(1_209), Fixed::ZERO),
            threshold,
        ));
        assert!(whole_unit_magnitude_exceeds(
            Dq::new(Fixed::from_bits((1_209 << 16) + 1), Fixed::ZERO),
            threshold,
        ));
        assert!(!whole_unit_magnitude_exceeds(
            Dq::new(Fixed::from_integer(900), Fixed::from_integer(800)),
            threshold,
        ));
        assert!(whole_unit_magnitude_exceeds(
            Dq::new(Fixed::from_integer(1_000), Fixed::from_integer(700)),
            threshold,
        ));
    }

    #[test]
    fn field_weakening_does_not_enter_for_low_speed_or_braking_saturation() {
        let mut weakening = FieldWeakening::new(
            Fixed::from_integer(200),
            6_000,
            Fixed::from_integer(50),
            Fixed::from_integer(10),
            Fixed::from_integer(5),
        );
        let saturated = Dq::new(Fixed::ZERO, Fixed::from_integer(100));

        weakening.update(saturated, -5_999, Fixed::from_integer(-100));
        assert_eq!(weakening.current(), Fixed::ZERO);
        weakening.update(saturated, -10_000, Fixed::from_integer(100));
        assert_eq!(weakening.current(), Fixed::ZERO);
    }

    #[test]
    fn active_field_weakening_holds_at_speed_and_unwinds_with_headroom() {
        let mut weakening = FieldWeakening::new(
            Fixed::from_integer(20),
            6_000,
            Fixed::from_integer(50),
            Fixed::from_integer(10),
            Fixed::from_integer(5),
        );
        let saturated = Dq::new(Fixed::ZERO, Fixed::from_integer(100));

        weakening.update(saturated, -10_000, Fixed::from_integer(-100));
        weakening.update(saturated, -10_000, Fixed::ZERO);
        weakening.update(saturated, -10_000, Fixed::ZERO);
        assert_eq!(weakening.current(), Fixed::from_integer(-20));

        weakening.update(Dq::default(), -10_000, Fixed::ZERO);
        assert_eq!(weakening.current(), Fixed::from_integer(-15));
        for _ in 0..3 {
            weakening.update(Dq::default(), -10_000, Fixed::ZERO);
        }
        assert_eq!(weakening.current(), Fixed::ZERO);
    }

    #[test]
    fn driver_reset_clears_field_weakening_current() {
        let proportional = PIController::new(Fixed::ONE, Fixed::ZERO);
        let mut driver: FocDriver<TestPhase> = FocDriver::new(
            FocController::new(proportional, proportional, Fixed::from_integer(100), 100),
            TestPhase,
            CurrentLimits::new(
                Fixed::from_integer(880),
                Fixed::from_integer(1_344),
                None,
                None,
            ),
            200,
            0,
        )
        .with_field_weakening(FieldWeakening::new(
            Fixed::from_integer(200),
            6_000,
            Fixed::from_integer(50),
            Fixed::from_integer(10),
            Fixed::from_integer(5),
        ));
        driver
            .step_current_control(
                Fixed::ZERO,
                Fixed::ZERO,
                PhaseEstimate {
                    angle: 0,
                    electrical_rpm: -10_000,
                    trustworthy: true,
                },
                Dq::new(Fixed::ZERO, Fixed::from_integer(-100)),
                100,
                62_500,
            )
            .unwrap();
        assert_eq!(driver.field_weakening_current(), Fixed::from_integer(-10));

        driver.reset();
        assert_eq!(driver.field_weakening_current(), Fixed::ZERO);
    }

    #[test]
    fn output_preserves_pre_limit_voltage_diagnostics() {
        let proportional = PIController::new(Fixed::ONE, Fixed::ZERO);
        let mut driver: FocDriver<TestPhase> = FocDriver::new(
            FocController::new(proportional, proportional, Fixed::from_integer(50), 100),
            TestPhase,
            CurrentLimits::new(
                Fixed::from_integer(100),
                Fixed::from_integer(200),
                None,
                None,
            ),
            200,
            0,
        );
        let output = driver
            .step_current_control(
                Fixed::ZERO,
                Fixed::ZERO,
                PhaseEstimate {
                    angle: 0,
                    electrical_rpm: -31_000,
                    trustworthy: true,
                },
                Dq::new(Fixed::from_integer(30), Fixed::from_integer(-40)),
                100,
                62_500,
            )
            .unwrap();

        assert_eq!(
            output.requested_voltage,
            Dq::new(Fixed::from_integer(30), Fixed::from_integer(-40))
        );
        assert_eq!(output.feedforward_voltage, Dq::default());
        assert_eq!(output.applied_voltage, output.requested_voltage);
    }

    #[test]
    fn observer_receives_previous_voltage_with_the_current_sample() {
        let proportional = PIController::new(Fixed::ONE, Fixed::ZERO);
        let mut driver: FocDriver<CapturingPhase> = FocDriver::new(
            FocController::new(proportional, proportional, Fixed::from_integer(100), 100),
            CapturingPhase::default(),
            CurrentLimits::new(
                Fixed::from_integer(100),
                Fixed::from_integer(200),
                None,
                None,
            ),
            200,
            0,
        )
        .with_observer_scales(Fixed::from_integer(2), Fixed::from_integer(3));

        driver
            .step_current_control(
                Fixed::ZERO,
                Fixed::ZERO,
                PhaseEstimate {
                    angle: 0,
                    electrical_rpm: 0,
                    trustworthy: true,
                },
                Dq::new(Fixed::from_integer(10), Fixed::ZERO),
                100,
                62_500,
            )
            .unwrap();
        assert_eq!(driver.phase().input.applied_voltage, AlphaBeta::default());

        driver
            .step_current_control(
                Fixed::from_integer(2),
                Fixed::from_integer(-1),
                PhaseEstimate {
                    angle: 0,
                    electrical_rpm: 0,
                    trustworthy: true,
                },
                Dq::default(),
                100,
                62_500,
            )
            .unwrap();
        let applied = driver.phase().input.applied_voltage;
        assert!((applied.alpha.to_bits() - Fixed::from_integer(20).to_bits()).unsigned_abs() <= 32);
        assert!(applied.beta.to_bits().unsigned_abs() <= 32);
        assert_eq!(
            driver.phase().input.measured_current,
            AlphaBeta {
                alpha: Fixed::from_integer(6),
                beta: Fixed::ZERO,
            }
        );
    }
}
