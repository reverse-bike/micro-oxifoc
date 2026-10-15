//! Motor-control policy and the synchronous FOC interrupt.

#[cfg(any(feature = "firmware", test))]
use oxifoc_core::foc::hall_sensor::HallEdge;
use oxifoc_core::foc::{Fixed, FocController, NoDecoupling, PIController};
use oxifoc_core::foc::{hall_sensor::HallSensor, phase::PhaseManager};
use oxifoc_core::motor::foc_driver::{CurrentLimits, FieldWeakening, FocDriver};

/// Cross-axis decoupling used by every current loop on this board.
pub type CurrentDecoupling = NoDecoupling;
pub type BoardFocController = FocController<
    { crate::config::FOC_DEAD_TIME_COMP_NUMERATOR },
    { crate::config::FOC_DEAD_TIME_COMP_DENOMINATOR },
    CurrentDecoupling,
>;

/// Current controller with the board's PI gains and voltage limits.
pub const fn board_foc_controller() -> BoardFocController {
    let pi = PIController::new(
        crate::config::CURRENT_PI_PROPORTIONAL_GAIN,
        crate::config::CURRENT_PI_INTEGRAL_GAIN_PER_CYCLE,
    );
    BoardFocController::new(
        pi,
        pi,
        crate::config::FOC_VECTOR_LIMIT_TICKS,
        crate::config::FOC_PHASE_LIMIT_TICKS,
    )
}

/// Ride current driver: Hall/observer phase, board controller, ride limits.
pub type RideFocDriver = FocDriver<
    PhaseManager<HallSensor>,
    { crate::config::FOC_DEAD_TIME_COMP_NUMERATOR },
    { crate::config::FOC_DEAD_TIME_COMP_DENOMINATOR },
    CurrentDecoupling,
>;

pub const fn ride_foc_driver(phase: PhaseManager<HallSensor>) -> RideFocDriver {
    RideFocDriver::new(
        board_foc_controller(),
        phase,
        ride_current_limits(),
        crate::config::PWM_ARR,
        crate::config::DC_LINK_CURRENT_FILTER_SHIFT,
    )
    .with_observer_scales(Fixed::ZERO, crate::config::PHASE_CURRENT_AMPS_PER_ADC_COUNT)
    .with_field_weakening(FieldWeakening::new(
        Fixed::from_integer(crate::config::FIELD_WEAKENING_MAXIMUM_CURRENT_COUNTS as i32),
        crate::config::FIELD_WEAKENING_MINIMUM_ERPM,
        crate::config::FIELD_WEAKENING_VOLTAGE_THRESHOLD_TICKS,
        crate::config::FIELD_WEAKENING_RAMP_IN_PER_CYCLE,
        crate::config::FIELD_WEAKENING_RAMP_OUT_PER_CYCLE,
    ))
}

const fn ride_current_limits() -> CurrentLimits {
    CurrentLimits::new(
        Fixed::from_integer(crate::config::RIDE_PHASE_CURRENT_LIMIT_COUNTS as i32),
        Fixed::from_integer(crate::config::PHASE_CURRENT_TRIP_COUNTS as i32),
        Some(Fixed::from_integer(
            crate::config::RIDE_DC_BUS_TRANSIENT_LIMIT_COUNTS as i32,
        )),
        Some(Fixed::from_integer(
            crate::config::RIDE_DC_BUS_REGEN_LIMIT_COUNTS as i32,
        )),
    )
}

/// Apply the foreground's published ceilings before a ride current step.
///
/// The phase ceiling becomes the radius of the complete dq command circle, so
/// field-weakening d current and q current share the thermally derated
/// envelope; a zero ceiling commands no current. Field weakening may use at
/// most half of the ceiling, which leaves at least √3/2 of it for torque-
/// producing q current. The measured overcurrent trip stays at the fixed
/// hardware-protection threshold.
#[inline]
pub fn apply_ride_current_ceilings(
    driver: &mut RideFocDriver,
    dc_current_limit_counts: u16,
    phase_current_limit_counts: u16,
) {
    let phase_ceiling =
        phase_current_limit_counts.min(crate::config::RIDE_PHASE_CURRENT_LIMIT_COUNTS);
    driver.set_max_current(Fixed::from_integer(i32::from(phase_ceiling)));
    driver.set_field_weakening_maximum_current(Fixed::from_integer(i32::from(
        (phase_ceiling / 2).min(crate::config::FIELD_WEAKENING_MAXIMUM_CURRENT_COUNTS),
    )));
    driver.set_bus_limits(
        Some(Fixed::from_integer(i32::from(dc_current_limit_counts))),
        Some(Fixed::from_integer(i32::from(
            crate::config::RIDE_DC_BUS_REGEN_LIMIT_COUNTS,
        ))),
    );
}

/// Q12 factor from electrical RPM to Q16.16 radians per PWM period,
/// `2*pi*65536*4096 / (60*PWM_HZ)` rounded, so the ISR needs no division.
const ACTUATION_ADVANCE_Q12_PER_ERPM: i32 = {
    const TAU_NANO: u64 = 6_283_185_307;
    let denominator = 60 * crate::config::PWM_HZ as u64 * 1_000_000_000;
    (((TAU_NANO << 28) + denominator / 2) / denominator) as i32
};
// Keeps the Q12 rounding error below 0.05%.
const _: () = assert!(ACTUATION_ADVANCE_Q12_PER_ERPM >= 1_000);

/// Electrical angle travelled during one PWM period at `electrical_rpm`.
#[inline]
pub fn actuation_advance_from_erpm(electrical_rpm: i32) -> Fixed {
    Fixed::from_bits((electrical_rpm * ACTUATION_ADVANCE_Q12_PER_ERPM) >> 12)
}

#[cfg(any(feature = "firmware", test))]
pub(crate) const fn stationary_phase_recovery_allowed(
    estimate_missing: bool,
    motor_outputs_disabled: bool,
) -> bool {
    estimate_missing && motor_outputs_disabled
}

/// Missed Hall edges are resynchronised to the live Hall state, rather than
/// latched as a capture fault, while the motor is undriven or its estimate
/// holds the sector center; the live sector is then the whole estimate. The
/// resynchronisation is published as [`HallEdge::Chatter`]; `None` latches
/// the fault.
#[cfg(any(feature = "firmware", test))]
pub(crate) const fn missed_hall_edges_resynchronisation(
    output_was_active: bool,
    holds_sector_center: bool,
) -> Option<HallEdge> {
    if !output_was_active || holds_sector_center {
        Some(HallEdge::Chatter)
    } else {
        None
    }
}

/// What an accepted Hall edge publishes to the ride path.
#[cfg(any(feature = "firmware", test))]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct HallEdgePublication {
    /// Advances the published Hall sequence, which renews ride Hall deadlines.
    pub renews_sequence: bool,
    /// Publishes the progress step, sector interval, and speed.
    pub updates_estimate: bool,
}

/// Only a transition counts as ride Hall progress. Chatter and
/// resynchronisation keep the progress sum consistent without renewing ride
/// deadlines; a repeated raw state publishes nothing.
#[cfg(any(feature = "firmware", test))]
pub(crate) const fn hall_edge_publication(edge: HallEdge) -> HallEdgePublication {
    HallEdgePublication {
        renews_sequence: matches!(edge, HallEdge::Transition),
        updates_estimate: !matches!(edge, HallEdge::Repeated),
    }
}

#[cfg(any(feature = "firmware", test))]
#[inline]
pub(crate) const fn observer_diagnostic_schedule(
    pending: bool,
    periodic_due: bool,
    command_started: bool,
    hall_edge_updated: bool,
) -> (bool, bool) {
    let pending = pending || periodic_due;
    let capture = pending && !command_started && !hall_edge_updated;
    (capture, pending && !capture)
}

#[cfg(feature = "firmware")]
pub mod foc;
pub mod ride;

#[cfg(test)]
mod tests {
    use super::{
        ACTUATION_ADVANCE_Q12_PER_ERPM, HallEdgePublication, RideFocDriver,
        actuation_advance_from_erpm, apply_ride_current_ceilings, hall_edge_publication,
        missed_hall_edges_resynchronisation, observer_diagnostic_schedule, ride_foc_driver,
        stationary_phase_recovery_allowed,
    };
    use crate::config as cfg;
    use crate::sensors::environment::{self, EnvironmentMonitor, RawLocalSensors};
    use oxifoc_core::foc::{
        Dq, Fixed,
        hall_sensor::{HallEdge, HallSensor},
        phase::{PhaseEstimate, PhaseManager},
    };

    const FIELD_WEAKENING_ESTIMATE: PhaseEstimate = PhaseEstimate {
        angle: 0,
        electrical_rpm: -60_000,
        trustworthy: true,
    };

    fn counts(value: u16) -> Fixed {
        Fixed::from_integer(i32::from(value))
    }

    /// Ride driver held at full motoring demand above base speed until field
    /// weakening has reached its configured maximum.
    fn field_weakened_driver() -> RideFocDriver {
        let mut driver = ride_foc_driver(PhaseManager::with_hall(HallSensor::new(
            &cfg::HALL_GEOMETRY,
        )));
        apply_ride_current_ceilings(
            &mut driver,
            cfg::RIDE_DC_BUS_TRANSIENT_LIMIT_COUNTS,
            cfg::RIDE_PHASE_CURRENT_LIMIT_COUNTS,
        );
        for _ in 0..2_000 {
            step(&mut driver, cfg::RIDE_PHASE_CURRENT_LIMIT_COUNTS);
        }
        assert_eq!(
            driver.field_weakening_current(),
            -counts(cfg::FIELD_WEAKENING_MAXIMUM_CURRENT_COUNTS)
        );
        driver
    }

    fn step(driver: &mut RideFocDriver, q_demand_counts: u16) -> Dq {
        driver
            .step_current_control(
                Fixed::ZERO,
                Fixed::ZERO,
                FIELD_WEAKENING_ESTIMATE,
                Dq::new(Fixed::ZERO, -counts(q_demand_counts)),
                cfg::PWM_NEUTRAL,
                62_500,
            )
            .unwrap()
            .target
    }

    #[test]
    fn thermal_phase_ceiling_bounds_field_weakening_current() {
        let mut environment = EnvironmentMonitor::new(0);
        let raw = (0..4096)
            .find(|&raw| environment::controller_temperature_deci_c(raw) == 789)
            .unwrap();
        let bus_limit = environment
            .update(
                0,
                RawLocalSensors {
                    valid: true,
                    bus_voltage_adc: 2945,
                    motor_temperature_adc: 2000,
                    controller_temperature_adc: raw,
                    sampled_at_ms: 0,
                },
                0,
            )
            .unwrap();
        let phase_limit = environment.phase_current_limit_counts();
        assert!(phase_limit > 0 && phase_limit < cfg::FIELD_WEAKENING_MAXIMUM_CURRENT_COUNTS);

        let mut driver = field_weakened_driver();
        apply_ride_current_ceilings(&mut driver, bus_limit, phase_limit);
        let target = step(&mut driver, phase_limit);

        assert!(!Fixed::magnitude_exceeds(
            target.d,
            target.q,
            counts(phase_limit)
        ));
        assert_eq!(
            driver.current_limits().overcurrent_threshold,
            counts(cfg::PHASE_CURRENT_TRIP_COUNTS)
        );
    }

    #[test]
    fn derated_phase_ceiling_reserves_half_for_field_weakening() {
        let phase_limit = cfg::FIELD_WEAKENING_MAXIMUM_CURRENT_COUNTS / 2;
        let mut driver = field_weakened_driver();
        apply_ride_current_ceilings(
            &mut driver,
            cfg::RIDE_DC_BUS_TRANSIENT_LIMIT_COUNTS,
            phase_limit,
        );
        for _ in 0..2_000 {
            step(&mut driver, cfg::RIDE_PHASE_CURRENT_LIMIT_COUNTS);
        }
        let target = step(&mut driver, cfg::RIDE_PHASE_CURRENT_LIMIT_COUNTS);

        assert_eq!(target.d, -counts(phase_limit / 2));
        assert!(target.q.abs() >= counts(phase_limit * 86 / 100));
        assert!(!Fixed::magnitude_exceeds(
            target.d,
            target.q,
            counts(phase_limit)
        ));
    }

    #[test]
    fn full_phase_ceiling_keeps_the_configured_field_weakening_limit() {
        let driver = field_weakened_driver();
        assert_eq!(
            driver.field_weakening_current(),
            -counts(cfg::FIELD_WEAKENING_MAXIMUM_CURRENT_COUNTS)
        );
    }

    #[test]
    fn zero_phase_ceiling_commands_no_field_weakening_current() {
        let mut driver = field_weakened_driver();
        apply_ride_current_ceilings(&mut driver, cfg::RIDE_DC_BUS_TRANSIENT_LIMIT_COUNTS, 0);
        assert_eq!(
            step(&mut driver, cfg::RIDE_PHASE_CURRENT_LIMIT_COUNTS),
            Dq::new(Fixed::ZERO, Fixed::ZERO)
        );
    }

    #[test]
    fn published_phase_ceiling_never_exceeds_the_configured_ride_limit() {
        let mut driver = field_weakened_driver();
        apply_ride_current_ceilings(
            &mut driver,
            cfg::RIDE_DC_BUS_TRANSIENT_LIMIT_COUNTS,
            u16::MAX,
        );
        assert_eq!(
            driver.current_limits().max_current,
            counts(cfg::RIDE_PHASE_CURRENT_LIMIT_COUNTS)
        );
    }

    #[test]
    fn actuation_advance_tracks_the_configured_pwm_rate() {
        let exact = 2.0 * core::f64::consts::PI * 65_536.0 * 4_096.0
            / (60.0 * f64::from(crate::config::PWM_HZ));
        assert!((f64::from(ACTUATION_ADVANCE_Q12_PER_ERPM) - exact).abs() <= 0.5);
        if crate::config::PWM_HZ == 16_000 {
            assert_eq!(ACTUATION_ADVANCE_Q12_PER_ERPM, 1_757);
        }
        // 60,000 erpm is 1,000 electrical revolutions per second.
        let radians = 2.0 * core::f64::consts::PI * 1_000.0 / f64::from(crate::config::PWM_HZ);
        let bits = actuation_advance_from_erpm(60_000).to_bits();
        assert!((f64::from(bits) - radians * 65_536.0).abs() < 20.0);
        assert_eq!(actuation_advance_from_erpm(-60_000).to_bits(), -bits - 1);
    }

    #[test]
    fn disabled_pwm_allows_a_missing_phase_to_recover_before_energizing() {
        assert!(stationary_phase_recovery_allowed(true, true));
        assert!(!stationary_phase_recovery_allowed(false, true));
        assert!(!stationary_phase_recovery_allowed(true, false));
    }

    #[test]
    fn missed_hall_edges_resynchronise_only_undriven_or_below_center_snap_speed() {
        assert_eq!(
            missed_hall_edges_resynchronisation(false, false),
            Some(HallEdge::Chatter)
        );
        assert_eq!(
            missed_hall_edges_resynchronisation(false, true),
            Some(HallEdge::Chatter)
        );
        assert_eq!(
            missed_hall_edges_resynchronisation(true, true),
            Some(HallEdge::Chatter)
        );
        assert_eq!(missed_hall_edges_resynchronisation(true, false), None);
    }

    #[test]
    fn only_a_real_transition_renews_the_ride_hall_sequence() {
        assert_eq!(
            hall_edge_publication(HallEdge::Transition),
            HallEdgePublication {
                renews_sequence: true,
                updates_estimate: true,
            }
        );
        let resynchronised = missed_hall_edges_resynchronisation(false, false).unwrap();
        for edge in [HallEdge::Chatter, resynchronised] {
            assert_eq!(
                hall_edge_publication(edge),
                HallEdgePublication {
                    renews_sequence: false,
                    updates_estimate: true,
                }
            );
        }
        assert_eq!(
            hall_edge_publication(HallEdge::Repeated),
            HallEdgePublication {
                renews_sequence: false,
                updates_estimate: false,
            }
        );
    }

    #[test]
    fn observer_diagnostics_defer_until_a_quiet_control_cycle() {
        assert_eq!(
            observer_diagnostic_schedule(false, true, false, true),
            (false, true)
        );
        assert_eq!(
            observer_diagnostic_schedule(true, false, false, false),
            (true, false)
        );
        assert_eq!(
            observer_diagnostic_schedule(false, true, true, false),
            (false, true)
        );
    }
}
