//! Motor-control policy and the synchronous FOC interrupt.

#[cfg(any(feature = "firmware", test))]
pub(crate) const fn stationary_phase_recovery_allowed(
    estimate_missing: bool,
    motor_outputs_disabled: bool,
) -> bool {
    estimate_missing && motor_outputs_disabled
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
    use super::{observer_diagnostic_schedule, stationary_phase_recovery_allowed};

    #[test]
    fn disabled_pwm_allows_a_missing_phase_to_recover_before_energizing() {
        assert!(stationary_phase_recovery_allowed(true, true));
        assert!(!stationary_phase_recovery_allowed(false, true));
        assert!(!stationary_phase_recovery_allowed(true, false));
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
