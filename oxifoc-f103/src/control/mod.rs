//! Motor-control policy and the synchronous FOC interrupt.

#[cfg(any(feature = "firmware", test))]
pub(crate) const fn stationary_phase_recovery_allowed(
    estimate_missing: bool,
    motor_outputs_disabled: bool,
) -> bool {
    estimate_missing && motor_outputs_disabled
}

#[cfg(feature = "firmware")]
pub mod foc;
pub mod ride;

#[cfg(test)]
mod tests {
    use super::stationary_phase_recovery_allowed;

    #[test]
    fn disabled_pwm_allows_a_missing_phase_to_recover_before_energizing() {
        assert!(stationary_phase_recovery_allowed(true, true));
        assert!(!stationary_phase_recovery_allowed(false, true));
        assert!(!stationary_phase_recovery_allowed(true, false));
    }
}
