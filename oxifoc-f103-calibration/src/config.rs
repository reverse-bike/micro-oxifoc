//! Calibration policy layered on the shared F103 board constants.

pub const PWM_HZ: u32 = oxifoc_f103::config::PWM_HZ;
pub const PWM_PERIOD_TICKS: u16 = oxifoc_f103::config::PWM_ARR;
pub const NOMINAL_CURRENT_MA_PER_COUNT: u32 =
    oxifoc_f103::config::PHASE_CURRENT_MA_PER_ADC_COUNT as u32;

pub const RESISTANCE_CURRENT_LOW_COUNTS: i16 = 50;
pub const RESISTANCE_CURRENT_HIGH_COUNTS: i16 = 250;
pub const CALIBRATION_PHASE_CURRENT_TRIP_COUNTS: u16 = 600;
pub const CALIBRATION_BUS_MINIMUM_MV: u32 =
    oxifoc_f103::sensors::environment::DC_BUS_UNDERVOLTAGE_MV;
pub const CALIBRATION_BUS_MAXIMUM_MV: u32 = 60_000;

/// Calibration enforces its own `CALIBRATION_BUS_MINIMUM_MV..=
/// CALIBRATION_BUS_MAXIMUM_MV` window on every control interrupt, so the ride
/// image's lower DC-bus overvoltage cutoff does not block it. Every other
/// environment reason, including thermal derating, still does.
pub const fn environment_allows_calibration(reasons: u8) -> bool {
    reasons & !oxifoc_f103::sensors::environment::reason::DC_BUS_OVERVOLTAGE == 0
}

pub const INDUCTANCE_HOLD_CURRENT_COUNTS: i16 = 100;
pub const INDUCTANCE_TARGET_DI_COUNTS: i16 = 20;
pub const INDUCTANCE_PULSES_PER_AXIS: u8 = 20;
pub const FLUX_CURRENT_COUNTS: i16 = 80;
pub const FLUX_TARGET_ERPM: i32 = 6_000;
pub const HALL_CALIBRATION_CURRENT_COUNTS: i16 = 80;

const _: () = assert!(NOMINAL_CURRENT_MA_PER_COUNT <= u8::MAX as u32);
const _: () = assert!(RESISTANCE_CURRENT_LOW_COUNTS > 0);
const _: () = assert!(RESISTANCE_CURRENT_HIGH_COUNTS > RESISTANCE_CURRENT_LOW_COUNTS);
const _: () =
    assert!((RESISTANCE_CURRENT_HIGH_COUNTS as u16) < CALIBRATION_PHASE_CURRENT_TRIP_COUNTS);
const _: () =
    assert!(CALIBRATION_PHASE_CURRENT_TRIP_COUNTS < oxifoc_f103::config::PHASE_CURRENT_TRIP_COUNTS);
const _: () = assert!(CALIBRATION_BUS_MAXIMUM_MV > CALIBRATION_BUS_MINIMUM_MV);
const _: () = assert!(INDUCTANCE_HOLD_CURRENT_COUNTS > 0);
const _: () = assert!(INDUCTANCE_TARGET_DI_COUNTS > 1);
const _: () = assert!(
    INDUCTANCE_HOLD_CURRENT_COUNTS as u16 + INDUCTANCE_TARGET_DI_COUNTS as u16 * 5
        < CALIBRATION_PHASE_CURRENT_TRIP_COUNTS
);
const _: () = assert!(FLUX_CURRENT_COUNTS > 0);
const _: () = assert!((FLUX_CURRENT_COUNTS as u16) < CALIBRATION_PHASE_CURRENT_TRIP_COUNTS);
const _: () = assert!(FLUX_TARGET_ERPM > 0);
const _: () = assert!(HALL_CALIBRATION_CURRENT_COUNTS > 0);
const _: () =
    assert!((HALL_CALIBRATION_CURRENT_COUNTS as u16) < CALIBRATION_PHASE_CURRENT_TRIP_COUNTS);

#[cfg(test)]
mod tests {
    use super::*;
    use oxifoc_f103::sensors::environment::{DC_BUS_OVERVOLTAGE_MV, reason};

    #[test]
    fn ride_overvoltage_does_not_block_the_wider_calibration_bus_window() {
        assert!(DC_BUS_OVERVOLTAGE_MV < CALIBRATION_BUS_MAXIMUM_MV);
        assert!(environment_allows_calibration(0));
        assert!(environment_allows_calibration(reason::DC_BUS_OVERVOLTAGE));
        for blocking in [
            reason::CONTROLLER_TEMPERATURE,
            reason::MOTOR_TEMPERATURE,
            reason::DC_BUS_TRANSIENT_RECOVERY,
            reason::DC_BUS_UNDERVOLTAGE,
            reason::LOCAL_DATA_MISSING,
        ] {
            assert!(!environment_allows_calibration(blocking));
            assert!(!environment_allows_calibration(
                blocking | reason::DC_BUS_OVERVOLTAGE
            ));
        }
    }
}
