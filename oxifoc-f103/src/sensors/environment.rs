//! Local voltage and temperature conversion plus the ride-current envelope.

use crate::config::{
    PWM_ARR, RIDE_DC_BUS_CONTINUOUS_LIMIT_COUNTS, RIDE_DC_BUS_TRANSIENT_DURATION_MS,
    RIDE_DC_BUS_TRANSIENT_LIMIT_COUNTS, RIDE_DC_BUS_TRANSIENT_RECOVERY_DIVISOR,
};

pub const UPDATE_PERIOD_MS: u32 = 50;
pub const DC_BUS_UNDERVOLTAGE_MV: u32 = 39_000;
pub const DC_BUS_UNDERVOLTAGE_SAMPLES: u8 = 20;
const CONTROLLER_DERATE_START_DECI_C: i16 = 700;
const CONTROLLER_STOP_DECI_C: i16 = 800;
const MOTOR_DERATE_START_DECI_C: i16 = 1_000;
const MOTOR_STOP_DECI_C: i16 = 1_300;
const TRANSIENT_BUDGET_SAMPLES: u16 = (RIDE_DC_BUS_TRANSIENT_DURATION_MS / UPDATE_PERIOD_MS) as u16;
const TRANSIENT_BUDGET_CAPACITY: u16 = (RIDE_DC_BUS_TRANSIENT_LIMIT_COUNTS
    - RIDE_DC_BUS_CONTINUOUS_LIMIT_COUNTS)
    * TRANSIENT_BUDGET_SAMPLES;

const _: () = assert!(RIDE_DC_BUS_TRANSIENT_DURATION_MS.is_multiple_of(UPDATE_PERIOD_MS));
const _: () = assert!(TRANSIENT_BUDGET_CAPACITY > 0);

pub mod reason {
    pub const CONTROLLER_TEMPERATURE: u8 = 1;
    pub const MOTOR_TEMPERATURE: u8 = 1 << 1;
    pub const DC_BUS_TRANSIENT_RECOVERY: u8 = 1 << 2;
    pub const DC_BUS_UNDERVOLTAGE: u8 = 1 << 5;
    pub const LOCAL_DATA_MISSING: u8 = 1 << 6;
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct RawLocalSensors {
    pub valid: bool,
    pub bus_voltage_adc: u16,
    pub motor_temperature_adc: u16,
    pub controller_temperature_adc: u16,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct EnvironmentMonitor {
    undervoltage_count: u8,
    undervoltage_active: bool,
    last_update_ms: u32,
    latest_limit: Option<u16>,
    derating_reasons: u8,
    transient_budget: u16,
    transient_recovery_active: bool,
}

impl EnvironmentMonitor {
    pub const fn new(now_ms: u32) -> Self {
        Self {
            undervoltage_count: 0,
            undervoltage_active: false,
            last_update_ms: now_ms.wrapping_sub(UPDATE_PERIOD_MS),
            latest_limit: None,
            derating_reasons: reason::LOCAL_DATA_MISSING,
            transient_budget: TRANSIENT_BUDGET_CAPACITY,
            transient_recovery_active: false,
        }
    }

    #[inline(never)]
    #[cfg_attr(feature = "firmware", optimize(size))]
    pub fn update(
        &mut self,
        now_ms: u32,
        sensors: RawLocalSensors,
        projected_dc_current_counts: u16,
    ) -> Option<u16> {
        if now_ms.wrapping_sub(self.last_update_ms) < UPDATE_PERIOD_MS {
            return self.latest_limit;
        }
        self.last_update_ms = now_ms;
        if !sensors.valid {
            self.latest_limit = None;
            self.derating_reasons = reason::LOCAL_DATA_MISSING;
            return None;
        }

        let bus_voltage = bus_voltage_mv(sensors.bus_voltage_adc);
        self.update_undervoltage(bus_voltage);
        let Some(motor_temperature) = motor_temperature_deci_c(sensors.motor_temperature_adc)
        else {
            self.latest_limit = None;
            self.derating_reasons = reason::LOCAL_DATA_MISSING;
            return None;
        };
        let controller_temperature =
            controller_temperature_deci_c(sensors.controller_temperature_adc);
        let mut reasons = 0;
        if controller_temperature > CONTROLLER_DERATE_START_DECI_C {
            reasons |= reason::CONTROLLER_TEMPERATURE;
        }
        if motor_temperature > MOTOR_DERATE_START_DECI_C {
            reasons |= reason::MOTOR_TEMPERATURE;
        }
        let transient_limit = self.update_transient_limit(projected_dc_current_counts);
        if self.transient_recovery_active {
            reasons |= reason::DC_BUS_TRANSIENT_RECOVERY;
        }
        let mut limit = transient_limit.min(thermal_limit_counts(
            controller_temperature,
            CONTROLLER_DERATE_START_DECI_C,
            CONTROLLER_STOP_DECI_C,
        ));
        limit = limit.min(thermal_limit_counts(
            motor_temperature,
            MOTOR_DERATE_START_DECI_C,
            MOTOR_STOP_DECI_C,
        ));
        if self.undervoltage_active {
            limit = 0;
            reasons |= reason::DC_BUS_UNDERVOLTAGE;
        }
        self.derating_reasons = reasons;
        self.latest_limit = Some(limit);
        self.latest_limit
    }

    pub const fn undervoltage_active(&self) -> bool {
        self.undervoltage_active
    }

    pub const fn derating_reasons(&self) -> u8 {
        self.derating_reasons
    }

    fn update_transient_limit(&mut self, current: u16) -> u16 {
        if current > RIDE_DC_BUS_CONTINUOUS_LIMIT_COUNTS {
            if !self.transient_recovery_active {
                self.transient_budget = self
                    .transient_budget
                    .saturating_sub(current.saturating_sub(RIDE_DC_BUS_CONTINUOUS_LIMIT_COUNTS));
                self.transient_recovery_active = self.transient_budget == 0;
            }
        } else {
            let headroom = RIDE_DC_BUS_CONTINUOUS_LIMIT_COUNTS - current;
            let recovery = headroom.div_ceil(RIDE_DC_BUS_TRANSIENT_RECOVERY_DIVISOR);
            self.transient_budget = self
                .transient_budget
                .saturating_add(recovery)
                .min(TRANSIENT_BUDGET_CAPACITY);
            if self.transient_budget == TRANSIENT_BUDGET_CAPACITY {
                self.transient_recovery_active = false;
            }
        }

        if self.transient_recovery_active {
            RIDE_DC_BUS_CONTINUOUS_LIMIT_COUNTS
        } else {
            RIDE_DC_BUS_TRANSIENT_LIMIT_COUNTS
        }
    }

    fn update_undervoltage(&mut self, voltage_mv: u32) {
        if voltage_mv < DC_BUS_UNDERVOLTAGE_MV {
            self.undervoltage_count = self
                .undervoltage_count
                .saturating_add(1)
                .min(DC_BUS_UNDERVOLTAGE_SAMPLES);
            if self.undervoltage_count == DC_BUS_UNDERVOLTAGE_SAMPLES {
                self.undervoltage_active = true;
            }
        } else if self.undervoltage_count == 0 {
            self.undervoltage_active = false;
        } else {
            self.undervoltage_count = self.undervoltage_count.saturating_sub(1);
        }
    }
}

impl Default for EnvironmentMonitor {
    fn default() -> Self {
        Self::new(0)
    }
}

pub fn bus_voltage_mv(raw: u16) -> u32 {
    u32::from(raw).saturating_mul(18_975) >> 10
}

/// Project motor-terminal input current from a coherent foreground copy of
/// measured dq current and applied dq voltage. Negative terminal power is
/// battery charging during rollback and does not consume the discharge budget.
pub fn projected_dc_current_counts(
    measured_d_counts: i16,
    measured_q_counts: i16,
    applied_d_ticks: i16,
    applied_q_ticks: i16,
) -> u16 {
    let terminal_power = i32::from(measured_d_counts)
        .saturating_mul(i32::from(applied_d_ticks))
        .saturating_add(i32::from(measured_q_counts).saturating_mul(i32::from(applied_q_ticks)));
    if terminal_power <= 0 {
        return 0;
    }
    terminal_power
        .unsigned_abs()
        .saturating_mul(3)
        .div_ceil(u32::from(PWM_ARR).saturating_mul(2))
        .min(u32::from(u16::MAX)) as u16
}

pub fn controller_temperature_deci_c(raw: u16) -> i16 {
    let sensor_mv = i32::from(raw).saturating_mul(3_300) / 4_096;
    let temperature =
        250_i32.saturating_add(1_430_i32.saturating_sub(sensor_mv).saturating_mul(100) / 43);
    temperature.clamp(i32::from(i16::MIN), i32::from(i16::MAX)) as i16
}

pub fn motor_temperature_deci_c(raw: u16) -> Option<i16> {
    if !(110..=3_124).contains(&raw) {
        return None;
    }
    let millivolts = i32::from(raw).saturating_mul(3_300) / 4_096;
    let whole_degrees = match millivolts {
        2_518.. => -40,
        2_476..=2_517 => thermistor_segment(619, 262, millivolts),
        2_411..=2_475 => thermistor_segment(390, 169, millivolts),
        2_271..=2_410 => thermistor_segment(206, 93, millivolts),
        1_930..=2_270 => thermistor_segment(122, 56, millivolts),
        1_041..=1_929 => thermistor_segment(90, 39, millivolts),
        763..=1_040 => thermistor_segment(98, 47, millivolts),
        547..=762 => thermistor_segment(108, 60, millivolts),
        352..=546 => thermistor_segment(123, 87, millivolts),
        222..=351 => thermistor_segment(141, 138, millivolts),
        147..=221 => thermistor_segment(160, 227, millivolts),
        88..=146 => thermistor_segment(184, 390, millivolts),
        _ => 150,
    };
    Some(whole_degrees.saturating_mul(10).min(1_500) as i16)
}

fn thermistor_segment(intercept: i32, slope: i32, millivolts: i32) -> i32 {
    intercept.saturating_sub(slope.saturating_mul(millivolts) / 1_000)
}

fn thermal_limit_counts(temperature: i16, start: i16, stop: i16) -> u16 {
    if temperature <= start {
        return RIDE_DC_BUS_TRANSIENT_LIMIT_COUNTS;
    }
    if temperature >= stop {
        return 0;
    }
    let remaining = u32::from(stop.saturating_sub(temperature) as u16);
    let span = u32::from(stop.saturating_sub(start) as u16);
    u32::from(RIDE_DC_BUS_TRANSIENT_LIMIT_COUNTS)
        .saturating_mul(remaining)
        .checked_div(span)
        .unwrap_or_default() as u16
}

#[cfg(test)]
mod tests {
    use super::*;

    fn nominal() -> RawLocalSensors {
        RawLocalSensors {
            valid: true,
            bus_voltage_adc: 2_945,
            motor_temperature_adc: 2_000,
            controller_temperature_adc: 1_774,
        }
    }

    #[test]
    fn recovered_sensor_conversions_match_reference_points() {
        assert_eq!(bus_voltage_mv(2_945), 54_571);
        assert_eq!(controller_temperature_deci_c(1_774), 252);
        assert_eq!(motor_temperature_deci_c(3_124), Some(-400));
        assert_eq!(motor_temperature_deci_c(2_000), Some(280));
        assert_eq!(motor_temperature_deci_c(110), Some(1_500));
        assert_eq!(motor_temperature_deci_c(3_125), None);
    }

    #[test]
    fn missing_local_sensor_data_removes_drive_authority() {
        let mut monitor = EnvironmentMonitor::new(0);
        assert_eq!(monitor.update(0, RawLocalSensors::default(), 0), None);
        assert_eq!(monitor.update(50, nominal(), 0), Some(640));
    }

    #[test]
    fn local_temperatures_derate_to_zero() {
        assert_eq!(thermal_limit_counts(700, 700, 800), 640);
        assert_eq!(thermal_limit_counts(750, 700, 800), 320);
        assert_eq!(thermal_limit_counts(800, 700, 800), 0);
        assert_eq!(thermal_limit_counts(1_150, 1_000, 1_300), 320);

        let mut monitor = EnvironmentMonitor::new(0);
        let mut hot = nominal();
        hot.controller_temperature_adc = 1_162;
        assert!(monitor.update(0, hot, 0).is_some());
        assert_ne!(
            monitor.derating_reasons() & reason::CONTROLLER_TEMPERATURE,
            0
        );
    }

    #[test]
    fn undervoltage_uses_twenty_samples_and_hysteretic_recovery() {
        let mut monitor = EnvironmentMonitor::new(0);
        let mut low = nominal();
        low.bus_voltage_adc = 2_000;
        for sample in 0..19 {
            assert_ne!(monitor.update(sample * 50, low, 0), Some(0));
        }
        assert_eq!(monitor.update(19 * 50, low, 0), Some(0));
        for sample in 20..40 {
            assert_eq!(monitor.update(sample * 50, nominal(), 0), Some(0));
        }
        assert_eq!(monitor.update(40 * 50, nominal(), 0), Some(640));
        assert_eq!(monitor.derating_reasons(), 0);
    }

    #[test]
    fn missing_local_data_is_reported_with_zero_authority() {
        let mut monitor = EnvironmentMonitor::new(0);
        assert_eq!(monitor.update(0, RawLocalSensors::default(), 0), None);
        assert_eq!(monitor.derating_reasons(), reason::LOCAL_DATA_MISSING);
    }

    #[test]
    fn projected_dc_current_uses_measured_terminal_power() {
        assert_eq!(projected_dc_current_counts(10, -880, -20, -1_100), 646);
        assert_eq!(projected_dc_current_counts(0, -880, 0, 1_100), 0);
        assert_eq!(projected_dc_current_counts(100, 0, 1_000, 0), 67);
        assert_eq!(projected_dc_current_counts(0, 0, 0, -1_100), 0);
    }

    #[test]
    fn full_transient_current_expires_after_seven_seconds() {
        let mut monitor = EnvironmentMonitor::new(0);
        for sample in 0..139 {
            assert_eq!(
                monitor.update(sample * UPDATE_PERIOD_MS, nominal(), 640),
                Some(640)
            );
        }
        assert_eq!(
            monitor.update(139 * UPDATE_PERIOD_MS, nominal(), 640),
            Some(431)
        );
        assert_ne!(
            monitor.derating_reasons() & reason::DC_BUS_TRANSIENT_RECOVERY,
            0
        );
    }

    #[test]
    fn excess_current_and_continuous_headroom_change_the_budget_proportionally() {
        let mut monitor = EnvironmentMonitor::new(0);
        assert_eq!(
            monitor.update(0, nominal(), RIDE_DC_BUS_CONTINUOUS_LIMIT_COUNTS + 10),
            Some(640)
        );
        assert_eq!(monitor.transient_budget, TRANSIENT_BUDGET_CAPACITY - 10);
        assert_eq!(
            monitor.update(
                UPDATE_PERIOD_MS,
                nominal(),
                RIDE_DC_BUS_CONTINUOUS_LIMIT_COUNTS - 40
            ),
            Some(640)
        );
        assert_eq!(monitor.transient_budget, TRANSIENT_BUDGET_CAPACITY);
    }

    #[test]
    fn exhausted_transient_budget_stays_continuous_until_fully_recovered() {
        let mut monitor = EnvironmentMonitor::new(0);
        for sample in 0..140 {
            let _ = monitor.update(sample * UPDATE_PERIOD_MS, nominal(), 640);
        }

        let mut sample = 140;
        assert_eq!(
            monitor.update(sample * UPDATE_PERIOD_MS, nominal(), 0),
            Some(431)
        );
        while monitor.transient_recovery_active {
            sample += 1;
            let _ = monitor.update(sample * UPDATE_PERIOD_MS, nominal(), 0);
            assert!(sample < 500);
        }
        assert_eq!(monitor.latest_limit, Some(640));
    }
}
