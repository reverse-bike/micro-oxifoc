//! Local voltage and temperature conversion plus the ride-current envelope.

use crate::config::RIDE_DC_BUS_CURRENT_LIMIT_COUNTS;

pub const UPDATE_PERIOD_MS: u32 = 50;
pub const DC_BUS_UNDERVOLTAGE_MV: u32 = 39_000;
pub const DC_BUS_UNDERVOLTAGE_SAMPLES: u8 = 20;
const CONTROLLER_DERATE_START_DECI_C: i16 = 700;
const CONTROLLER_STOP_DECI_C: i16 = 800;
const MOTOR_DERATE_START_DECI_C: i16 = 1_000;
const MOTOR_STOP_DECI_C: i16 = 1_300;

pub mod reason {
    pub const CONTROLLER_TEMPERATURE: u8 = 1;
    pub const MOTOR_TEMPERATURE: u8 = 1 << 1;
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
}

impl EnvironmentMonitor {
    pub const fn new(now_ms: u32) -> Self {
        Self {
            undervoltage_count: 0,
            undervoltage_active: false,
            last_update_ms: now_ms.wrapping_sub(UPDATE_PERIOD_MS),
            latest_limit: None,
            derating_reasons: reason::LOCAL_DATA_MISSING,
        }
    }

    pub fn update(&mut self, now_ms: u32, sensors: RawLocalSensors) -> Option<u16> {
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
        let mut limit = thermal_limit_counts(
            controller_temperature,
            CONTROLLER_DERATE_START_DECI_C,
            CONTROLLER_STOP_DECI_C,
        )
        .min(thermal_limit_counts(
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
        return RIDE_DC_BUS_CURRENT_LIMIT_COUNTS;
    }
    if temperature >= stop {
        return 0;
    }
    let remaining = u32::from(stop.saturating_sub(temperature) as u16);
    let span = u32::from(stop.saturating_sub(start) as u16);
    u32::from(RIDE_DC_BUS_CURRENT_LIMIT_COUNTS)
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
        assert_eq!(monitor.update(0, RawLocalSensors::default()), None);
        assert_eq!(monitor.update(50, nominal()), Some(392));
    }

    #[test]
    fn local_temperatures_derate_to_zero() {
        assert_eq!(thermal_limit_counts(700, 700, 800), 392);
        assert_eq!(thermal_limit_counts(750, 700, 800), 196);
        assert_eq!(thermal_limit_counts(800, 700, 800), 0);
        assert_eq!(thermal_limit_counts(1_150, 1_000, 1_300), 196);

        let mut monitor = EnvironmentMonitor::new(0);
        let mut hot = nominal();
        hot.controller_temperature_adc = 1_162;
        assert!(monitor.update(0, hot).is_some());
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
            assert_ne!(monitor.update(sample * 50, low), Some(0));
        }
        assert_eq!(monitor.update(19 * 50, low), Some(0));
        for sample in 20..40 {
            assert_eq!(monitor.update(sample * 50, nominal()), Some(0));
        }
        assert_eq!(monitor.update(40 * 50, nominal()), Some(392));
        assert_eq!(monitor.derating_reasons(), 0);
    }

    #[test]
    fn missing_local_data_is_reported_with_zero_authority() {
        let mut monitor = EnvironmentMonitor::new(0);
        assert_eq!(monitor.update(0, RawLocalSensors::default()), None);
        assert_eq!(monitor.derating_reasons(), reason::LOCAL_DATA_MISSING);
    }
}
