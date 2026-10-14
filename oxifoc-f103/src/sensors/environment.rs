//! Local voltage and temperature conversion plus the ride-current envelope.

use crate::config::{
    ANALOG_SAMPLE_PERIOD_MS, BIKE_CONFIG, PWM_ARR, RIDE_DC_BUS_CONTINUOUS_LIMIT_COUNTS,
    RIDE_DC_BUS_TRANSIENT_DURATION_MS, RIDE_DC_BUS_TRANSIENT_LIMIT_COUNTS,
    RIDE_DC_BUS_TRANSIENT_RECOVERY_DIVISOR, RIDE_PHASE_CURRENT_LIMIT_COUNTS,
    VBUS_MILLIVOLTS_Q10_PER_COUNT,
};

pub const UPDATE_PERIOD_MS: u32 = 50;
pub const DC_BUS_UNDERVOLTAGE_MV: u32 = BIKE_CONFIG.battery.minimum_bus_voltage_millivolts;
pub const DC_BUS_UNDERVOLTAGE_SAMPLES: u8 =
    (BIKE_CONFIG.battery.undervoltage_debounce_milliseconds / UPDATE_PERIOD_MS) as u8;
pub const DC_BUS_OVERVOLTAGE_MV: u32 = BIKE_CONFIG.battery.maximum_bus_voltage_millivolts;
pub const DC_BUS_OVERVOLTAGE_HYSTERESIS_MV: u32 =
    BIKE_CONFIG.battery.overvoltage_hysteresis_millivolts;
/// Overvoltage is checked on every 1 ms ride pass rather than the 50 ms
/// envelope period, but counts only fresh bus-voltage conversions, so one
/// sample is one analog sample period.
pub const DC_BUS_OVERVOLTAGE_SAMPLES: u8 =
    (BIKE_CONFIG.battery.overvoltage_debounce_milliseconds / ANALOG_SAMPLE_PERIOD_MS) as u8;
/// Q16 thermal fraction granting the full current ceiling.
pub const THERMAL_FRACTION_ONE: u32 = 1 << 16;
pub const CONTROLLER_DERATE_START_DECI_C: i16 =
    BIKE_CONFIG.thermal.controller_derate_start_celsius * 10;
pub const CONTROLLER_STOP_DECI_C: i16 = BIKE_CONFIG.thermal.controller_stop_celsius * 10;
pub const MOTOR_DERATE_START_DECI_C: i16 = BIKE_CONFIG.thermal.motor_derate_start_celsius * 10;
pub const MOTOR_STOP_DECI_C: i16 = BIKE_CONFIG.thermal.motor_stop_celsius * 10;
const TRANSIENT_BUDGET_SAMPLES: u16 = (RIDE_DC_BUS_TRANSIENT_DURATION_MS / UPDATE_PERIOD_MS) as u16;
const TRANSIENT_BUDGET_CAPACITY: u16 = (RIDE_DC_BUS_TRANSIENT_LIMIT_COUNTS
    - RIDE_DC_BUS_CONTINUOUS_LIMIT_COUNTS)
    * TRANSIENT_BUDGET_SAMPLES;

const _: () = assert!(RIDE_DC_BUS_TRANSIENT_DURATION_MS.is_multiple_of(UPDATE_PERIOD_MS));
const _: () = assert!(
    BIKE_CONFIG
        .battery
        .undervoltage_debounce_milliseconds
        .is_multiple_of(UPDATE_PERIOD_MS)
);
const _: () = assert!(
    BIKE_CONFIG
        .battery
        .overvoltage_debounce_milliseconds
        .is_multiple_of(ANALOG_SAMPLE_PERIOD_MS)
);
const _: () = assert!(
    BIKE_CONFIG.battery.overvoltage_debounce_milliseconds / ANALOG_SAMPLE_PERIOD_MS > 0
        && BIKE_CONFIG.battery.overvoltage_debounce_milliseconds / ANALOG_SAMPLE_PERIOD_MS
            <= u8::MAX as u32
);
const _: () = assert!(RIDE_DC_BUS_TRANSIENT_DURATION_MS <= u16::MAX as u32);
const _: () = assert!(TRANSIENT_BUDGET_CAPACITY > 0);

pub mod reason {
    pub const CONTROLLER_TEMPERATURE: u8 = 1;
    pub const MOTOR_TEMPERATURE: u8 = 1 << 1;
    pub const DC_BUS_TRANSIENT_RECOVERY: u8 = 1 << 2;
    pub const DC_BUS_OVERVOLTAGE: u8 = 1 << 3;
    pub const DC_BUS_UNDERVOLTAGE: u8 = 1 << 5;
    pub const LOCAL_DATA_MISSING: u8 = 1 << 6;
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct RawLocalSensors {
    pub valid: bool,
    pub bus_voltage_adc: u16,
    pub motor_temperature_adc: u16,
    pub controller_temperature_adc: u16,
    /// Time the analog conversion was captured; a repeated timestamp is the
    /// same conversion observed again.
    pub sampled_at_ms: u32,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct EnvironmentMonitor {
    undervoltage_count: u8,
    undervoltage_active: bool,
    overvoltage_count: u8,
    overvoltage_active: bool,
    last_overvoltage_sample_ms: u32,
    last_update_ms: u32,
    latest_limit: Option<u16>,
    phase_current_limit: u16,
    derating_reasons: u8,
    transient_budget: u16,
    transient_recovery_active: bool,
}

impl EnvironmentMonitor {
    pub const fn new(now_ms: u32) -> Self {
        Self {
            undervoltage_count: 0,
            undervoltage_active: false,
            overvoltage_count: 0,
            overvoltage_active: false,
            last_overvoltage_sample_ms: now_ms.wrapping_sub(1),
            last_update_ms: now_ms.wrapping_sub(UPDATE_PERIOD_MS),
            latest_limit: None,
            phase_current_limit: 0,
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
        let bus_voltage = bus_voltage_mv(sensors.bus_voltage_adc);
        if sensors.valid && sensors.sampled_at_ms != self.last_overvoltage_sample_ms {
            self.last_overvoltage_sample_ms = sensors.sampled_at_ms;
            self.update_overvoltage(bus_voltage);
        }
        if now_ms.wrapping_sub(self.last_update_ms) < UPDATE_PERIOD_MS {
            return self.limit();
        }
        self.last_update_ms = now_ms;
        self.latest_limit = None;
        self.phase_current_limit = 0;
        self.derating_reasons = reason::LOCAL_DATA_MISSING;
        if !sensors.valid {
            return None;
        }

        self.update_undervoltage(bus_voltage);
        let motor_temperature = motor_temperature_deci_c(sensors.motor_temperature_adc)?;
        let controller_temperature =
            controller_temperature_deci_c(sensors.controller_temperature_adc);
        let mut reasons = 0;
        if derating(controller_temperature, CONTROLLER_DERATE_START_DECI_C) {
            reasons |= reason::CONTROLLER_TEMPERATURE;
        }
        if derating(motor_temperature, MOTOR_DERATE_START_DECI_C) {
            reasons |= reason::MOTOR_TEMPERATURE;
        }
        let transient_limit = self.update_transient_limit(projected_dc_current_counts);
        if self.transient_recovery_active {
            reasons |= reason::DC_BUS_TRANSIENT_RECOVERY;
        }
        let thermal_fraction = thermal_fraction_q16(
            controller_temperature,
            CONTROLLER_DERATE_START_DECI_C,
            CONTROLLER_STOP_DECI_C,
        )
        .min(thermal_fraction_q16(
            motor_temperature,
            MOTOR_DERATE_START_DECI_C,
            MOTOR_STOP_DECI_C,
        ));
        self.phase_current_limit =
            scale_by_thermal_fraction(RIDE_PHASE_CURRENT_LIMIT_COUNTS, thermal_fraction);
        let mut limit = transient_limit.min(scale_by_thermal_fraction(
            RIDE_DC_BUS_TRANSIENT_LIMIT_COUNTS,
            thermal_fraction,
        ));
        if self.undervoltage_active {
            limit = 0;
            reasons |= reason::DC_BUS_UNDERVOLTAGE;
        }
        self.derating_reasons = reasons;
        self.latest_limit = Some(limit);
        self.limit()
    }

    /// Overvoltage is applied on every 1 ms pass over the 50 ms envelope.
    fn limit(&self) -> Option<u16> {
        if self.overvoltage_active {
            self.latest_limit.map(|_| 0)
        } else {
            self.latest_limit
        }
    }

    pub const fn undervoltage_active(&self) -> bool {
        self.undervoltage_active
    }

    pub const fn overvoltage_active(&self) -> bool {
        self.overvoltage_active
    }

    /// Thermally derated phase-current ceiling in current counts.
    pub const fn phase_current_limit_counts(&self) -> u16 {
        self.phase_current_limit
    }

    pub const fn derating_reasons(&self) -> u8 {
        if self.overvoltage_active {
            self.derating_reasons | reason::DC_BUS_OVERVOLTAGE
        } else {
            self.derating_reasons
        }
    }

    /// Remaining time at the full transient-minus-continuous current excess.
    pub const fn transient_budget_ms(&self) -> u16 {
        let peak_excess =
            (RIDE_DC_BUS_TRANSIENT_LIMIT_COUNTS - RIDE_DC_BUS_CONTINUOUS_LIMIT_COUNTS) as u32;
        (self.transient_budget as u32 * UPDATE_PERIOD_MS / peak_excess) as u16
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

    fn update_overvoltage(&mut self, voltage_mv: u32) {
        if voltage_mv > DC_BUS_OVERVOLTAGE_MV {
            self.overvoltage_count = self
                .overvoltage_count
                .saturating_add(1)
                .min(DC_BUS_OVERVOLTAGE_SAMPLES);
            if self.overvoltage_count == DC_BUS_OVERVOLTAGE_SAMPLES {
                self.overvoltage_active = true;
            }
        } else if voltage_mv <= DC_BUS_OVERVOLTAGE_MV - DC_BUS_OVERVOLTAGE_HYSTERESIS_MV {
            if self.overvoltage_count == 0 {
                self.overvoltage_active = false;
            } else {
                self.overvoltage_count -= 1;
            }
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
    (u32::from(raw) * VBUS_MILLIVOLTS_Q10_PER_COUNT) >> 10
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

/// Drive current is reduced only above the derating start temperature.
pub const fn derating(temperature_deci_c: i16, start_deci_c: i16) -> bool {
    temperature_deci_c > start_deci_c
}

/// Drive current reaches zero at the stop temperature.
pub const fn stopped(temperature_deci_c: i16, stop_deci_c: i16) -> bool {
    temperature_deci_c >= stop_deci_c
}

/// Q16 share of the current ceiling: full at or below the start temperature,
/// zero at the stop temperature, and linear between them.
fn thermal_fraction_q16(temperature: i16, start: i16, stop: i16) -> u32 {
    if !derating(temperature, start) {
        return THERMAL_FRACTION_ONE;
    }
    if stopped(temperature, stop) {
        return 0;
    }
    let remaining = u32::from(stop.saturating_sub(temperature) as u16);
    let span = u32::from(stop.saturating_sub(start) as u16);
    (remaining << 16).checked_div(span).unwrap_or_default()
}

fn scale_by_thermal_fraction(limit: u16, fraction_q16: u32) -> u16 {
    ((u32::from(limit) * fraction_q16.min(THERMAL_FRACTION_ONE)) >> 16) as u16
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
            sampled_at_ms: 0,
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
        assert_eq!(
            monitor.update(UPDATE_PERIOD_MS, nominal(), 0),
            Some(RIDE_DC_BUS_TRANSIENT_LIMIT_COUNTS)
        );
    }

    #[test]
    fn local_temperatures_derate_to_zero() {
        let thermal_limit_counts = |temperature, start, stop| {
            scale_by_thermal_fraction(
                RIDE_DC_BUS_TRANSIENT_LIMIT_COUNTS,
                thermal_fraction_q16(temperature, start, stop),
            )
        };
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
        assert_eq!(
            monitor.update(40 * UPDATE_PERIOD_MS, nominal(), 0),
            Some(RIDE_DC_BUS_TRANSIENT_LIMIT_COUNTS)
        );
        assert_eq!(monitor.derating_reasons(), 0);
    }

    fn bus_adc(millivolts: u32) -> u16 {
        ((millivolts << 10).div_ceil(VBUS_MILLIVOLTS_Q10_PER_COUNT)) as u16
    }

    /// One analog conversion observed on every 1 ms ride pass until the
    /// next conversion replaces it.
    fn observe_sample(
        monitor: &mut EnvironmentMonitor,
        now: &mut u32,
        sensors: RawLocalSensors,
    ) -> Option<u16> {
        let sample = RawLocalSensors {
            sampled_at_ms: *now,
            ..sensors
        };
        let mut limit = None;
        for _ in 0..ANALOG_SAMPLE_PERIOD_MS {
            limit = monitor.update(*now, sample, 0);
            *now += 1;
        }
        limit
    }

    #[test]
    fn overvoltage_debounce_is_thirty_milliseconds_of_fresh_samples() {
        assert_eq!(DC_BUS_OVERVOLTAGE_SAMPLES, 3);
        assert_eq!(
            u32::from(DC_BUS_OVERVOLTAGE_SAMPLES) * ANALOG_SAMPLE_PERIOD_MS,
            BIKE_CONFIG.battery.overvoltage_debounce_milliseconds
        );
    }

    #[test]
    fn one_noisy_conversion_seen_on_many_passes_does_not_trip_overvoltage() {
        let mut monitor = EnvironmentMonitor::new(0);
        let mut now = 0;
        assert!(observe_sample(&mut monitor, &mut now, nominal()).is_some());
        let mut high = nominal();
        high.bus_voltage_adc = bus_adc(DC_BUS_OVERVOLTAGE_MV + 100);
        assert_ne!(observe_sample(&mut monitor, &mut now, high), Some(0));
        assert!(!monitor.overvoltage_active());
        assert_ne!(observe_sample(&mut monitor, &mut now, nominal()), Some(0));
        assert!(!monitor.overvoltage_active());
    }

    #[test]
    fn overvoltage_removes_authority_within_the_debounce_and_clears_with_hysteresis() {
        let mut monitor = EnvironmentMonitor::new(0);
        let mut now = 0;
        assert!(observe_sample(&mut monitor, &mut now, nominal()).is_some());
        let mut high = nominal();
        high.bus_voltage_adc = bus_adc(DC_BUS_OVERVOLTAGE_MV + 100);
        assert!(bus_voltage_mv(high.bus_voltage_adc) > DC_BUS_OVERVOLTAGE_MV);
        for _ in 1..DC_BUS_OVERVOLTAGE_SAMPLES {
            assert_ne!(observe_sample(&mut monitor, &mut now, high), Some(0));
        }
        let tripping = RawLocalSensors {
            sampled_at_ms: now,
            ..high
        };
        assert_eq!(monitor.update(now, tripping, 0), Some(0));
        assert!(monitor.overvoltage_active());
        assert_ne!(monitor.derating_reasons() & reason::DC_BUS_OVERVOLTAGE, 0);
        now += ANALOG_SAMPLE_PERIOD_MS;

        let mut within_hysteresis = nominal();
        within_hysteresis.bus_voltage_adc =
            bus_adc(DC_BUS_OVERVOLTAGE_MV - DC_BUS_OVERVOLTAGE_HYSTERESIS_MV + 100);
        for _ in 0..10 {
            assert_eq!(
                observe_sample(&mut monitor, &mut now, within_hysteresis),
                Some(0)
            );
        }
        for _ in 0..DC_BUS_OVERVOLTAGE_SAMPLES {
            assert_eq!(observe_sample(&mut monitor, &mut now, nominal()), Some(0));
        }
        now += UPDATE_PERIOD_MS;
        let released = RawLocalSensors {
            sampled_at_ms: now,
            ..nominal()
        };
        assert_eq!(
            monitor.update(now, released, 0),
            Some(RIDE_DC_BUS_TRANSIENT_LIMIT_COUNTS)
        );
        assert_eq!(monitor.derating_reasons(), 0);
    }

    #[test]
    fn thermal_derating_scales_the_phase_current_ceiling() {
        let mut monitor = EnvironmentMonitor::new(0);
        assert_eq!(monitor.phase_current_limit_counts(), 0);
        monitor.update(0, nominal(), 0);
        assert_eq!(
            monitor.phase_current_limit_counts(),
            RIDE_PHASE_CURRENT_LIMIT_COUNTS
        );
        assert_eq!(thermal_fraction_q16(700, 700, 800), THERMAL_FRACTION_ONE);
        assert_eq!(
            thermal_fraction_q16(750, 700, 800),
            THERMAL_FRACTION_ONE / 2
        );
        assert_eq!(thermal_fraction_q16(800, 700, 800), 0);
        assert_eq!(
            scale_by_thermal_fraction(RIDE_PHASE_CURRENT_LIMIT_COUNTS, THERMAL_FRACTION_ONE / 2),
            RIDE_PHASE_CURRENT_LIMIT_COUNTS / 2
        );

        let mut warm = nominal();
        warm.controller_temperature_adc = 1_508;
        assert!(derating(
            controller_temperature_deci_c(1_508),
            CONTROLLER_DERATE_START_DECI_C
        ));
        let limit = monitor.update(UPDATE_PERIOD_MS, warm, 0).unwrap();
        let phase_limit = monitor.phase_current_limit_counts();
        assert!(phase_limit > 0 && phase_limit < RIDE_PHASE_CURRENT_LIMIT_COUNTS);
        let expected = u32::from(phase_limit) * u32::from(RIDE_DC_BUS_TRANSIENT_LIMIT_COUNTS)
            / u32::from(RIDE_PHASE_CURRENT_LIMIT_COUNTS);
        assert!(u32::from(limit).abs_diff(expected) <= 1);

        assert_eq!(
            monitor.update(2 * UPDATE_PERIOD_MS, RawLocalSensors::default(), 0),
            None
        );
        assert_eq!(monitor.phase_current_limit_counts(), 0);
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
    fn full_transient_current_expires_when_the_budget_is_consumed() {
        let mut monitor = EnvironmentMonitor::new(0);
        assert_eq!(
            monitor.transient_budget_ms(),
            RIDE_DC_BUS_TRANSIENT_DURATION_MS as u16
        );
        for sample in 0..u32::from(TRANSIENT_BUDGET_SAMPLES - 1) {
            assert_eq!(
                monitor.update(
                    sample * UPDATE_PERIOD_MS,
                    nominal(),
                    RIDE_DC_BUS_TRANSIENT_LIMIT_COUNTS
                ),
                Some(RIDE_DC_BUS_TRANSIENT_LIMIT_COUNTS)
            );
        }
        assert_eq!(
            monitor.update(
                u32::from(TRANSIENT_BUDGET_SAMPLES - 1) * UPDATE_PERIOD_MS,
                nominal(),
                RIDE_DC_BUS_TRANSIENT_LIMIT_COUNTS
            ),
            Some(RIDE_DC_BUS_CONTINUOUS_LIMIT_COUNTS)
        );
        assert_eq!(monitor.transient_budget_ms(), 0);
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
            Some(RIDE_DC_BUS_TRANSIENT_LIMIT_COUNTS)
        );
        assert_eq!(monitor.transient_budget, TRANSIENT_BUDGET_CAPACITY - 10);
        assert_eq!(
            monitor.update(
                UPDATE_PERIOD_MS,
                nominal(),
                RIDE_DC_BUS_CONTINUOUS_LIMIT_COUNTS - 40
            ),
            Some(RIDE_DC_BUS_TRANSIENT_LIMIT_COUNTS)
        );
        let recovery = 40_u16.div_ceil(RIDE_DC_BUS_TRANSIENT_RECOVERY_DIVISOR);
        assert_eq!(
            monitor.transient_budget,
            TRANSIENT_BUDGET_CAPACITY - 10 + recovery
        );
    }

    #[test]
    fn repeated_pull_budget_reaches_continuous_before_the_third_pull_ends() {
        fn run_samples(
            monitor: &mut EnvironmentMonitor,
            sample: &mut u32,
            current: u16,
            count: u32,
        ) {
            for _ in 0..count {
                let _ = monitor.update(*sample * UPDATE_PERIOD_MS, nominal(), current);
                *sample += 1;
            }
        }

        let mut monitor = EnvironmentMonitor::new(0);
        let mut sample = 0;
        run_samples(&mut monitor, &mut sample, 550, 160);
        run_samples(&mut monitor, &mut sample, 0, 600);
        run_samples(&mut monitor, &mut sample, 570, 120);
        run_samples(&mut monitor, &mut sample, 0, 170);

        for _ in 0..168 {
            assert_eq!(
                monitor.update(sample * UPDATE_PERIOD_MS, nominal(), 560),
                Some(RIDE_DC_BUS_TRANSIENT_LIMIT_COUNTS)
            );
            sample += 1;
        }
        assert_eq!(
            monitor.update(sample * UPDATE_PERIOD_MS, nominal(), 560),
            Some(RIDE_DC_BUS_CONTINUOUS_LIMIT_COUNTS)
        );
        assert_eq!(monitor.transient_budget_ms(), 0);
    }

    #[test]
    fn exhausted_transient_budget_stays_continuous_until_fully_recovered() {
        let mut monitor = EnvironmentMonitor::new(0);
        for sample in 0..u32::from(TRANSIENT_BUDGET_SAMPLES) {
            let _ = monitor.update(
                sample * UPDATE_PERIOD_MS,
                nominal(),
                RIDE_DC_BUS_TRANSIENT_LIMIT_COUNTS,
            );
        }

        let recovery_per_sample =
            RIDE_DC_BUS_CONTINUOUS_LIMIT_COUNTS.div_ceil(RIDE_DC_BUS_TRANSIENT_RECOVERY_DIVISOR);
        let recovery_samples = TRANSIENT_BUDGET_CAPACITY.div_ceil(recovery_per_sample);
        let expected_final_sample =
            u32::from(TRANSIENT_BUDGET_SAMPLES - 1) + u32::from(recovery_samples);
        let mut sample = u32::from(TRANSIENT_BUDGET_SAMPLES);
        assert_eq!(
            monitor.update(sample * UPDATE_PERIOD_MS, nominal(), 0),
            Some(RIDE_DC_BUS_CONTINUOUS_LIMIT_COUNTS)
        );
        while monitor.transient_recovery_active {
            sample += 1;
            let _ = monitor.update(sample * UPDATE_PERIOD_MS, nominal(), 0);
            assert!(sample <= expected_final_sample);
        }
        assert_eq!(sample, expected_final_sample);
        assert_eq!(
            monitor.latest_limit,
            Some(RIDE_DC_BUS_TRANSIENT_LIMIT_COUNTS)
        );
    }
}
