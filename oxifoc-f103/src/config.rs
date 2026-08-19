//! Board constants and power-stage interlock for the recovered F103RE design.
//!
//! This module contains the fixed hardware map and the conservative ride
//! envelope selected for this controller and motor. Current offsets remain a
//! per-boot measurement; the Hall boundaries and current limits below are
//! explicit, reviewed application constants.

use oxifoc_core::foc::{Fixed, hall_sensor::HallGeometry};

pub const SYSCLK_HZ: u32 = 72_000_000;
pub const APPLICATION_FLASH_ORIGIN: u32 = 0x0800_3800;
pub const PCLK1_HZ: u32 = 36_000_000;
pub const APB1_TIMER_HZ: u32 = 72_000_000;
pub const ADC_HZ: u32 = 12_000_000;

pub const PWM_HZ: u32 = 16_000;
pub const PWM_ARR: u16 = 2_250;
pub const PWM_NEUTRAL: u16 = PWM_ARR / 2;
pub const PWM_DEAD_TIME_TICKS: u8 = 25;
pub const PWM_SAMPLE_CC4: u16 = 2_248;
// OxiFOC's normalized t_dead*f_pwm factor expressed in this controller's
// phase-voltage tick domain: (25 / 36 MHz) * 16 kHz * (2 * 2,250 / 3).
pub const FOC_DEAD_TIME_COMP_NUMERATOR: i32 = 50;
pub const FOC_DEAD_TIME_COMP_DENOMINATOR: i32 = 3;
pub const FOC_DEAD_TIME_COMP_TICKS: Fixed =
    Fixed::ratio(FOC_DEAD_TIME_COMP_NUMERATOR, FOC_DEAD_TIME_COMP_DENOMINATOR);
// The S310 application uses a 1,273-tick voltage circle and approximately
// 22..2,227 timer compares with this same 2,250-tick PWM period.
pub const FOC_PHASE_LIMIT_TICKS: u16 = 1_103;
pub const FOC_HARD_PHASE_LIMIT_TICKS: u16 = 1_103;
pub const FOC_VECTOR_LIMIT_TICKS: Fixed = Fixed::from_integer(1_273);
pub const CURRENT_PI_PROPORTIONAL_GAIN: Fixed = Fixed::ratio(512, 1_024);
pub const CURRENT_PI_INTEGRAL_GAIN_PER_CYCLE: Fixed = Fixed::ratio(605, 16_384);
pub const TARGET_RAMP_CYCLES_PER_STEP: u8 = (PWM_HZ / 1_000) as u8;
pub const TARGET_RAMP_COUNTS_PER_STEP: i32 = 4;

pub const HALL_TIMER_HZ: u32 = 1_000_000;
pub const HALL_TIMER_PRESCALER: u16 = (APB1_TIMER_HZ / HALL_TIMER_HZ - 1) as u16;
pub static HALL_GEOMETRY: HallGeometry = HallGeometry::new(
    [5, 1, 3, 2, 6, 4],
    [5_699, 16_526, 26_499, 37_754, 49_151, 59_124],
    -1,
);

// Loaded terminal power balance gives approximately 94--100 mA per phase-ADC
// count. Inverter losses bias that estimate upward, so 100 mA/count remains a
// nominal conversion; the protection and current-loop envelopes stay in the
// directly observed ADC-count domain.
pub const PHASE_CURRENT_MA_PER_ADC_COUNT: i32 = 100;
pub const PHASE_CURRENT_AMPS_PER_ADC_COUNT: Fixed =
    Fixed::ratio(PHASE_CURRENT_MA_PER_ADC_COUNT, 1_000);
// Effective loaded terminal model. These values reproduce 4.3 mV/count of
// resistive drop and 7.5 uH*A/count of inductive flux in the observer's input
// domain without claiming an independently measured winding R or L.
pub const MOTOR_PHASE_RESISTANCE_OHMS: Fixed = Fixed::ratio(43, 1_000);
pub const MOTOR_PHASE_INDUCTANCE_MILLIHENRIES: Fixed = Fixed::ratio(75, 1_000);
pub const MOTOR_FLUX_LINKAGE_MILLIWEBERS: Fixed = Fixed::ratio(134, 10);
// 75 uH * 0.10 A/count = 0.0075 mWb/count, rounded to Q16.16.
pub const MOTOR_INDUCTIVE_FLUX_MWB_PER_COUNT_BITS: i32 = 492;
pub const OBSERVER_BLEND_LOW_ERPM: i32 = 3_000;
pub const OBSERVER_BLEND_HIGH_ERPM: i32 = 6_000;
pub const PHASE_CURRENT_TRIP_COUNTS: u16 = 1_344;
pub const RIDE_PHASE_CURRENT_LIMIT_COUNTS: u16 = 838;
pub const RIDE_DC_BUS_CURRENT_LIMIT_COUNTS: u16 = 480;
pub const VBUS_UV_PER_COUNT: u32 = 18_530;

const _: () = assert!(RIDE_PHASE_CURRENT_LIMIT_COUNTS < PHASE_CURRENT_TRIP_COUNTS);
const _: () =
    assert!((PHASE_CURRENT_TRIP_COUNTS as u32) * 5 >= (RIDE_PHASE_CURRENT_LIMIT_COUNTS as u32) * 8);
const _: () = assert!(FOC_PHASE_LIMIT_TICKS <= FOC_HARD_PHASE_LIMIT_TICKS);
const _: () = assert!(TARGET_RAMP_CYCLES_PER_STEP > 0);
const _: () = assert!(OBSERVER_BLEND_LOW_ERPM > 0);
const _: () = assert!(OBSERVER_BLEND_HIGH_ERPM > OBSERVER_BLEND_LOW_ERPM);
const _: () = assert!(PWM_NEUTRAL + FOC_HARD_PHASE_LIMIT_TICKS < PWM_SAMPLE_CC4);
const _: () = assert!(PWM_SAMPLE_CC4 < PWM_ARR);

pub const THROTTLE_ADC_CHANNEL: u8 = 15;
pub const THROTTLE_GPIO_PORT: u8 = b'C';
pub const THROTTLE_GPIO_PIN: u8 = 5;
pub const MOTOR_TEMPERATURE_ADC_CHANNEL: u8 = 13;
pub const MOTOR_TEMPERATURE_GPIO_PIN: u8 = 3;
pub const VBUS_ADC_CHANNEL: u8 = 8;
pub const VBUS_GPIO_PORT: u8 = b'B';
pub const VBUS_GPIO_PIN: u8 = 0;
pub const UNUSED_TORQUE_ADC_CHANNEL: u8 = 5;
pub const UNUSED_TORQUE_GPIO_PIN: u8 = 5;
pub const CONTROLLER_TEMPERATURE_ADC_CHANNEL: u8 = 16;
pub const REGULAR_ADC_CHANNELS: [u8; 5] = [
    MOTOR_TEMPERATURE_ADC_CHANNEL,
    THROTTLE_ADC_CHANNEL,
    VBUS_ADC_CHANNEL,
    UNUSED_TORQUE_ADC_CHANNEL,
    CONTROLLER_TEMPERATURE_ADC_CHANNEL,
];
pub const BRAKE_GPIO_PORT: u8 = b'C';
pub const BRAKE_GPIO_PIN: u8 = 4;
pub const WHEEL_SPEED_GPIO_PORT: u8 = b'B';
pub const WHEEL_SPEED_GPIO_PIN: u8 = 4;
pub const CADENCE_GPIO_PORT: u8 = b'B';
pub const CADENCE_GPIO_PIN: u8 = 9;

pub const CAN_BITRATE: u32 = 250_000;
pub const CAN_PRESCALER: u16 = 12;
pub const CAN_BS1_TQ: u8 = 8;
pub const CAN_BS2_TQ: u8 = 3;
pub const CAN_SJW_TQ: u8 = 1;

/// TIM1 updates at both extrema in center-aligned mode. The completed CC4
/// injected sample is consumed only at underflow, where DIR reads clear.
pub const fn sample_injected_on_timer_update(timer_counting_down: bool) -> bool {
    !timer_counting_down
}

/// Q16.16 phase volts represented by one commanded PWM timer tick.
pub const fn observer_volts_per_pwm_tick_bits(bus_voltage_mv: u32) -> i32 {
    // Reduce 65536/1000 by eight before multiplying. The board ADC cannot
    // report above 75.9 V, so this exact u32 form cannot overflow.
    let bounded_mv = if bus_voltage_mv > 75_900 {
        75_900
    } else {
        bus_voltage_mv
    };
    (bounded_mv * 8_192 / (PWM_ARR as u32 * 125)) as i32
}

pub const CAN_RIDE_TUNING_ID: u16 = 0x2f1;
pub const CAN_STOP_CALIBRATION_ID: u16 = 0x2f2;
pub const CAN_FOC_STATUS_ID: u16 = 0x2f7;

pub const fn regular_adc_sequence_register_1() -> u32 {
    (REGULAR_ADC_CHANNELS.len() as u32 - 1) << 20
}

pub const fn regular_adc_sequence_register_3() -> u32 {
    REGULAR_ADC_CHANNELS[0] as u32
        | ((REGULAR_ADC_CHANNELS[1] as u32) << 5)
        | ((REGULAR_ADC_CHANNELS[2] as u32) << 10)
        | ((REGULAR_ADC_CHANNELS[3] as u32) << 15)
        | ((REGULAR_ADC_CHANNELS[4] as u32) << 20)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CanTiming {
    pub prescaler: u16,
    pub bs1_tq: u8,
    pub bs2_tq: u8,
    pub sjw_tq: u8,
}

impl CanTiming {
    pub const F103_250K: Self = Self {
        prescaler: CAN_PRESCALER,
        bs1_tq: CAN_BS1_TQ,
        bs2_tq: CAN_BS2_TQ,
        sjw_tq: CAN_SJW_TQ,
    };

    pub const fn bitrate(self, peripheral_clock_hz: u32) -> u32 {
        peripheral_clock_hz / self.prescaler as u32 / (1 + self.bs1_tq as u32 + self.bs2_tq as u32)
    }
}

/// Prerequisites for energising the inverter. Every field starts false, so a
/// reset, malformed CAN frame, or incomplete commissioning leaves PA2 high
/// (power stage disabled).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SafetyInterlock {
    pub current_offsets_valid: bool,
    pub hall_calibration_valid: bool,
    pub command_fresh: bool,
    pub break_active: bool,
    pub software_fault_active: bool,
}

impl SafetyInterlock {
    #[inline]
    pub const fn may_enable_power_stage(self) -> bool {
        self.current_offsets_valid
            && self.hall_calibration_valid
            && self.command_fresh
            && !self.break_active
            && !self.software_fault_active
    }

    #[inline]
    pub fn note_command_timeout(&mut self) {
        self.command_fresh = false;
    }

    #[inline]
    pub fn note_break(&mut self) {
        self.break_active = true;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use oxifoc_core::foc::{
        AlphaBeta,
        svpwm::space_vector_pwm_ticks,
        transforms::inverse_park,
        trig::{CordicSinCos, SinCos},
    };

    #[test]
    fn can_timing_matches_the_recovered_bus() {
        assert_eq!(CanTiming::F103_250K.bitrate(PCLK1_HZ), CAN_BITRATE);
    }

    #[test]
    fn interlock_defaults_to_safe() {
        let mut interlock = SafetyInterlock {
            current_offsets_valid: true,
            hall_calibration_valid: true,
            command_fresh: true,
            ..SafetyInterlock::default()
        };
        assert!(interlock.may_enable_power_stage());
        interlock.note_break();
        assert!(!interlock.may_enable_power_stage());
    }

    #[test]
    fn injected_sample_is_consumed_only_at_underflow() {
        assert!(sample_injected_on_timer_update(false));
        assert!(!sample_injected_on_timer_update(true));
    }

    #[test]
    fn observer_voltage_scale_uses_the_live_bus_and_pwm_period() {
        assert_eq!(observer_volts_per_pwm_tick_bits(0), 0);
        assert_eq!(observer_volts_per_pwm_tick_bits(52_300), 1_523);
        assert_eq!(observer_volts_per_pwm_tick_bits(100_000), 2_210);
    }

    #[test]
    fn hall_geometry_matches_the_hardware_validated_stock_table() {
        assert_eq!(HALL_GEOMETRY.electrical_states(), &[5, 1, 3, 2, 6, 4]);
        assert_eq!(
            HALL_GEOMETRY.boundaries_q16(),
            &[5_699, 16_526, 26_499, 37_754, 49_151, 59_124]
        );
    }

    #[test]
    fn regular_adc_rank_contract_matches_the_recovered_board() {
        assert_eq!(REGULAR_ADC_CHANNELS, [13, 15, 8, 5, 16]);
        assert_eq!(regular_adc_sequence_register_1(), 0x0040_0000);
        assert_eq!(regular_adc_sequence_register_3(), 0x0102_a1ed);
    }

    #[test]
    fn ride_current_envelope_matches_the_full_power_configuration() {
        assert_eq!(RIDE_PHASE_CURRENT_LIMIT_COUNTS, 838);
        assert_eq!(RIDE_DC_BUS_CURRENT_LIMIT_COUNTS, 480);
        assert_eq!(PHASE_CURRENT_TRIP_COUNTS, 1_344);
        assert_eq!(
            i32::from(RIDE_PHASE_CURRENT_LIMIT_COUNTS) * PHASE_CURRENT_MA_PER_ADC_COUNT,
            83_800
        );
        assert_eq!(
            i32::from(RIDE_DC_BUS_CURRENT_LIMIT_COUNTS) * PHASE_CURRENT_MA_PER_ADC_COUNT,
            48_000
        );
        assert!(
            u32::from(PHASE_CURRENT_TRIP_COUNTS) * 5
                >= u32::from(RIDE_PHASE_CURRENT_LIMIT_COUNTS) * 8
        );
    }

    #[test]
    fn observer_model_uses_the_loaded_terminal_fit_in_the_adc_count_domain() {
        assert_eq!(PHASE_CURRENT_MA_PER_ADC_COUNT, 100);
        assert_eq!(PHASE_CURRENT_AMPS_PER_ADC_COUNT, Fixed::ratio(1, 10));
        assert_eq!(MOTOR_PHASE_RESISTANCE_OHMS, Fixed::ratio(43, 1_000));
        assert_eq!(MOTOR_PHASE_INDUCTANCE_MILLIHENRIES, Fixed::ratio(75, 1_000));
        assert_eq!(MOTOR_FLUX_LINKAGE_MILLIWEBERS, Fixed::ratio(134, 10));
        assert_eq!(MOTOR_INDUCTIVE_FLUX_MWB_PER_COUNT_BITS, 492);

        let resistive_volts_per_count =
            MOTOR_PHASE_RESISTANCE_OHMS * PHASE_CURRENT_AMPS_PER_ADC_COUNT;
        assert!(
            (resistive_volts_per_count.to_bits() - Fixed::ratio(43, 10_000).to_bits()).abs() <= 2
        );
        let inductive_flux_mwb_per_count =
            MOTOR_PHASE_INDUCTANCE_MILLIHENRIES * PHASE_CURRENT_AMPS_PER_ADC_COUNT;
        assert!(
            (inductive_flux_mwb_per_count.to_bits() - MOTOR_INDUCTIVE_FLUX_MWB_PER_COUNT_BITS)
                .abs()
                <= 1
        );
    }

    #[test]
    fn voltage_envelope_matches_the_stock_f103_circle() {
        assert_eq!(FOC_VECTOR_LIMIT_TICKS.integer(), 1_273);
        assert_eq!(FOC_PHASE_LIMIT_TICKS, 1_103);
        assert_eq!(FOC_HARD_PHASE_LIMIT_TICKS, 1_103);
        assert_eq!(FOC_DEAD_TIME_COMP_TICKS, Fixed::ratio(50, 3));
        assert!(PWM_NEUTRAL + FOC_HARD_PHASE_LIMIT_TICKS < PWM_SAMPLE_CC4);

        let mut maximum_span = 0;
        for step in 0..4_096_u32 {
            let (sin, cos) = CordicSinCos::sin_cos(step << 20);
            let (alpha, beta) = inverse_park(FOC_VECTOR_LIMIT_TICKS, Fixed::ZERO, sin, cos);
            let duty = space_vector_pwm_ticks(
                AlphaBeta { alpha, beta },
                PWM_NEUTRAL,
                FOC_PHASE_LIMIT_TICKS,
            );
            let compares = duty.as_array();
            let minimum = compares.into_iter().min().unwrap();
            let maximum = compares.into_iter().max().unwrap();
            maximum_span = maximum_span.max(maximum - minimum);
            assert!(maximum < PWM_SAMPLE_CC4);
            assert!(
                compares
                    .into_iter()
                    .all(|compare| compare.abs_diff(PWM_NEUTRAL) <= FOC_HARD_PHASE_LIMIT_TICKS)
            );
        }
        assert!(maximum_span >= 2_204);
    }
}
