//! Physical bike configuration and derived STM32F103 implementation constants.
//!
//! [`BIKE_CONFIG`] is the tuning and controller-adaptation surface. Its values
//! use SI-derived engineering units and named GPIO assignments. Timer ticks,
//! ADC counts, fixed-point coefficients, and peripheral remap bits are derived
//! below so they cannot become a second, conflicting configuration.

use oxifoc_core::foc::{Fixed, hall_sensor::HallGeometry};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum GpioPort {
    A,
    B,
    C,
    D,
    E,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct GpioPin {
    pub port: GpioPort,
    pub number: u8,
}

impl GpioPin {
    pub const fn new(port: GpioPort, number: u8) -> Self {
        Self { port, number }
    }

    pub const fn is(self, port: GpioPort, number: u8) -> bool {
        self.port as u8 == port as u8 && self.number == number
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AdcInput {
    pub pin: GpioPin,
    pub channel: u8,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DigitalOutput {
    pub pin: GpioPin,
    pub active_low: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct InverterPinMap {
    /// TIM1 CH1, CH2, and CH3 in phase order.
    pub high_side_pwm: [GpioPin; 3],
    /// TIM1 CH1N, CH2N, and CH3N in phase order.
    pub low_side_pwm: [GpioPin; 3],
    pub hardware_break: GpioPin,
    pub power_stage_enable: DigitalOutput,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CanPinMap {
    pub rx: GpioPin,
    pub tx: GpioPin,
    pub transceiver_enable: DigitalOutput,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PinMap {
    pub inverter: InverterPinMap,
    pub phase_current: [AdcInput; 2],
    /// TIM2 CH1, CH2, and CH3 in logical Hall-bit order.
    pub hall: [GpioPin; 3],
    pub throttle: AdcInput,
    pub motor_temperature: AdcInput,
    pub bus_voltage: AdcInput,
    pub auxiliary_analog: AdcInput,
    pub controller_temperature_adc_channel: u8,
    pub brake: GpioPin,
    pub wheel_speed: GpioPin,
    pub cadence: GpioPin,
    pub can: CanPinMap,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MotorConfig {
    pub phase_resistance_milliohms: u32,
    pub phase_inductance_microhenries: u32,
    pub flux_linkage_microwebers: u32,
    pub hall_sequence: [u8; 6],
    pub hall_boundaries_electrical_millidegrees: [u32; 6],
    pub forward_hall_direction: i8,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SensorConfig {
    pub adc_reference_microvolts: u32,
    pub adc_levels: u32,
    pub phase_current_milliamps_per_adc_count: u32,
    pub bus_voltage_nanovolts_per_adc_count: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct InverterConfig {
    pub pwm_frequency_hz: u32,
    pub dead_time_nanoseconds: u32,
    /// Maximum requested phase-voltage magnitude as parts per thousand of DC bus voltage.
    pub voltage_utilization_per_mille: u16,
    /// Maximum phase duty displacement from 50% as parts per thousand of a PWM period.
    pub duty_deviation_per_mille: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CurrentControlConfig {
    pub phase_current_limit_milliamps: u32,
    pub software_phase_current_trip_milliamps: u32,
    pub target_slew_milliamps_per_millisecond: u32,
    pub tuning_bus_voltage_millivolts: u32,
    pub proportional_gain_microvolts_per_amp: u32,
    pub integral_gain_millivolts_per_amp_second: u32,
    pub dc_link_current_filter_microseconds: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ObserverConfig {
    pub blend_start_electrical_rpm: i32,
    pub blend_end_electrical_rpm: i32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FieldWeakeningConfig {
    pub maximum_current_milliamps: u32,
    pub minimum_electrical_rpm: u32,
    pub voltage_threshold_per_mille: u16,
    pub ramp_in_milliseconds: u32,
    pub ramp_out_milliseconds: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BatteryConfig {
    /// Locally projected terminal-current envelope; no proprietary BMS data is required.
    pub continuous_discharge_milliamps: u32,
    pub transient_discharge_milliamps: u32,
    pub transient_duration_milliseconds: u32,
    pub transient_recovery_ratio: u16,
    pub maximum_regen_milliamps: u32,
    pub minimum_bus_voltage_millivolts: u32,
    pub undervoltage_debounce_milliseconds: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ThermalConfig {
    pub controller_derate_start_celsius: i16,
    pub controller_stop_celsius: i16,
    pub motor_derate_start_celsius: i16,
    pub motor_stop_celsius: i16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RideConfig {
    pub command_lease_milliseconds: u32,
    pub brake_debounce_milliseconds: u8,
    pub startup_first_hall_timeout_milliseconds: u32,
    pub startup_timeout_milliseconds: u32,
    pub startup_forward_hall_transitions: i32,
    pub minimum_running_hall_timeout_milliseconds: u32,
    pub maximum_running_hall_timeout_milliseconds: u32,
    pub maximum_entry_electrical_rpm: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ThrottleConfig {
    pub plausible_minimum_microvolts: u32,
    pub rest_microvolts: u32,
    pub rest_maximum_microvolts: u32,
    pub full_scale_microvolts: u32,
    pub plausible_maximum_microvolts: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CadenceConfig {
    pub debounce_milliseconds: u8,
    pub qualifying_pulses: u8,
    pub maximum_low_milliseconds: u32,
    pub slow_timeout_milliseconds: u32,
    pub fast_timeout_milliseconds: u32,
    pub fast_timeout_threshold_kph: u16,
    pub pulses_per_revolution: u32,
    pub full_demand_low_milliseconds: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct WheelConfig {
    pub distance_per_pulse_millimeters: u16,
    pub minimum_interval_microseconds: u32,
    pub quiet_timeout_microseconds: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CommunicationConfig {
    pub can_bitrate: u32,
    pub can_time_quanta_per_bit: u8,
    pub can_sample_point_per_mille: u16,
    pub can_synchronization_jump_width_time_quanta: u8,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BikeConfig {
    pub motor: MotorConfig,
    pub sensors: SensorConfig,
    pub inverter: InverterConfig,
    pub current_control: CurrentControlConfig,
    pub observer: ObserverConfig,
    pub field_weakening: FieldWeakeningConfig,
    pub battery: BatteryConfig,
    pub thermal: ThermalConfig,
    pub ride: RideConfig,
    pub throttle: ThrottleConfig,
    pub cadence: CadenceConfig,
    pub wheel: WheelConfig,
    pub communication: CommunicationConfig,
    pub pins: PinMap,
}

const PA: GpioPort = GpioPort::A;
const PB: GpioPort = GpioPort::B;
const PC: GpioPort = GpioPort::C;

/// The complete physical configuration for the validated controller and bike.
pub const BIKE_CONFIG: BikeConfig = BikeConfig {
    motor: MotorConfig {
        phase_resistance_milliohms: 43,
        phase_inductance_microhenries: 75,
        flux_linkage_microwebers: 13_400,
        hall_sequence: [5, 1, 3, 2, 6, 4],
        hall_boundaries_electrical_millidegrees: [
            30_125, 84_683, 145_448, 209_877, 273_384, 334_396,
        ],
        forward_hall_direction: -1,
    },
    sensors: SensorConfig {
        adc_reference_microvolts: 3_300_000,
        adc_levels: 4_096,
        phase_current_milliamps_per_adc_count: 100,
        bus_voltage_nanovolts_per_adc_count: 18_530_273,
    },
    inverter: InverterConfig {
        pwm_frequency_hz: 16_000,
        dead_time_nanoseconds: 694,
        voltage_utilization_per_mille: 566,
        duty_deviation_per_mille: 490,
    },
    current_control: CurrentControlConfig {
        phase_current_limit_milliamps: 90_000,
        software_phase_current_trip_milliamps: 134_400,
        target_slew_milliamps_per_millisecond: 800,
        tuning_bus_voltage_millivolts: 52_300,
        proportional_gain_microvolts_per_amp: 116_223,
        integral_gain_millivolts_per_amp_second: 137_333,
        dc_link_current_filter_microseconds: 2_000,
    },
    observer: ObserverConfig {
        blend_start_electrical_rpm: 3_000,
        blend_end_electrical_rpm: 6_000,
    },
    field_weakening: FieldWeakeningConfig {
        maximum_current_milliamps: 20_000,
        minimum_electrical_rpm: 6_000,
        voltage_threshold_per_mille: 950,
        ramp_in_milliseconds: 50,
        ramp_out_milliseconds: 200,
    },
    battery: BatteryConfig {
        continuous_discharge_milliamps: 43_100,
        transient_discharge_milliamps: 64_000,
        transient_duration_milliseconds: 7_000,
        transient_recovery_ratio: 8,
        maximum_regen_milliamps: 4_000,
        minimum_bus_voltage_millivolts: 39_000,
        undervoltage_debounce_milliseconds: 1_000,
    },
    thermal: ThermalConfig {
        controller_derate_start_celsius: 70,
        controller_stop_celsius: 80,
        motor_derate_start_celsius: 100,
        motor_stop_celsius: 130,
    },
    ride: RideConfig {
        command_lease_milliseconds: 10,
        brake_debounce_milliseconds: 4,
        startup_first_hall_timeout_milliseconds: 500,
        startup_timeout_milliseconds: 2_000,
        startup_forward_hall_transitions: 12,
        minimum_running_hall_timeout_milliseconds: 100,
        maximum_running_hall_timeout_milliseconds: 500,
        maximum_entry_electrical_rpm: 2_500,
    },
    throttle: ThrottleConfig {
        plausible_minimum_microvolts: 201_416,
        rest_microvolts: 584_912,
        rest_maximum_microvolts: 684_814,
        full_scale_microvolts: 2_620_020,
        plausible_maximum_microvolts: 3_021_240,
    },
    cadence: CadenceConfig {
        debounce_milliseconds: 4,
        qualifying_pulses: 3,
        maximum_low_milliseconds: 400,
        slow_timeout_milliseconds: 500,
        fast_timeout_milliseconds: 300,
        fast_timeout_threshold_kph: 20,
        pulses_per_revolution: 24,
        full_demand_low_milliseconds: 16,
    },
    wheel: WheelConfig {
        distance_per_pulse_millimeters: 312,
        minimum_interval_microseconds: 1_000,
        quiet_timeout_microseconds: 851_968,
    },
    communication: CommunicationConfig {
        can_bitrate: 250_000,
        can_time_quanta_per_bit: 12,
        can_sample_point_per_mille: 750,
        can_synchronization_jump_width_time_quanta: 1,
    },
    pins: PinMap {
        inverter: InverterPinMap {
            high_side_pwm: [
                GpioPin::new(PA, 8),
                GpioPin::new(PA, 9),
                GpioPin::new(PA, 10),
            ],
            low_side_pwm: [
                GpioPin::new(PB, 13),
                GpioPin::new(PB, 14),
                GpioPin::new(PB, 15),
            ],
            hardware_break: GpioPin::new(PB, 12),
            power_stage_enable: DigitalOutput {
                pin: GpioPin::new(PA, 2),
                active_low: true,
            },
        },
        phase_current: [
            AdcInput {
                pin: GpioPin::new(PC, 1),
                channel: 11,
            },
            AdcInput {
                pin: GpioPin::new(PC, 2),
                channel: 12,
            },
        ],
        hall: [
            GpioPin::new(PA, 15),
            GpioPin::new(PB, 3),
            GpioPin::new(PB, 10),
        ],
        throttle: AdcInput {
            pin: GpioPin::new(PC, 5),
            channel: 15,
        },
        motor_temperature: AdcInput {
            pin: GpioPin::new(PC, 3),
            channel: 13,
        },
        bus_voltage: AdcInput {
            pin: GpioPin::new(PB, 0),
            channel: 8,
        },
        auxiliary_analog: AdcInput {
            pin: GpioPin::new(PA, 5),
            channel: 5,
        },
        controller_temperature_adc_channel: 16,
        brake: GpioPin::new(PC, 4),
        wheel_speed: GpioPin::new(PB, 4),
        cadence: GpioPin::new(PB, 9),
        can: CanPinMap {
            rx: GpioPin::new(PA, 11),
            tx: GpioPin::new(PA, 12),
            transceiver_enable: DigitalOutput {
                pin: GpioPin::new(PA, 13),
                active_low: true,
            },
        },
    },
};

const fn current_counts_from_milliamps(milliamps: u32) -> u16 {
    (milliamps / BIKE_CONFIG.sensors.phase_current_milliamps_per_adc_count) as u16
}

const fn hall_boundary_q16(millidegrees: u32) -> u16 {
    ((millidegrees as u64 * 65_536 + 180_000) / 360_000) as u16
}

const fn adc_pin_matches(input: AdcInput) -> bool {
    match input.channel {
        0..=7 => input.pin.is(PA, input.channel),
        8..=9 => input.pin.is(PB, input.channel - 8),
        10..=15 => input.pin.is(PC, input.channel - 10),
        _ => false,
    }
}

const fn pins_equal(a: GpioPin, b: GpioPin) -> bool {
    a.port as u8 == b.port as u8 && a.number == b.number
}

const fn pins_are_valid_and_unique(pins: &[GpioPin]) -> bool {
    let mut outer = 0;
    while outer < pins.len() {
        if pins[outer].number >= 16 {
            return false;
        }
        let mut inner = outer + 1;
        while inner < pins.len() {
            if pins_equal(pins[outer], pins[inner]) {
                return false;
            }
            inner += 1;
        }
        outer += 1;
    }
    true
}

const fn pin_triplet_is(pins: [GpioPin; 3], expected: [(GpioPort, u8); 3]) -> bool {
    pins[0].is(expected[0].0, expected[0].1)
        && pins[1].is(expected[1].0, expected[1].1)
        && pins[2].is(expected[2].0, expected[2].1)
}

/// AFIO MAPR TIM1_REMAP field selected by the configured six-switch bridge.
pub const fn tim1_remap_bits() -> u32 {
    let inverter = BIKE_CONFIG.pins.inverter;
    if pin_triplet_is(inverter.high_side_pwm, [(PA, 8), (PA, 9), (PA, 10)])
        && pin_triplet_is(inverter.low_side_pwm, [(PB, 13), (PB, 14), (PB, 15)])
        && inverter.hardware_break.is(PB, 12)
    {
        0
    } else if pin_triplet_is(inverter.high_side_pwm, [(PA, 8), (PA, 9), (PA, 10)])
        && pin_triplet_is(inverter.low_side_pwm, [(PA, 7), (PB, 0), (PB, 1)])
        && inverter.hardware_break.is(PA, 6)
    {
        0b01 << 6
    } else if pin_triplet_is(
        inverter.high_side_pwm,
        [(GpioPort::E, 9), (GpioPort::E, 11), (GpioPort::E, 13)],
    ) && pin_triplet_is(
        inverter.low_side_pwm,
        [(GpioPort::E, 8), (GpioPort::E, 10), (GpioPort::E, 12)],
    ) && inverter.hardware_break.is(GpioPort::E, 15)
    {
        0b11 << 6
    } else {
        u32::MAX
    }
}

/// AFIO MAPR TIM2_REMAP field selected by the three Hall inputs.
pub const fn tim2_remap_bits() -> u32 {
    let hall = BIKE_CONFIG.pins.hall;
    if pin_triplet_is(hall, [(PA, 0), (PA, 1), (PA, 2)]) {
        0
    } else if pin_triplet_is(hall, [(PA, 15), (PB, 3), (PA, 2)]) {
        0b01 << 8
    } else if pin_triplet_is(hall, [(PA, 0), (PA, 1), (PB, 10)]) {
        0b10 << 8
    } else if pin_triplet_is(hall, [(PA, 15), (PB, 3), (PB, 10)]) {
        0b11 << 8
    } else {
        u32::MAX
    }
}

/// AFIO MAPR TIM3_REMAP field selected by the wheel-speed input on CH1.
pub const fn tim3_remap_bits() -> u32 {
    let wheel = BIKE_CONFIG.pins.wheel_speed;
    if wheel.is(PA, 6) {
        0
    } else if wheel.is(PB, 4) {
        0b10 << 10
    } else if wheel.is(PC, 6) {
        0b11 << 10
    } else {
        u32::MAX
    }
}

/// AFIO MAPR CAN_REMAP field selected by bxCAN RX/TX.
pub const fn can_remap_bits() -> u32 {
    let can = BIKE_CONFIG.pins.can;
    if can.rx.is(PA, 11) && can.tx.is(PA, 12) {
        0
    } else if can.rx.is(PB, 8) && can.tx.is(PB, 9) {
        0b10 << 13
    } else if can.rx.is(GpioPort::D, 0) && can.tx.is(GpioPort::D, 1) {
        0b11 << 13
    } else {
        u32::MAX
    }
}

pub const SYSCLK_HZ: u32 = 72_000_000;
pub const APPLICATION_FLASH_ORIGIN: u32 = 0x0800_3800;
pub const PCLK1_HZ: u32 = 36_000_000;
pub const APB1_TIMER_HZ: u32 = 72_000_000;
pub const ADC_HZ: u32 = 12_000_000;

pub const PWM_HZ: u32 = BIKE_CONFIG.inverter.pwm_frequency_hz;
pub const PWM_ARR: u16 = (APB1_TIMER_HZ / (2 * PWM_HZ)) as u16;
pub const PWM_NEUTRAL: u16 = PWM_ARR / 2;
pub const PWM_DEAD_TIME_TICKS: u8 =
    ((BIKE_CONFIG.inverter.dead_time_nanoseconds as u64 * (APB1_TIMER_HZ / 2) as u64 + 500_000_000)
        / 1_000_000_000) as u8;
pub const PWM_SAMPLE_CC4: u16 = PWM_ARR - 2;
pub const FOC_DEAD_TIME_COMP_NUMERATOR: i32 = (BIKE_CONFIG.inverter.dead_time_nanoseconds as u64
    * PWM_HZ as u64
    * 2
    * PWM_ARR as u64
    / 1_000) as i32;
pub const FOC_DEAD_TIME_COMP_DENOMINATOR: i32 = 3_000_000;
pub const FOC_DEAD_TIME_COMP_TICKS: Fixed =
    Fixed::ratio(FOC_DEAD_TIME_COMP_NUMERATOR, FOC_DEAD_TIME_COMP_DENOMINATOR);
// The S310 application uses a 1,273-tick voltage circle and approximately
// 22..2,227 timer compares with this same 2,250-tick PWM period.
pub const FOC_PHASE_LIMIT_TICKS: u16 =
    ((PWM_ARR as u32 * BIKE_CONFIG.inverter.duty_deviation_per_mille as u32 + 500) / 1_000) as u16;
pub const FOC_HARD_PHASE_LIMIT_TICKS: u16 = FOC_PHASE_LIMIT_TICKS;
pub const FOC_VECTOR_LIMIT_TICKS: Fixed = Fixed::from_integer(
    (PWM_ARR as i32 * BIKE_CONFIG.inverter.voltage_utilization_per_mille as i32) / 1_000,
);
pub const CURRENT_PI_PROPORTIONAL_GAIN: Fixed = Fixed::ratio(
    (BIKE_CONFIG
        .current_control
        .proportional_gain_microvolts_per_amp as u64
        * BIKE_CONFIG.sensors.phase_current_milliamps_per_adc_count as u64
        * PWM_ARR as u64
        / 1_000) as i32,
    (1_000 * BIKE_CONFIG.current_control.tuning_bus_voltage_millivolts) as i32,
);
pub const CURRENT_PI_INTEGRAL_GAIN_PER_CYCLE: Fixed = Fixed::ratio(
    (BIKE_CONFIG
        .current_control
        .integral_gain_millivolts_per_amp_second as u64
        * BIKE_CONFIG.sensors.phase_current_milliamps_per_adc_count as u64
        * PWM_ARR as u64
        / 1_000) as i32,
    (PWM_HZ * BIKE_CONFIG.current_control.tuning_bus_voltage_millivolts) as i32,
);
pub const TARGET_RAMP_CYCLES_PER_STEP: u8 = (PWM_HZ / 1_000) as u8;
pub const TARGET_RAMP_COUNTS_PER_STEP: i32 = (BIKE_CONFIG
    .current_control
    .target_slew_milliamps_per_millisecond
    / BIKE_CONFIG.sensors.phase_current_milliamps_per_adc_count)
    as i32;
pub const DC_LINK_CURRENT_FILTER_CYCLES: u32 = PWM_HZ
    * BIKE_CONFIG
        .current_control
        .dc_link_current_filter_microseconds
    / 1_000_000;
pub const DC_LINK_CURRENT_FILTER_SHIFT: u8 = DC_LINK_CURRENT_FILTER_CYCLES.ilog2() as u8;

pub const HALL_TIMER_HZ: u32 = 1_000_000;
pub const HALL_TIMER_PRESCALER: u16 = (APB1_TIMER_HZ / HALL_TIMER_HZ - 1) as u16;
pub static HALL_GEOMETRY: HallGeometry = HallGeometry::new(
    BIKE_CONFIG.motor.hall_sequence,
    [
        hall_boundary_q16(BIKE_CONFIG.motor.hall_boundaries_electrical_millidegrees[0]),
        hall_boundary_q16(BIKE_CONFIG.motor.hall_boundaries_electrical_millidegrees[1]),
        hall_boundary_q16(BIKE_CONFIG.motor.hall_boundaries_electrical_millidegrees[2]),
        hall_boundary_q16(BIKE_CONFIG.motor.hall_boundaries_electrical_millidegrees[3]),
        hall_boundary_q16(BIKE_CONFIG.motor.hall_boundaries_electrical_millidegrees[4]),
        hall_boundary_q16(BIKE_CONFIG.motor.hall_boundaries_electrical_millidegrees[5]),
    ],
    BIKE_CONFIG.motor.forward_hall_direction,
);

// Loaded terminal power balance gives approximately 94--100 mA per phase-ADC
// count. Inverter losses bias that estimate upward, so 100 mA/count remains a
// nominal conversion; the protection and current-loop envelopes stay in the
// directly observed ADC-count domain.
pub const PHASE_CURRENT_MA_PER_ADC_COUNT: i32 =
    BIKE_CONFIG.sensors.phase_current_milliamps_per_adc_count as i32;
pub const PHASE_CURRENT_AMPS_PER_ADC_COUNT: Fixed =
    Fixed::ratio(PHASE_CURRENT_MA_PER_ADC_COUNT, 1_000);
// Effective loaded terminal model. These values reproduce 4.3 mV/count of
// resistive drop and 7.5 uH*A/count of inductive flux in the observer's input
// domain without claiming an independently measured winding R or L.
pub const MOTOR_PHASE_RESISTANCE_OHMS: Fixed =
    Fixed::ratio(BIKE_CONFIG.motor.phase_resistance_milliohms as i32, 1_000);
pub const MOTOR_PHASE_INDUCTANCE_MILLIHENRIES: Fixed = Fixed::ratio(
    BIKE_CONFIG.motor.phase_inductance_microhenries as i32,
    1_000,
);
pub const MOTOR_FLUX_LINKAGE_MILLIWEBERS: Fixed =
    Fixed::ratio(BIKE_CONFIG.motor.flux_linkage_microwebers as i32, 1_000);
pub const MOTOR_INDUCTIVE_FLUX_MWB_PER_COUNT_BITS: i32 =
    ((BIKE_CONFIG.motor.phase_inductance_microhenries
        * BIKE_CONFIG.sensors.phase_current_milliamps_per_adc_count
        * 65_536
        + 500_000)
        / 1_000_000) as i32;
pub const OBSERVER_BLEND_LOW_ERPM: i32 = BIKE_CONFIG.observer.blend_start_electrical_rpm;
pub const OBSERVER_BLEND_HIGH_ERPM: i32 = BIKE_CONFIG.observer.blend_end_electrical_rpm;
pub const PHASE_CURRENT_TRIP_COUNTS: u16 = current_counts_from_milliamps(
    BIKE_CONFIG
        .current_control
        .software_phase_current_trip_milliamps,
);
pub const RIDE_PHASE_CURRENT_LIMIT_COUNTS: u16 =
    current_counts_from_milliamps(BIKE_CONFIG.current_control.phase_current_limit_milliamps);
pub const FIELD_WEAKENING_MAXIMUM_CURRENT_COUNTS: u16 =
    current_counts_from_milliamps(BIKE_CONFIG.field_weakening.maximum_current_milliamps);
pub const FIELD_WEAKENING_MINIMUM_ERPM: u32 = BIKE_CONFIG.field_weakening.minimum_electrical_rpm;
pub const FIELD_WEAKENING_VOLTAGE_THRESHOLD_TICKS: Fixed = Fixed::ratio(
    FOC_VECTOR_LIMIT_TICKS.integer()
        * BIKE_CONFIG.field_weakening.voltage_threshold_per_mille as i32,
    1_000,
);
// The d target reaches its initial 20 A ceiling in 50 ms and releases over
// 200 ms once voltage headroom returns.
pub const FIELD_WEAKENING_RAMP_IN_PER_CYCLE: Fixed = Fixed::ratio(
    FIELD_WEAKENING_MAXIMUM_CURRENT_COUNTS as i32,
    (PWM_HZ * BIKE_CONFIG.field_weakening.ramp_in_milliseconds / 1_000) as i32,
);
pub const FIELD_WEAKENING_RAMP_OUT_PER_CYCLE: Fixed = Fixed::ratio(
    FIELD_WEAKENING_MAXIMUM_CURRENT_COUNTS as i32,
    (PWM_HZ * BIKE_CONFIG.field_weakening.ramp_out_milliseconds / 1_000) as i32,
);
// The terminal-power projection reads approximately three percent below the
// BMS current. These limits therefore target about 44 A continuous and 65 A
// peak at the battery instead of treating the nominal 100 mA/count phase scale
// as an exact DC-current conversion.
pub const RIDE_DC_BUS_CONTINUOUS_LIMIT_COUNTS: u16 =
    current_counts_from_milliamps(BIKE_CONFIG.battery.continuous_discharge_milliamps);
pub const RIDE_DC_BUS_TRANSIENT_LIMIT_COUNTS: u16 =
    current_counts_from_milliamps(BIKE_CONFIG.battery.transient_discharge_milliamps);
pub const RIDE_DC_BUS_TRANSIENT_DURATION_MS: u32 =
    BIKE_CONFIG.battery.transient_duration_milliseconds;
// Unused continuous-current headroom restores the transient bucket at one
// eighth rate, requiring about 27 seconds at zero drive current after complete
// exhaustion.
pub const RIDE_DC_BUS_TRANSIENT_RECOVERY_DIVISOR: u16 =
    BIKE_CONFIG.battery.transient_recovery_ratio;
pub const RIDE_DC_BUS_REGEN_LIMIT_COUNTS: u16 =
    current_counts_from_milliamps(BIKE_CONFIG.battery.maximum_regen_milliamps);
pub const VBUS_UV_PER_COUNT: u32 = BIKE_CONFIG.sensors.bus_voltage_nanovolts_per_adc_count / 1_000;
pub const VBUS_MILLIVOLTS_Q10_PER_COUNT: u32 = ((BIKE_CONFIG
    .sensors
    .bus_voltage_nanovolts_per_adc_count as u64
    * 1_024
    + 500_000)
    / 1_000_000) as u32;

const _: () = assert!(BIKE_CONFIG.sensors.adc_reference_microvolts > 0);
const _: () = assert!(BIKE_CONFIG.sensors.adc_levels == 4_096);
const _: () = assert!(BIKE_CONFIG.sensors.phase_current_milliamps_per_adc_count > 0);
const _: () = assert!(BIKE_CONFIG.sensors.bus_voltage_nanovolts_per_adc_count > 0);
const _: () = assert!(VBUS_MILLIVOLTS_Q10_PER_COUNT <= u32::MAX / u16::MAX as u32);
const _: () = assert!(BIKE_CONFIG.motor.phase_resistance_milliohms > 0);
const _: () = assert!(BIKE_CONFIG.motor.phase_inductance_microhenries > 0);
const _: () = assert!(BIKE_CONFIG.motor.flux_linkage_microwebers > 0);
const _: () = assert!(BIKE_CONFIG.inverter.pwm_frequency_hz >= 8_000);
const _: () = assert!(BIKE_CONFIG.inverter.pwm_frequency_hz <= 40_000);
const _: () = assert!(BIKE_CONFIG.inverter.dead_time_nanoseconds > 0);
const _: () = assert!(BIKE_CONFIG.inverter.dead_time_nanoseconds <= 3_000);
const _: () = assert!(BIKE_CONFIG.inverter.voltage_utilization_per_mille <= 577);
const _: () = assert!(BIKE_CONFIG.inverter.duty_deviation_per_mille < 500);
const _: () = assert!(BIKE_CONFIG.current_control.phase_current_limit_milliamps > 0);
const _: () = assert!(
    BIKE_CONFIG
        .current_control
        .target_slew_milliamps_per_millisecond
        > 0
);
const _: () = assert!(BIKE_CONFIG.current_control.tuning_bus_voltage_millivolts > 0);
const _: () = assert!(
    BIKE_CONFIG
        .current_control
        .dc_link_current_filter_microseconds
        > 0
);
const _: () = assert!(
    BIKE_CONFIG
        .current_control
        .target_slew_milliamps_per_millisecond
        .is_multiple_of(BIKE_CONFIG.sensors.phase_current_milliamps_per_adc_count)
);
const _: () = assert!(BIKE_CONFIG.field_weakening.voltage_threshold_per_mille < 1_000);
const _: () = assert!(BIKE_CONFIG.field_weakening.ramp_in_milliseconds > 0);
const _: () = assert!(BIKE_CONFIG.field_weakening.ramp_out_milliseconds > 0);
const _: () = assert!(BIKE_CONFIG.battery.undervoltage_debounce_milliseconds > 0);
const _: () = assert!(
    BIKE_CONFIG.thermal.controller_derate_start_celsius
        < BIKE_CONFIG.thermal.controller_stop_celsius
);
const _: () = assert!(
    BIKE_CONFIG.thermal.motor_derate_start_celsius < BIKE_CONFIG.thermal.motor_stop_celsius
);
const _: () = assert!(
    BIKE_CONFIG.throttle.plausible_minimum_microvolts <= BIKE_CONFIG.throttle.rest_microvolts
);
const _: () =
    assert!(BIKE_CONFIG.throttle.rest_microvolts < BIKE_CONFIG.throttle.rest_maximum_microvolts);
const _: () = assert!(
    BIKE_CONFIG.throttle.rest_maximum_microvolts < BIKE_CONFIG.throttle.full_scale_microvolts
);
const _: () = assert!(
    BIKE_CONFIG.throttle.full_scale_microvolts <= BIKE_CONFIG.throttle.plausible_maximum_microvolts
);
const _: () = assert!(
    BIKE_CONFIG.throttle.plausible_maximum_microvolts
        < BIKE_CONFIG.sensors.adc_reference_microvolts
);
const _: () = assert!(BIKE_CONFIG.cadence.debounce_milliseconds > 0);
const _: () = assert!(BIKE_CONFIG.cadence.qualifying_pulses > 0);
const _: () = assert!(
    (BIKE_CONFIG.cadence.full_demand_low_milliseconds as u32)
        < BIKE_CONFIG.cadence.maximum_low_milliseconds
);
const _: () = assert!(BIKE_CONFIG.wheel.distance_per_pulse_millimeters > 0);
const _: () = assert!(BIKE_CONFIG.wheel.minimum_interval_microseconds > 0);
const _: () = assert!(BIKE_CONFIG.wheel.quiet_timeout_microseconds > 0);
const _: () = assert!(BIKE_CONFIG.communication.can_bitrate > 0);
const _: () = assert!(BIKE_CONFIG.communication.can_time_quanta_per_bit >= 8);
const _: () = assert!(BIKE_CONFIG.communication.can_time_quanta_per_bit <= 25);
const _: () = assert!(BIKE_CONFIG.communication.can_sample_point_per_mille > 500);
const _: () = assert!(BIKE_CONFIG.communication.can_sample_point_per_mille < 1_000);
const _: () = assert!(
    (BIKE_CONFIG.communication.can_sample_point_per_mille as u32
        * BIKE_CONFIG.communication.can_time_quanta_per_bit as u32)
        .is_multiple_of(1_000)
);
const _: () = assert!(
    BIKE_CONFIG
        .communication
        .can_synchronization_jump_width_time_quanta
        > 0
);
const _: () = assert!(
    BIKE_CONFIG
        .communication
        .can_synchronization_jump_width_time_quanta
        <= CAN_BS2_TQ
);
const _: () = assert!(CAN_BS1_TQ >= 1 && CAN_BS1_TQ <= 16);
const _: () = assert!(CAN_BS2_TQ >= 1 && CAN_BS2_TQ <= 8);
const _: () = assert!(CAN_SJW_TQ <= 4);
const _: () = assert!(CAN_PRESCALER >= 1 && CAN_PRESCALER <= 1_024);
const _: () = assert!(BIKE_CONFIG.ride.command_lease_milliseconds > 0);
const _: () = assert!(BIKE_CONFIG.ride.brake_debounce_milliseconds > 0);
const _: () = assert!(BIKE_CONFIG.pins.controller_temperature_adc_channel == 16);
const _: () = assert!(adc_pin_matches(BIKE_CONFIG.pins.phase_current[0]));
const _: () = assert!(adc_pin_matches(BIKE_CONFIG.pins.phase_current[1]));
const _: () = assert!(adc_pin_matches(BIKE_CONFIG.pins.throttle));
const _: () = assert!(adc_pin_matches(BIKE_CONFIG.pins.motor_temperature));
const _: () = assert!(adc_pin_matches(BIKE_CONFIG.pins.bus_voltage));
const _: () = assert!(adc_pin_matches(BIKE_CONFIG.pins.auxiliary_analog));
const _: () = assert!(tim1_remap_bits() != u32::MAX);
const _: () = assert!(tim2_remap_bits() != u32::MAX);
const _: () = assert!(tim3_remap_bits() != u32::MAX);
const _: () = assert!(can_remap_bits() != u32::MAX);
const _: () = assert!(pins_are_valid_and_unique(&[
    BIKE_CONFIG.pins.inverter.high_side_pwm[0],
    BIKE_CONFIG.pins.inverter.high_side_pwm[1],
    BIKE_CONFIG.pins.inverter.high_side_pwm[2],
    BIKE_CONFIG.pins.inverter.low_side_pwm[0],
    BIKE_CONFIG.pins.inverter.low_side_pwm[1],
    BIKE_CONFIG.pins.inverter.low_side_pwm[2],
    BIKE_CONFIG.pins.inverter.hardware_break,
    BIKE_CONFIG.pins.inverter.power_stage_enable.pin,
    BIKE_CONFIG.pins.phase_current[0].pin,
    BIKE_CONFIG.pins.phase_current[1].pin,
    BIKE_CONFIG.pins.hall[0],
    BIKE_CONFIG.pins.hall[1],
    BIKE_CONFIG.pins.hall[2],
    BIKE_CONFIG.pins.throttle.pin,
    BIKE_CONFIG.pins.motor_temperature.pin,
    BIKE_CONFIG.pins.bus_voltage.pin,
    BIKE_CONFIG.pins.auxiliary_analog.pin,
    BIKE_CONFIG.pins.brake,
    BIKE_CONFIG.pins.wheel_speed,
    BIKE_CONFIG.pins.cadence,
    BIKE_CONFIG.pins.can.rx,
    BIKE_CONFIG.pins.can.tx,
    BIKE_CONFIG.pins.can.transceiver_enable.pin,
]));
const _: () = assert!(RIDE_PHASE_CURRENT_LIMIT_COUNTS < PHASE_CURRENT_TRIP_COUNTS);
const _: () = assert!(
    BIKE_CONFIG
        .current_control
        .phase_current_limit_milliamps
        .is_multiple_of(BIKE_CONFIG.sensors.phase_current_milliamps_per_adc_count)
);
const _: () = assert!(
    BIKE_CONFIG
        .current_control
        .software_phase_current_trip_milliamps
        .is_multiple_of(BIKE_CONFIG.sensors.phase_current_milliamps_per_adc_count)
);
const _: () = assert!(
    BIKE_CONFIG
        .field_weakening
        .maximum_current_milliamps
        .is_multiple_of(BIKE_CONFIG.sensors.phase_current_milliamps_per_adc_count)
);
const _: () = assert!(
    BIKE_CONFIG
        .battery
        .continuous_discharge_milliamps
        .is_multiple_of(BIKE_CONFIG.sensors.phase_current_milliamps_per_adc_count)
);
const _: () = assert!(
    BIKE_CONFIG
        .battery
        .transient_discharge_milliamps
        .is_multiple_of(BIKE_CONFIG.sensors.phase_current_milliamps_per_adc_count)
);
const _: () = assert!(
    BIKE_CONFIG
        .battery
        .maximum_regen_milliamps
        .is_multiple_of(BIKE_CONFIG.sensors.phase_current_milliamps_per_adc_count)
);
const _: () = assert!(FIELD_WEAKENING_MAXIMUM_CURRENT_COUNTS < RIDE_PHASE_CURRENT_LIMIT_COUNTS);
const _: () = assert!(FIELD_WEAKENING_MINIMUM_ERPM >= OBSERVER_BLEND_HIGH_ERPM as u32);
const _: () = assert!(FIELD_WEAKENING_VOLTAGE_THRESHOLD_TICKS.to_bits() > 0);
const _: () =
    assert!(FIELD_WEAKENING_VOLTAGE_THRESHOLD_TICKS.to_bits() < FOC_VECTOR_LIMIT_TICKS.to_bits());
const _: () = assert!(FIELD_WEAKENING_RAMP_IN_PER_CYCLE.to_bits() > 0);
const _: () = assert!(FIELD_WEAKENING_RAMP_OUT_PER_CYCLE.to_bits() > 0);
const _: () = assert!(RIDE_DC_BUS_REGEN_LIMIT_COUNTS < RIDE_DC_BUS_CONTINUOUS_LIMIT_COUNTS);
const _: () = assert!(RIDE_DC_BUS_CONTINUOUS_LIMIT_COUNTS < RIDE_DC_BUS_TRANSIENT_LIMIT_COUNTS);
const _: () = assert!(RIDE_DC_BUS_TRANSIENT_RECOVERY_DIVISOR > 0);
const _: () =
    assert!((PHASE_CURRENT_TRIP_COUNTS as u32) * 5 >= (RIDE_PHASE_CURRENT_LIMIT_COUNTS as u32) * 7);
const _: () = assert!(FOC_PHASE_LIMIT_TICKS <= FOC_HARD_PHASE_LIMIT_TICKS);
const _: () = assert!(TARGET_RAMP_CYCLES_PER_STEP > 0);
const _: () = assert!(APB1_TIMER_HZ.is_multiple_of(2 * PWM_HZ));
const _: () = assert!(PWM_HZ.is_multiple_of(1_000));
const _: () = assert!(PWM_ARR > 0);
const _: () = assert!(PWM_DEAD_TIME_TICKS <= 127);
const _: () = assert!(
    (PWM_HZ
        * BIKE_CONFIG
            .current_control
            .dc_link_current_filter_microseconds)
        .is_multiple_of(1_000_000)
);
const _: () = assert!(DC_LINK_CURRENT_FILTER_CYCLES.is_power_of_two());
const _: () = assert!(OBSERVER_BLEND_LOW_ERPM > 0);
const _: () = assert!(OBSERVER_BLEND_HIGH_ERPM > OBSERVER_BLEND_LOW_ERPM);
const _: () = assert!(PWM_NEUTRAL + FOC_HARD_PHASE_LIMIT_TICKS < PWM_SAMPLE_CC4);
const _: () = assert!(PWM_SAMPLE_CC4 < PWM_ARR);
const _: () = assert!(PCLK1_HZ.is_multiple_of(
    CAN_BITRATE * CAN_PRESCALER as u32 * (1 + CAN_BS1_TQ as u32 + CAN_BS2_TQ as u32)
));

pub const THROTTLE_ADC_CHANNEL: u8 = BIKE_CONFIG.pins.throttle.channel;
pub const MOTOR_TEMPERATURE_ADC_CHANNEL: u8 = BIKE_CONFIG.pins.motor_temperature.channel;
pub const VBUS_ADC_CHANNEL: u8 = BIKE_CONFIG.pins.bus_voltage.channel;
pub const UNUSED_TORQUE_ADC_CHANNEL: u8 = BIKE_CONFIG.pins.auxiliary_analog.channel;
pub const CONTROLLER_TEMPERATURE_ADC_CHANNEL: u8 =
    BIKE_CONFIG.pins.controller_temperature_adc_channel;
pub const REGULAR_ADC_CHANNELS: [u8; 5] = [
    MOTOR_TEMPERATURE_ADC_CHANNEL,
    THROTTLE_ADC_CHANNEL,
    VBUS_ADC_CHANNEL,
    UNUSED_TORQUE_ADC_CHANNEL,
    CONTROLLER_TEMPERATURE_ADC_CHANNEL,
];
pub const CAN_BITRATE: u32 = BIKE_CONFIG.communication.can_bitrate;
pub const CAN_BS1_TQ: u8 = (BIKE_CONFIG.communication.can_sample_point_per_mille as u32
    * BIKE_CONFIG.communication.can_time_quanta_per_bit as u32
    / 1_000
    - 1) as u8;
pub const CAN_BS2_TQ: u8 = BIKE_CONFIG.communication.can_time_quanta_per_bit - 1 - CAN_BS1_TQ;
pub const CAN_SJW_TQ: u8 = BIKE_CONFIG
    .communication
    .can_synchronization_jump_width_time_quanta;
pub const CAN_PRESCALER: u16 =
    (PCLK1_HZ / CAN_BITRATE / BIKE_CONFIG.communication.can_time_quanta_per_bit as u32) as u16;

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
    pub const CONFIGURED: Self = Self {
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
        assert_eq!(CanTiming::CONFIGURED.bitrate(PCLK1_HZ), CAN_BITRATE);
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
    fn regular_adc_rank_contract_matches_the_recovered_board() {
        assert_eq!(regular_adc_sequence_register_1(), 0x0040_0000);
        assert_eq!(regular_adc_sequence_register_3(), 0x0102_a1ed);
    }

    #[test]
    fn observer_inductive_coefficient_matches_the_motor_model() {
        assert_eq!(
            PHASE_CURRENT_AMPS_PER_ADC_COUNT,
            Fixed::ratio(PHASE_CURRENT_MA_PER_ADC_COUNT, 1_000)
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
        assert!(maximum_span >= FOC_PHASE_LIMIT_TICKS.saturating_mul(2).saturating_sub(2));
    }
}
