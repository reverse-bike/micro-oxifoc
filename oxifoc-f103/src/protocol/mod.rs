//! Minimal stock-bike CAN wire contract.

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Frame {
    pub id: u16,
    pub len: u8,
    pub data: [u8; 8],
}

impl Frame {
    pub const fn new(id: u16, len: u8, data: [u8; 8]) -> Self {
        Self { id, len, data }
    }
}

pub const fn identity_response(id: u16) -> Option<Frame> {
    match id {
        0x210 => Some(Frame::new(0x210, 8, *b"S73RX_22")),
        0x211 => Some(Frame::new(0x211, 8, *b"05060002")),
        0x212 => Some(Frame::new(0x212, 4, [b'U', b'S', 0, 0, 0, 0, 0, 0])),
        _ => None,
    }
}

pub const fn is_updater_reset(frame: Frame) -> bool {
    frame.id == 0x67f
        && frame.len >= 5
        && frame.data[0] == 0xaa
        && frame.data[1] == 0x55
        && frame.data[2] == 0x2a
        && frame.data[3] == 0
        && frame.data[4] == 0x2a
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct StockTelemetry {
    pub vehicle_speed_tenths_kph: u16,
    pub distance_counter: u32,
    pub brake_active: bool,
    pub controller_temperature_deci_c: Option<i16>,
    pub motor_temperature_deci_c: Option<i16>,
    pub fault_page: u8,
    pub hardware_break: bool,
    pub dc_bus_undervoltage: bool,
    pub hall_invalid: bool,
}

pub fn telemetry_slot(slot: u8, snapshot: StockTelemetry) -> [Option<Frame>; 3] {
    let motion_flags = 0x60 | (u8::from(snapshot.brake_active) << 2);
    let speed = snapshot
        .vehicle_speed_tenths_kph
        .saturating_mul(10)
        .to_le_bytes();
    let motion = Frame::new(0x201, 5, [speed[0], speed[1], 0, 0, motion_flags, 0, 0, 0]);
    let profile = Frame::new(
        0x64a,
        8,
        [
            0x2b,
            0x41,
            0x63,
            0,
            if snapshot.brake_active { 0x40 } else { 0 },
            0,
            0,
            0,
        ],
    );
    let primary = if slot & 1 == 0 {
        [Some(motion), Some(profile), None]
    } else {
        [Some(Frame::new(0x203, 8, [0; 8])), None, None]
    };
    match slot % 60 {
        1 => [
            primary[0],
            Some(controller_status(snapshot)),
            Some(Frame::new(
                0x265,
                8,
                [0, b'H', b'T', b'M', b'T', 0x13, 0, 0],
            )),
        ],
        2 | 22 | 42 => [primary[0], primary[1], Some(controller_limits(snapshot))],
        3 => [
            primary[0],
            Some(Frame::new(
                0x204,
                8,
                [
                    encode_temperature(snapshot.controller_temperature_deci_c),
                    0,
                    3,
                    0,
                    0,
                    0,
                    0,
                    0,
                ],
            )),
            None,
        ],
        7 => [
            primary[0],
            Some(Frame::new(0x266, 8, [0, 3, 12, 0, 0, 0x20, 0x10, 0x22])),
            None,
        ],
        _ => primary,
    }
}

pub const fn next_telemetry_slot(slot: u8) -> u8 {
    if slot >= 59 { 0 } else { slot + 1 }
}

pub const fn next_project_telemetry_page(page: u8) -> u8 {
    if page >= 11 { 0 } else { page + 1 }
}

pub const PROJECT_TELEMETRY_SCHEMA: u8 = 2;

fn controller_status(snapshot: StockTelemetry) -> Frame {
    let faults = stock_fault_word(snapshot).to_le_bytes();
    Frame::new(0x200, 8, [0, 0, 0, 0, 0, 0, faults[0], faults[1]])
}

fn stock_fault_word(snapshot: StockTelemetry) -> u16 {
    let page = snapshot.fault_page & 3;
    let mut word = u16::from(page);
    match page {
        0 => {
            word |= u16::from(snapshot.hardware_break) << 3;
            word |= u16::from(snapshot.dc_bus_undervoltage) << 4;
            word |= u16::from(snapshot.hall_invalid) << 6;
        }
        1 => {
            if let Some(temperature) = snapshot.controller_temperature_deci_c {
                word |= u16::from(temperature >= 700) << 6;
                word |= u16::from(temperature >= 800) << 7;
            }
        }
        2 => {
            if let Some(temperature) = snapshot.motor_temperature_deci_c {
                word |= u16::from(temperature > 1_000 && temperature < 1_300) << 6;
                word |= u16::from(temperature >= 1_300) << 7;
            }
        }
        _ => {}
    }
    word
}

fn controller_limits(snapshot: StockTelemetry) -> Frame {
    let distance_low = (snapshot.distance_counter as u16).to_le_bytes();
    let distance = snapshot.distance_counter.to_le_bytes();
    Frame::new(
        0x202,
        8,
        [
            0x22,
            encode_temperature(snapshot.motor_temperature_deci_c),
            distance_low[0],
            distance_low[1],
            distance[0],
            distance[1],
            distance[2],
            distance[3],
        ],
    )
}

fn encode_temperature(temperature_deci_c: Option<i16>) -> u8 {
    temperature_deci_c.map_or(0, |temperature| {
        (temperature / 10)
            .saturating_add(40)
            .clamp(0, i16::from(u8::MAX)) as u8
    })
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PassiveInputTelemetry {
    pub analog_valid: bool,
    pub brake_active: bool,
    pub throttle_valid: bool,
    pub throttle_at_rest: bool,
    pub throttle_raw: u16,
    pub throttle_demand: u8,
    pub throttle_current_limit_counts: u16,
    pub live_hall_state: u8,
    pub bus_voltage_mv: u16,
    pub effective_current_limit: u16,
    pub derating_reasons: u8,
    pub controller_temperature_deci_c: Option<i16>,
    pub motor_temperature_deci_c: Option<i16>,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ControlTimingTelemetry {
    pub current_trips: u32,
    pub maximum_cycles: u32,
    pub warning_count: u32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ControlLiveTelemetry {
    pub hall_valid: bool,
    pub current_valid: bool,
    pub output_active: bool,
    pub voltage_limited: bool,
    pub ride_stage: u8,
    pub target_q_counts: i16,
    pub measured_d_counts: i16,
    pub measured_q_counts: i16,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ControlOutputTelemetry {
    pub phase_limit_counts: u16,
    pub applied_d_ticks: i16,
    pub applied_q_ticks: i16,
    pub pwm_span_ticks: u16,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ControlFaultTelemetry {
    pub fault_flags: u32,
    pub safety_events: u16,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ControlPeakTelemetry {
    pub maximum_phase_current_abs: u16,
    pub maximum_direct_current_abs: u16,
    pub maximum_quadrature_error_abs: u16,
    pub maximum_pwm_span_ticks: u16,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ControlPeakEventTelemetry {
    pub generation: u8,
    pub measured_d_counts: i16,
    pub measured_q_counts: i16,
    pub target_q_counts: i16,
    pub hall_raw: u8,
    pub hall_angle_direction: i8,
    pub edge_age_us: u16,
    pub hall_interval_us: u16,
    pub measurement_angle_q16: u16,
    pub unlimited_angle_q16: u16,
    pub phase_a_counts: i16,
    pub phase_b_counts: i16,
    pub applied_d_ticks: i16,
    pub applied_q_ticks: i16,
    pub voltage_limited: bool,
    pub angle_rate_limited: bool,
}

/// The two passive commissioning pages retain the established `0x2F7` page-8
/// and page-9 field locations without claiming the complete ride telemetry
/// schema. They never grant motor authority.
pub fn passive_input_telemetry(page_index: u8, snapshot: PassiveInputTelemetry) -> Frame {
    if page_index & 1 == 0 {
        let raw = snapshot.throttle_raw.to_le_bytes();
        let current_limit = (snapshot.throttle_current_limit_counts / 2).to_le_bytes();
        let interlocks = u8::from(snapshot.throttle_at_rest)
            | (u8::from(snapshot.brake_active) << 1)
            | (u8::from(snapshot.analog_valid) << 2)
            | (u8::from(snapshot.throttle_valid) << 4);
        Frame::new(
            0x2f7,
            8,
            [
                8,
                0,
                raw[0],
                raw[1],
                snapshot.throttle_demand,
                current_limit[0],
                interlocks,
                (snapshot.live_hall_state & 7) | ((current_limit[1] & 1) << 3),
            ],
        )
    } else {
        let bus_voltage = (snapshot.bus_voltage_mv / 100).to_le_bytes();
        // Byte 1 remains a saturating legacy view. Byte 3 carries the exact
        // overflow above 255 so widened limits remain lossless to new tools.
        let legacy_current_limit = snapshot.effective_current_limit.min(u16::from(u8::MAX)) as u8;
        let current_limit_overflow = snapshot
            .effective_current_limit
            .saturating_sub(u16::from(u8::MAX))
            .min(u16::from(u8::MAX)) as u8;
        Frame::new(
            0x2f7,
            8,
            [
                9,
                legacy_current_limit,
                snapshot.derating_reasons,
                current_limit_overflow,
                bus_voltage[0],
                bus_voltage[1],
                encode_temperature(snapshot.controller_temperature_deci_c),
                encode_temperature(snapshot.motor_temperature_deci_c),
            ],
        )
    }
}

/// Established page 6 fields used by the bench tooling to verify that the
/// 16 kHz ISR has margin on the target MCU.
pub fn control_timing_telemetry(snapshot: ControlTimingTelemetry) -> Frame {
    let current_trips = snapshot
        .current_trips
        .min(u32::from(u16::MAX))
        .to_le_bytes();
    let maximum_cycles = snapshot
        .maximum_cycles
        .min(u32::from(u16::MAX))
        .to_le_bytes();
    let warnings = snapshot
        .warning_count
        .min(u32::from(u16::MAX))
        .to_le_bytes();
    Frame::new(
        0x2f7,
        8,
        [
            6,
            0,
            current_trips[0],
            current_trips[1],
            maximum_cycles[0],
            maximum_cycles[1],
            warnings[0],
            warnings[1],
        ],
    )
}

/// Page 10: instantaneous current-loop demand and response. The ride stage is
/// packed into the upper flag nibble and therefore remains limited to 0--7.
pub fn control_live_telemetry(snapshot: ControlLiveTelemetry) -> Frame {
    let flags = u8::from(snapshot.hall_valid)
        | (u8::from(snapshot.current_valid) << 1)
        | (u8::from(snapshot.output_active) << 2)
        | (u8::from(snapshot.voltage_limited) << 3)
        | ((snapshot.ride_stage & 7) << 4);
    let target = snapshot.target_q_counts.to_le_bytes();
    let direct = snapshot.measured_d_counts.to_le_bytes();
    let quadrature = snapshot.measured_q_counts.to_le_bytes();
    Frame::new(
        0x2f7,
        8,
        [
            10,
            flags,
            target[0],
            target[1],
            direct[0],
            direct[1],
            quadrature[0],
            quadrature[1],
        ],
    )
}

/// Page 11: instantaneous dynamic current limit and applied modulation. PWM
/// span is encoded in 16-tick units so the complete timer range fits in one
/// byte.
pub fn control_output_telemetry(snapshot: ControlOutputTelemetry) -> Frame {
    let phase_limit = snapshot.phase_limit_counts.to_le_bytes();
    let direct = snapshot.applied_d_ticks.to_le_bytes();
    let quadrature = snapshot.applied_q_ticks.to_le_bytes();
    Frame::new(
        0x2f7,
        8,
        [
            11,
            phase_limit[0],
            phase_limit[1],
            direct[0],
            direct[1],
            quadrature[0],
            quadrature[1],
            (snapshot.pwm_span_ticks / 16).min(u16::from(u8::MAX)) as u8,
        ],
    )
}

/// Page 12: the complete internal hardware/software fault mask and saturated
/// safety-loss count.
pub fn control_fault_telemetry(snapshot: ControlFaultTelemetry) -> Frame {
    let faults = snapshot.fault_flags.to_le_bytes();
    let safety_events = snapshot.safety_events.to_le_bytes();
    Frame::new(
        0x2f7,
        8,
        [
            12,
            faults[0],
            faults[1],
            faults[2],
            faults[3],
            safety_events[0],
            safety_events[1],
            0,
        ],
    )
}

/// Page 13: boot-retained control peaks. PWM span uses the same 16-tick scale
/// as page 11.
pub fn control_peak_telemetry(snapshot: ControlPeakTelemetry) -> Frame {
    let phase = snapshot.maximum_phase_current_abs.to_le_bytes();
    let direct = snapshot.maximum_direct_current_abs.to_le_bytes();
    let quadrature = snapshot.maximum_quadrature_error_abs.to_le_bytes();
    Frame::new(
        0x2f7,
        8,
        [
            13,
            phase[0],
            phase[1],
            direct[0],
            direct[1],
            quadrature[0],
            quadrature[1],
            (snapshot.maximum_pwm_span_ticks / 16).min(u16::from(u8::MAX)) as u8,
        ],
    )
}

/// Pages 14--17: fields captured from the exact 16 kHz cycle that established
/// the boot-retained maximum |d|. Every page repeats the event generation so
/// readers never combine fields from two different peaks.
pub fn control_peak_event_telemetry(page_index: u8, snapshot: ControlPeakEventTelemetry) -> Frame {
    match page_index {
        0 => {
            let direct = snapshot.measured_d_counts.to_le_bytes();
            let quadrature = snapshot.measured_q_counts.to_le_bytes();
            let target = snapshot.target_q_counts.to_le_bytes();
            Frame::new(
                0x2f7,
                8,
                [
                    14,
                    snapshot.generation,
                    direct[0],
                    direct[1],
                    quadrature[0],
                    quadrature[1],
                    target[0],
                    target[1],
                ],
            )
        }
        1 => {
            let edge_age = snapshot.edge_age_us.to_le_bytes();
            let interval = snapshot.hall_interval_us.to_le_bytes();
            let hall_flags = (snapshot.hall_raw & 7)
                | (u8::from(snapshot.hall_angle_direction < 0) << 3)
                | (u8::from(snapshot.hall_angle_direction > 0) << 4)
                | (u8::from(snapshot.voltage_limited) << 5)
                | (u8::from(snapshot.angle_rate_limited) << 6);
            let angle_error_q8 = (snapshot
                .unlimited_angle_q16
                .wrapping_sub(snapshot.measurement_angle_q16)
                as i16
                >> 8) as u8;
            Frame::new(
                0x2f7,
                8,
                [
                    15,
                    snapshot.generation,
                    hall_flags,
                    edge_age[0],
                    edge_age[1],
                    interval[0],
                    interval[1],
                    angle_error_q8,
                ],
            )
        }
        2 => {
            let phase_a = snapshot.phase_a_counts.to_le_bytes();
            let phase_b = snapshot.phase_b_counts.to_le_bytes();
            let direct = snapshot.applied_d_ticks.to_le_bytes();
            Frame::new(
                0x2f7,
                8,
                [
                    16,
                    snapshot.generation,
                    phase_a[0],
                    phase_a[1],
                    phase_b[0],
                    phase_b[1],
                    direct[0],
                    direct[1],
                ],
            )
        }
        _ => {
            let quadrature = snapshot.applied_q_ticks.to_le_bytes();
            let measurement_angle = snapshot.measurement_angle_q16.to_le_bytes();
            let unlimited_angle = snapshot.unlimited_angle_q16.to_le_bytes();
            Frame::new(
                0x2f7,
                8,
                [
                    17,
                    snapshot.generation,
                    quadrature[0],
                    quadrature[1],
                    measurement_angle[0],
                    measurement_angle[1],
                    unlimited_angle[0],
                    unlimited_angle[1],
                ],
            )
        }
    }
}

/// Page 18 identifies the exact crate version and project telemetry schema in
/// ride logs without changing the stock identity responses.
pub fn firmware_version_telemetry() -> Frame {
    let version = env!("CARGO_PKG_VERSION").as_bytes();
    let mut data = [0_u8; 8];
    data[0] = 18;
    data[1] = PROJECT_TELEMETRY_SCHEMA;
    let mut index = 0;
    while index < 6 && index < version.len() {
        data[index + 2] = version[index];
        index += 1;
    }
    Frame::new(0x2f7, 8, data)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn updater_magic_requires_exact_prefix_and_minimum_length() {
        let valid = Frame::new(0x67f, 5, [0xaa, 0x55, 0x2a, 0, 0x2a, 0, 0, 0]);
        assert!(is_updater_reset(valid));
        assert!(is_updater_reset(Frame::new(
            0x67f,
            8,
            [0xaa, 0x55, 0x2a, 0, 0x2a, 0, 0x55, 0xaa],
        )));
        assert!(!is_updater_reset(Frame { id: 0x67e, ..valid }));
        assert!(!is_updater_reset(Frame { len: 4, ..valid }));
        assert!(!is_updater_reset(Frame::new(
            0x67f,
            8,
            [0xaa, 0x55, 0x2a, 0, 0, 0, 0x55, 0xaa],
        )));
    }

    #[test]
    fn identity_matches_stock_capture() {
        assert_eq!(identity_response(0x210).unwrap().data, *b"S73RX_22");
        assert_eq!(identity_response(0x211).unwrap().data, *b"05060002");
        assert_eq!(identity_response(0x212).unwrap().len, 4);
    }

    #[test]
    fn telemetry_schedule_contains_required_ids() {
        let mut found = [false; 8];
        let ids = [0x200, 0x201, 0x202, 0x203, 0x204, 0x265, 0x266, 0x64a];
        for slot in 0..60 {
            for frame in telemetry_slot(slot, StockTelemetry::default())
                .into_iter()
                .flatten()
            {
                for (index, id) in ids.iter().enumerate() {
                    found[index] |= frame.id == *id;
                }
            }
        }
        assert!(found.into_iter().all(|value| value));
    }

    #[test]
    fn telemetry_schedule_keeps_its_sixty_slot_phase_across_u8_wrap() {
        let mut slot = 0;
        let mut visits = [0_u16; 60];
        for _ in 0..600 {
            visits[usize::from(slot)] += 1;
            slot = next_telemetry_slot(slot);
        }
        assert_eq!(slot, 0);
        assert!(visits.into_iter().all(|count| count == 10));
    }

    #[test]
    fn project_telemetry_rotates_through_all_twelve_pages() {
        let mut page = 0;
        let mut visits = [0_u8; 12];
        for _ in 0..120 {
            visits[usize::from(page)] += 1;
            page = next_project_telemetry_page(page);
        }
        assert_eq!(page, 0);
        assert!(visits.into_iter().all(|count| count == 10));
    }

    #[test]
    fn local_brake_temperatures_and_faults_reach_stock_frames() {
        let snapshot = StockTelemetry {
            vehicle_speed_tenths_kph: 14,
            distance_counter: 0x1234_5678,
            brake_active: true,
            controller_temperature_deci_c: Some(410),
            motor_temperature_deci_c: Some(270),
            fault_page: 0,
            hardware_break: true,
            dc_bus_undervoltage: false,
            hall_invalid: true,
        };
        assert_eq!(
            telemetry_slot(0, snapshot)[0].unwrap().data,
            [0x8c, 0, 0, 0, 0x64, 0, 0, 0]
        );
        assert_eq!(telemetry_slot(0, snapshot)[1].unwrap().data[4], 0x40);
        assert_eq!(telemetry_slot(1, snapshot)[1].unwrap().data[6], 0x48);
        assert_eq!(
            telemetry_slot(2, snapshot)[2].unwrap().data,
            [0x22, 67, 0x78, 0x56, 0x78, 0x56, 0x34, 0x12]
        );
        assert_eq!(telemetry_slot(3, snapshot)[1].unwrap().data[0], 81);
    }

    #[test]
    fn stock_fault_pages_report_local_protection_state() {
        let mut snapshot = StockTelemetry {
            controller_temperature_deci_c: Some(800),
            motor_temperature_deci_c: Some(1_300),
            hardware_break: true,
            dc_bus_undervoltage: true,
            hall_invalid: true,
            ..StockTelemetry::default()
        };
        assert_eq!(telemetry_slot(1, snapshot)[1].unwrap().data[6], 0x58);
        snapshot.fault_page = 1;
        assert_eq!(telemetry_slot(1, snapshot)[1].unwrap().data[6], 0xc1);
        snapshot.fault_page = 2;
        assert_eq!(telemetry_slot(1, snapshot)[1].unwrap().data[6], 0x82);
    }

    #[test]
    fn passive_input_pages_use_the_established_field_locations() {
        let snapshot = PassiveInputTelemetry {
            analog_valid: true,
            brake_active: false,
            throttle_valid: true,
            throttle_at_rest: true,
            throttle_raw: 726,
            throttle_demand: 0,
            throttle_current_limit_counts: 838,
            live_hall_state: 5,
            bus_voltage_mv: 52_300,
            effective_current_limit: 250,
            derating_reasons: 0,
            controller_temperature_deci_c: Some(410),
            motor_temperature_deci_c: Some(270),
        };
        assert_eq!(
            passive_input_telemetry(0, snapshot).data,
            [8, 0, 0xd6, 0x02, 0, 0xa3, 0x15, 0x0d]
        );
        assert_eq!(
            passive_input_telemetry(1, snapshot).data,
            [9, 250, 0, 0, 0x0b, 0x02, 81, 67]
        );
    }

    #[test]
    fn passive_input_page_preserves_wide_dc_current_limits() {
        let snapshot = PassiveInputTelemetry {
            effective_current_limit: 480,
            ..PassiveInputTelemetry::default()
        };
        assert_eq!(
            passive_input_telemetry(1, snapshot).data[..4],
            [9, 255, 0, 225]
        );
    }

    #[test]
    fn timing_page_uses_the_established_field_locations() {
        assert_eq!(
            control_timing_telemetry(ControlTimingTelemetry {
                current_trips: 1,
                maximum_cycles: 1_396,
                warning_count: 2,
            })
            .data,
            [6, 0, 1, 0, 0x74, 0x05, 2, 0]
        );
    }

    #[test]
    fn control_commissioning_pages_are_compact_and_lossless() {
        let live = ControlLiveTelemetry {
            hall_valid: true,
            current_valid: true,
            output_active: true,
            voltage_limited: true,
            ride_stage: 4,
            target_q_counts: -838,
            measured_d_counts: -17,
            measured_q_counts: -801,
        };
        assert_eq!(
            control_live_telemetry(live).data,
            [10, 0x4f, 0xba, 0xfc, 0xef, 0xff, 0xdf, 0xfc]
        );

        let output = ControlOutputTelemetry {
            phase_limit_counts: 838,
            applied_d_ticks: -123,
            applied_q_ticks: -1_240,
            pwm_span_ticks: 2_168,
        };
        assert_eq!(
            control_output_telemetry(output).data,
            [11, 0x46, 0x03, 0x85, 0xff, 0x28, 0xfb, 135]
        );

        let faults = ControlFaultTelemetry {
            fault_flags: 0x1234_5678,
            safety_events: 0xabcd,
        };
        assert_eq!(
            control_fault_telemetry(faults).data,
            [12, 0x78, 0x56, 0x34, 0x12, 0xcd, 0xab, 0]
        );

        let peaks = ControlPeakTelemetry {
            maximum_phase_current_abs: 1_337,
            maximum_direct_current_abs: 55,
            maximum_quadrature_error_abs: 222,
            maximum_pwm_span_ticks: 2_168,
        };
        assert_eq!(
            control_peak_telemetry(peaks).data,
            [13, 0x39, 0x05, 55, 0, 222, 0, 135]
        );

        let event = ControlPeakEventTelemetry {
            generation: 7,
            measured_d_counts: -782,
            measured_q_counts: -358,
            target_q_counts: -732,
            hall_raw: 5,
            hall_angle_direction: -1,
            edge_age_us: 321,
            hall_interval_us: 1_234,
            measurement_angle_q16: 0x1234,
            unlimited_angle_q16: 0x2345,
            phase_a_counts: 900,
            phase_b_counts: -420,
            applied_d_ticks: -123,
            applied_q_ticks: -1_273,
            voltage_limited: true,
            angle_rate_limited: true,
        };
        assert_eq!(
            control_peak_event_telemetry(0, event).data,
            [14, 7, 0xf2, 0xfc, 0x9a, 0xfe, 0x24, 0xfd]
        );
        assert_eq!(
            control_peak_event_telemetry(1, event).data,
            [15, 7, 0x6d, 0x41, 0x01, 0xd2, 0x04, 0x11]
        );
        assert_eq!(
            control_peak_event_telemetry(2, event).data,
            [16, 7, 0x84, 0x03, 0x5c, 0xfe, 0x85, 0xff]
        );
        assert_eq!(
            control_peak_event_telemetry(3, event).data,
            [17, 7, 0x07, 0xfb, 0x34, 0x12, 0x45, 0x23]
        );
        assert_eq!(
            firmware_version_telemetry().data,
            [18, 2, b'0', b'.', b'1', b'.', b'1', 0]
        );
    }
}
