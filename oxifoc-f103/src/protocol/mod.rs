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
    pub effective_current_limit: u8,
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
        Frame::new(
            0x2f7,
            8,
            [
                9,
                snapshot.effective_current_limit,
                snapshot.derating_reasons,
                0,
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
            throttle_current_limit_counts: 480,
            live_hall_state: 5,
            bus_voltage_mv: 52_300,
            effective_current_limit: 240,
            derating_reasons: 0,
            controller_temperature_deci_c: Some(410),
            motor_temperature_deci_c: Some(270),
        };
        assert_eq!(
            passive_input_telemetry(0, snapshot).data,
            [8, 0, 0xd6, 0x02, 0, 240, 0x15, 5]
        );
        assert_eq!(
            passive_input_telemetry(1, snapshot).data,
            [9, 240, 0, 0, 0x0b, 0x02, 81, 67]
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
}
