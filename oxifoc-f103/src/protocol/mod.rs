//! Minimal stock-bike CAN wire contract.

use crate::config::CAN_FOC_STATUS_ID;
use crate::sensors::environment::{
    CONTROLLER_DERATE_START_DECI_C, CONTROLLER_STOP_DECI_C, MOTOR_DERATE_START_DECI_C,
    MOTOR_STOP_DECI_C, derating, stopped,
};

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

pub const fn headlight_command(frame: Frame) -> Option<bool> {
    if frame.id == 0x300 && frame.len >= 3 {
        Some(frame.data[2] != 0)
    } else {
        None
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct StockTelemetry {
    pub vehicle_speed_tenths_kph: u16,
    pub distance_counter: u32,
    pub cadence_rpm: u16,
    /// `0x201.b4` input bits: headlight in bit 1 and brake in bit 2.
    pub digital_input_flags: u8,
    pub controller_temperature_deci_c: Option<i16>,
    pub motor_temperature_deci_c: Option<i16>,
    pub fault_page: u8,
    pub hardware_break: bool,
    pub dc_bus_undervoltage: bool,
    pub hall_invalid: bool,
}

pub fn telemetry_slot(slot: u8, snapshot: StockTelemetry) -> [Option<Frame>; 3] {
    let motion_flags = 0x60 | snapshot.digital_input_flags;
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
            ((snapshot.digital_input_flags & 4) << 4) | ((snapshot.digital_input_flags & 2) << 2),
            (snapshot.digital_input_flags & 2) << 4,
            0,
            0,
        ],
    );
    let primary = if slot & 1 == 0 {
        [Some(motion), Some(profile), None]
    } else {
        let cadence = snapshot.cadence_rpm.saturating_mul(40).to_le_bytes();
        [
            Some(Frame::new(
                0x203,
                8,
                [cadence[0], cadence[1], 0, 0, 0, 0, 0, 0],
            )),
            None,
            None,
        ]
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
    if page >= 14 { 0 } else { page + 1 }
}

pub const PROJECT_TELEMETRY_SCHEMA: u8 = 16;

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
                word |= u16::from(derating(temperature, CONTROLLER_DERATE_START_DECI_C)) << 6;
                word |= u16::from(stopped(temperature, CONTROLLER_STOP_DECI_C)) << 7;
            }
        }
        2 => {
            if let Some(temperature) = snapshot.motor_temperature_deci_c {
                let stop = stopped(temperature, MOTOR_STOP_DECI_C);
                word |= u16::from(derating(temperature, MOTOR_DERATE_START_DECI_C) && !stop) << 6;
                word |= u16::from(stop) << 7;
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
    pub throttle_demand_percent: u8,
    pub throttle_current_limit_counts: u16,
    pub cadence_active: bool,
    pub cadence_pin_low: bool,
    pub cadence_raw_pin_low: bool,
    pub cadence_demand_percent: u8,
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
    pub transient_budget_ms: u16,
    pub applied_d_ticks: i16,
    pub applied_q_ticks: i16,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ControlFaultTelemetry {
    pub fault_flags: u32,
    pub fault_lockout: bool,
    pub safety_events: u16,
    pub last_safety_loss_reason: u8,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ControlPeakTelemetry {
    pub maximum_phase_current_abs: u16,
    pub maximum_direct_current_abs: u16,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct FieldWeakeningTelemetry {
    pub active: bool,
    pub at_current_limit: bool,
    pub voltage_limited: bool,
    pub target_d_counts: i16,
    pub requested_d_ticks: i16,
    pub requested_q_ticks: i16,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ObserverStatusTelemetry {
    pub configured: bool,
    pub ready: bool,
    pub active: bool,
    pub blend: u8,
    pub confidence: u8,
    pub electrical_rpm: i16,
    pub hall_error_q16: i16,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ObserverModelTelemetry {
    pub flux_centi_mwb: u16,
    pub bemf_q_mv: i16,
    pub phase_error_q16: u16,
    pub validity_progress: u8,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ResetSummaryTelemetry {
    pub reset_flags: u8,
    pub retained_context_valid: bool,
    pub fatal_reason: u8,
    pub checkpoint: u8,
    pub last_control_cycles: u16,
    pub maximum_control_cycles: u16,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CrashContextTelemetry {
    pub detail: i16,
    pub control_cycle: u32,
    pub program_counter: u32,
    pub link_register: u32,
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
            | (u8::from(snapshot.throttle_valid) << 4)
            | (u8::from(snapshot.cadence_active) << 5)
            | (u8::from(snapshot.cadence_pin_low) << 6)
            | (u8::from(snapshot.cadence_raw_pin_low) << 7);
        Frame::new(
            CAN_FOC_STATUS_ID,
            8,
            [
                8,
                snapshot.cadence_demand_percent,
                raw[0],
                raw[1],
                snapshot.throttle_demand_percent,
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
            CAN_FOC_STATUS_ID,
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

/// Page 6 reports the hard current-trip count and worst 16 kHz control time.
pub fn control_timing_telemetry(snapshot: ControlTimingTelemetry) -> Frame {
    let current_trips = snapshot
        .current_trips
        .min(u32::from(u16::MAX))
        .to_le_bytes();
    let maximum_cycles = snapshot
        .maximum_cycles
        .min(u32::from(u16::MAX))
        .to_le_bytes();
    Frame::new(
        CAN_FOC_STATUS_ID,
        8,
        [
            6,
            0,
            current_trips[0],
            current_trips[1],
            maximum_cycles[0],
            maximum_cycles[1],
            0,
            0,
        ],
    )
}

/// Page 10: instantaneous current-loop demand and response. The ride stage is
/// packed into flag bits 4--6 and therefore remains limited to 0--7; every
/// value is used.
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
        CAN_FOC_STATUS_ID,
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

/// Page 11 reports the remaining full-transient-equivalent time and the
/// applied dq modulation used by the local DC-current projection.
pub fn control_output_telemetry(snapshot: ControlOutputTelemetry) -> Frame {
    let transient_budget = snapshot.transient_budget_ms.to_le_bytes();
    let direct = snapshot.applied_d_ticks.to_le_bytes();
    let quadrature = snapshot.applied_q_ticks.to_le_bytes();
    Frame::new(
        CAN_FOC_STATUS_ID,
        8,
        [
            11,
            transient_budget[0],
            transient_budget[1],
            direct[0],
            direct[1],
            quadrature[0],
            quadrature[1],
            0,
        ],
    )
}

/// Page 12: the complete internal hardware/software fault mask, saturated
/// safety-loss count, and most recent loss reason. The mask also carries the
/// ride retry lockout as `fault_flag::RETRY_LOCKOUT`.
pub fn control_fault_telemetry(snapshot: ControlFaultTelemetry) -> Frame {
    let lockout = if snapshot.fault_lockout {
        crate::safety::fault_flag::RETRY_LOCKOUT
    } else {
        0
    };
    let faults = (snapshot.fault_flags | lockout).to_le_bytes();
    let safety_events = snapshot.safety_events.to_le_bytes();
    Frame::new(
        CAN_FOC_STATUS_ID,
        8,
        [
            12,
            faults[0],
            faults[1],
            faults[2],
            faults[3],
            safety_events[0],
            safety_events[1],
            snapshot.last_safety_loss_reason,
        ],
    )
}

/// Page 13 reports the boot-retained phase-current peak and the current fault
/// episode's maximum |d|.
pub fn control_peak_telemetry(snapshot: ControlPeakTelemetry) -> Frame {
    let phase = snapshot.maximum_phase_current_abs.to_le_bytes();
    let direct = snapshot.maximum_direct_current_abs.to_le_bytes();
    Frame::new(
        CAN_FOC_STATUS_ID,
        8,
        [13, phase[0], phase[1], direct[0], direct[1], 0, 0, 0],
    )
}

/// Page 14: field-weakening current target and the pre-limit voltage request
/// that drives its saturation feedback.
pub fn field_weakening_telemetry(snapshot: FieldWeakeningTelemetry) -> Frame {
    let flags = u8::from(snapshot.active)
        | (u8::from(snapshot.at_current_limit) << 1)
        | (u8::from(snapshot.voltage_limited) << 2);
    let target = snapshot.target_d_counts.to_le_bytes();
    let direct = snapshot.requested_d_ticks.to_le_bytes();
    let quadrature = snapshot.requested_q_ticks.to_le_bytes();
    Frame::new(
        CAN_FOC_STATUS_ID,
        8,
        [
            14,
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

/// Page 19: active Hall-to-observer handoff state and angle disagreement.
pub fn observer_status_telemetry(snapshot: ObserverStatusTelemetry) -> Frame {
    let flags = u8::from(snapshot.configured)
        | (u8::from(snapshot.ready) << 1)
        | (u8::from(snapshot.active) << 2);
    let rpm = snapshot.electrical_rpm.to_le_bytes();
    let hall_error = snapshot.hall_error_q16.to_le_bytes();
    Frame::new(
        CAN_FOC_STATUS_ID,
        8,
        [
            19,
            flags,
            snapshot.blend,
            snapshot.confidence,
            rpm[0],
            rpm[1],
            hall_error[0],
            hall_error[1],
        ],
    )
}

/// Page 20: observer model magnitude, back-EMF corroboration, PLL lock error,
/// and progress toward the two-revolution external-validity threshold.
pub fn observer_model_telemetry(snapshot: ObserverModelTelemetry) -> Frame {
    let flux = snapshot.flux_centi_mwb.to_le_bytes();
    let bemf = snapshot.bemf_q_mv.to_le_bytes();
    let phase_error = snapshot.phase_error_q16.to_le_bytes();
    Frame::new(
        CAN_FOC_STATUS_ID,
        8,
        [
            20,
            flux[0],
            flux[1],
            bemf[0],
            bemf[1],
            phase_error[0],
            phase_error[1],
            snapshot.validity_progress,
        ],
    )
}

/// Page 22 identifies why the MCU reset and, for watchdog resets, where the
/// preceding control interrupt last made progress. Bit 7 marks retained
/// context as valid; the low six bits retain the decoded RCC reset causes.
pub fn reset_summary_telemetry(snapshot: ResetSummaryTelemetry) -> Frame {
    let flags = (snapshot.reset_flags & 0x3f) | (u8::from(snapshot.retained_context_valid) << 7);
    let last = snapshot.last_control_cycles.to_le_bytes();
    let maximum = snapshot.maximum_control_cycles.to_le_bytes();
    Frame::new(
        CAN_FOC_STATUS_ID,
        8,
        [
            22,
            flags,
            snapshot.fatal_reason,
            snapshot.checkpoint,
            last[0],
            last[1],
            maximum[0],
            maximum[1],
        ],
    )
}

/// Page 23 carries compact exception context from the preceding watchdog
/// reset. Application addresses fit below 0x0801_0000, so their low 16 bits
/// identify the exact instruction in this application image. The low 16 bits
/// also distinguish normal link addresses from Cortex-M EXC_RETURN values.
pub fn crash_context_telemetry(snapshot: CrashContextTelemetry) -> Frame {
    let cycle = snapshot.control_cycle.to_le_bytes();
    let pc = snapshot.program_counter.to_le_bytes();
    let lr = snapshot.link_register.to_le_bytes();
    Frame::new(
        CAN_FOC_STATUS_ID,
        8,
        [
            23,
            snapshot
                .detail
                .clamp(i16::from(i8::MIN), i16::from(i8::MAX)) as i8 as u8,
            cycle[0],
            cycle[1],
            pc[0],
            pc[1],
            lr[0],
            lr[1],
        ],
    )
}

/// Pages 24 and 25 preserve the first rejected output state from the latest
/// fault episode. Page 24 carries the predicate and relevant TIM1 state; page
/// 25 carries all three attempted compares.
pub fn pwm_failure_telemetry(page: u8, words: [u32; 4]) -> Frame {
    let header = words[0].to_le_bytes();
    let timer = words[1].to_le_bytes();
    let ccer_a = words[2].to_le_bytes();
    let b_c = words[3].to_le_bytes();
    let data = if page == 24 {
        [
            24, header[0], header[1], header[2], timer[0], timer[3], ccer_a[0], ccer_a[1],
        ]
    } else {
        [
            25, header[0], ccer_a[2], ccer_a[3], b_c[0], b_c[1], b_c[2], b_c[3],
        ]
    };
    Frame::new(CAN_FOC_STATUS_ID, 8, data)
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
    Frame::new(CAN_FOC_STATUS_ID, 8, data)
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
    fn display_headlight_command_requires_the_stock_byte() {
        let enabled = Frame::new(0x300, 8, [3, 0x5a, 1, 0x5a, 0, 0, 100, 0]);
        let disabled = Frame::new(0x300, 8, [3, 0x5a, 0, 0x5a, 0, 0, 100, 0]);
        assert_eq!(headlight_command(enabled), Some(true));
        assert_eq!(headlight_command(disabled), Some(false));
        assert_eq!(
            headlight_command(Frame {
                id: 0x301,
                ..enabled
            }),
            None
        );
        assert_eq!(headlight_command(Frame { len: 3, ..enabled }), Some(true));
        assert_eq!(headlight_command(Frame { len: 2, ..enabled }), None);
        assert_eq!(headlight_command(Frame { len: 0, ..enabled }), None);
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
    fn stock_power_status_reports_cadence_rpm_times_forty() {
        let snapshot = StockTelemetry {
            cadence_rpm: 75,
            ..StockTelemetry::default()
        };
        assert_eq!(
            telemetry_slot(1, snapshot)[0].unwrap(),
            Frame::new(0x203, 8, [0xb8, 0x0b, 0, 0, 0, 0, 0, 0])
        );
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
    fn project_telemetry_rotates_through_all_fifteen_slots() {
        let mut page = 0;
        let mut visits = [0_u8; 15];
        for _ in 0..150 {
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
            cadence_rpm: 0,
            digital_input_flags: 4,
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
    fn headlight_state_reaches_both_stock_status_frames() {
        let snapshot = StockTelemetry {
            digital_input_flags: 6,
            ..StockTelemetry::default()
        };
        assert_eq!(telemetry_slot(0, snapshot)[0].unwrap().data[4], 0x66);
        assert_eq!(telemetry_slot(0, snapshot)[1].unwrap().data[4], 0x48);
        assert_eq!(telemetry_slot(0, snapshot)[1].unwrap().data[5], 0x20);
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
    fn stock_temperature_fault_bits_follow_the_configured_derating_envelope() {
        let thermal = crate::config::BIKE_CONFIG.thermal;
        let controller_start = thermal.controller_derate_start_celsius * 10;
        let controller_stop = thermal.controller_stop_celsius * 10;
        let motor_start = thermal.motor_derate_start_celsius * 10;
        let motor_stop = thermal.motor_stop_celsius * 10;
        let fault_bits = |fault_page, controller, motor| {
            let snapshot = StockTelemetry {
                fault_page,
                controller_temperature_deci_c: Some(controller),
                motor_temperature_deci_c: Some(motor),
                ..StockTelemetry::default()
            };
            telemetry_slot(1, snapshot)[1].unwrap().data[6] & 0xc0
        };

        assert_eq!(fault_bits(1, controller_start, 0), 0);
        assert_eq!(fault_bits(1, controller_start + 1, 0), 0x40);
        assert_eq!(fault_bits(1, controller_stop - 1, 0), 0x40);
        assert_eq!(fault_bits(1, controller_stop, 0), 0xc0);

        assert_eq!(fault_bits(2, 0, motor_start), 0);
        assert_eq!(fault_bits(2, 0, motor_start + 1), 0x40);
        assert_eq!(fault_bits(2, 0, motor_stop - 1), 0x40);
        assert_eq!(fault_bits(2, 0, motor_stop), 0x80);
    }

    #[test]
    fn passive_input_pages_use_the_established_field_locations() {
        let snapshot = PassiveInputTelemetry {
            analog_valid: true,
            brake_active: false,
            throttle_valid: true,
            throttle_at_rest: true,
            throttle_raw: 726,
            throttle_demand_percent: 0,
            throttle_current_limit_counts: 838,
            cadence_active: true,
            cadence_pin_low: false,
            cadence_raw_pin_low: true,
            cadence_demand_percent: 44,
            live_hall_state: 5,
            bus_voltage_mv: 52_300,
            effective_current_limit: 250,
            derating_reasons: 0,
            controller_temperature_deci_c: Some(410),
            motor_temperature_deci_c: Some(270),
        };
        assert_eq!(
            passive_input_telemetry(0, snapshot).data,
            [8, 44, 0xd6, 0x02, 0, 0xa3, 0xb5, 0x0d]
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

        let snapshot = PassiveInputTelemetry {
            effective_current_limit: 640,
            ..PassiveInputTelemetry::default()
        };
        assert_eq!(
            passive_input_telemetry(1, snapshot).data[..4],
            [9, 255, 0, 255]
        );
    }

    #[test]
    fn timing_page_uses_the_established_field_locations() {
        assert_eq!(
            control_timing_telemetry(ControlTimingTelemetry {
                current_trips: 1,
                maximum_cycles: 1_396,
            })
            .data,
            [6, 0, 1, 0, 0x74, 0x05, 0, 0]
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

        for stage in [6, 7] {
            let held = ControlLiveTelemetry {
                ride_stage: stage,
                ..live
            };
            assert_eq!(control_live_telemetry(held).data[1], 0x0f | (stage << 4));
        }

        let output = ControlOutputTelemetry {
            transient_budget_ms: 0x1234,
            applied_d_ticks: -123,
            applied_q_ticks: -1_240,
        };
        assert_eq!(
            control_output_telemetry(output).data,
            [11, 0x34, 0x12, 0x85, 0xff, 0x28, 0xfb, 0]
        );

        let faults = ControlFaultTelemetry {
            fault_flags: 0x1234_5678,
            fault_lockout: false,
            safety_events: 0xabcd,
            last_safety_loss_reason: 7,
        };
        assert_eq!(
            control_fault_telemetry(faults).data,
            [12, 0x78, 0x56, 0x34, 0x12, 0xcd, 0xab, 7]
        );
        let locked = ControlFaultTelemetry {
            fault_flags: crate::safety::fault_flag::HARDWARE_BREAK,
            fault_lockout: true,
            ..faults
        };
        assert_eq!(
            control_fault_telemetry(locked).data,
            [12, 0x81, 0, 0, 0, 0xcd, 0xab, 7]
        );

        let peaks = ControlPeakTelemetry {
            maximum_phase_current_abs: 1_337,
            maximum_direct_current_abs: 55,
        };
        assert_eq!(
            control_peak_telemetry(peaks).data,
            [13, 0x39, 0x05, 55, 0, 0, 0, 0]
        );

        let weakening = FieldWeakeningTelemetry {
            active: true,
            at_current_limit: false,
            voltage_limited: true,
            target_d_counts: -173,
            requested_d_ticks: -600,
            requested_q_ticks: -1_800,
        };
        assert_eq!(
            field_weakening_telemetry(weakening).data,
            [14, 5, 0x53, 0xff, 0xa8, 0xfd, 0xf8, 0xf8]
        );
        assert_eq!(
            observer_status_telemetry(ObserverStatusTelemetry {
                configured: true,
                ready: true,
                active: true,
                blend: 192,
                confidence: 247,
                electrical_rpm: -7_500,
                hall_error_q16: -1_024,
            })
            .data,
            [19, 7, 192, 247, 0xb4, 0xe2, 0x00, 0xfc]
        );
        assert_eq!(
            observer_model_telemetry(ObserverModelTelemetry {
                flux_centi_mwb: 1_220,
                bemf_q_mv: -9_876,
                phase_error_q16: 456,
                validity_progress: 255,
            })
            .data,
            [20, 0xc4, 0x04, 0x6c, 0xd9, 0xc8, 0x01, 255]
        );
    }

    #[test]
    fn reset_forensics_pages_preserve_the_previous_boot_context() {
        assert_eq!(
            reset_summary_telemetry(ResetSummaryTelemetry {
                reset_flags: 0x18,
                retained_context_valid: true,
                fatal_reason: 2,
                checkpoint: 4,
                last_control_cycles: 4_321,
                maximum_control_cycles: 4_498,
            })
            .data,
            [22, 0x98, 2, 4, 0xe1, 0x10, 0x92, 0x11]
        );
        assert_eq!(
            crash_context_telemetry(CrashContextTelemetry {
                detail: -3,
                control_cycle: 0x1234_5678,
                program_counter: 0x0800_9abc,
                link_register: 0xffff_fff9,
            })
            .data,
            [23, 0xfd, 0x78, 0x56, 0xbc, 0x9a, 0xf9, 0xff]
        );

        let pwm = [
            u32::from_le_bytes([6, 0x0b, 3, 0]),
            u32::from(0x0081_u16) | (u32::from(0x9d19_u16) << 16),
            u32::from(0x1ddd_u16) | (u32::from(22_u16) << 16),
            u32::from(1_125_u16) | (u32::from(2_228_u16) << 16),
        ];
        assert_eq!(
            pwm_failure_telemetry(24, pwm).data,
            [24, 6, 0x0b, 3, 0x81, 0x9d, 0xdd, 0x1d]
        );
        assert_eq!(
            pwm_failure_telemetry(25, pwm).data,
            [25, 6, 22, 0, 0x65, 0x04, 0xb4, 0x08]
        );
    }
}
