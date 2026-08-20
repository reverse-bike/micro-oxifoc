//! Compact CAN contract for an explicitly armed calibration image.

use oxifoc_f103::protocol::Frame;

use crate::calibration::inductance::{PULSE_DIAGNOSTIC_COUNT, PulseDiagnostic};

pub const SCHEMA: u8 = 4;
pub const FIRMWARE_VERSION: [u8; 3] = [0, 3, 2];
pub const ARM_WINDOW_MS: u32 = 10_000;
pub const COMMAND_ID: u16 = 0x2f2;
pub const STATUS_ID: u16 = 0x2f7;
pub const STATUS_PAGE_COUNT: u8 = 16;
const TRAILER: [u8; 2] = [0xa5, 0x5a];

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Command {
    Arm { challenge: u16 },
    RunResistance { challenge: u16 },
    RunInductance { challenge: u16 },
    RunFluxLinkage { challenge: u16 },
    RunHall { challenge: u16 },
    Stop,
}

pub fn decode_command(frame: Frame) -> Option<Command> {
    if frame.id != COMMAND_ID {
        return None;
    }
    if frame.len >= 4 && frame.data[..4] == *b"STOP" {
        return Some(Command::Stop);
    }
    if frame.len != 8 || frame.data[6..8] != TRAILER {
        return None;
    }
    let challenge = u16::from_le_bytes([frame.data[4], frame.data[5]]);
    match &frame.data[..4] {
        b"ARMC" => Some(Command::Arm { challenge }),
        b"RUNR" => Some(Command::RunResistance { challenge }),
        b"RUNL" => Some(Command::RunInductance { challenge }),
        b"RUNF" => Some(Command::RunFluxLinkage { challenge }),
        b"RUNH" => Some(Command::RunHall { challenge }),
        _ => None,
    }
}

pub const fn challenge(offset_a: u16, offset_b: u16, boot_nonce: u16) -> u16 {
    offset_a.rotate_left(5) ^ offset_b.rotate_right(3) ^ boot_nonce ^ 0xa55a
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Status {
    pub routine: u8,
    pub state: u8,
    pub failure: u8,
    pub challenge: u16,
    pub armed: bool,
    pub local_ready: bool,
    pub output_active: bool,
    pub voltage_limited: bool,
    pub hall_quiet: bool,
    pub motor_outputs_disabled: bool,
    pub can_rx_fifo_overrun: bool,
    pub can_command_queue_drop: bool,
    pub low_current_counts: i16,
    pub low_voltage_ticks: i16,
    pub high_current_counts: i16,
    pub high_voltage_ticks: i16,
    pub effective_uv_per_count: u32,
    pub nominal_resistance_uohm: u32,
    pub target_d_counts: i16,
    pub measured_d_counts: i16,
    pub applied_d_ticks: i16,
    pub maximum_phase_current_abs: u16,
    pub bus_voltage_mv: u16,
    pub offset_a: u16,
    pub offset_b: u16,
    pub environment_reasons: u8,
    pub sample_progress: u16,
    pub fault_flags: u32,
    pub maximum_control_cycles: u16,
    pub timing_overruns: u16,
    pub inductance_d_nwb_per_count: u32,
    pub inductance_q_nwb_per_count: u32,
    pub residual_dead_time_uv: u32,
    pub pulse_step_d_ticks: i16,
    pub pulse_step_q_ticks: i16,
    pub last_pulse_di_counts: i16,
    pub flux_linkage_nwb: u32,
    pub average_bemf_d_uv: i32,
    pub average_bemf_q_uv: i32,
    pub flux_measurement_erpm: i16,
    pub hall_measurement_erpm: i16,
    pub sync_minimum_percent: u8,
    pub hall_forward_centers_q16: [u16; 8],
    pub hall_reverse_centers_q16: [u16; 8],
    pub hall_valid_mask: u8,
    pub hall_forward_minimum_samples: u8,
    pub hall_reverse_minimum_samples: u8,
    pub pulse_diagnostic_slot: u8,
    pub pulse_diagnostic: PulseDiagnostic,
}

pub fn status_frame(page: u8, status: Status) -> Frame {
    let mut data = [0_u8; 8];
    data[0] = 0xc0 | (page % STATUS_PAGE_COUNT);
    match page % STATUS_PAGE_COUNT {
        0 => {
            data[1] = SCHEMA;
            data[2] = status.state;
            data[3] = status.failure;
            data[4..6].copy_from_slice(&status.challenge.to_le_bytes());
            data[6] = u8::from(status.armed)
                | (u8::from(status.local_ready) << 1)
                | (u8::from(status.output_active) << 2)
                | (u8::from(status.voltage_limited) << 3)
                | (u8::from(status.hall_quiet) << 4)
                | (u8::from(status.motor_outputs_disabled) << 5)
                | (u8::from(status.can_rx_fifo_overrun) << 6)
                | (u8::from(status.can_command_queue_drop) << 7);
            data[7] = status.routine;
        }
        1 => {
            data[1..3].copy_from_slice(&status.low_current_counts.to_le_bytes());
            data[3..5].copy_from_slice(&status.low_voltage_ticks.to_le_bytes());
            data[5..7].copy_from_slice(&status.high_current_counts.to_le_bytes());
        }
        2 => {
            data[1..5].copy_from_slice(&status.effective_uv_per_count.to_le_bytes());
            let nominal_milliohm =
                (status.nominal_resistance_uohm / 1_000).min(u32::from(u16::MAX)) as u16;
            data[5..7].copy_from_slice(&nominal_milliohm.to_le_bytes());
            data[7] = 100;
        }
        3 => {
            data[1..3].copy_from_slice(&status.target_d_counts.to_le_bytes());
            data[3..5].copy_from_slice(&status.measured_d_counts.to_le_bytes());
            data[5..7].copy_from_slice(&status.applied_d_ticks.to_le_bytes());
            data[7] = (status.maximum_phase_current_abs / 4).min(u16::from(u8::MAX)) as u8;
        }
        4 => {
            data[1..3].copy_from_slice(&status.bus_voltage_mv.to_le_bytes());
            data[3..5].copy_from_slice(&status.offset_a.to_le_bytes());
            data[5..7].copy_from_slice(&status.offset_b.to_le_bytes());
            data[7] = status.environment_reasons;
        }
        5 => {
            data[1..5].copy_from_slice(&status.fault_flags.to_le_bytes());
            data[5..7].copy_from_slice(&status.maximum_control_cycles.to_le_bytes());
            data[7] = status.timing_overruns.min(u16::from(u8::MAX)) as u8;
        }
        6 => {
            data[1..3].copy_from_slice(&status.high_voltage_ticks.to_le_bytes());
            data[3..5].copy_from_slice(&status.sample_progress.to_le_bytes());
            data[5..8].copy_from_slice(&FIRMWARE_VERSION);
        }
        7 => {
            data[1..5].copy_from_slice(&status.inductance_d_nwb_per_count.to_le_bytes());
            copy_u24(
                &mut data[5..8],
                status.inductance_q_nwb_per_count.min(0x00ff_ffff),
            );
        }
        8 => {
            let residual_dead_time_mv = (status.residual_dead_time_uv / 1_000) as u16;
            data[1..3].copy_from_slice(&residual_dead_time_mv.to_le_bytes());
            data[3..5].copy_from_slice(&status.pulse_step_d_ticks.to_le_bytes());
            data[5..7].copy_from_slice(&status.pulse_step_q_ticks.to_le_bytes());
            data[7] = status
                .last_pulse_di_counts
                .clamp(i16::from(i8::MIN), i16::from(i8::MAX)) as i8 as u8;
        }
        9 => {
            data[1..3].copy_from_slice(&status.hall_forward_centers_q16[1].to_le_bytes());
            data[3..5].copy_from_slice(&status.hall_forward_centers_q16[2].to_le_bytes());
            data[5..7].copy_from_slice(&status.hall_forward_centers_q16[3].to_le_bytes());
            data[7] = status.hall_forward_minimum_samples;
        }
        10 => {
            data[1..3].copy_from_slice(&status.hall_forward_centers_q16[4].to_le_bytes());
            data[3..5].copy_from_slice(&status.hall_forward_centers_q16[5].to_le_bytes());
            data[5..7].copy_from_slice(&status.hall_forward_centers_q16[6].to_le_bytes());
            data[7] = status.hall_valid_mask;
        }
        11 => {
            let flux_centi_mwb = (status.flux_linkage_nwb / 10_000) as u16;
            data[1..3].copy_from_slice(&flux_centi_mwb.to_le_bytes());
            data[3..5].copy_from_slice(&status.flux_measurement_erpm.to_le_bytes());
            data[5..7].copy_from_slice(&status.hall_measurement_erpm.to_le_bytes());
            data[7] = status.sync_minimum_percent;
        }
        12 => {
            data[1..5].copy_from_slice(&status.average_bemf_d_uv.to_le_bytes());
            copy_i24(&mut data[5..8], status.average_bemf_q_uv);
        }
        13 => {
            data[1..3].copy_from_slice(&status.hall_reverse_centers_q16[1].to_le_bytes());
            data[3..5].copy_from_slice(&status.hall_reverse_centers_q16[2].to_le_bytes());
            data[5..7].copy_from_slice(&status.hall_reverse_centers_q16[3].to_le_bytes());
            data[7] = status.hall_reverse_minimum_samples;
        }
        14 => {
            data[1..3].copy_from_slice(&status.hall_reverse_centers_q16[4].to_le_bytes());
            data[3..5].copy_from_slice(&status.hall_reverse_centers_q16[5].to_le_bytes());
            data[5..7].copy_from_slice(&status.hall_reverse_centers_q16[6].to_le_bytes());
            data[7] = status.hall_valid_mask;
        }
        _ => {
            data[1] = status.pulse_diagnostic_slot;
            data[2] = PULSE_DIAGNOSTIC_COUNT;
            data[3..5].copy_from_slice(&status.pulse_diagnostic.pulse_step_ticks.to_le_bytes());
            data[5] = status
                .pulse_diagnostic
                .average_di_counts
                .clamp(i16::from(i8::MIN), i16::from(i8::MAX)) as i8 as u8;
            let inductance_deca_nwb = (status.pulse_diagnostic.inductance_nwb_per_count / 10)
                .min(u32::from(u16::MAX)) as u16;
            data[6..8].copy_from_slice(&inductance_deca_nwb.to_le_bytes());
        }
    }
    Frame::new(STATUS_ID, 8, data)
}

fn copy_u24(destination: &mut [u8], value: u32) {
    destination.copy_from_slice(&value.to_le_bytes()[..3]);
}

fn copy_i24(destination: &mut [u8], value: i32) {
    let value = value.clamp(-0x0080_0000, 0x007f_ffff);
    destination.copy_from_slice(&value.to_le_bytes()[..3]);
}

#[cfg(test)]
mod tests {
    use super::*;

    fn command(tag: [u8; 4], challenge: u16) -> Frame {
        let challenge = challenge.to_le_bytes();
        Frame::new(
            COMMAND_ID,
            8,
            [
                tag[0],
                tag[1],
                tag[2],
                tag[3],
                challenge[0],
                challenge[1],
                TRAILER[0],
                TRAILER[1],
            ],
        )
    }

    #[test]
    fn arm_and_run_require_the_complete_magic_frame() {
        assert_eq!(
            decode_command(command(*b"ARMC", 0x1234)),
            Some(Command::Arm { challenge: 0x1234 })
        );
        assert_eq!(
            decode_command(command(*b"RUNR", 0xabcd)),
            Some(Command::RunResistance { challenge: 0xabcd })
        );
        assert_eq!(
            decode_command(command(*b"RUNL", 0x4321)),
            Some(Command::RunInductance { challenge: 0x4321 })
        );
        assert_eq!(
            decode_command(command(*b"RUNF", 0x5678)),
            Some(Command::RunFluxLinkage { challenge: 0x5678 })
        );
        assert_eq!(
            decode_command(command(*b"RUNH", 0x9abc)),
            Some(Command::RunHall { challenge: 0x9abc })
        );
        let mut malformed = command(*b"RUNR", 1);
        malformed.data[7] = 0;
        assert_eq!(decode_command(malformed), None);
    }

    #[test]
    fn stop_is_intentionally_immediate() {
        assert_eq!(
            decode_command(Frame::new(COMMAND_ID, 4, *b"STOP\0\0\0\0")),
            Some(Command::Stop)
        );
    }

    #[test]
    fn status_page_zero_carries_the_interlocks_and_challenge() {
        let frame = status_frame(
            0,
            Status {
                state: 3,
                failure: 0,
                challenge: 0x1234,
                armed: true,
                local_ready: true,
                output_active: true,
                hall_quiet: true,
                motor_outputs_disabled: true,
                can_rx_fifo_overrun: true,
                can_command_queue_drop: true,
                ..Status::default()
            },
        );
        assert_eq!(frame.id, STATUS_ID);
        assert_eq!(frame.data, [0xc0, SCHEMA, 3, 0, 0x34, 0x12, 0xf7, 0]);
    }

    #[test]
    fn inductance_pages_preserve_native_results_and_pulse_diagnostics() {
        let status = Status {
            inductance_d_nwb_per_count: 7_500,
            inductance_q_nwb_per_count: 8_250,
            residual_dead_time_uv: 25_184,
            pulse_step_d_ticks: 53,
            pulse_step_q_ticks: 53,
            last_pulse_di_counts: 27,
            pulse_diagnostic_slot: 7,
            pulse_diagnostic: PulseDiagnostic {
                pulse_step_ticks: 40,
                average_di_counts: 18,
                inductance_nwb_per_count: 2_750,
            },
            ..Status::default()
        };
        assert_eq!(
            &status_frame(7, status).data[1..5],
            &7_500_u32.to_le_bytes()
        );
        assert_eq!(
            &status_frame(7, status).data[5..8],
            &8_250_u32.to_le_bytes()[..3]
        );
        assert_eq!(
            status_frame(8, status).data,
            [0xc8, 25, 0, 53, 0, 53, 0, 27]
        );
        assert_eq!(
            status_frame(15, status).data,
            [0xcf, 7, 12, 40, 0, 18, 19, 1]
        );
    }

    #[test]
    fn flux_and_hall_pages_preserve_signed_vectors_and_raw_state_indexing() {
        let status = Status {
            flux_linkage_nwb: 13_400_000,
            average_bemf_d_uv: -1_234_567,
            average_bemf_q_uv: 7_654_321,
            flux_measurement_erpm: 6_000,
            hall_measurement_erpm: 5_960,
            sync_minimum_percent: 84,
            hall_forward_centers_q16: [0, 5_461, 27_307, 16_384, 49_152, 60_075, 38_229, 0],
            hall_reverse_centers_q16: [0, 5_400, 27_200, 16_300, 49_000, 60_000, 38_100, 0],
            hall_valid_mask: 0x7e,
            hall_forward_minimum_samples: 180,
            hall_reverse_minimum_samples: 175,
            ..Status::default()
        };
        assert_eq!(
            status_frame(11, status).data,
            [0xcb, 0x3c, 0x05, 0x70, 0x17, 0x48, 0x17, 84]
        );
        assert_eq!(
            &status_frame(12, status).data[1..5],
            &(-1_234_567_i32).to_le_bytes()
        );
        assert_eq!(status_frame(9, status).data[1..3], 5_461_u16.to_le_bytes());
        assert_eq!(status_frame(9, status).data[7], 180);
        assert_eq!(status_frame(13, status).data[1..3], 5_400_u16.to_le_bytes());
        assert_eq!(
            status_frame(14, status).data[5..7],
            38_100_u16.to_le_bytes()
        );
        assert_eq!(status_frame(14, status).data[7], 0x7e);
    }

    #[test]
    fn telemetry_version_matches_the_crate_version() {
        assert_eq!(env!("CARGO_PKG_VERSION"), "0.3.2");
        assert_eq!(FIRMWARE_VERSION, [0, 3, 2]);
    }
}
