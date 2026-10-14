//! Stock-bike CAN policy for the ride application.

use core::sync::atomic::{AtomicBool, Ordering};

use crate::{config, control::foc, hardware::peripherals, protocol, safety, sensors};

use super::can::{BxCan, FilterConfig, transmit};

static RESET_REQUESTED: AtomicBool = AtomicBool::new(false);
static HEADLIGHT_COMMAND: AtomicBool = AtomicBool::new(false);

pub struct RideCan {
    telemetry_slot: u8,
    stock_fault_page: u8,
    project_telemetry_page: u8,
    next_telemetry_ms: u32,
}

impl Default for RideCan {
    fn default() -> Self {
        Self::new()
    }
}

impl RideCan {
    pub const fn new() -> Self {
        Self {
            telemetry_slot: 0,
            stock_fault_page: 0,
            project_telemetry_page: 0,
            next_telemetry_ms: 0,
        }
    }

    pub fn initialize(&self) -> bool {
        BxCan::initialize(FilterConfig::Mask32 {
            identifier: 0,
            mask: (1 << 2) | (1 << 1),
        })
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct RuntimeTelemetry {
    pub inputs: sensors::Snapshot,
    pub vehicle_speed_tenths_kph: u16,
    pub distance_counter: u32,
    pub cadence: sensors::cadence::Observation,
    pub dc_bus_undervoltage: bool,
    pub effective_current_limit: u16,
    pub transient_budget_ms: u16,
    pub derating_reasons: u8,
    pub ride_stage: u8,
    pub fault_lockout: bool,
}

impl RideCan {
    #[inline(never)]
    #[optimize(size)]
    /// `runtime` is evaluated only when a telemetry slot is due.
    pub fn service(&mut self, now_ms: u32, runtime: impl FnOnce() -> RuntimeTelemetry) {
        let next = self.next_telemetry_ms;
        if next != now_ms && next.wrapping_sub(now_ms) < 0x8000_0000 {
            return;
        }
        let RuntimeTelemetry {
            inputs,
            vehicle_speed_tenths_kph,
            distance_counter,
            cadence,
            dc_bus_undervoltage,
            effective_current_limit,
            transient_budget_ms,
            derating_reasons,
            ride_stage,
            fault_lockout,
        } = runtime();
        self.next_telemetry_ms = now_ms.wrapping_add(50);
        let slot = self.telemetry_slot;
        self.telemetry_slot = protocol::next_telemetry_slot(slot);
        let control = foc::diagnostic_snapshot();
        let fault_page = self.stock_fault_page;
        let controller_temperature = inputs.analog_valid.then(|| {
            sensors::environment::controller_temperature_deci_c(inputs.controller_temperature_adc)
        });
        let motor_temperature = inputs
            .analog_valid
            .then(|| sensors::environment::motor_temperature_deci_c(inputs.motor_temperature_adc))
            .flatten();
        let stock = protocol::StockTelemetry {
            vehicle_speed_tenths_kph,
            distance_counter,
            cadence_rpm: cadence.rpm,
            digital_input_flags: (u8::from(HEADLIGHT_COMMAND.load(Ordering::Relaxed)) << 1)
                | (u8::from(inputs.brake_active) << 2),
            controller_temperature_deci_c: controller_temperature,
            motor_temperature_deci_c: motor_temperature,
            fault_page,
            hardware_break: control.fault_flags & peripherals::FAULT_HARDWARE_BREAK != 0,
            dc_bus_undervoltage,
            hall_invalid: !control.hall_valid,
        };
        let mut mailboxes_used = 0;
        for frame in protocol::telemetry_slot(slot, stock).into_iter().flatten() {
            mailboxes_used += u8::from(transmit(frame));
        }
        if slot == 1 {
            self.stock_fault_page = fault_page.wrapping_add(1) & 3;
        }
        if mailboxes_used < 3 {
            let page = self.project_telemetry_page;
            let frame = match page {
                0 | 1 => protocol::passive_input_telemetry(
                    page,
                    protocol::PassiveInputTelemetry {
                        analog_valid: inputs.analog_valid,
                        brake_active: inputs.brake_active,
                        throttle_valid: inputs.throttle.is_valid(),
                        throttle_at_rest: inputs.throttle.is_at_rest(),
                        throttle_raw: inputs.throttle.raw(),
                        throttle_demand_percent: inputs.throttle.demand_percent(),
                        throttle_current_limit_counts: config::RIDE_PHASE_CURRENT_LIMIT_COUNTS,
                        cadence_active: cadence.is_active(),
                        cadence_pin_low: cadence.pin_low,
                        cadence_raw_pin_low: cadence.raw_pin_low,
                        cadence_demand_percent: cadence.demand_percent,
                        live_hall_state: peripherals::live_hall_state(),
                        bus_voltage_mv: if inputs.analog_valid {
                            sensors::bus_voltage_mv(inputs.bus_voltage_adc).min(u32::from(u16::MAX))
                                as u16
                        } else {
                            0
                        },
                        effective_current_limit,
                        derating_reasons,
                        controller_temperature_deci_c: controller_temperature,
                        motor_temperature_deci_c: motor_temperature,
                    },
                ),
                2 => protocol::control_timing_telemetry(protocol::ControlTimingTelemetry {
                    current_trips: control.phase_current_trips,
                    maximum_cycles: control.control_max_cycles,
                }),
                3 => protocol::control_live_telemetry(protocol::ControlLiveTelemetry {
                    hall_valid: control.hall_valid,
                    current_valid: control.current_valid,
                    output_active: control.output_active,
                    voltage_limited: control.voltage_limited,
                    ride_stage,
                    target_q_counts: control.target_q_counts,
                    measured_d_counts: control.measured_d_counts,
                    measured_q_counts: control.measured_q_counts,
                }),
                4 => protocol::control_output_telemetry(protocol::ControlOutputTelemetry {
                    transient_budget_ms,
                    applied_d_ticks: control.applied_d_ticks,
                    applied_q_ticks: control.applied_q_ticks,
                }),
                5 => protocol::control_fault_telemetry(protocol::ControlFaultTelemetry {
                    fault_flags: control.fault_flags,
                    fault_lockout,
                    safety_events: control.safety_events.min(u32::from(u16::MAX)) as u16,
                    last_safety_loss_reason: control.last_safety_loss_reason,
                }),
                6 => protocol::control_peak_telemetry(protocol::ControlPeakTelemetry {
                    maximum_phase_current_abs: control.maximum_phase_current_abs,
                    maximum_direct_current_abs: control.maximum_direct_current_abs,
                }),
                7 => protocol::field_weakening_telemetry(protocol::FieldWeakeningTelemetry {
                    active: control.target_d_counts < 0,
                    at_current_limit: control.target_d_counts
                        <= -(config::FIELD_WEAKENING_MAXIMUM_CURRENT_COUNTS as i16),
                    voltage_limited: control.voltage_limited,
                    target_d_counts: control.target_d_counts,
                    requested_d_ticks: control.requested_d_ticks,
                    requested_q_ticks: control.requested_q_ticks,
                }),
                8 => protocol::observer_status_telemetry(protocol::ObserverStatusTelemetry {
                    configured: control.observer_configured,
                    ready: control.observer_ready,
                    active: control.observer_active,
                    blend: control.observer_blend,
                    confidence: control.observer_confidence,
                    electrical_rpm: control.observer_electrical_rpm,
                    hall_error_q16: control.observer_hall_error_q16,
                }),
                9 => protocol::observer_model_telemetry(protocol::ObserverModelTelemetry {
                    flux_centi_mwb: control.observer_flux_centi_mwb,
                    bemf_q_mv: control.observer_bemf_q_mv,
                    phase_error_q16: control.observer_phase_error_q16,
                    validity_progress: control.observer_validity_progress,
                }),
                10 | 11 => {
                    let reset = safety::boot_diagnostics_snapshot();
                    if page == 10 {
                        protocol::reset_summary_telemetry(protocol::ResetSummaryTelemetry {
                            reset_flags: reset.reset_flags,
                            retained_context_valid: reset.retained_context_valid,
                            fatal_reason: reset.fatal_reason,
                            checkpoint: reset.checkpoint,
                            last_control_cycles: reset.last_control_cycles.min(u32::from(u16::MAX))
                                as u16,
                            maximum_control_cycles: reset
                                .maximum_control_cycles
                                .min(u32::from(u16::MAX))
                                as u16,
                        })
                    } else {
                        protocol::crash_context_telemetry(protocol::CrashContextTelemetry {
                            detail: reset.detail,
                            control_cycle: reset.control_cycle,
                            program_counter: reset.program_counter,
                            link_register: reset.link_register,
                        })
                    }
                }
                12 | 13 => {
                    let failure = safety::latest_pwm_failure_snapshot();
                    protocol::pwm_failure_telemetry(page + 12, failure.words())
                }
                _ => protocol::firmware_version_telemetry(),
            };
            if transmit(frame) {
                self.project_telemetry_page = protocol::next_project_telemetry_page(page);
            }
        }
    }
}

impl RideCan {
    pub fn take_reset_request(&self) -> bool {
        RESET_REQUESTED.swap(false, Ordering::AcqRel)
    }
}

pub fn receive_interrupt() {
    let _ = BxCan::drain_fifo0(|frame| {
        if frame.len == 0 {
            if let Some(response) = protocol::identity_response(frame.id) {
                let _ = BxCan::transmit(response);
            }
        } else if protocol::is_updater_reset(frame) {
            RESET_REQUESTED.store(true, Ordering::Release);
        } else if let Some(enabled) = protocol::headlight_command(frame) {
            HEADLIGHT_COMMAND.store(enabled, Ordering::Relaxed);
        }
    });
}
