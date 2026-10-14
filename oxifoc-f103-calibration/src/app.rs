//! Foreground calibration application and supervision state.

use cortex_m::peripheral::scb::SystemHandler;
use oxifoc_f103::{
    hardware::peripherals,
    safety,
    sensors::{self, InputMonitor, environment},
};

use crate::{
    calibration::inductance::PULSE_DIAGNOSTIC_COUNT,
    config, control,
    protocol::{self, Command, Status},
    transport,
};

const TELEMETRY_PERIOD_MS: u32 = 50;

pub struct CalibrationApp {
    input_monitor: Option<InputMonitor>,
    environment_monitor: environment::EnvironmentMonitor,
    offsets: peripherals::CurrentOffsets,
    control_started: bool,
    watchdog: safety::MainLoopWatchdog,
    armed: bool,
    arm_deadline_ms: u32,
    telemetry_page: u8,
    pulse_diagnostic_slot: u8,
    next_telemetry_ms: u32,
    challenge: u16,
}

impl Default for CalibrationApp {
    fn default() -> Self {
        Self::new()
    }
}

impl CalibrationApp {
    pub const fn new() -> Self {
        Self {
            input_monitor: None,
            environment_monitor: environment::EnvironmentMonitor::new(0),
            offsets: peripherals::CurrentOffsets {
                phase_a: 0,
                phase_b: 0,
            },
            control_started: false,
            watchdog: safety::MainLoopWatchdog::new(),
            armed: false,
            arm_deadline_ms: 0,
            telemetry_page: 0,
            pulse_diagnostic_slot: 0,
            next_telemetry_ms: 0,
            challenge: 0,
        }
    }

    pub fn initialize(&mut self) {
        safety::capture_boot_diagnostics();
        peripherals::select_application_vector_table();
        peripherals::disable_power_stage();

        if peripherals::configure_72mhz_clock().is_ok() {
            peripherals::configure_tim1_passive();
            if let Ok(offsets) = peripherals::configure_and_calibrate_current_adcs() {
                self.offsets = offsets;
                peripherals::configure_hall_capture();
                self.input_monitor = Some(InputMonitor::initialize(0));
                control::start(offsets);
                self.control_started = true;
            }
            let _ = transport::initialize();
            if let Some(mut cp) = cortex_m::Peripherals::take() {
                // SAFETY: SysTick only drives the foreground state machine and
                // cannot preempt the current loop, break, Hall, or CAN handlers.
                unsafe { cp.SCB.set_priority(SystemHandler::SysTick, 0xf0) };
                cp.SYST.set_reload(71_999);
                cp.SYST.clear_current();
                cp.SYST
                    .set_clock_source(cortex_m::peripheral::syst::SystClkSource::Core);
                cp.SYST.enable_interrupt();
                cp.SYST.enable_counter();
                if self.control_started {
                    self.watchdog.start(control_progress());
                }
                // SAFETY: the resident bootloader enters with PRIMASK set and
                // every unmasked interrupt now has its complete state.
                unsafe { cortex_m::interrupt::enable() };
            }
        }

        self.challenge = protocol::challenge(
            self.offsets.phase_a,
            self.offsets.phase_b,
            peripherals::cycle_count() as u16,
        );
    }

    pub fn service(&mut self, now_ms: u32) {
        if let Some(inputs) = self.input_monitor.as_mut() {
            inputs.service(now_ms);
        }
        let local = self
            .input_monitor
            .as_ref()
            .map(InputMonitor::latest)
            .unwrap_or_default();
        let bus_voltage_mv = if local.analog_valid {
            sensors::bus_voltage_mv(local.bus_voltage_adc)
        } else {
            0
        };
        let environment_limit = self.environment_monitor.update(
            now_ms,
            environment::RawLocalSensors {
                valid: local.analog_valid,
                bus_voltage_adc: local.bus_voltage_adc,
                motor_temperature_adc: local.motor_temperature_adc,
                controller_temperature_adc: local.controller_temperature_adc,
                sampled_at_ms: local.sampled_at_ms,
            },
            0,
        );
        let environment_reasons = self.environment_monitor.derating_reasons();
        control::set_bus_voltage_mv(bus_voltage_mv);
        let local_ready = self.control_started
            && local.analog_valid
            && local.throttle.is_at_rest()
            && !local.brake_active
            && environment_limit.is_some()
            && config::environment_allows_calibration(environment_reasons)
            && (config::CALIBRATION_BUS_MINIMUM_MV..=config::CALIBRATION_BUS_MAXIMUM_MV)
                .contains(&bus_voltage_mv);
        let hall_quiet = peripherals::hall_is_quiet(500_000);
        let motor_outputs_disabled = peripherals::motor_outputs_disabled();
        let control_snapshot = control::snapshot();
        if control_snapshot.active && !local_ready {
            control::abort(control::Abort::LocalInterlock);
            self.armed = false;
        }
        if self.armed && !deadline_active(now_ms, self.arm_deadline_ms) {
            self.armed = false;
        }

        while let Some(frame) = transport::take_received_frame() {
            let Some(command) = protocol::decode_command(frame) else {
                continue;
            };
            self.process_command(
                command,
                now_ms,
                local_ready,
                hall_quiet,
                motor_outputs_disabled,
            );
        }

        if deadline_due(now_ms, self.next_telemetry_ms) {
            self.transmit_status(
                now_ms,
                local_ready,
                hall_quiet,
                motor_outputs_disabled,
                bus_voltage_mv,
                environment_reasons,
            );
        }
        self.watchdog.service(now_ms, control_progress);
    }

    fn process_command(
        &mut self,
        command: Command,
        now_ms: u32,
        local_ready: bool,
        hall_quiet: bool,
        motor_outputs_disabled: bool,
    ) {
        match command {
            Command::Stop => {
                control::cancel_request();
                if control::snapshot().active {
                    control::abort(control::Abort::Stopped);
                }
                self.armed = false;
                if local_ready {
                    let _ = peripherals::acknowledge_faults();
                }
            }
            Command::Arm {
                challenge: received,
            } => {
                let snapshot = control::snapshot();
                self.armed = received == self.challenge
                    && local_ready
                    && !snapshot.active
                    && snapshot.fault_flags == 0
                    && motor_outputs_disabled
                    && hall_quiet;
                if self.armed {
                    self.arm_deadline_ms = now_ms.wrapping_add(protocol::ARM_WINDOW_MS);
                }
            }
            Command::RunResistance {
                challenge: received,
            } => {
                if self.start_ready(received, local_ready, hall_quiet) {
                    let _ = control::request_resistance();
                }
                self.armed = false;
            }
            Command::RunInductance {
                challenge: received,
            } => {
                if self.start_ready(received, local_ready, hall_quiet) {
                    let _ = control::request_inductance();
                }
                self.armed = false;
            }
            Command::RunFluxLinkage {
                challenge: received,
            } => {
                if self.start_ready(received, local_ready, hall_quiet) {
                    let _ = control::request_flux_linkage();
                }
                self.armed = false;
            }
            Command::RunHall {
                challenge: received,
            } => {
                if self.start_ready(received, local_ready, hall_quiet) {
                    let _ = control::request_hall();
                }
                self.armed = false;
            }
        }
    }

    /// `service` disarms an expired window before commands are processed, so
    /// `armed` already implies the arm deadline is active.
    fn start_ready(&self, received_challenge: u16, local_ready: bool, hall_quiet: bool) -> bool {
        self.armed && received_challenge == self.challenge && local_ready && hall_quiet
    }

    #[inline(never)]
    fn transmit_status(
        &mut self,
        now_ms: u32,
        local_ready: bool,
        hall_quiet: bool,
        motor_outputs_disabled: bool,
        bus_voltage_mv: u32,
        environment_reasons: u8,
    ) {
        let control = control::snapshot();
        let pulse_diagnostic = if self.telemetry_page == 15 {
            control::pulse_diagnostic(self.pulse_diagnostic_slot)
        } else {
            Default::default()
        };
        let status = Status {
            routine: control.routine as u8,
            state: control.state,
            failure: control.failure as u8,
            challenge: self.challenge,
            armed: self.armed,
            local_ready,
            output_active: control.output_active,
            voltage_limited: control.voltage_limited,
            hall_quiet,
            motor_outputs_disabled,
            can_rx_fifo_overrun: transport::can::rx_fifo_overrun_seen(),
            can_command_queue_drop: transport::can::command_queue_drop_seen(),
            low_current_counts: control.low.current_counts,
            low_voltage_ticks: control.low.voltage_ticks,
            high_current_counts: control.high.current_counts,
            high_voltage_ticks: control.high.voltage_ticks,
            effective_uv_per_count: control.effective_uv_per_count,
            nominal_resistance_uohm: control.nominal_resistance_uohm,
            target_d_counts: control.target_d_counts,
            measured_d_counts: control.measured_d_counts,
            applied_d_ticks: control.applied_d_ticks,
            maximum_phase_current_abs: control.maximum_phase_current_abs,
            bus_voltage_mv: bus_voltage_mv.min(u32::from(u16::MAX)) as u16,
            offset_a: self.offsets.phase_a,
            offset_b: self.offsets.phase_b,
            environment_reasons,
            sample_progress: control.sample_progress,
            fault_flags: control.fault_flags,
            maximum_control_cycles: control.maximum_control_cycles.min(u32::from(u16::MAX)) as u16,
            timing_overruns: control.timing_overruns,
            inductance_d_nwb_per_count: control.inductance_d_nwb_per_count,
            inductance_q_nwb_per_count: control.inductance_q_nwb_per_count,
            residual_dead_time_uv: control.residual_dead_time_uv,
            pulse_step_d_ticks: control.pulse_step_d_ticks,
            pulse_step_q_ticks: control.pulse_step_q_ticks,
            last_pulse_di_counts: control.last_pulse_di_counts,
            flux_linkage_nwb: control.flux_linkage_nwb,
            average_bemf_d_uv: control.average_bemf_d_uv,
            average_bemf_q_uv: control.average_bemf_q_uv,
            flux_measurement_erpm: control.flux_measurement_erpm,
            hall_measurement_erpm: control.hall_measurement_erpm,
            sync_minimum_percent: control.sync_minimum_percent,
            hall_forward_centers_q16: control.hall_forward_centers_q16,
            hall_reverse_centers_q16: control.hall_reverse_centers_q16,
            hall_valid_mask: control.hall_valid_mask,
            hall_forward_minimum_samples: control.hall_forward_minimum_samples,
            hall_reverse_minimum_samples: control.hall_reverse_minimum_samples,
            pulse_diagnostic_slot: self.pulse_diagnostic_slot,
            pulse_diagnostic,
        };
        if transport::can::transmit(protocol::status_frame(self.telemetry_page, status)) {
            if self.telemetry_page == 15 {
                self.pulse_diagnostic_slot =
                    (self.pulse_diagnostic_slot + 1) % PULSE_DIAGNOSTIC_COUNT;
            }
            self.telemetry_page = (self.telemetry_page + 1) % protocol::STATUS_PAGE_COUNT;
        }
        self.next_telemetry_ms = now_ms.wrapping_add(TELEMETRY_PERIOD_MS);
    }
}

const fn deadline_active(now: u32, deadline: u32) -> bool {
    let remaining = deadline.wrapping_sub(now);
    remaining != 0 && remaining < 0x8000_0000
}

const fn deadline_due(now: u32, deadline: u32) -> bool {
    deadline == now || deadline.wrapping_sub(now) >= 0x8000_0000
}

fn control_progress() -> safety::ControlProgress {
    let snapshot = control::snapshot();
    safety::ControlProgress {
        control_cycles: snapshot.control_cycles,
        injected_samples: snapshot.injected_samples,
        fault_flags: snapshot.fault_flags,
        output_active: snapshot.output_active,
    }
}
