#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};

use cortex_m::asm;
use cortex_m::peripheral::scb::SystemHandler;
use cortex_m_rt::{entry, exception};
use oxifoc_f103::{
    hardware::peripherals,
    safety,
    sensors::{self, InputMonitor, environment},
    transport,
};
use oxifoc_f103_calibration::{
    calibration::types::Failure,
    config as calibration_config, control,
    protocol::{self, Command, Status},
};
#[allow(unused_imports, reason = "links the PAC interrupt-vector table")]
use stm32f1::stm32f103 as _;

static MILLIS: AtomicU32 = AtomicU32::new(0);

#[entry]
fn main() -> ! {
    const WATCHDOG_FEED_PERIOD_MS: u32 = 10;
    const TELEMETRY_PERIOD_MS: u32 = 50;

    safety::capture_boot_diagnostics();
    peripherals::select_application_vector_table();
    peripherals::disable_power_stage();

    let mut input_monitor: Option<InputMonitor> = None;
    let mut environment_monitor = environment::EnvironmentMonitor::new(0);
    let mut offsets = peripherals::CurrentOffsets::default();
    let mut control_started = false;
    let mut watchdog_started = false;
    let mut last_watchdog_feed_ms = 0;
    let mut last_watchdog_control_cycles = 0;
    let mut last_watchdog_injected_samples = 0;
    let mut armed = false;
    let mut arm_deadline_ms = 0;
    let mut telemetry_page = 0_u8;
    let mut next_telemetry_ms = 0_u32;

    if peripherals::configure_72mhz_clock().is_ok() {
        peripherals::configure_tim1_passive();
        if let Ok(calibrated_offsets) = peripherals::configure_and_calibrate_current_adcs() {
            offsets = calibrated_offsets;
            peripherals::configure_hall_capture();
            input_monitor = Some(InputMonitor::initialize(0));
            control::start(offsets);
            control_started = true;
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
            if control_started {
                let progress = control::snapshot();
                last_watchdog_control_cycles = progress.control_cycles;
                last_watchdog_injected_samples = progress.injected_samples;
                safety::start();
                watchdog_started = true;
            }
            // SAFETY: the resident bootloader enters with PRIMASK set and every
            // unmasked interrupt now has its complete state.
            unsafe { cortex_m::interrupt::enable() };
        }
    }

    let challenge = protocol::challenge(
        offsets.phase_a,
        offsets.phase_b,
        peripherals::cycle_count() as u16,
    );
    loop {
        let now = MILLIS.load(Ordering::Relaxed);
        if let Some(inputs) = input_monitor.as_mut() {
            inputs.service(now);
        }
        let local = sensors::latest();
        let bus_voltage_mv = if local.analog_valid {
            sensors::bus_voltage_mv(local.bus_voltage_adc)
        } else {
            0
        };
        let environment_limit = environment_monitor.update(
            now,
            environment::RawLocalSensors {
                valid: local.analog_valid,
                bus_voltage_adc: local.bus_voltage_adc,
                motor_temperature_adc: local.motor_temperature_adc,
                controller_temperature_adc: local.controller_temperature_adc,
            },
        );
        let environment_reasons = environment_monitor.derating_reasons();
        control::set_bus_voltage_mv(bus_voltage_mv);
        let local_ready = control_started
            && local.analog_valid
            && local.throttle.is_at_rest()
            && !local.brake_active
            && environment_limit.is_some()
            && environment_reasons == 0
            && (calibration_config::CALIBRATION_BUS_MINIMUM_MV
                ..=calibration_config::CALIBRATION_BUS_MAXIMUM_MV)
                .contains(&bus_voltage_mv);
        let control_snapshot = control::snapshot();
        if control_snapshot.active && !local_ready {
            control::abort(Failure::LocalInterlock);
            armed = false;
        }
        if armed && !deadline_active(now, arm_deadline_ms) {
            armed = false;
        }

        while let Some(frame) = transport::take_received_frame() {
            let Some(command) = protocol::decode_command(frame) else {
                continue;
            };
            match command {
                Command::Stop => {
                    if control::snapshot().active {
                        control::abort(Failure::Stopped);
                    }
                    armed = false;
                    if local_ready {
                        let _ = peripherals::acknowledge_faults();
                    }
                }
                Command::Arm {
                    challenge: received,
                } => {
                    let snapshot = control::snapshot();
                    armed = received == challenge
                        && local_ready
                        && !snapshot.active
                        && snapshot.fault_flags == 0
                        && peripherals::motor_outputs_disabled()
                        && peripherals::hall_is_quiet(500_000);
                    if armed {
                        arm_deadline_ms = now.wrapping_add(protocol::ARM_WINDOW_MS);
                    }
                }
                Command::RunResistance {
                    challenge: received,
                } => {
                    let start_ready = armed
                        && deadline_active(now, arm_deadline_ms)
                        && received == challenge
                        && local_ready
                        && peripherals::hall_is_quiet(500_000);
                    if start_ready {
                        let _ = control::request_resistance();
                    }
                    armed = false;
                }
                Command::RunInductance {
                    challenge: received,
                } => {
                    let start_ready = armed
                        && deadline_active(now, arm_deadline_ms)
                        && received == challenge
                        && local_ready
                        && peripherals::hall_is_quiet(500_000);
                    if start_ready {
                        let _ = control::request_inductance();
                    }
                    armed = false;
                }
                Command::RunFluxLinkage {
                    challenge: received,
                } => {
                    let start_ready = armed
                        && deadline_active(now, arm_deadline_ms)
                        && received == challenge
                        && local_ready
                        && peripherals::hall_is_quiet(500_000);
                    if start_ready {
                        let _ = control::request_flux_linkage();
                    }
                    armed = false;
                }
                Command::RunHall {
                    challenge: received,
                } => {
                    let start_ready = armed
                        && deadline_active(now, arm_deadline_ms)
                        && received == challenge
                        && local_ready
                        && peripherals::hall_is_quiet(500_000);
                    if start_ready {
                        let _ = control::request_hall();
                    }
                    armed = false;
                }
            }
        }

        if deadline_due(now, next_telemetry_ms) {
            let control = control::snapshot();
            let status = Status {
                routine: control.routine as u8,
                state: control.state,
                failure: control.failure as u8,
                challenge,
                armed,
                local_ready,
                output_active: control.output_active,
                voltage_limited: control.voltage_limited,
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
                offset_a: offsets.phase_a,
                offset_b: offsets.phase_b,
                environment_reasons,
                sample_progress: control.sample_progress,
                fault_flags: control.fault_flags,
                maximum_control_cycles: control.maximum_control_cycles.min(u32::from(u16::MAX))
                    as u16,
                timing_overruns: control.timing_overruns,
                inductance_d_nwb_per_count: control.inductance_d_nwb_per_count,
                inductance_q_nwb_per_count: control.inductance_q_nwb_per_count,
                residual_dead_time_uv: control.residual_dead_time_uv,
                pulse_step_tick_bits: control.pulse_step_tick_bits,
                last_pulse_di_counts: control.last_pulse_di_counts,
                proportional_d_q16: control.proportional_d_q16,
                proportional_q_q16: control.proportional_q_q16,
                integral_per_cycle_q16: control.integral_per_cycle_q16,
                gain_bus_voltage_mv: control.gain_bus_voltage_mv,
                tuning_bandwidth_rad_s: control.tuning_bandwidth_rad_s,
                flux_linkage_nwb: control.flux_linkage_nwb,
                average_bemf_d_uv: control.average_bemf_d_uv,
                average_bemf_q_uv: control.average_bemf_q_uv,
                flux_measurement_erpm: control.flux_measurement_erpm,
                sync_minimum_percent: control.sync_minimum_percent,
                hall_centers_q16: control.hall_centers_q16,
                hall_valid_mask: control.hall_valid_mask,
                hall_minimum_samples: control.hall_minimum_samples,
            };
            if transport::can::transmit(protocol::status_frame(telemetry_page, status)) {
                telemetry_page = (telemetry_page + 1) % protocol::STATUS_PAGE_COUNT;
            }
            next_telemetry_ms = now.wrapping_add(TELEMETRY_PERIOD_MS);
        }

        if watchdog_started && now.wrapping_sub(last_watchdog_feed_ms) >= WATCHDOG_FEED_PERIOD_MS {
            let progress = control::snapshot();
            let latched_safe_off = progress.fault_flags != 0
                && !progress.output_active
                && peripherals::motor_outputs_disabled();
            if safety::watchdog_progressed(
                last_watchdog_control_cycles,
                progress.control_cycles,
                last_watchdog_injected_samples,
                progress.injected_samples,
                latched_safe_off,
            ) {
                safety::feed_main_loop();
                last_watchdog_feed_ms = now;
                last_watchdog_control_cycles = progress.control_cycles;
                last_watchdog_injected_samples = progress.injected_samples;
            }
        }
        asm::wfi();
    }
}

const fn deadline_active(now: u32, deadline: u32) -> bool {
    let remaining = deadline.wrapping_sub(now);
    remaining != 0 && remaining < 0x8000_0000
}

const fn deadline_due(now: u32, deadline: u32) -> bool {
    deadline == now || deadline.wrapping_sub(now) >= 0x8000_0000
}

#[exception]
fn SysTick() {
    MILLIS.fetch_add(1, Ordering::Relaxed);
}
