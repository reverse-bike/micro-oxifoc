//! Foreground ride application built from the synchronous F103 components.

use cortex_m::peripheral::scb::SystemHandler;

use crate::{
    control::{foc, ride},
    hardware::peripherals,
    safety,
    sensors::{self, cadence, cadence_input, environment, wheel, wheel_capture},
    transport::{RideCan, RuntimeTelemetry},
};

const WATCHDOG_FEED_PERIOD_MS: u32 = 10;

pub struct RideApp {
    inputs: Option<sensors::InputMonitor>,
    environment: Option<environment::EnvironmentMonitor>,
    ride_controller: Option<ride::RideController>,
    last_ride_ms: u32,
    watchdog_started: bool,
    last_watchdog_feed_ms: u32,
    last_watchdog_control_cycles: u32,
    last_watchdog_injected_samples: u32,
    wheel_initialized: bool,
    wheel_estimator: wheel::Estimator,
    wheel_state: wheel::State,
    wheel_distance: wheel::DistanceCounter,
    cadence_tracker: cadence::Tracker,
    cadence: cadence::Observation,
    last_cadence_ms: u32,
    dc_bus_undervoltage: bool,
    effective_current_limit: u16,
    transient_budget_ms: u16,
    derating_reasons: u8,
    ride_stage: ride::Stage,
    can: RideCan,
}

impl Default for RideApp {
    fn default() -> Self {
        Self::new()
    }
}

impl RideApp {
    pub const fn new() -> Self {
        Self {
            inputs: None,
            environment: None,
            ride_controller: None,
            last_ride_ms: u32::MAX,
            watchdog_started: false,
            last_watchdog_feed_ms: 0,
            last_watchdog_control_cycles: 0,
            last_watchdog_injected_samples: 0,
            wheel_initialized: false,
            wheel_estimator: wheel::Estimator::new(),
            wheel_state: wheel::State::Uninitialized,
            wheel_distance: wheel::DistanceCounter::new(),
            cadence_tracker: cadence::Tracker::new(),
            cadence: cadence::Observation::INACTIVE,
            last_cadence_ms: u32::MAX,
            dc_bus_undervoltage: false,
            effective_current_limit: 0,
            transient_budget_ms: 0,
            derating_reasons: environment::reason::LOCAL_DATA_MISSING,
            ride_stage: ride::Stage::Disarmed,
            can: RideCan::new(),
        }
    }

    pub fn initialize(&mut self) {
        safety::capture_boot_diagnostics();
        peripherals::select_application_vector_table();
        peripherals::disable_power_stage();
        if peripherals::configure_72mhz_clock().is_err() {
            return;
        }

        peripherals::configure_tim1_passive();
        wheel_capture::initialize();
        cadence_input::initialize();
        self.wheel_initialized = true;
        if let Ok(offsets) = peripherals::configure_and_calibrate_current_adcs() {
            peripherals::configure_hall_capture();
            self.inputs = Some(sensors::InputMonitor::initialize(0));
            foc::start(offsets);
            let control = foc::snapshot();
            self.environment = Some(environment::EnvironmentMonitor::new(0));
            self.ride_controller = Some(ride::RideController::new(control.safety_events));
        }
        let _ = self.can.initialize();

        let Some(mut cp) = cortex_m::Peripherals::take() else {
            return;
        };
        // SAFETY: SysTick only advances foreground time and must not preempt
        // Hall capture, hardware break, or the 16 kHz current loop.
        unsafe { cp.SCB.set_priority(SystemHandler::SysTick, 0xf0) };
        cp.SYST.set_reload(71_999);
        cp.SYST.clear_current();
        cp.SYST
            .set_clock_source(cortex_m::peripheral::syst::SystClkSource::Core);
        cp.SYST.enable_interrupt();
        cp.SYST.enable_counter();
        if self.ride_controller.is_some() {
            let progress = foc::snapshot();
            self.last_watchdog_control_cycles = progress.control_cycles;
            self.last_watchdog_injected_samples = progress.injected_samples;
            safety::start();
            self.watchdog_started = true;
        }
        // The resident bootloader masks interrupts before branching to the
        // application, and cortex-m-rt deliberately preserves PRIMASK. Every
        // enabled interrupt has its handler and shared state ready now.
        unsafe { cortex_m::interrupt::enable() };
    }

    pub fn service(&mut self, now_ms: u32) {
        if let Some(inputs) = self.inputs.as_mut() {
            inputs.service(now_ms);
        }
        self.service_wheel();
        self.service_cadence(now_ms);
        self.service_ride(now_ms);
        self.can.service(
            now_ms,
            RuntimeTelemetry {
                vehicle_speed_tenths_kph: self.wheel_state.speed_tenths_kph(),
                distance_counter: self.wheel_distance.value(),
                cadence: self.cadence,
                dc_bus_undervoltage: self.dc_bus_undervoltage,
                effective_current_limit: self.effective_current_limit,
                transient_budget_ms: self.transient_budget_ms,
                derating_reasons: self.derating_reasons,
                ride_stage: self.ride_stage as u8,
            },
        );
        self.service_watchdog(now_ms);
        self.service_updater_reset();
    }

    fn service_ride(&mut self, now_ms: u32) {
        if now_ms == self.last_ride_ms {
            return;
        }
        let (Some(environment), Some(ride_controller)) =
            (self.environment.as_mut(), self.ride_controller.as_mut())
        else {
            return;
        };

        let local = sensors::latest();
        let control = foc::snapshot();
        foc::set_bus_voltage_mv(if local.analog_valid {
            sensors::bus_voltage_mv(local.bus_voltage_adc)
        } else {
            0
        });
        let environment_limit = environment.update(
            now_ms,
            environment::RawLocalSensors {
                valid: local.analog_valid,
                bus_voltage_adc: local.bus_voltage_adc,
                motor_temperature_adc: local.motor_temperature_adc,
                controller_temperature_adc: local.controller_temperature_adc,
            },
            environment::projected_dc_current_counts(
                control.measured_d_counts,
                control.measured_q_counts,
                control.applied_d_ticks,
                control.applied_q_ticks,
            ),
        );
        self.dc_bus_undervoltage = environment.undervoltage_active();
        self.effective_current_limit = environment_limit.unwrap_or(0);
        self.transient_budget_ms = environment.transient_budget_ms();
        self.derating_reasons = environment.derating_reasons();
        let command = ride_controller.update(ride::Observation {
            now_ms,
            throttle: local.throttle,
            cadence_demand_percent: self.cadence.demand_percent,
            brake_active: local.brake_active,
            environment_dc_limit_counts: environment_limit,
            hall_valid: control.hall_valid,
            current_valid: control.current_valid,
            fault_flags: control.fault_flags,
            safety_events: control.safety_events,
            hall_sequence: control.hall_sequence,
            hall_progress: control.hall_progress,
            hall_interval_us: control.hall_interval_us,
            electrical_rpm: control.electrical_rpm,
        });
        self.ride_stage = command.stage;
        if command.energize {
            foc::authorize_ride_target(
                command.target_q_counts,
                command.dc_current_limit_counts,
                ride::OUTPUT_LEASE_CYCLES,
                control.safety_events,
            );
        } else {
            foc::revoke_ride_authority();
            if command.acknowledge_faults && peripherals::acknowledge_faults() {
                foc::begin_fault_diagnostic_episode();
            }
        }
        self.last_ride_ms = now_ms;
    }

    fn service_wheel(&mut self) {
        if self.wheel_initialized {
            let capture = wheel_capture::snapshot();
            self.wheel_state = self.wheel_estimator.update(capture);
            self.wheel_distance.update(capture.pulse_count);
        }
    }

    fn service_cadence(&mut self, now_ms: u32) {
        if now_ms == self.last_cadence_ms {
            return;
        }
        self.cadence = self.cadence_tracker.update(
            now_ms,
            cadence_input::pin_is_low(),
            self.wheel_state.speed_tenths_kph(),
        );
        self.last_cadence_ms = now_ms;
    }

    fn service_watchdog(&mut self, now_ms: u32) {
        if !self.watchdog_started
            || now_ms.wrapping_sub(self.last_watchdog_feed_ms) < WATCHDOG_FEED_PERIOD_MS
        {
            return;
        }
        let progress = foc::snapshot();
        let latched_safe_off = progress.fault_flags != 0
            && !progress.output_active
            && peripherals::motor_outputs_disabled();
        if safety::watchdog_progressed(
            self.last_watchdog_control_cycles,
            progress.control_cycles,
            self.last_watchdog_injected_samples,
            progress.injected_samples,
            latched_safe_off,
        ) {
            safety::feed_main_loop();
            self.last_watchdog_feed_ms = now_ms;
            self.last_watchdog_control_cycles = progress.control_cycles;
            self.last_watchdog_injected_samples = progress.injected_samples;
        }
    }

    fn service_updater_reset(&self) {
        if self.can.take_reset_request()
            && self.wheel_state.safe_for_update()
            && foc::safe_for_updater_reset()
        {
            peripherals::emergency_shutdown();
            cortex_m::peripheral::SCB::sys_reset();
        }
    }
}
