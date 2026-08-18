#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m::asm;
use cortex_m::peripheral::scb::SystemHandler;
use cortex_m_rt::{entry, exception};
use oxifoc_f103::{
    control::{foc, ride},
    hardware::peripherals,
    safety,
    sensors::{self, environment, wheel, wheel_capture},
    transport,
};
#[allow(unused_imports, reason = "links the PAC interrupt-vector table")]
use stm32f1::stm32f103 as _;

static MILLIS: AtomicU32 = AtomicU32::new(0);

#[entry]
fn main() -> ! {
    const WATCHDOG_FEED_PERIOD_MS: u32 = 10;

    let mut inputs = None;
    let mut environment = None;
    let mut ride = None;
    let mut last_ride_ms = u32::MAX;
    let mut watchdog_started = false;
    let mut last_watchdog_feed_ms = 0;
    let mut last_watchdog_control_cycles = 0;
    let mut last_watchdog_injected_samples = 0;
    let mut wheel_initialized = false;
    let mut wheel_estimator = wheel::Estimator::new();
    let mut wheel_state = wheel::State::Uninitialized;
    let mut wheel_distance = wheel::DistanceCounter::new();
    let mut dc_bus_undervoltage = false;
    let mut effective_current_limit = 0;
    let mut derating_reasons = environment::reason::LOCAL_DATA_MISSING;
    peripherals::select_application_vector_table();
    peripherals::disable_power_stage();
    if peripherals::configure_72mhz_clock().is_ok() {
        peripherals::configure_tim1_passive();
        wheel_capture::initialize();
        wheel_initialized = true;
        if let Ok(offsets) = peripherals::configure_and_calibrate_current_adcs() {
            peripherals::configure_hall_capture();
            inputs = Some(sensors::InputMonitor::initialize(0));
            foc::start(offsets);
            let control = foc::snapshot();
            environment = Some(environment::EnvironmentMonitor::new(0));
            ride = Some(ride::RideController::new(control.safety_events));
        }
        let _ = transport::initialize();
        if let Some(mut cp) = cortex_m::Peripherals::take() {
            // SAFETY: SysTick only advances foreground time and must not
            // preempt Hall capture, hardware break, or the 16 kHz current loop.
            unsafe { cp.SCB.set_priority(SystemHandler::SysTick, 0xf0) };
            cp.SYST.set_reload(71_999);
            cp.SYST.clear_current();
            cp.SYST
                .set_clock_source(cortex_m::peripheral::syst::SystClkSource::Core);
            cp.SYST.enable_interrupt();
            cp.SYST.enable_counter();
            if ride.is_some() {
                let progress = foc::snapshot();
                last_watchdog_control_cycles = progress.control_cycles;
                last_watchdog_injected_samples = progress.injected_samples;
                safety::start();
                watchdog_started = true;
            }
            // The resident bootloader masks interrupts before branching to the
            // application, and cortex-m-rt deliberately preserves PRIMASK.
            // Every enabled interrupt has its handler and shared state ready at
            // this point, so release the bootloader's global mask explicitly.
            unsafe { cortex_m::interrupt::enable() };
        }
    }
    loop {
        let now = MILLIS.load(Ordering::Relaxed);
        if let Some(inputs) = inputs.as_mut() {
            inputs.service(now);
        }
        if now != last_ride_ms
            && let (Some(environment), Some(ride)) = (environment.as_mut(), ride.as_mut())
        {
            let local = sensors::latest();
            let environment_limit = environment.update(
                now,
                environment::RawLocalSensors {
                    valid: local.analog_valid,
                    bus_voltage_adc: local.bus_voltage_adc,
                    motor_temperature_adc: local.motor_temperature_adc,
                    controller_temperature_adc: local.controller_temperature_adc,
                },
            );
            dc_bus_undervoltage = environment.undervoltage_active();
            effective_current_limit = environment_limit.unwrap_or(0);
            derating_reasons = environment.derating_reasons();
            let control = foc::snapshot();
            let command = ride.update(ride::Observation {
                now_ms: now,
                throttle: local.throttle,
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
            if command.energize {
                foc::authorize_ride_target(
                    command.target_q_counts,
                    command.dc_current_limit_counts,
                    ride::OUTPUT_LEASE_CYCLES,
                    control.safety_events,
                );
            } else {
                foc::revoke_ride_authority();
            }
            last_ride_ms = now;
        }
        if wheel_initialized {
            let capture = wheel_capture::snapshot();
            wheel_state = wheel_estimator.update(capture);
            wheel_distance.update(capture.pulse_count);
        }
        transport::service(
            now,
            wheel_state.speed_tenths_kph(),
            wheel_distance.value(),
            dc_bus_undervoltage,
            effective_current_limit,
            derating_reasons,
        );
        if watchdog_started && now.wrapping_sub(last_watchdog_feed_ms) >= WATCHDOG_FEED_PERIOD_MS {
            let progress = foc::snapshot();
            if progress.control_cycles != last_watchdog_control_cycles
                && progress.injected_samples != last_watchdog_injected_samples
            {
                safety::feed_main_loop();
                last_watchdog_feed_ms = now;
                last_watchdog_control_cycles = progress.control_cycles;
                last_watchdog_injected_samples = progress.injected_samples;
            }
        }
        if transport::take_reset_request()
            && wheel_state.safe_for_update()
            && foc::safe_for_updater_reset()
        {
            peripherals::emergency_shutdown();
            cortex_m::peripheral::SCB::sys_reset();
        }
        asm::wfi();
    }
}

#[exception]
fn SysTick() {
    MILLIS.fetch_add(1, Ordering::Relaxed);
}
