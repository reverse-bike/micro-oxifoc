//! 16 kHz Hall/back-EMF fixed-point current-control interrupt.

use core::cell::UnsafeCell;
use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicBool, AtomicI32, AtomicU32, Ordering};

use crate::hardware::peripherals::{self as hardware, CurrentOffsets};
use oxifoc_core::foc::{
    Dq, Fixed, FixedFocController, NoDecoupling, PIController, Scalar,
    hall_sensor::HallSensor,
    offset_tracker::CurrentOffsetTracker,
    phase::{BackEmfObserver, ObserverDiagnostics, PhaseEstimate, PhaseManager, PhaseSource},
    ramp::QuadratureTargetRamp,
};
use oxifoc_core::motor::foc_driver::{CurrentLimits, FocDriver, StepError};
use stm32f1::stm32f103::interrupt;

const CONTROL_HALL_VALID: u32 = 1;
const CONTROL_CURRENT_VALID: u32 = 1 << 1;
const CONTROL_OUTPUT_ACTIVE: u32 = 1 << 2;
const CONTROL_VOLTAGE_LIMITED: u32 = 1 << 3;
const CONTROL_DIRECT_EVENT_PENDING: u32 = 1 << 4;
const HALL_RECOVERY_STABLE_CYCLES: u8 = 5;
const CONTROL_BUDGET_WARNING_CYCLES: u32 = 3_900;
const CONTROL_BUDGET_CYCLES: u32 = crate::config::SYSCLK_HZ / crate::config::PWM_HZ;
const CONTROL_PERIOD_NS: u32 = 1_000_000_000 / crate::config::PWM_HZ;
// One IIR step per 1/32 of the error gives the 2 ms bus-modulation filter at 16 kHz.
const BUS_MODULATION_FILTER_SHIFT: u8 = 5;

type RideDecoupling = NoDecoupling;
type RideFocController = FixedFocController<
    { crate::config::FOC_DEAD_TIME_COMP_NUMERATOR },
    { crate::config::FOC_DEAD_TIME_COMP_DENOMINATOR },
    RideDecoupling,
>;
type RideFocDriver = FocDriver<
    PhaseManager<HallSensor>,
    { crate::config::FOC_DEAD_TIME_COMP_NUMERATOR },
    { crate::config::FOC_DEAD_TIME_COMP_DENOMINATOR },
    RideDecoupling,
>;

static TARGET_Q_COUNTS: AtomicI32 = AtomicI32::new(0);
static DC_CURRENT_LIMIT_COUNTS: AtomicU32 = AtomicU32::new(0);
static CONTROL_CYCLE: AtomicU32 = AtomicU32::new(0);
static COMMAND_DEADLINE: AtomicU32 = AtomicU32::new(0);
static COMMAND_SAFETY_EPOCH: AtomicU32 = AtomicU32::new(0);
static COMMAND_ENABLED: AtomicBool = AtomicBool::new(false);
static OBSERVER_VOLTS_PER_PWM_TICK_BITS: AtomicI32 = AtomicI32::new(0);

struct IsrCell<T>(UnsafeCell<T>);

// SAFETY: each instance below documents how interrupt/foreground exclusion is
// maintained for its contained value.
unsafe impl<T> Sync for IsrCell<T> {}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[repr(u8)]
pub enum SafetyLossReason {
    #[default]
    None = 0,
    ControlTiming = 1,
    CurrentSample = 2,
    PhaseOvercurrent = 3,
    HallCaptureSequence = 4,
    HallTransition = 5,
    HallStationaryMismatch = 6,
    PhaseEstimate = 7,
    HardwareFault = 8,
    OutputLease = 9,
    PwmOutput = 10,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct DirectCurrentPeakEvent {
    pub generation: u8,
    pub measured_d_counts: i16,
    pub measured_q_counts: i16,
    pub target_q_counts: i16,
    pub hall_raw: u8,
    pub hall_angle_direction: i8,
    pub edge_age_us: u16,
    pub hall_interval_us: u16,
    pub angle_error_q8: i8,
    pub requested_d_ticks: i16,
    pub requested_q_ticks: i16,
    pub feedforward_d_ticks: i16,
    pub feedforward_q_ticks: i16,
    pub applied_d_ticks: i16,
    pub applied_q_ticks: i16,
    pub voltage_limited: bool,
    pub angle_rate_limited: bool,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
struct TimingMaxima {
    pre_driver_cycles: u16,
    driver_step_cycles: u16,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Snapshot {
    pub hall_valid: bool,
    pub current_valid: bool,
    pub output_active: bool,
    pub voltage_limited: bool,
    pub target_q_counts: i16,
    pub phase_current_limit_counts: u16,
    pub measured_d_counts: i16,
    pub measured_q_counts: i16,
    pub applied_d_ticks: i16,
    pub applied_q_ticks: i16,
    pub pwm_span_ticks: u16,
    pub maximum_phase_current_abs: u16,
    pub maximum_direct_current_abs: u16,
    pub maximum_quadrature_error_abs: u16,
    pub maximum_pwm_span_ticks: u16,
    pub maximum_direct_event: DirectCurrentPeakEvent,
    pub safety_events: u32,
    pub last_safety_loss_reason: u8,
    pub phase_current_trips: u32,
    pub hall_sequence: u32,
    pub hall_interval_us: u32,
    pub hall_progress: i32,
    pub electrical_rpm: i32,
    pub observer_configured: bool,
    pub observer_ready: bool,
    pub observer_active: bool,
    pub observer_blend: u8,
    pub observer_confidence: u8,
    pub observer_validity_progress: u8,
    pub observer_electrical_rpm: i16,
    pub observer_hall_error_q16: i16,
    pub observer_flux_centi_mwb: u16,
    pub observer_bemf_q_mv: i16,
    pub observer_phase_error_q16: u16,
    pub maximum_pre_driver_cycles: u16,
    pub maximum_driver_step_cycles: u16,
    pub fault_flags: u32,
    pub control_cycles: u32,
    pub injected_samples: u32,
    pub control_max_cycles: u32,
    pub control_budget_warnings: u32,
    pub control_budget_overruns: u32,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
struct ControlDiagnostics {
    flags: u32,
    target_q_counts: i16,
    phase_current_limit_counts: u16,
    measured_d_counts: i16,
    measured_q_counts: i16,
    applied_d_ticks: i16,
    applied_q_ticks: i16,
    pwm_span_ticks: u16,
    maximum_phase_current_abs: u32,
    maximum_direct_current_abs: u32,
    maximum_quadrature_error_abs: u32,
    maximum_pwm_span_ticks: u32,
    maximum_direct_event: DirectCurrentPeakEvent,
    fault_direct_event: DirectCurrentPeakEvent,
    fault_direct_event_valid: bool,
    safety_events: u32,
    last_safety_loss_reason: u8,
    phase_current_trips: u32,
    hall_sequence: u32,
    hall_interval_us: u32,
    hall_progress: i32,
    electrical_rpm: i32,
    observer_configured: bool,
    observer_ready: bool,
    observer_active: bool,
    observer_blend: u8,
    observer_confidence: u8,
    observer_validity_progress: u8,
    observer_electrical_rpm: i16,
    observer_hall_error_q16: i16,
    observer_flux_centi_mwb: u16,
    observer_bemf_q_mv: i16,
    observer_phase_error_q16: u16,
    timing_maxima: TimingMaxima,
    injected_samples: u32,
    control_max_cycles: u32,
    control_budget_warnings: u32,
    control_budget_overruns: u32,
}

struct ControlState {
    driver: RideFocDriver,
    target_ramp: QuadratureTargetRamp,
    offsets: CurrentOffsets,
    offset_tracker: CurrentOffsetTracker,
    hardware_hall_sequence: u32,
    hall_recovery_raw: u8,
    hall_recovery_cycles: u8,
    command_was_enabled: bool,
    diagnostics: ControlDiagnostics,
}

static CONTROL: IsrCell<MaybeUninit<ControlState>> =
    IsrCell(UnsafeCell::new(MaybeUninit::uninit()));
static CONTROL_INITIALIZED: AtomicBool = AtomicBool::new(false);

pub fn start(offsets: CurrentOffsets) {
    let mut hall = HallSensor::new(&crate::config::HALL_GEOMETRY);
    let hall_valid = hall.seed(hardware::live_hall_state()).is_ok();
    let mut phase = PhaseManager::with_hall(hall);
    phase.set_observer(BackEmfObserver::new(
        crate::config::MOTOR_PHASE_RESISTANCE_OHMS,
        crate::config::MOTOR_PHASE_INDUCTANCE_MILLIHENRIES,
        crate::config::MOTOR_FLUX_LINKAGE_MILLIWEBERS,
        crate::config::PWM_HZ,
    ));
    let source_configured = phase
        .set_source(PhaseSource::HallToObserver {
            blend_low_erpm: crate::config::OBSERVER_BLEND_LOW_ERPM,
            blend_high_erpm: crate::config::OBSERVER_BLEND_HIGH_ERPM,
        })
        .is_ok();
    debug_assert!(source_configured);
    let initial_flags = if hall_valid { CONTROL_HALL_VALID } else { 0 };
    // SAFETY: TIM1_UP remains masked until start_tim1_control_loop returns.
    unsafe {
        (*CONTROL.0.get()).write(ControlState {
            driver: ride_foc_driver(phase),
            target_ramp: QuadratureTargetRamp::new(
                crate::config::TARGET_RAMP_CYCLES_PER_STEP,
                crate::config::TARGET_RAMP_COUNTS_PER_STEP,
            ),
            offsets,
            offset_tracker: CurrentOffsetTracker::new(offsets.phase_a, offsets.phase_b),
            hardware_hall_sequence: 0,
            hall_recovery_raw: 0,
            hall_recovery_cycles: 0,
            command_was_enabled: false,
            diagnostics: ControlDiagnostics {
                flags: initial_flags,
                observer_configured: source_configured,
                ..ControlDiagnostics::default()
            },
        });
    }
    CONTROL_INITIALIZED.store(true, Ordering::Release);
    hardware::start_tim1_control_loop();
}

/// Refresh the observer's conversion from commanded PWM ticks to phase volts.
/// The foreground owns the relatively expensive divider; the 16 kHz ISR only
/// consumes the resulting Q16.16 scale.
pub fn set_bus_voltage_mv(bus_voltage_mv: u32) {
    let bits = crate::config::observer_volts_per_pwm_tick_bits(bus_voltage_mv);
    OBSERVER_VOLTS_PER_PWM_TICK_BITS.store(bits, Ordering::Release);
}

/// Grants a short, renewable local-ride lease. Only the foreground ride state
/// machine calls this; CAN frames never create motor authority.
pub fn authorize_ride_target(
    target_q_counts: i16,
    dc_current_limit_counts: u16,
    lifetime_cycles: u32,
    safety_epoch: u32,
) {
    TARGET_Q_COUNTS.store(
        i32::from(target_q_counts).clamp(
            -i32::from(crate::config::RIDE_PHASE_CURRENT_LIMIT_COUNTS),
            0,
        ),
        Ordering::Relaxed,
    );
    DC_CURRENT_LIMIT_COUNTS.store(
        u32::from(dc_current_limit_counts.min(crate::config::RIDE_DC_BUS_CURRENT_LIMIT_COUNTS)),
        Ordering::Relaxed,
    );
    COMMAND_SAFETY_EPOCH.store(safety_epoch, Ordering::Relaxed);
    let now = CONTROL_CYCLE.load(Ordering::Relaxed);
    COMMAND_DEADLINE.store(now.wrapping_add(lifetime_cycles), Ordering::Release);
    COMMAND_ENABLED.store(true, Ordering::Release);
}

pub fn revoke_ride_authority() {
    COMMAND_ENABLED.store(false, Ordering::Release);
    TARGET_Q_COUNTS.store(0, Ordering::Relaxed);
    DC_CURRENT_LIMIT_COUNTS.store(0, Ordering::Relaxed);
    COMMAND_DEADLINE.store(CONTROL_CYCLE.load(Ordering::Relaxed), Ordering::Release);
}

/// Starts a fresh control-peak capture after the hardware fault latch has been
/// acknowledged. The first active control sample is captured even when d is
/// exactly zero, then normal maximum-|d| ownership resumes for the episode.
pub fn begin_fault_diagnostic_episode() {
    cortex_m::interrupt::free(|_| {
        if CONTROL_INITIALIZED.load(Ordering::Acquire) {
            // SAFETY: CONTROL was initialized before its flag was published,
            // and TIM1_UP remains masked by this critical section.
            let state = unsafe { &mut *(*CONTROL.0.get()).as_mut_ptr() };
            let generation = state.diagnostics.maximum_direct_event.generation;
            state.diagnostics.maximum_direct_current_abs = 0;
            state.diagnostics.maximum_direct_event = DirectCurrentPeakEvent {
                generation,
                ..DirectCurrentPeakEvent::default()
            };
            state.diagnostics.fault_direct_event_valid = false;
            state.diagnostics.flags |= CONTROL_DIRECT_EVENT_PENDING;
        }
    });
}

pub fn snapshot() -> Snapshot {
    // SAFETY: TIM1_UP is the only writer after start(). Global interrupt
    // masking makes this one coherent copy and keeps atomics out of the 16 kHz
    // diagnostic path.
    let diagnostics = cortex_m::interrupt::free(|_| {
        if CONTROL_INITIALIZED.load(Ordering::Acquire) {
            unsafe { (*(*CONTROL.0.get()).as_ptr()).diagnostics }
        } else {
            ControlDiagnostics::default()
        }
    });
    let flags = diagnostics.flags;
    Snapshot {
        hall_valid: flags & CONTROL_HALL_VALID != 0,
        current_valid: flags & CONTROL_CURRENT_VALID != 0,
        output_active: flags & CONTROL_OUTPUT_ACTIVE != 0,
        voltage_limited: flags & CONTROL_VOLTAGE_LIMITED != 0,
        target_q_counts: diagnostics.target_q_counts,
        phase_current_limit_counts: diagnostics.phase_current_limit_counts,
        measured_d_counts: diagnostics.measured_d_counts,
        measured_q_counts: diagnostics.measured_q_counts,
        applied_d_ticks: diagnostics.applied_d_ticks,
        applied_q_ticks: diagnostics.applied_q_ticks,
        pwm_span_ticks: diagnostics.pwm_span_ticks,
        maximum_phase_current_abs: saturating_u32_to_u16(diagnostics.maximum_phase_current_abs),
        maximum_direct_current_abs: saturating_u32_to_u16(diagnostics.maximum_direct_current_abs),
        maximum_quadrature_error_abs: saturating_u32_to_u16(
            diagnostics.maximum_quadrature_error_abs,
        ),
        maximum_pwm_span_ticks: saturating_u32_to_u16(diagnostics.maximum_pwm_span_ticks),
        maximum_direct_event: if diagnostics.fault_direct_event_valid {
            diagnostics.fault_direct_event
        } else {
            diagnostics.maximum_direct_event
        },
        safety_events: diagnostics.safety_events,
        last_safety_loss_reason: diagnostics.last_safety_loss_reason,
        phase_current_trips: diagnostics.phase_current_trips,
        hall_sequence: diagnostics.hall_sequence,
        hall_interval_us: diagnostics.hall_interval_us,
        hall_progress: diagnostics.hall_progress,
        electrical_rpm: diagnostics.electrical_rpm,
        observer_configured: diagnostics.observer_configured,
        observer_ready: diagnostics.observer_ready,
        observer_active: diagnostics.observer_active,
        observer_blend: diagnostics.observer_blend,
        observer_confidence: diagnostics.observer_confidence,
        observer_validity_progress: diagnostics.observer_validity_progress,
        observer_electrical_rpm: diagnostics.observer_electrical_rpm,
        observer_hall_error_q16: diagnostics.observer_hall_error_q16,
        observer_flux_centi_mwb: diagnostics.observer_flux_centi_mwb,
        observer_bemf_q_mv: diagnostics.observer_bemf_q_mv,
        observer_phase_error_q16: diagnostics.observer_phase_error_q16,
        maximum_pre_driver_cycles: diagnostics.timing_maxima.pre_driver_cycles,
        maximum_driver_step_cycles: diagnostics.timing_maxima.driver_step_cycles,
        fault_flags: hardware::fault_flags(),
        control_cycles: CONTROL_CYCLE.load(Ordering::Relaxed),
        injected_samples: diagnostics.injected_samples,
        control_max_cycles: diagnostics.control_max_cycles,
        control_budget_warnings: diagnostics.control_budget_warnings,
        control_budget_overruns: diagnostics.control_budget_overruns,
    }
}

pub fn safe_for_updater_reset() -> bool {
    !COMMAND_ENABLED.load(Ordering::Acquire)
        && hardware::motor_outputs_disabled()
        && hardware::hall_is_quiet(500_000)
}

fn publish_flag(diagnostics: &mut ControlDiagnostics, flag: u32, value: bool) {
    if value {
        diagnostics.flags |= flag;
    } else {
        diagnostics.flags &= !flag;
    }
}

#[inline(never)]
fn note_safety_loss(diagnostics: &mut ControlDiagnostics, reason: SafetyLossReason) {
    diagnostics.last_safety_loss_reason = reason as u8;
    diagnostics.safety_events = diagnostics.safety_events.wrapping_add(1);
    crate::safety::record_safety_loss(reason as u8);
    revoke_ride_authority();
}

#[inline(never)]
fn publish_stopped_output(diagnostics: &mut ControlDiagnostics) {
    diagnostics.flags &= !(CONTROL_OUTPUT_ACTIVE | CONTROL_VOLTAGE_LIMITED);
    diagnostics.target_q_counts = 0;
    diagnostics.phase_current_limit_counts = 0;
    diagnostics.measured_d_counts = 0;
    diagnostics.measured_q_counts = 0;
    diagnostics.applied_d_ticks = 0;
    diagnostics.applied_q_ticks = 0;
    diagnostics.pwm_span_ticks = 0;
}

const fn lease_is_active(now: u32, deadline: u32) -> bool {
    let remaining = deadline.wrapping_sub(now);
    remaining != 0 && remaining < 0x8000_0000
}

fn stop_control(state: &mut ControlState, safety_loss: Option<SafetyLossReason>) {
    if hardware::fault_flags() != 0 {
        state.diagnostics.fault_direct_event = state.diagnostics.maximum_direct_event;
        state.diagnostics.fault_direct_event_valid = true;
    }
    hardware::disable_motor_outputs();
    hardware::write_pwm_neutral();
    publish_stopped_output(&mut state.diagnostics);
    state.driver.reset();
    state.target_ramp.reset();
    state.command_was_enabled = false;
    if let Some(reason) = safety_loss {
        note_safety_loss(&mut state.diagnostics, reason);
    }
}

#[interrupt]
fn TIM1_UP() {
    let started = hardware::cycle_count();
    hardware::clear_tim1_update_flag();
    crate::safety::timer_update_entered();
    if !crate::config::sample_injected_on_timer_update(hardware::tim1_counting_down()) {
        return;
    }
    control_cycle(started);
    let elapsed = hardware::cycle_count().wrapping_sub(started);
    if CONTROL_INITIALIZED.load(Ordering::Acquire) {
        // SAFETY: CONTROL is initialized before TIM1_UP is unmasked and this
        // handler is its only writer.
        let state = unsafe { &mut *(*CONTROL.0.get()).as_mut_ptr() };
        let _ = update_max(&mut state.diagnostics.control_max_cycles, elapsed);
        if elapsed > CONTROL_BUDGET_WARNING_CYCLES {
            state.diagnostics.control_budget_warnings =
                state.diagnostics.control_budget_warnings.wrapping_add(1);
        }
        if elapsed > CONTROL_BUDGET_CYCLES {
            state.diagnostics.control_budget_overruns =
                state.diagnostics.control_budget_overruns.wrapping_add(1);
            if hardware::fault_flags() & hardware::FAULT_CONTROL_TIMING == 0 {
                hardware::latch_control_timing_fault();
                state.diagnostics.fault_direct_event = state.diagnostics.maximum_direct_event;
                state.diagnostics.fault_direct_event_valid = true;
                publish_stopped_output(&mut state.diagnostics);
                note_safety_loss(&mut state.diagnostics, SafetyLossReason::ControlTiming);
            }
        }
        crate::safety::record_control_timing(elapsed, state.diagnostics.control_max_cycles);
    }
}

fn control_cycle(started: u32) {
    let control_cycle = CONTROL_CYCLE.load(Ordering::Relaxed).wrapping_add(1);
    CONTROL_CYCLE.store(control_cycle, Ordering::Relaxed);
    crate::safety::record_control_cycle(control_cycle);

    if !CONTROL_INITIALIZED.load(Ordering::Acquire) {
        hardware::emergency_shutdown();
        return;
    }
    // SAFETY: start() publishes a fully initialized value before unmasking
    // TIM1_UP, and only this interrupt accesses it afterwards.
    let state = unsafe { &mut *(*CONTROL.0.get()).as_mut_ptr() };
    let command_requested = COMMAND_ENABLED.load(Ordering::Acquire);
    let safety_events = state.diagnostics.safety_events;
    let command_enabled =
        command_requested && COMMAND_SAFETY_EPOCH.load(Ordering::Relaxed) == safety_events;
    let lease_active =
        command_enabled && lease_is_active(control_cycle, COMMAND_DEADLINE.load(Ordering::Acquire));
    let output_was_active = state.diagnostics.flags & CONTROL_OUTPUT_ACTIVE != 0;

    let current = match hardware::read_phase_currents(state.offsets) {
        Ok(sample) => {
            state.diagnostics.injected_samples = state.diagnostics.injected_samples.wrapping_add(1);
            publish_flag(&mut state.diagnostics, CONTROL_CURRENT_VALID, true);
            crate::safety::record_control_checkpoint(crate::safety::checkpoint::CURRENT_SAMPLED);
            sample
        }
        Err(_) => {
            publish_flag(&mut state.diagnostics, CONTROL_CURRENT_VALID, false);
            let timing_lost = lease_active || output_was_active;
            if timing_lost && hardware::fault_flags() == 0 {
                hardware::latch_control_timing_fault();
            }
            stop_control(
                state,
                timing_lost.then_some(SafetyLossReason::CurrentSample),
            );
            return;
        }
    };
    if lease_active || output_was_active {
        let _ = update_max(
            &mut state.diagnostics.maximum_phase_current_abs,
            u32::from(
                current
                    .phase_a
                    .unsigned_abs()
                    .max(current.phase_b.unsigned_abs())
                    .max(current.phase_c.unsigned_abs()),
            ),
        );
    }
    if current.exceeds_limit(crate::config::PHASE_CURRENT_TRIP_COUNTS)
        && (lease_active || output_was_active)
    {
        state.diagnostics.phase_current_trips =
            state.diagnostics.phase_current_trips.wrapping_add(1);
        stop_control(state, Some(SafetyLossReason::PhaseOvercurrent));
        return;
    }
    if !lease_active && hardware::motor_outputs_disabled() {
        state
            .offset_tracker
            .observe_undriven(current.raw_a, current.raw_b);
        let (phase_a, phase_b) = state.offset_tracker.offsets();
        state.offsets = CurrentOffsets { phase_a, phase_b };
    }

    let (sequence, raw, interval_us) = hardware::hall_edge_snapshot();
    if sequence != state.hardware_hall_sequence {
        if state.hardware_hall_sequence != 0
            && sequence.wrapping_sub(state.hardware_hall_sequence) != 1
        {
            hardware::latch_hall_capture_fault();
            publish_flag(&mut state.diagnostics, CONTROL_HALL_VALID, false);
            stop_control(state, Some(SafetyLossReason::HallCaptureSequence));
            return;
        }
        state.hardware_hall_sequence = sequence;
        if state
            .driver
            .phase_mut()
            .hall_mut()
            .update_edge(raw, interval_us)
            .is_err()
        {
            publish_flag(&mut state.diagnostics, CONTROL_HALL_VALID, false);
            stop_control(state, Some(SafetyLossReason::HallTransition));
            return;
        }
        state.hall_recovery_cycles = 0;
        state.diagnostics.hall_sequence = sequence;
        state.diagnostics.hall_interval_us = state.driver.phase().hall().sector_interval_us();
        state.diagnostics.hall_progress = state
            .diagnostics
            .hall_progress
            .wrapping_add(i32::from(state.driver.phase().hall().physical_direction()));
        state.diagnostics.electrical_rpm = state.driver.phase().hall().electrical_rpm();
    }
    let command_started = command_enabled && !state.command_was_enabled;
    if command_started && state.driver.phase().hall().is_stationary() {
        let live_before = hardware::live_hall_state();
        if live_before != state.driver.phase().hall().raw_state() {
            publish_flag(&mut state.diagnostics, CONTROL_HALL_VALID, false);
            stop_control(state, Some(SafetyLossReason::HallStationaryMismatch));
            return;
        }
        hardware::restart_stationary_hall_interval();
        if hardware::live_hall_state() != live_before {
            publish_flag(&mut state.diagnostics, CONTROL_HALL_VALID, false);
            stop_control(state, Some(SafetyLossReason::HallStationaryMismatch));
            return;
        }
        state.driver.phase_mut().hall_mut().discard_next_interval();
    }
    if command_started {
        state.driver.phase_mut().request_observer_seed();
    }
    state.command_was_enabled = command_enabled;
    let edge_age_us = hardware::hall_edge_age_us().saturating_add(3);
    let mut estimate = state
        .driver
        .estimate_for_control(edge_age_us, CONTROL_PERIOD_NS);
    if estimate.is_none()
        && !lease_active
        && hardware::motor_outputs_disabled()
        && recover_stationary_hall(state)
    {
        estimate = state.driver.estimate_for_control(0, CONTROL_PERIOD_NS);
    }
    let Some(estimate) = estimate else {
        publish_flag(&mut state.diagnostics, CONTROL_HALL_VALID, false);
        stop_control(
            state,
            (lease_active || output_was_active).then_some(SafetyLossReason::PhaseEstimate),
        );
        return;
    };
    state.hall_recovery_cycles = 0;
    let angle = estimate.angle;
    publish_flag(&mut state.diagnostics, CONTROL_HALL_VALID, true);
    crate::safety::record_control_checkpoint(crate::safety::checkpoint::PHASE_ESTIMATED);

    if hardware::fault_flags() != 0 {
        stop_control(
            state,
            output_was_active.then_some(SafetyLossReason::HardwareFault),
        );
        return;
    }

    if !lease_active {
        stop_control(
            state,
            command_enabled.then_some(SafetyLossReason::OutputLease),
        );
        return;
    }

    if control_cycle & 0xff == 0 && !command_started {
        publish_observer_diagnostics(
            &mut state.diagnostics,
            state.driver.phase().observer_diagnostics(),
        );
    }

    let dc_current_limit = DC_CURRENT_LIMIT_COUNTS
        .load(Ordering::Relaxed)
        .min(u32::from(u16::MAX)) as u16;
    state.driver.set_bus_limits(
        Some(Fixed::from_integer(i32::from(dc_current_limit))),
        Some(Fixed::from_integer(i32::from(
            crate::config::RIDE_DC_BUS_REGEN_LIMIT_COUNTS,
        ))),
    );
    let requested_q = state
        .target_ramp
        .next(TARGET_Q_COUNTS.load(Ordering::Relaxed));
    // Bound the phase-derived speed before it reaches timing-dependent
    // controller paths.
    let control_electrical_rpm = estimate.electrical_rpm.clamp(-76_000, 76_000);
    let control_estimate = PhaseEstimate {
        electrical_rpm: control_electrical_rpm,
        ..estimate
    };
    let actuation_advance = actuation_advance_from_erpm(control_electrical_rpm);
    state.driver.set_actuation_advance(actuation_advance);
    state.driver.set_volts_per_pwm_tick(Fixed::from_bits(
        OBSERVER_VOLTS_PER_PWM_TICK_BITS.load(Ordering::Relaxed),
    ));
    note_timing_maximum(
        &mut state.diagnostics.timing_maxima.pre_driver_cycles,
        started,
    );
    let driver_step_started = hardware::cycle_count();
    let output = match state.driver.step_current_control(
        Fixed::from_integer(i32::from(current.phase_a)),
        Fixed::from_integer(i32::from(current.phase_b)),
        control_estimate,
        Dq::new(Fixed::ZERO, Fixed::from_integer(requested_q)),
        crate::config::PWM_NEUTRAL,
        CONTROL_PERIOD_NS,
    ) {
        Ok(output) => output,
        Err(StepError::Overcurrent) => {
            note_timing_maximum(
                &mut state.diagnostics.timing_maxima.driver_step_cycles,
                driver_step_started,
            );
            state.diagnostics.phase_current_trips =
                state.diagnostics.phase_current_trips.wrapping_add(1);
            stop_control(state, Some(SafetyLossReason::PhaseOvercurrent));
            return;
        }
    };
    note_timing_maximum(
        &mut state.diagnostics.timing_maxima.driver_step_cycles,
        driver_step_started,
    );
    crate::safety::record_control_checkpoint(crate::safety::checkpoint::DRIVER_COMPLETE);
    let phase_limit = output
        .quadrature_limit
        .abs_ceil_u32()
        .min(u32::from(u16::MAX)) as u16;
    let target_q = output.target.q.integer();
    let measured_current = output.measured_current;
    let duty = output.duties;
    let measured_d_counts = fixed_to_i16(measured_current.d);
    let measured_q_counts = fixed_to_i16(measured_current.q);
    let applied_voltage = output.applied_voltage;
    let applied_d_ticks = fixed_to_i16(applied_voltage.d);
    let applied_q_ticks = fixed_to_i16(applied_voltage.q);
    let pwm_span_ticks = pwm_span(duty);
    state.diagnostics.target_q_counts = target_q as i16;
    state.diagnostics.phase_current_limit_counts = phase_limit;
    state.diagnostics.measured_d_counts = measured_d_counts;
    state.diagnostics.measured_q_counts = measured_q_counts;
    state.diagnostics.applied_d_ticks = applied_d_ticks;
    state.diagnostics.applied_q_ticks = applied_q_ticks;
    state.diagnostics.pwm_span_ticks = pwm_span_ticks;
    publish_flag(
        &mut state.diagnostics,
        CONTROL_VOLTAGE_LIMITED,
        output.voltage_limited,
    );
    let direct_is_max = update_max(
        &mut state.diagnostics.maximum_direct_current_abs,
        u32::from(measured_d_counts.unsigned_abs()),
    );
    if direct_is_max || state.diagnostics.flags & CONTROL_DIRECT_EVENT_PENDING != 0 {
        state.diagnostics.flags &= !CONTROL_DIRECT_EVENT_PENDING;
        let unlimited_angle = state.driver.phase().hall().unlimited_angle();
        let measurement_angle_q16 = (angle >> 16) as u16;
        let unlimited_angle_q16 = (unlimited_angle >> 16) as u16;
        capture_direct_current_peak(
            &mut state.diagnostics,
            DirectCurrentPeakEvent {
                measured_d_counts,
                measured_q_counts,
                target_q_counts: target_q as i16,
                hall_raw: state.driver.phase().hall().raw_state(),
                hall_angle_direction: state.driver.phase().hall().angle_direction(),
                edge_age_us: saturating_u32_to_u16(edge_age_us),
                hall_interval_us: saturating_u32_to_u16(
                    state.driver.phase().hall().sector_interval_us(),
                ),
                angle_error_q8: (unlimited_angle_q16.wrapping_sub(measurement_angle_q16) as i16
                    >> 8) as i8,
                requested_d_ticks: fixed_to_i16(output.requested_voltage.d),
                requested_q_ticks: fixed_to_i16(output.requested_voltage.q),
                feedforward_d_ticks: fixed_to_i16(output.feedforward_voltage.d),
                feedforward_q_ticks: fixed_to_i16(output.feedforward_voltage.q),
                applied_d_ticks,
                applied_q_ticks,
                voltage_limited: output.voltage_limited,
                angle_rate_limited: angle != unlimited_angle,
                ..DirectCurrentPeakEvent::default()
            },
        );
    }
    let _ = update_max(
        &mut state.diagnostics.maximum_quadrature_error_abs,
        target_q
            .saturating_sub(i32::from(measured_q_counts))
            .unsigned_abs(),
    );
    let _ = update_max(
        &mut state.diagnostics.maximum_pwm_span_ticks,
        u32::from(pwm_span_ticks),
    );
    if !hardware::write_pwm_duties(duty) || !hardware::enable_motor_outputs() {
        stop_control(state, Some(SafetyLossReason::PwmOutput));
        return;
    }
    crate::safety::record_control_checkpoint(crate::safety::checkpoint::PWM_WRITTEN);
    publish_flag(&mut state.diagnostics, CONTROL_OUTPUT_ACTIVE, true);
}

fn publish_observer_diagnostics(
    published: &mut ControlDiagnostics,
    diagnostics: ObserverDiagnostics,
) {
    published.observer_configured = diagnostics.configured;
    published.observer_ready = diagnostics.ready;
    published.observer_active = diagnostics.active;
    published.observer_blend = fixed_unit_to_u8(diagnostics.blend);
    published.observer_confidence = fixed_unit_to_u8(diagnostics.confidence);
    published.observer_validity_progress = diagnostics.validity_progress;
    published.observer_electrical_rpm = saturating_i32_to_i16(diagnostics.electrical_rpm);
    published.observer_hall_error_q16 = (diagnostics.hall_error_q32 >> 16) as i16;
    let flux_centi_mwb = fixed_to_scaled_i32(diagnostics.flux_magnitude_mwb, 100)
        .clamp(0, i32::from(u16::MAX)) as u16;
    let bemf_q_mv = saturating_i32_to_i16(fixed_to_scaled_i32(diagnostics.bemf_q_v, 1_000));
    published.observer_flux_centi_mwb = flux_centi_mwb;
    published.observer_bemf_q_mv = bemf_q_mv;
    published.observer_phase_error_q16 =
        (diagnostics.phase_error_filtered_q32 >> 16).min(u32::from(u16::MAX)) as u16;
}

#[inline(never)]
fn note_timing_maximum(maximum: &mut u16, started: u32) {
    let elapsed = hardware::cycle_count().wrapping_sub(started);
    *maximum = (*maximum).max(saturating_u32_to_u16(elapsed));
}

fn fixed_unit_to_u8(value: Fixed) -> u8 {
    let bits = value.to_bits().clamp(0, Fixed::ONE.to_bits());
    ((i64::from(bits) * i64::from(u8::MAX) + (1 << 15)) >> 16) as u8
}

fn fixed_to_scaled_i32(value: Fixed, scale: i32) -> i32 {
    ((i64::from(value.to_bits()) * i64::from(scale)) >> 16)
        .clamp(i64::from(i32::MIN), i64::from(i32::MAX)) as i32
}

fn saturating_i32_to_i16(value: i32) -> i16 {
    value.clamp(i32::from(i16::MIN), i32::from(i16::MAX)) as i16
}

fn actuation_advance_from_erpm(electrical_rpm: i32) -> Fixed {
    // Q16.16 radians per control period. 1757/4096 approximates
    // 2*pi*65536/(60*16000) to 0.006%, with no division in the ISR.
    let bits = (electrical_rpm * 1_757) >> 12;
    Fixed::from_bits(bits)
}

const fn ride_foc_controller() -> RideFocController {
    let pi = PIController::new(
        crate::config::CURRENT_PI_PROPORTIONAL_GAIN,
        crate::config::CURRENT_PI_INTEGRAL_GAIN_PER_CYCLE,
    );
    RideFocController::new(
        pi,
        pi,
        crate::config::FOC_VECTOR_LIMIT_TICKS,
        crate::config::FOC_PHASE_LIMIT_TICKS,
    )
}

const fn ride_foc_driver(phase: PhaseManager<HallSensor>) -> RideFocDriver {
    RideFocDriver::new(
        ride_foc_controller(),
        phase,
        ride_current_limits(),
        crate::config::PWM_ARR,
        BUS_MODULATION_FILTER_SHIFT,
    )
    .with_observer_scales(Fixed::ZERO, crate::config::PHASE_CURRENT_AMPS_PER_ADC_COUNT)
}

const fn ride_current_limits() -> CurrentLimits {
    CurrentLimits::new(
        Fixed::from_integer(crate::config::RIDE_PHASE_CURRENT_LIMIT_COUNTS as i32),
        Fixed::from_integer(crate::config::PHASE_CURRENT_TRIP_COUNTS as i32),
        Some(Fixed::from_integer(
            crate::config::RIDE_DC_BUS_CURRENT_LIMIT_COUNTS as i32,
        )),
        Some(Fixed::from_integer(
            crate::config::RIDE_DC_BUS_REGEN_LIMIT_COUNTS as i32,
        )),
    )
}

fn update_max(target: &mut u32, value: u32) -> bool {
    if value > *target {
        *target = value;
        true
    } else {
        false
    }
}

fn capture_direct_current_peak(
    diagnostics: &mut ControlDiagnostics,
    mut event: DirectCurrentPeakEvent,
) {
    event.generation = diagnostics
        .maximum_direct_event
        .generation
        .wrapping_add(1)
        .max(1);
    diagnostics.maximum_direct_event = event;
}

const fn saturating_u32_to_u16(value: u32) -> u16 {
    if value > u16::MAX as u32 {
        u16::MAX
    } else {
        value as u16
    }
}

fn fixed_to_i16(value: Fixed) -> i16 {
    value
        .integer()
        .clamp(i32::from(i16::MIN), i32::from(i16::MAX)) as i16
}

fn pwm_span(duty: oxifoc_core::foc::PwmDuty) -> u16 {
    let minimum = duty.a.min(duty.b).min(duty.c);
    let maximum = duty.a.max(duty.b).max(duty.c);
    maximum - minimum
}

fn recover_stationary_hall(state: &mut ControlState) -> bool {
    let raw = hardware::live_hall_state();
    if !crate::config::HALL_GEOMETRY.raw_is_valid(raw) {
        state.hall_recovery_raw = 0;
        state.hall_recovery_cycles = 0;
        return false;
    }
    if raw != state.hall_recovery_raw {
        state.hall_recovery_raw = raw;
        state.hall_recovery_cycles = 1;
        return false;
    }
    state.hall_recovery_cycles = state.hall_recovery_cycles.saturating_add(1);
    if state.hall_recovery_cycles < HALL_RECOVERY_STABLE_CYCLES {
        return false;
    }
    if state.driver.phase_mut().hall_mut().seed(raw).is_err() {
        return false;
    }
    state.hall_recovery_cycles = 0;
    state.diagnostics.hall_interval_us = 0;
    state.diagnostics.electrical_rpm = 0;
    true
}
