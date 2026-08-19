//! 16 kHz Hall-only fixed-point current-control interrupt.

use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicBool, AtomicI32, AtomicU32, Ordering};

use crate::hardware::peripherals::{self as hardware, CurrentOffsets};
use oxifoc_core::foc::{
    Dq, Fixed, FocController, PIController, Scalar,
    current_limits::{CurrentLimiter, CurrentLimits},
    hall_sensor::HallSensor,
    offset_tracker::CurrentOffsetTracker,
    phase::{PhaseInput, PhaseManager, PhaseProvider},
    ramp::QuadratureTargetRamp,
};
use stm32f1::stm32f103::interrupt;

const CONTROL_HALL_VALID: u32 = 1;
const CONTROL_CURRENT_VALID: u32 = 1 << 1;
const CONTROL_OUTPUT_ACTIVE: u32 = 1 << 2;
const CONTROL_VOLTAGE_LIMITED: u32 = 1 << 3;
const HALL_RECOVERY_STABLE_CYCLES: u8 = 5;
const CONTROL_BUDGET_WARNING_CYCLES: u32 = 3_900;
const CONTROL_BUDGET_CYCLES: u32 = crate::config::SYSCLK_HZ / crate::config::PWM_HZ;
const CONTROL_PERIOD_NS: u32 = 1_000_000_000 / crate::config::PWM_HZ;
// One IIR step per 1/32 of the error gives the 2 ms bus-modulation filter at 16 kHz.
const BUS_MODULATION_FILTER_SHIFT: u8 = 5;

static TARGET_Q_COUNTS: AtomicI32 = AtomicI32::new(0);
static DC_CURRENT_LIMIT_COUNTS: AtomicU32 = AtomicU32::new(0);
static CONTROL_CYCLE: AtomicU32 = AtomicU32::new(0);
static COMMAND_DEADLINE: AtomicU32 = AtomicU32::new(0);
static COMMAND_SAFETY_EPOCH: AtomicU32 = AtomicU32::new(0);
static COMMAND_ENABLED: AtomicBool = AtomicBool::new(false);
static CONTROL_FLAGS: AtomicU32 = AtomicU32::new(0);
static CONTROL_SAFETY_EVENTS: AtomicU32 = AtomicU32::new(0);
static PHASE_CURRENT_TRIPS: AtomicU32 = AtomicU32::new(0);
static INJECTED_SAMPLES: AtomicU32 = AtomicU32::new(0);
static CONTROL_MAX_CYCLES: AtomicU32 = AtomicU32::new(0);
static CONTROL_BUDGET_WARNINGS: AtomicU32 = AtomicU32::new(0);
static CONTROL_BUDGET_OVERRUNS: AtomicU32 = AtomicU32::new(0);
static LIVE_TARGET_AND_LIMIT: AtomicU32 = AtomicU32::new(0);
static LIVE_MEASURED_DQ: AtomicU32 = AtomicU32::new(0);
static LIVE_APPLIED_DQ: AtomicU32 = AtomicU32::new(0);
static LIVE_PWM_SPAN: AtomicU32 = AtomicU32::new(0);
static MAXIMUM_PHASE_CURRENT_ABS: AtomicU32 = AtomicU32::new(0);
static MAXIMUM_DIRECT_CURRENT_ABS: AtomicU32 = AtomicU32::new(0);
static MAXIMUM_QUADRATURE_ERROR_ABS: AtomicU32 = AtomicU32::new(0);
static MAXIMUM_PWM_SPAN: AtomicU32 = AtomicU32::new(0);
static VALIDATED_HALL_SEQUENCE: AtomicU32 = AtomicU32::new(0);
static VALIDATED_HALL_INTERVAL_US: AtomicU32 = AtomicU32::new(0);
static VALIDATED_HALL_PROGRESS: AtomicI32 = AtomicI32::new(0);
static ELECTRICAL_RPM: AtomicI32 = AtomicI32::new(0);

struct IsrCell<T>(UnsafeCell<T>);

// SAFETY: each instance below documents how interrupt/foreground exclusion is
// maintained for its contained value.
unsafe impl<T> Sync for IsrCell<T> {}

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
    pub measurement_angle_q16: u16,
    pub unlimited_angle_q16: u16,
    pub phase_a_counts: i16,
    pub phase_b_counts: i16,
    pub applied_d_ticks: i16,
    pub applied_q_ticks: i16,
    pub voltage_limited: bool,
    pub angle_rate_limited: bool,
}

static PEAK_DIRECT_EVENT: IsrCell<DirectCurrentPeakEvent> =
    IsrCell(UnsafeCell::new(DirectCurrentPeakEvent {
        generation: 0,
        measured_d_counts: 0,
        measured_q_counts: 0,
        target_q_counts: 0,
        hall_raw: 0,
        hall_angle_direction: 0,
        edge_age_us: 0,
        hall_interval_us: 0,
        measurement_angle_q16: 0,
        unlimited_angle_q16: 0,
        phase_a_counts: 0,
        phase_b_counts: 0,
        applied_d_ticks: 0,
        applied_q_ticks: 0,
        voltage_limited: false,
        angle_rate_limited: false,
    }));

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
    pub phase_current_trips: u32,
    pub hall_sequence: u32,
    pub hall_interval_us: u32,
    pub hall_progress: i32,
    pub electrical_rpm: i32,
    pub fault_flags: u32,
    pub control_cycles: u32,
    pub injected_samples: u32,
    pub control_max_cycles: u32,
    pub control_budget_warnings: u32,
    pub control_budget_overruns: u32,
}

struct ControlState {
    controller: FocController,
    current_limiter: CurrentLimiter,
    target_ramp: QuadratureTargetRamp,
    phase: PhaseManager<HallSensor>,
    offsets: CurrentOffsets,
    offset_tracker: CurrentOffsetTracker,
    hardware_hall_sequence: u32,
    hall_recovery_raw: u8,
    hall_recovery_cycles: u8,
    command_was_enabled: bool,
}

static CONTROL: IsrCell<Option<ControlState>> = IsrCell(UnsafeCell::new(None));

pub fn start(offsets: CurrentOffsets) {
    let mut hall = HallSensor::new(&crate::config::HALL_GEOMETRY);
    let hall_valid = hall.seed(hardware::live_hall_state()).is_ok();
    let initial_flags = if hall_valid { CONTROL_HALL_VALID } else { 0 };
    CONTROL_FLAGS.store(initial_flags, Ordering::Release);
    // SAFETY: TIM1_UP remains masked until start_tim1_control_loop returns.
    unsafe {
        CONTROL.0.get().write(Some(ControlState {
            controller: ride_foc_controller(),
            current_limiter: ride_current_limiter(),
            target_ramp: QuadratureTargetRamp::new(
                crate::config::TARGET_RAMP_CYCLES_PER_STEP,
                crate::config::TARGET_RAMP_COUNTS_PER_STEP,
            ),
            phase: PhaseManager::with_hall(hall),
            offsets,
            offset_tracker: CurrentOffsetTracker::new(offsets.phase_a, offsets.phase_b),
            hardware_hall_sequence: 0,
            hall_recovery_raw: 0,
            hall_recovery_cycles: 0,
            command_was_enabled: false,
        }));
    }
    hardware::start_tim1_control_loop();
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
    let now = CONTROL_CYCLE.load(Ordering::Acquire);
    COMMAND_DEADLINE.store(now.wrapping_add(lifetime_cycles), Ordering::Release);
    COMMAND_ENABLED.store(true, Ordering::Release);
}

pub fn revoke_ride_authority() {
    COMMAND_ENABLED.store(false, Ordering::Release);
    TARGET_Q_COUNTS.store(0, Ordering::Relaxed);
    DC_CURRENT_LIMIT_COUNTS.store(0, Ordering::Relaxed);
    COMMAND_DEADLINE.store(CONTROL_CYCLE.load(Ordering::Acquire), Ordering::Release);
}

pub fn snapshot() -> Snapshot {
    let flags = CONTROL_FLAGS.load(Ordering::Acquire);
    let (target_q_counts, phase_current_limit_counts) =
        unpack_target_and_limit(LIVE_TARGET_AND_LIMIT.load(Ordering::Relaxed));
    let (measured_d_counts, measured_q_counts) =
        unpack_i16_pair(LIVE_MEASURED_DQ.load(Ordering::Relaxed));
    let (applied_d_ticks, applied_q_ticks) =
        unpack_i16_pair(LIVE_APPLIED_DQ.load(Ordering::Relaxed));
    // SAFETY: TIM1_UP is the only writer, and global interrupt masking keeps
    // this multiword copy coherent with the exact cycle that set the peak.
    let maximum_direct_event = cortex_m::interrupt::free(|_| unsafe { *PEAK_DIRECT_EVENT.0.get() });
    Snapshot {
        hall_valid: flags & CONTROL_HALL_VALID != 0,
        current_valid: flags & CONTROL_CURRENT_VALID != 0,
        output_active: flags & CONTROL_OUTPUT_ACTIVE != 0,
        voltage_limited: flags & CONTROL_VOLTAGE_LIMITED != 0,
        target_q_counts,
        phase_current_limit_counts,
        measured_d_counts,
        measured_q_counts,
        applied_d_ticks,
        applied_q_ticks,
        pwm_span_ticks: LIVE_PWM_SPAN.load(Ordering::Relaxed) as u16,
        maximum_phase_current_abs: MAXIMUM_PHASE_CURRENT_ABS.load(Ordering::Relaxed) as u16,
        maximum_direct_current_abs: MAXIMUM_DIRECT_CURRENT_ABS.load(Ordering::Relaxed) as u16,
        maximum_quadrature_error_abs: MAXIMUM_QUADRATURE_ERROR_ABS.load(Ordering::Relaxed) as u16,
        maximum_pwm_span_ticks: MAXIMUM_PWM_SPAN.load(Ordering::Relaxed) as u16,
        maximum_direct_event,
        safety_events: CONTROL_SAFETY_EVENTS.load(Ordering::Acquire),
        phase_current_trips: PHASE_CURRENT_TRIPS.load(Ordering::Relaxed),
        hall_sequence: VALIDATED_HALL_SEQUENCE.load(Ordering::Acquire),
        hall_interval_us: VALIDATED_HALL_INTERVAL_US.load(Ordering::Relaxed),
        hall_progress: VALIDATED_HALL_PROGRESS.load(Ordering::Relaxed),
        electrical_rpm: ELECTRICAL_RPM.load(Ordering::Relaxed),
        fault_flags: hardware::fault_flags(),
        control_cycles: CONTROL_CYCLE.load(Ordering::Relaxed),
        injected_samples: INJECTED_SAMPLES.load(Ordering::Relaxed),
        control_max_cycles: CONTROL_MAX_CYCLES.load(Ordering::Relaxed),
        control_budget_warnings: CONTROL_BUDGET_WARNINGS.load(Ordering::Relaxed),
        control_budget_overruns: CONTROL_BUDGET_OVERRUNS.load(Ordering::Relaxed),
    }
}

pub fn safe_for_updater_reset() -> bool {
    !COMMAND_ENABLED.load(Ordering::Acquire)
        && hardware::motor_outputs_disabled()
        && hardware::hall_is_quiet(500_000)
}

fn publish_flag(flag: u32, value: bool) {
    if value {
        CONTROL_FLAGS.fetch_or(flag, Ordering::Release);
    } else {
        CONTROL_FLAGS.fetch_and(!flag, Ordering::Release);
    }
}

fn note_safety_loss() {
    CONTROL_SAFETY_EVENTS.fetch_add(1, Ordering::Release);
    revoke_ride_authority();
}

const fn lease_is_active(now: u32, deadline: u32) -> bool {
    let remaining = deadline.wrapping_sub(now);
    remaining != 0 && remaining < 0x8000_0000
}

fn stop_control(state: &mut ControlState, safety_loss: bool) {
    hardware::disable_motor_outputs();
    hardware::write_pwm_neutral();
    CONTROL_FLAGS.fetch_and(
        !(CONTROL_OUTPUT_ACTIVE | CONTROL_VOLTAGE_LIMITED),
        Ordering::Release,
    );
    LIVE_TARGET_AND_LIMIT.store(0, Ordering::Relaxed);
    LIVE_MEASURED_DQ.store(0, Ordering::Relaxed);
    LIVE_APPLIED_DQ.store(0, Ordering::Relaxed);
    LIVE_PWM_SPAN.store(0, Ordering::Relaxed);
    state.controller.reset();
    state.current_limiter.reset();
    state.target_ramp.reset();
    state.command_was_enabled = false;
    if safety_loss {
        note_safety_loss();
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
    control_cycle();
    let elapsed = hardware::cycle_count().wrapping_sub(started);
    let _ = update_max(&CONTROL_MAX_CYCLES, elapsed);
    if elapsed > CONTROL_BUDGET_WARNING_CYCLES {
        CONTROL_BUDGET_WARNINGS.fetch_add(1, Ordering::Relaxed);
    }
    if elapsed > CONTROL_BUDGET_CYCLES {
        CONTROL_BUDGET_OVERRUNS.fetch_add(1, Ordering::Relaxed);
        if hardware::fault_flags() & hardware::FAULT_CONTROL_TIMING == 0 {
            hardware::latch_control_timing_fault();
            note_safety_loss();
        }
    }
}

fn control_cycle() {
    let control_cycle = CONTROL_CYCLE.fetch_add(1, Ordering::AcqRel).wrapping_add(1);

    // SAFETY: only this interrupt accesses the state after initialization.
    let Some(state) = (unsafe { &mut *CONTROL.0.get() }).as_mut() else {
        hardware::emergency_shutdown();
        return;
    };
    let command_requested = COMMAND_ENABLED.load(Ordering::Acquire);
    let safety_events = CONTROL_SAFETY_EVENTS.load(Ordering::Acquire);
    let command_enabled =
        command_requested && COMMAND_SAFETY_EPOCH.load(Ordering::Relaxed) == safety_events;
    let lease_active =
        command_enabled && lease_is_active(control_cycle, COMMAND_DEADLINE.load(Ordering::Acquire));
    let output_was_active = CONTROL_FLAGS.load(Ordering::Relaxed) & CONTROL_OUTPUT_ACTIVE != 0;

    let current = match hardware::read_phase_currents(state.offsets) {
        Ok(sample) => {
            INJECTED_SAMPLES.fetch_add(1, Ordering::Relaxed);
            publish_flag(CONTROL_CURRENT_VALID, true);
            sample
        }
        Err(_) => {
            publish_flag(CONTROL_CURRENT_VALID, false);
            let timing_lost = lease_active || output_was_active;
            if timing_lost {
                hardware::latch_control_timing_fault();
            }
            stop_control(state, timing_lost);
            return;
        }
    };
    if lease_active || output_was_active {
        let _ = update_max(
            &MAXIMUM_PHASE_CURRENT_ABS,
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
        PHASE_CURRENT_TRIPS.fetch_add(1, Ordering::Relaxed);
        stop_control(state, true);
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
            publish_flag(CONTROL_HALL_VALID, false);
            stop_control(state, true);
            return;
        }
        state.hardware_hall_sequence = sequence;
        if state
            .phase
            .hall_mut()
            .update_edge(raw, interval_us)
            .is_err()
        {
            publish_flag(CONTROL_HALL_VALID, false);
            stop_control(state, true);
            return;
        }
        state.hall_recovery_cycles = 0;
        VALIDATED_HALL_SEQUENCE.store(sequence, Ordering::Release);
        VALIDATED_HALL_INTERVAL_US
            .store(state.phase.hall().sector_interval_us(), Ordering::Relaxed);
        VALIDATED_HALL_PROGRESS.fetch_add(
            i32::from(state.phase.hall().physical_direction()),
            Ordering::Relaxed,
        );
        ELECTRICAL_RPM.store(state.phase.hall().electrical_rpm(), Ordering::Relaxed);
    }
    if command_enabled && !state.command_was_enabled && state.phase.hall().is_stationary() {
        let live_before = hardware::live_hall_state();
        if live_before != state.phase.hall().raw_state() {
            publish_flag(CONTROL_HALL_VALID, false);
            stop_control(state, true);
            return;
        }
        hardware::restart_stationary_hall_interval();
        if hardware::live_hall_state() != live_before {
            publish_flag(CONTROL_HALL_VALID, false);
            stop_control(state, true);
            return;
        }
        state.phase.hall_mut().discard_next_interval();
    }
    state.command_was_enabled = command_enabled;
    let edge_age_us = hardware::hall_edge_age_us().saturating_add(3);
    let angle = match state
        .phase
        .estimate_for_control(edge_age_us, CONTROL_PERIOD_NS)
    {
        Some(estimate) => {
            state.hall_recovery_cycles = 0;
            estimate.angle
        }
        None if !lease_active
            && hardware::motor_outputs_disabled()
            && recover_stationary_hall(state) =>
        {
            state
                .phase
                .estimate_for_control(0, CONTROL_PERIOD_NS)
                .map(|estimate| estimate.angle)
                .unwrap_or_default()
        }
        None => {
            publish_flag(CONTROL_HALL_VALID, false);
            stop_control(state, lease_active || output_was_active);
            return;
        }
    };
    publish_flag(CONTROL_HALL_VALID, true);

    if hardware::fault_flags() != 0 {
        stop_control(state, output_was_active);
        return;
    }

    if !lease_active {
        stop_control(state, command_enabled);
        return;
    }

    let dc_current_limit = DC_CURRENT_LIMIT_COUNTS
        .load(Ordering::Relaxed)
        .min(u32::from(u16::MAX)) as u16;
    state.current_limiter.set_bus_limits(
        Some(Fixed::from_integer(i32::from(dc_current_limit))),
        Some(Fixed::ZERO),
    );
    let limited_target = state.current_limiter.clamp_targets_with_limit(Dq::new(
        Fixed::ZERO,
        Fixed::from_integer(TARGET_Q_COUNTS.load(Ordering::Relaxed)),
    ));
    let phase_limit = limited_target
        .quadrature_limit
        .abs_ceil_u32()
        .min(u32::from(u16::MAX)) as u16;
    let target_q = state.target_ramp.next(limited_target.target.q.integer());
    state
        .controller
        .set_actuation_advance(state.phase.hall().actuation_advance());
    let (measured_current, duty) = state.controller.step_with_injection(
        Fixed::from_integer(i32::from(current.phase_a)),
        Fixed::from_integer(i32::from(current.phase_b)),
        angle,
        Dq {
            d: Fixed::ZERO,
            q: Fixed::from_integer(target_q),
        },
        state.phase.injection(),
        crate::config::PWM_NEUTRAL,
    );
    if state.current_limiter.is_overcurrent(measured_current) {
        PHASE_CURRENT_TRIPS.fetch_add(1, Ordering::Relaxed);
        stop_control(state, true);
        return;
    }
    let measured_d_counts = fixed_to_i16(measured_current.d);
    let measured_q_counts = fixed_to_i16(measured_current.q);
    let applied_voltage = state.controller.applied_voltage();
    state.current_limiter.note_applied_voltage(applied_voltage);
    let applied_d_ticks = fixed_to_i16(applied_voltage.d);
    let applied_q_ticks = fixed_to_i16(applied_voltage.q);
    let pwm_span_ticks = pwm_span(duty);
    LIVE_TARGET_AND_LIMIT.store(
        pack_target_and_limit(target_q as i16, phase_limit),
        Ordering::Relaxed,
    );
    LIVE_MEASURED_DQ.store(
        pack_i16_pair(measured_d_counts, measured_q_counts),
        Ordering::Relaxed,
    );
    LIVE_APPLIED_DQ.store(
        pack_i16_pair(applied_d_ticks, applied_q_ticks),
        Ordering::Relaxed,
    );
    LIVE_PWM_SPAN.store(u32::from(pwm_span_ticks), Ordering::Relaxed);
    publish_flag(CONTROL_VOLTAGE_LIMITED, state.controller.voltage_limited());
    if update_max(
        &MAXIMUM_DIRECT_CURRENT_ABS,
        u32::from(measured_d_counts.unsigned_abs()),
    ) {
        capture_direct_current_peak(DirectCurrentPeakEvent {
            measured_d_counts,
            measured_q_counts,
            target_q_counts: target_q as i16,
            hall_raw: state.phase.hall().raw_state(),
            hall_angle_direction: state.phase.hall().angle_direction(),
            edge_age_us: saturating_u32_to_u16(edge_age_us),
            hall_interval_us: saturating_u32_to_u16(state.phase.hall().sector_interval_us()),
            measurement_angle_q16: (angle >> 16) as u16,
            unlimited_angle_q16: (state.phase.hall().unlimited_angle() >> 16) as u16,
            phase_a_counts: current.phase_a,
            phase_b_counts: current.phase_b,
            applied_d_ticks,
            applied_q_ticks,
            voltage_limited: state.controller.voltage_limited(),
            angle_rate_limited: angle != state.phase.hall().unlimited_angle(),
            ..DirectCurrentPeakEvent::default()
        });
    }
    let _ = update_max(
        &MAXIMUM_QUADRATURE_ERROR_ABS,
        target_q
            .saturating_sub(i32::from(measured_q_counts))
            .unsigned_abs(),
    );
    let _ = update_max(&MAXIMUM_PWM_SPAN, u32::from(pwm_span_ticks));
    state.phase.update(&PhaseInput {
        applied_voltage,
        measured_current,
        electrical_angle: angle,
        control_period_ns: CONTROL_PERIOD_NS,
    });
    if !hardware::write_pwm_duties(duty) || !hardware::enable_motor_outputs() {
        stop_control(state, true);
        return;
    }
    publish_flag(CONTROL_OUTPUT_ACTIVE, true);
}

const fn ride_foc_controller() -> FocController {
    let pi = PIController::new(
        crate::config::CURRENT_PI_PROPORTIONAL_GAIN,
        crate::config::CURRENT_PI_INTEGRAL_GAIN_PER_CYCLE,
    );
    FocController::new(
        pi,
        pi,
        crate::config::FOC_VECTOR_LIMIT_TICKS,
        crate::config::FOC_PHASE_LIMIT_TICKS,
    )
}

const fn ride_current_limiter() -> CurrentLimiter {
    CurrentLimiter::new(
        CurrentLimits::new(
            Fixed::from_integer(crate::config::RIDE_PHASE_CURRENT_LIMIT_COUNTS as i32),
            Fixed::from_integer(crate::config::PHASE_CURRENT_TRIP_COUNTS as i32),
            Some(Fixed::from_integer(
                crate::config::RIDE_DC_BUS_CURRENT_LIMIT_COUNTS as i32,
            )),
            Some(Fixed::ZERO),
        ),
        crate::config::PWM_ARR,
        BUS_MODULATION_FILTER_SHIFT,
    )
}

fn update_max(target: &AtomicU32, value: u32) -> bool {
    let mut current = target.load(Ordering::Relaxed);
    while value > current {
        match target.compare_exchange_weak(current, value, Ordering::Relaxed, Ordering::Relaxed) {
            Ok(_) => return true,
            Err(updated) => current = updated,
        }
    }
    false
}

fn capture_direct_current_peak(mut event: DirectCurrentPeakEvent) {
    // SAFETY: called only by TIM1_UP. Foreground copies this cell with global
    // interrupts masked in snapshot().
    unsafe {
        let previous = *PEAK_DIRECT_EVENT.0.get();
        event.generation = previous.generation.wrapping_add(1).max(1);
        *PEAK_DIRECT_EVENT.0.get() = event;
    }
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

fn pack_i16_pair(low: i16, high: i16) -> u32 {
    let low = low.to_le_bytes();
    let high = high.to_le_bytes();
    u32::from_le_bytes([low[0], low[1], high[0], high[1]])
}

fn unpack_i16_pair(packed: u32) -> (i16, i16) {
    let bytes = packed.to_le_bytes();
    (
        i16::from_le_bytes([bytes[0], bytes[1]]),
        i16::from_le_bytes([bytes[2], bytes[3]]),
    )
}

fn pack_target_and_limit(target: i16, limit: u16) -> u32 {
    let target = target.to_le_bytes();
    let limit = limit.to_le_bytes();
    u32::from_le_bytes([target[0], target[1], limit[0], limit[1]])
}

fn unpack_target_and_limit(packed: u32) -> (i16, u16) {
    let bytes = packed.to_le_bytes();
    (
        i16::from_le_bytes([bytes[0], bytes[1]]),
        u16::from_le_bytes([bytes[2], bytes[3]]),
    )
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
    if state.phase.hall_mut().seed(raw).is_err() {
        return false;
    }
    state.hall_recovery_cycles = 0;
    VALIDATED_HALL_INTERVAL_US.store(0, Ordering::Relaxed);
    ELECTRICAL_RPM.store(0, Ordering::Relaxed);
    true
}
