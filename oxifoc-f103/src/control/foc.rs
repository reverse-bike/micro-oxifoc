//! 16 kHz Hall-only fixed-point current-control interrupt.

use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicBool, AtomicI32, AtomicU32, Ordering};

use crate::hardware::peripherals::{self as hardware, CurrentOffsets};
use oxifoc_core::foc::{
    Dq, Fixed, FocKernel, Pi,
    offset_tracker::CurrentOffsetTracker,
    phase::{ControlPhaseInput, ControlPhaseProvider, HallTracker},
    ramp::QuadratureTargetRamp,
};
use stm32f1::stm32f103::interrupt;

const CONTROL_HALL_VALID: u32 = 1;
const CONTROL_CURRENT_VALID: u32 = 1 << 1;
const CONTROL_OUTPUT_ACTIVE: u32 = 1 << 2;
const HALL_RECOVERY_STABLE_CYCLES: u8 = 5;
const CONTROL_BUDGET_WARNING_CYCLES: u32 = 3_900;
const CONTROL_BUDGET_CYCLES: u32 = crate::config::SYSCLK_HZ / crate::config::PWM_HZ;
const CONTROL_PERIOD_NS: u32 = 1_000_000_000 / crate::config::PWM_HZ;

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
static VALIDATED_HALL_SEQUENCE: AtomicU32 = AtomicU32::new(0);
static VALIDATED_HALL_INTERVAL_US: AtomicU32 = AtomicU32::new(0);
static VALIDATED_HALL_PROGRESS: AtomicI32 = AtomicI32::new(0);
static ELECTRICAL_RPM: AtomicI32 = AtomicI32::new(0);

struct IsrCell<T>(UnsafeCell<T>);

// SAFETY: the contained state is accessed only by TIM1_UP after initialization
// completes with that interrupt masked.
unsafe impl<T> Sync for IsrCell<T> {}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Snapshot {
    pub hall_valid: bool,
    pub current_valid: bool,
    pub output_active: bool,
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
    kernel: FocKernel,
    target_ramp: QuadratureTargetRamp,
    hall: HallTracker,
    offsets: CurrentOffsets,
    offset_tracker: CurrentOffsetTracker,
    hardware_hall_sequence: u32,
    hall_recovery_raw: u8,
    hall_recovery_cycles: u8,
    command_was_enabled: bool,
}

static CONTROL: IsrCell<Option<ControlState>> = IsrCell(UnsafeCell::new(None));

pub fn start(offsets: CurrentOffsets) {
    let mut hall = HallTracker::new(&crate::config::HALL_GEOMETRY);
    let hall_valid = hall.seed(hardware::live_hall_state()).is_ok();
    let initial_flags = if hall_valid { CONTROL_HALL_VALID } else { 0 };
    CONTROL_FLAGS.store(initial_flags, Ordering::Release);
    // SAFETY: TIM1_UP remains masked until start_tim1_control_loop returns.
    unsafe {
        CONTROL.0.get().write(Some(ControlState {
            kernel: ride_foc_kernel(),
            target_ramp: QuadratureTargetRamp::new(
                crate::config::TARGET_RAMP_CYCLES_PER_STEP,
                crate::config::TARGET_RAMP_COUNTS_PER_STEP,
            ),
            hall,
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
    dc_current_limit_counts: u8,
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
    DC_CURRENT_LIMIT_COUNTS.store(u32::from(dc_current_limit_counts), Ordering::Relaxed);
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
    Snapshot {
        hall_valid: flags & CONTROL_HALL_VALID != 0,
        current_valid: flags & CONTROL_CURRENT_VALID != 0,
        output_active: flags & CONTROL_OUTPUT_ACTIVE != 0,
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
    publish_flag(CONTROL_OUTPUT_ACTIVE, false);
    state.kernel.reset();
    state.target_ramp.reset();
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
    update_max(&CONTROL_MAX_CYCLES, elapsed);
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
        if state.hall.update_edge(raw, interval_us).is_err() {
            publish_flag(CONTROL_HALL_VALID, false);
            stop_control(state, true);
            return;
        }
        state.hall_recovery_cycles = 0;
        VALIDATED_HALL_SEQUENCE.store(sequence, Ordering::Release);
        VALIDATED_HALL_INTERVAL_US.store(state.hall.sector_interval_us(), Ordering::Relaxed);
        VALIDATED_HALL_PROGRESS.fetch_add(
            i32::from(state.hall.physical_direction()),
            Ordering::Relaxed,
        );
        ELECTRICAL_RPM.store(state.hall.electrical_rpm(), Ordering::Relaxed);
    }
    if command_enabled && !state.command_was_enabled && state.hall.is_stationary() {
        let live_before = hardware::live_hall_state();
        if live_before != state.hall.raw() {
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
        state.hall.discard_next_interval();
    }
    state.command_was_enabled = command_enabled;
    let edge_age_us = hardware::hall_edge_age_us().saturating_add(3);
    let angle = match state.hall.estimate(edge_age_us) {
        Some(estimate) => {
            state.hall_recovery_cycles = 0;
            estimate.angle
        }
        None if !lease_active
            && hardware::motor_outputs_disabled()
            && recover_stationary_hall(state) =>
        {
            state
                .hall
                .estimate(0)
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

    let phase_limit = state.kernel.phase_current_limit_from_dc(
        DC_CURRENT_LIMIT_COUNTS
            .load(Ordering::Relaxed)
            .min(u32::from(u8::MAX)) as u8,
        crate::config::RIDE_PHASE_CURRENT_LIMIT_COUNTS,
        crate::config::PWM_ARR,
    );
    let requested = TARGET_Q_COUNTS
        .load(Ordering::Relaxed)
        .max(-i32::from(phase_limit));
    let target_q = state.target_ramp.next(requested);
    let (measured_current, duty) = state.kernel.step_with_injection(
        Fixed::from_integer(i32::from(current.phase_a)),
        Fixed::from_integer(i32::from(current.phase_b)),
        angle,
        Dq {
            d: Fixed::ZERO,
            q: Fixed::from_integer(target_q),
        },
        state.hall.injection(),
        crate::config::PWM_NEUTRAL,
    );
    state.hall.update(&ControlPhaseInput {
        applied_voltage: state.kernel.applied_voltage(),
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

const fn ride_foc_kernel() -> FocKernel {
    let pi = Pi::new(
        crate::config::CURRENT_PI_PROPORTIONAL_GAIN,
        crate::config::CURRENT_PI_INTEGRAL_GAIN_PER_CYCLE,
    );
    FocKernel::new(
        pi,
        pi,
        crate::config::FOC_VECTOR_LIMIT_TICKS,
        crate::config::FOC_PHASE_LIMIT_TICKS,
    )
}

fn update_max(target: &AtomicU32, value: u32) {
    let mut current = target.load(Ordering::Relaxed);
    while value > current {
        match target.compare_exchange_weak(current, value, Ordering::Relaxed, Ordering::Relaxed) {
            Ok(_) => break,
            Err(updated) => current = updated,
        }
    }
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
    if state.hall.seed(raw).is_err() {
        return false;
    }
    state.hall_recovery_cycles = 0;
    VALIDATED_HALL_INTERVAL_US.store(0, Ordering::Relaxed);
    ELECTRICAL_RPM.store(0, Ordering::Relaxed);
    true
}
