//! Synchronous OxiFOC current and direct-voltage paths for calibration.

use core::cell::UnsafeCell;
use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU32, Ordering};

use oxifoc_core::foc::{Dq, Fixed, FixedFocController, NoDecoupling, PIController};
use oxifoc_f103::{config as board_config, hardware::peripherals as hardware, safety};
use stm32f1::stm32f103::interrupt;

use crate::calibration::{
    flux_linkage::{FluxLinkageCalibration, Observation as FluxObservation},
    hall_calibration::HallCalibration,
    inductance::{InductanceCalibration, Observation as InductanceObservation},
    resistance::{Point, ResistanceCalibration, Sample},
    types::{Actuation, Failure, Routine},
};
use crate::config as calibration_config;

const CONTROL_BUDGET_CYCLES: u32 = board_config::SYSCLK_HZ / board_config::PWM_HZ;

type CalibrationFocController = FixedFocController<
    { board_config::FOC_DEAD_TIME_COMP_NUMERATOR },
    { board_config::FOC_DEAD_TIME_COMP_DENOMINATOR },
    NoDecoupling,
>;

static REQUESTED_ROUTINE: AtomicU8 = AtomicU8::new(Routine::None as u8);
static ABORT_REASON: AtomicU8 = AtomicU8::new(Failure::None as u8);
static BUS_VOLTAGE_MV: AtomicU32 = AtomicU32::new(0);
static CONTROL_CYCLES: AtomicU32 = AtomicU32::new(0);

struct IsrCell<T>(UnsafeCell<T>);

// SAFETY: TIM1_UP is the only writer after initialization; foreground copies
// the diagnostic subset with interrupts masked.
unsafe impl<T> Sync for IsrCell<T> {}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Snapshot {
    pub routine: Routine,
    pub state: u8,
    pub failure: Failure,
    pub active: bool,
    pub output_active: bool,
    pub voltage_limited: bool,
    pub target_d_counts: i16,
    pub measured_d_counts: i16,
    pub applied_d_ticks: i16,
    pub maximum_phase_current_abs: u16,
    pub low: Point,
    pub high: Point,
    pub effective_uv_per_count: u32,
    pub nominal_resistance_uohm: u32,
    pub sample_progress: u16,
    pub inductance_d_nwb_per_count: u32,
    pub inductance_q_nwb_per_count: u32,
    pub residual_dead_time_uv: u32,
    pub pulse_step_d_ticks: i16,
    pub pulse_step_q_ticks: i16,
    pub last_pulse_di_counts: i16,
    pub proportional_d_q16: i32,
    pub proportional_q_q16: i32,
    pub integral_per_cycle_q16: i32,
    pub gain_bus_voltage_mv: u16,
    pub tuning_bandwidth_rad_s: u16,
    pub flux_linkage_nwb: u32,
    pub average_bemf_d_uv: i32,
    pub average_bemf_q_uv: i32,
    pub flux_measurement_erpm: i16,
    pub hall_measurement_erpm: i16,
    pub sync_minimum_percent: u8,
    pub hall_centers_q16: [u16; 8],
    pub hall_valid_mask: u8,
    pub hall_minimum_samples: u8,
    pub injected_samples: u32,
    pub control_cycles: u32,
    pub maximum_control_cycles: u32,
    pub timing_overruns: u16,
    pub fault_flags: u32,
}

struct ControlState {
    controller: CalibrationFocController,
    routine: Routine,
    resistance: ResistanceCalibration,
    inductance: InductanceCalibration,
    flux_linkage: FluxLinkageCalibration,
    hall: HallCalibration,
    offsets: hardware::CurrentOffsets,
    output_active: bool,
    voltage_limited: bool,
    target_d_counts: i16,
    measured_d_counts: i16,
    applied_d_ticks: i16,
    maximum_phase_current_abs: u16,
    injected_samples: u32,
    maximum_control_cycles: u32,
    timing_overruns: u16,
}

static CONTROL: IsrCell<MaybeUninit<ControlState>> =
    IsrCell(UnsafeCell::new(MaybeUninit::uninit()));
static CONTROL_INITIALIZED: AtomicBool = AtomicBool::new(false);

pub fn start(offsets: hardware::CurrentOffsets) {
    // SAFETY: TIM1_UP remains masked until start_tim1_control_loop returns.
    unsafe {
        (*CONTROL.0.get()).write(ControlState {
            controller: calibration_controller(),
            routine: Routine::None,
            resistance: ResistanceCalibration::new(),
            inductance: InductanceCalibration::new(),
            flux_linkage: FluxLinkageCalibration::new(),
            hall: HallCalibration::new(),
            offsets,
            output_active: false,
            voltage_limited: false,
            target_d_counts: 0,
            measured_d_counts: 0,
            applied_d_ticks: 0,
            maximum_phase_current_abs: 0,
            injected_samples: 0,
            maximum_control_cycles: 0,
            timing_overruns: 0,
        });
    }
    CONTROL_INITIALIZED.store(true, Ordering::Release);
    hardware::start_tim1_control_loop();
}

pub fn set_bus_voltage_mv(bus_voltage_mv: u32) {
    BUS_VOLTAGE_MV.store(bus_voltage_mv, Ordering::Release);
}

pub fn request_resistance() -> bool {
    let snapshot = snapshot();
    if snapshot.active || snapshot.fault_flags != 0 {
        return false;
    }
    REQUESTED_ROUTINE.store(Routine::Resistance as u8, Ordering::Release);
    true
}

pub fn request_inductance() -> bool {
    let snapshot = snapshot();
    if snapshot.active || snapshot.fault_flags != 0 || snapshot.effective_uv_per_count == 0 {
        return false;
    }
    REQUESTED_ROUTINE.store(Routine::Inductance as u8, Ordering::Release);
    true
}

pub fn request_flux_linkage() -> bool {
    let snapshot = snapshot();
    if snapshot.active
        || snapshot.fault_flags != 0
        || snapshot.effective_uv_per_count == 0
        || snapshot.inductance_d_nwb_per_count == 0
        || snapshot.inductance_q_nwb_per_count == 0
    {
        return false;
    }
    REQUESTED_ROUTINE.store(Routine::FluxLinkage as u8, Ordering::Release);
    true
}

pub fn request_hall() -> bool {
    let snapshot = snapshot();
    if snapshot.active || snapshot.fault_flags != 0 {
        return false;
    }
    REQUESTED_ROUTINE.store(Routine::Hall as u8, Ordering::Release);
    true
}

pub fn abort(reason: Failure) {
    let reason = if reason == Failure::None {
        Failure::Stopped
    } else {
        reason
    };
    ABORT_REASON.store(reason as u8, Ordering::Release);
}

pub fn snapshot() -> Snapshot {
    cortex_m::interrupt::free(|_| {
        if !CONTROL_INITIALIZED.load(Ordering::Acquire) {
            return Snapshot::default();
        }
        // SAFETY: initialization is published before TIM1_UP is unmasked and
        // the interrupt cannot preempt this complete diagnostic copy.
        // SAFETY: the outer cell remains initialized for the application life.
        let state_ptr = unsafe { (*CONTROL.0.get()).as_ptr() };
        // SAFETY: foreground holds the critical section for this shared read.
        let state = unsafe { &*state_ptr };
        let resistance = state.resistance.result();
        let inductance = state.inductance.result();
        let flux_linkage = state.flux_linkage.result();
        let hall = state.hall.result();
        Snapshot {
            routine: state.routine,
            state: sequence_state(state),
            failure: sequence_failure(state),
            active: sequence_active(state),
            output_active: state.output_active,
            voltage_limited: state.voltage_limited,
            target_d_counts: state.target_d_counts,
            measured_d_counts: state.measured_d_counts,
            applied_d_ticks: state.applied_d_ticks,
            maximum_phase_current_abs: state.maximum_phase_current_abs,
            low: resistance.low,
            high: resistance.high,
            effective_uv_per_count: resistance.effective_uv_per_count,
            nominal_resistance_uohm: resistance.nominal_resistance_uohm,
            sample_progress: sequence_progress(state),
            inductance_d_nwb_per_count: inductance.inductance_d_nwb_per_count,
            inductance_q_nwb_per_count: inductance.inductance_q_nwb_per_count,
            residual_dead_time_uv: inductance.residual_dead_time_uv,
            pulse_step_d_ticks: inductance.pulse_step_d_ticks,
            pulse_step_q_ticks: inductance.pulse_step_q_ticks,
            last_pulse_di_counts: inductance.last_pulse_di_counts,
            proportional_d_q16: inductance.proportional_d_q16,
            proportional_q_q16: inductance.proportional_q_q16,
            integral_per_cycle_q16: inductance.integral_per_cycle_q16,
            gain_bus_voltage_mv: inductance.gain_bus_voltage_mv,
            tuning_bandwidth_rad_s: inductance.bandwidth_rad_s,
            flux_linkage_nwb: flux_linkage.flux_linkage_nwb,
            average_bemf_d_uv: flux_linkage.average_bemf_d_uv,
            average_bemf_q_uv: flux_linkage.average_bemf_q_uv,
            flux_measurement_erpm: flux_linkage.measurement_erpm,
            hall_measurement_erpm: flux_linkage.hall_measurement_erpm,
            sync_minimum_percent: flux_linkage.sync_minimum_percent,
            hall_centers_q16: hall.centers_q16,
            hall_valid_mask: hall.valid_mask,
            hall_minimum_samples: hall.minimum_samples,
            injected_samples: state.injected_samples,
            control_cycles: CONTROL_CYCLES.load(Ordering::Relaxed),
            maximum_control_cycles: state.maximum_control_cycles,
            timing_overruns: state.timing_overruns,
            fault_flags: hardware::fault_flags(),
        }
    })
}

#[interrupt]
fn TIM1_UP() {
    let started = hardware::cycle_count();
    hardware::clear_tim1_update_flag();
    safety::timer_update_entered();
    if !board_config::sample_injected_on_timer_update(hardware::tim1_counting_down()) {
        return;
    }
    control_cycle();
    let elapsed = hardware::cycle_count().wrapping_sub(started);
    if CONTROL_INITIALIZED.load(Ordering::Acquire) {
        // SAFETY: CONTROL is initialized before this interrupt is unmasked and
        // this handler remains its only writer.
        // SAFETY: the outer cell remains initialized for the application life.
        let state_ptr = unsafe { (*CONTROL.0.get()).as_mut_ptr() };
        // SAFETY: TIM1_UP is the sole mutable accessor after initialization.
        let state = unsafe { &mut *state_ptr };
        state.maximum_control_cycles = state.maximum_control_cycles.max(elapsed);
        if elapsed > CONTROL_BUDGET_CYCLES {
            state.timing_overruns = state.timing_overruns.saturating_add(1);
            hardware::latch_control_timing_fault();
            fail_and_stop(state, Failure::ControlTiming);
        }
        safety::record_control_timing(elapsed, state.maximum_control_cycles);
    }
}

fn control_cycle() {
    let control_cycle = CONTROL_CYCLES
        .fetch_add(1, Ordering::Relaxed)
        .wrapping_add(1);
    safety::record_control_cycle(control_cycle);
    if !CONTROL_INITIALIZED.load(Ordering::Acquire) {
        hardware::emergency_shutdown();
        return;
    }
    // SAFETY: start() publishes the complete state before unmasking TIM1_UP.
    // SAFETY: the outer cell remains initialized for the application life.
    let state_ptr = unsafe { (*CONTROL.0.get()).as_mut_ptr() };
    // SAFETY: TIM1_UP is the sole mutable accessor after initialization.
    let state = unsafe { &mut *state_ptr };

    let Ok(current) = hardware::read_phase_currents(state.offsets) else {
        fail_and_stop(state, Failure::CurrentSample);
        return;
    };
    state.injected_samples = state.injected_samples.wrapping_add(1);
    safety::record_control_checkpoint(safety::checkpoint::CURRENT_SAMPLED);

    // Consume every request exactly once even if an abort or hardware fault
    // wins this cycle. Clearing a later fault must never revive an old start.
    let requested_routine = take_requested_routine();
    if let Some(reason) = take_abort_reason() {
        fail_and_stop(state, reason);
        return;
    }
    if hardware::fault_flags() != 0 {
        fail_and_stop(state, Failure::HardwareFault);
        return;
    }
    if requested_routine != Routine::None {
        state.controller.reset();
        state.maximum_phase_current_abs = 0;
        match requested_routine {
            Routine::Resistance => {
                state.routine = Routine::Resistance;
                state.resistance.start();
            }
            Routine::Inductance => {
                state.routine = Routine::Inductance;
                state
                    .inductance
                    .start(state.resistance.result().effective_uv_per_count);
            }
            Routine::FluxLinkage => {
                let inductance = state.inductance.result();
                state.routine = Routine::FluxLinkage;
                state.flux_linkage.start(
                    state.resistance.result().effective_uv_per_count,
                    inductance.inductance_d_nwb_per_count,
                    inductance.inductance_q_nwb_per_count,
                );
            }
            Routine::Hall => {
                state.routine = Routine::Hall;
                state.hall.start();
            }
            Routine::None => {}
        }
    }
    if !sequence_active(state) {
        stop_output(state);
        return;
    }

    let maximum_phase = current
        .phase_a
        .unsigned_abs()
        .max(current.phase_b.unsigned_abs())
        .max(current.phase_c.unsigned_abs());
    state.maximum_phase_current_abs = state.maximum_phase_current_abs.max(maximum_phase);
    if maximum_phase > calibration_config::CALIBRATION_PHASE_CURRENT_TRIP_COUNTS {
        fail_and_stop(state, Failure::PhaseOvercurrent);
        return;
    }

    let bus_voltage_mv = BUS_VOLTAGE_MV.load(Ordering::Acquire);
    if !(calibration_config::CALIBRATION_BUS_MINIMUM_MV
        ..=calibration_config::CALIBRATION_BUS_MAXIMUM_MV)
        .contains(&bus_voltage_mv)
    {
        fail_and_stop(state, Failure::BusVoltage);
        return;
    }

    let phase_a = Fixed::from_integer(i32::from(current.phase_a));
    let phase_b = Fixed::from_integer(i32::from(current.phase_b));
    let actuation = sequence_actuation(state);
    state
        .controller
        .set_actuation_advance(actuation_advance_from_erpm(sequence_electrical_rpm(state)));
    let (measured, duties) = match actuation {
        Actuation::Off => {
            stop_output(state);
            observe_sequence(state, 0, 0, 0, 0, bus_voltage_mv);
            return;
        }
        Actuation::Current {
            angle,
            direct_counts,
            quadrature_counts,
        } => {
            state.target_d_counts = direct_counts;
            state.controller.step(
                phase_a,
                phase_b,
                angle,
                Dq::new(
                    Fixed::from_integer(i32::from(direct_counts)),
                    Fixed::from_integer(i32::from(quadrature_counts)),
                ),
                board_config::PWM_NEUTRAL,
            )
        }
        Actuation::DirectVoltage {
            angle,
            direct_tick_bits,
        } => {
            state.target_d_counts = 0;
            state.controller.step_direct_voltage(
                phase_a,
                phase_b,
                angle,
                Dq::new(Fixed::from_bits(direct_tick_bits), Fixed::ZERO),
                board_config::PWM_NEUTRAL,
            )
        }
    };
    safety::record_control_checkpoint(safety::checkpoint::DRIVER_COMPLETE);
    let applied = state.controller.applied_voltage();
    state.measured_d_counts = saturating_i32_to_i16(measured.d.integer());
    state.applied_d_ticks = saturating_i32_to_i16(applied.d.integer());
    state.voltage_limited = state.controller.voltage_limited();
    observe_sequence(
        state,
        state.measured_d_counts,
        saturating_i32_to_i16(measured.q.integer()),
        applied.d.to_bits(),
        applied.q.to_bits(),
        bus_voltage_mv,
    );

    if !sequence_active(state) {
        stop_output(state);
        return;
    }
    if !hardware::write_pwm_duties(duties) || !hardware::enable_motor_outputs() {
        fail_and_stop(state, Failure::PwmOutput);
        return;
    }
    state.output_active = true;
    safety::record_control_checkpoint(safety::checkpoint::PWM_WRITTEN);
}

fn fail_and_stop(state: &mut ControlState, failure: Failure) {
    match state.routine {
        Routine::Inductance => state.inductance.fail(failure),
        Routine::FluxLinkage => state.flux_linkage.fail(failure),
        Routine::Hall => state.hall.fail(failure),
        Routine::None | Routine::Resistance => {
            state.resistance.fail(failure);
        }
    }
    safety::record_safety_loss(failure as u8);
    stop_output(state);
}

fn stop_output(state: &mut ControlState) {
    if state.output_active {
        hardware::disable_motor_outputs();
        hardware::write_pwm_neutral();
        state.controller.reset();
    }
    state.output_active = false;
    state.voltage_limited = false;
    state.target_d_counts = 0;
}

fn take_abort_reason() -> Option<Failure> {
    match ABORT_REASON.swap(Failure::None as u8, Ordering::AcqRel) {
        0 => None,
        1 => Some(Failure::Stopped),
        2 => Some(Failure::LocalInterlock),
        3 => Some(Failure::CurrentSample),
        4 => Some(Failure::PhaseOvercurrent),
        5 => Some(Failure::HardwareFault),
        6 => Some(Failure::PwmOutput),
        7 => Some(Failure::ControlTiming),
        8 => Some(Failure::CurrentDidNotSettle),
        9 => Some(Failure::InvalidSlope),
        10 => Some(Failure::BusVoltage),
        11 => Some(Failure::MissingPrerequisite),
        12 => Some(Failure::PulseResponse),
        13 => Some(Failure::InductanceRange),
        14 => Some(Failure::MotorNotResponding),
        15 => Some(Failure::FluxRange),
        _ => Some(Failure::HallStates),
    }
}

fn take_requested_routine() -> Routine {
    match REQUESTED_ROUTINE.swap(Routine::None as u8, Ordering::AcqRel) {
        1 => Routine::Resistance,
        2 => Routine::Inductance,
        3 => Routine::FluxLinkage,
        4 => Routine::Hall,
        _ => Routine::None,
    }
}

fn sequence_active(state: &ControlState) -> bool {
    match state.routine {
        Routine::Resistance => state.resistance.active(),
        Routine::Inductance => state.inductance.active(),
        Routine::FluxLinkage => state.flux_linkage.active(),
        Routine::Hall => state.hall.active(),
        Routine::None => false,
    }
}

fn sequence_state(state: &ControlState) -> u8 {
    match state.routine {
        Routine::Resistance => state.resistance.state() as u8,
        Routine::Inductance => state.inductance.state() as u8,
        Routine::FluxLinkage => state.flux_linkage.state() as u8,
        Routine::Hall => state.hall.state() as u8,
        Routine::None => 0,
    }
}

fn sequence_failure(state: &ControlState) -> Failure {
    match state.routine {
        Routine::Resistance => state.resistance.failure(),
        Routine::Inductance => state.inductance.failure(),
        Routine::FluxLinkage => state.flux_linkage.failure(),
        Routine::Hall => state.hall.failure(),
        Routine::None => Failure::None,
    }
}

fn sequence_progress(state: &ControlState) -> u16 {
    match state.routine {
        Routine::Resistance => state.resistance.sample_progress(),
        Routine::Inductance => state.inductance.pulse_progress(),
        Routine::FluxLinkage => state.flux_linkage.progress(),
        Routine::Hall => state.hall.progress(),
        Routine::None => 0,
    }
}

fn sequence_actuation(state: &ControlState) -> Actuation {
    match state.routine {
        Routine::Resistance => Actuation::Current {
            angle: 0,
            direct_counts: state.resistance.target_counts(),
            quadrature_counts: 0,
        },
        Routine::Inductance => state.inductance.actuation(),
        Routine::FluxLinkage => state.flux_linkage.actuation(),
        Routine::Hall => state.hall.actuation(),
        Routine::None => Actuation::Off,
    }
}

fn observe_sequence(
    state: &mut ControlState,
    measured_d_counts: i16,
    measured_q_counts: i16,
    applied_d_tick_bits: i32,
    applied_q_tick_bits: i32,
    bus_voltage_mv: u32,
) {
    match state.routine {
        Routine::Resistance => state.resistance.tick(Sample {
            measured_d_counts,
            applied_d_tick_bits,
            bus_voltage_mv,
        }),
        Routine::Inductance => state.inductance.observe(InductanceObservation {
            measured_d_counts,
            applied_d_tick_bits,
            bus_voltage_mv,
        }),
        Routine::FluxLinkage => {
            let (hall_sequence, _, _) = hardware::hall_edge_snapshot();
            state.flux_linkage.observe(FluxObservation {
                measured_d_counts,
                measured_q_counts,
                applied_d_tick_bits,
                applied_q_tick_bits,
                bus_voltage_mv,
                hall_sequence,
            });
        }
        Routine::Hall => state.hall.observe(hardware::live_hall_state()),
        Routine::None => {}
    }
}

fn sequence_electrical_rpm(state: &ControlState) -> i32 {
    match state.routine {
        Routine::FluxLinkage => state.flux_linkage.electrical_rpm(),
        Routine::None | Routine::Resistance | Routine::Inductance | Routine::Hall => 0,
    }
}

fn actuation_advance_from_erpm(electrical_rpm: i32) -> Fixed {
    Fixed::from_bits((electrical_rpm * 1_757) >> 12)
}

const fn calibration_controller() -> CalibrationFocController {
    let pi = PIController::new(
        board_config::CURRENT_PI_PROPORTIONAL_GAIN,
        board_config::CURRENT_PI_INTEGRAL_GAIN_PER_CYCLE,
    );
    CalibrationFocController::new(
        pi,
        pi,
        board_config::FOC_VECTOR_LIMIT_TICKS,
        board_config::FOC_PHASE_LIMIT_TICKS,
    )
}

fn saturating_i32_to_i16(value: i32) -> i16 {
    value.clamp(i32::from(i16::MIN), i32::from(i16::MAX)) as i16
}
