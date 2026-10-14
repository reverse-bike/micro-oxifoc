//! Local ride-demand authority and Hall-progress lifecycle.

use crate::config::{BIKE_CONFIG, RideConfig};
use crate::safety::fault_flag;
use crate::sensors::throttle::{Demand, Observation as ThrottleObservation};

const RIDE: RideConfig = BIKE_CONFIG.ride;
/// Latched faults that indicate a power-stage problem rather than a sensing
/// or scheduling interruption; only these count toward the retry lockout.
const RETRY_LIMITED_FAULTS: u32 = fault_flag::HARDWARE_BREAK | fault_flag::PWM_RANGE;

pub const OUTPUT_LEASE_CYCLES: u32 =
    RIDE.command_lease_milliseconds * crate::config::PWM_HZ / 1_000;

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[repr(u8)]
pub enum Stage {
    #[default]
    Disarmed = 0,
    Ready = 1,
    AwaitingFirstEdge = 2,
    StartupTracking = 3,
    Tracking = 4,
    Coasting = 5,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Observation {
    pub now_ms: u32,
    pub throttle: ThrottleObservation,
    pub cadence_demand_percent: u8,
    pub brake_active: bool,
    pub environment_dc_limit_counts: Option<u16>,
    pub phase_current_limit_counts: u16,
    pub hall_valid: bool,
    pub current_valid: bool,
    pub fault_flags: u32,
    pub safety_events: u32,
    pub phase_current_trips: u32,
    pub hall_sequence: u32,
    pub hall_progress: i32,
    pub hall_interval_us: u32,
    pub electrical_rpm: i32,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Command {
    pub energize: bool,
    pub acknowledge_faults: bool,
    pub target_q_counts: i16,
    pub dc_current_limit_counts: u16,
    /// Thermally derated radius of the complete dq current command.
    pub phase_current_limit_counts: u16,
    pub stage: Stage,
}

impl Command {
    pub const OFF: Self = Self {
        energize: false,
        acknowledge_faults: false,
        target_q_counts: 0,
        dc_current_limit_counts: 0,
        phase_current_limit_counts: 0,
        stage: Stage::Disarmed,
    };
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct Run {
    stage: Stage,
    startup_deadline_ms: u32,
    hall_deadline_ms: u32,
    last_hall_sequence: u32,
    starting_hall_progress: i32,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum State {
    Disarmed,
    Ready,
    Active(Run),
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RideController {
    state: State,
    observed_safety_events: u32,
    observed_phase_current_trips: u32,
    retry_limited_fault_latched: bool,
    /// A power-stage fault has been counted since the controller last became
    /// Ready, so one event seen as both a software phase trip and a hardware
    /// break in adjacent passes counts once.
    fault_counted: bool,
    consecutive_faults: u8,
}

impl RideController {
    pub const fn new(initial_safety_events: u32) -> Self {
        Self {
            state: State::Disarmed,
            observed_safety_events: initial_safety_events,
            observed_phase_current_trips: 0,
            retry_limited_fault_latched: false,
            fault_counted: false,
            consecutive_faults: 0,
        }
    }

    /// Repeated power-stage faults have stopped acknowledgement and arming
    /// until the next reset.
    pub const fn fault_lockout(&self) -> bool {
        self.consecutive_faults >= RIDE.consecutive_fault_limit
    }

    pub fn update(&mut self, observation: Observation) -> Command {
        let command = self.update_authority(observation);
        if command.stage == Stage::Tracking {
            self.consecutive_faults = 0;
        }
        command
    }

    fn update_authority(&mut self, observation: Observation) -> Command {
        let safety_event = observation.safety_events != self.observed_safety_events;
        self.observed_safety_events = observation.safety_events;
        // At most one power-stage fault counts per arming attempt; a fault
        // stays latched until acknowledged.
        let phase_overcurrent =
            observation.phase_current_trips != self.observed_phase_current_trips;
        self.observed_phase_current_trips = observation.phase_current_trips;
        let fault_latched = observation.fault_flags & RETRY_LIMITED_FAULTS != 0;
        let new_fault_latch = fault_latched && !self.retry_limited_fault_latched;
        self.retry_limited_fault_latched = fault_latched;
        if (phase_overcurrent || new_fault_latch) && !self.fault_counted {
            self.fault_counted = true;
            self.consecutive_faults = self.consecutive_faults.saturating_add(1);
        }
        if self.fault_lockout() {
            self.state = State::Disarmed;
            return Command::OFF;
        }
        if observation.fault_flags != 0 {
            self.state = State::Disarmed;
            return Command {
                acknowledge_faults: observation.throttle.is_valid()
                    && observation.throttle.is_at_rest(),
                ..Command::OFF
            };
        }
        let environment_limit = observation.environment_dc_limit_counts.unwrap_or(0);
        // A zero DC or phase ceiling leaves no current authority to grant.
        let safety_ready = !safety_event
            && observation.hall_valid
            && observation.current_valid
            && environment_limit != 0
            && observation.phase_current_limit_counts != 0;
        if !observation.throttle.is_valid() || !safety_ready {
            self.state = State::Disarmed;
            return Command::OFF;
        }
        if observation.brake_active && !matches!(self.state, State::Active(_)) {
            self.state = State::Disarmed;
            return Command::OFF;
        }

        match self.state {
            State::Disarmed => {
                if observation.throttle.is_at_rest() && observation.cadence_demand_percent == 0 {
                    self.state = State::Ready;
                    self.fault_counted = false;
                    Command {
                        stage: Stage::Ready,
                        ..Command::OFF
                    }
                } else {
                    Command::OFF
                }
            }
            State::Ready => self.update_ready(observation, environment_limit),
            State::Active(run) => self.update_active(run, observation, environment_limit),
        }
    }

    fn update_ready(&mut self, observation: Observation, environment_limit: u16) -> Command {
        let Some(demand) = ride_demand(observation) else {
            if !observation.throttle.is_at_rest() {
                self.state = State::Disarmed;
                return Command::OFF;
            }
            return Command {
                stage: Stage::Ready,
                ..Command::OFF
            };
        };
        if observation.electrical_rpm.unsigned_abs() > RIDE.maximum_entry_electrical_rpm {
            return Command {
                stage: Stage::Ready,
                ..Command::OFF
            };
        }

        let run = Run {
            stage: Stage::AwaitingFirstEdge,
            startup_deadline_ms: observation
                .now_ms
                .wrapping_add(RIDE.startup_timeout_milliseconds),
            hall_deadline_ms: observation
                .now_ms
                .wrapping_add(RIDE.startup_first_hall_timeout_milliseconds),
            last_hall_sequence: observation.hall_sequence,
            starting_hall_progress: observation.hall_progress,
        };
        self.state = State::Active(run);
        active_command(
            demand.negative_q_target(observation.phase_current_limit_counts),
            environment_limit,
            observation.phase_current_limit_counts,
            run.stage,
        )
    }

    fn update_active(
        &mut self,
        mut run: Run,
        observation: Observation,
        environment_limit: u16,
    ) -> Command {
        let demand = if observation.brake_active {
            None
        } else {
            ride_demand(observation)
        };
        if matches!(run.stage, Stage::AwaitingFirstEdge | Stage::StartupTracking)
            && demand.is_none()
        {
            self.state = State::Disarmed;
            return Command::OFF;
        }

        let startup_progress = observation
            .hall_progress
            .wrapping_sub(run.starting_hall_progress);
        if observation.hall_sequence != run.last_hall_sequence {
            let first_edge = matches!(run.stage, Stage::AwaitingFirstEdge);
            run.last_hall_sequence = observation.hall_sequence;
            run.hall_deadline_ms = observation.now_ms.wrapping_add(if first_edge {
                RIDE.startup_first_hall_timeout_milliseconds
            } else {
                hall_timeout_ms(observation.hall_interval_us)
            });
            if first_edge {
                run.stage = Stage::StartupTracking;
            }
            if matches!(run.stage, Stage::StartupTracking)
                && startup_progress >= RIDE.startup_forward_hall_transitions
            {
                run.stage = Stage::Tracking;
            }
        }

        let timed_out = match run.stage {
            // The seeded Hall sector is sufficient for low-speed commutation.
            // Waiting for the first edge must cover a rollback that reverses
            // within that sector, where no transition occurs around zero speed.
            Stage::AwaitingFirstEdge => {
                deadline_reached(observation.now_ms, run.startup_deadline_ms)
            }
            Stage::StartupTracking => {
                // Reversing a rolling wheel necessarily has a no-edge interval at zero speed.
                // The absolute startup deadline bounds that interval until forward recovery.
                deadline_reached(observation.now_ms, run.startup_deadline_ms)
                    || (startup_progress >= 0
                        && deadline_reached(observation.now_ms, run.hall_deadline_ms))
            }
            Stage::Tracking | Stage::Coasting => {
                deadline_reached(observation.now_ms, run.hall_deadline_ms)
            }
            Stage::Disarmed | Stage::Ready => true,
        };
        if timed_out {
            self.state = State::Disarmed;
            return Command::OFF;
        }

        match (run.stage, demand) {
            (Stage::Tracking, None) => {
                if observation.electrical_rpm.unsigned_abs() <= RIDE.maximum_entry_electrical_rpm {
                    self.state = State::Disarmed;
                    Command::OFF
                } else {
                    run.stage = Stage::Coasting;
                    self.state = State::Active(run);
                    active_command(
                        0,
                        environment_limit,
                        observation.phase_current_limit_counts,
                        Stage::Coasting,
                    )
                }
            }
            (Stage::Coasting, None) => {
                if observation.electrical_rpm.unsigned_abs() <= RIDE.maximum_entry_electrical_rpm {
                    self.state = State::Disarmed;
                    Command::OFF
                } else {
                    self.state = State::Active(run);
                    active_command(
                        0,
                        environment_limit,
                        observation.phase_current_limit_counts,
                        Stage::Coasting,
                    )
                }
            }
            (Stage::Coasting, Some(demand)) => {
                run.stage = Stage::Tracking;
                self.state = State::Active(run);
                active_command(
                    demand.negative_q_target(observation.phase_current_limit_counts),
                    environment_limit,
                    observation.phase_current_limit_counts,
                    Stage::Tracking,
                )
            }
            (_, Some(demand)) => {
                self.state = State::Active(run);
                active_command(
                    demand.negative_q_target(observation.phase_current_limit_counts),
                    environment_limit,
                    observation.phase_current_limit_counts,
                    run.stage,
                )
            }
            _ => {
                self.state = State::Disarmed;
                Command::OFF
            }
        }
    }
}

fn ride_demand(observation: Observation) -> Option<Demand> {
    let throttle = observation.throttle.demand();
    if throttle.is_some_and(|demand| demand.percent() >= observation.cadence_demand_percent) {
        throttle
    } else {
        Demand::from_percent(observation.cadence_demand_percent)
    }
}

fn active_command(
    target_q_counts: i16,
    environment_limit: u16,
    phase_current_limit_counts: u16,
    stage: Stage,
) -> Command {
    Command {
        energize: true,
        acknowledge_faults: false,
        target_q_counts,
        dc_current_limit_counts: environment_limit,
        phase_current_limit_counts,
        stage,
    }
}

pub const fn hall_timeout_ms(interval_us: u32) -> u32 {
    let interval_ms = interval_us.saturating_add(999) / 1_000;
    let timeout = interval_ms.saturating_mul(2);
    if timeout < RIDE.minimum_running_hall_timeout_milliseconds {
        RIDE.minimum_running_hall_timeout_milliseconds
    } else if timeout > RIDE.maximum_running_hall_timeout_milliseconds {
        RIDE.maximum_running_hall_timeout_milliseconds
    } else {
        timeout
    }
}

const fn deadline_reached(now: u32, deadline: u32) -> bool {
    now.wrapping_sub(deadline) < 0x8000_0000
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::RIDE_PHASE_CURRENT_LIMIT_COUNTS;
    use crate::sensors::throttle::{FULL_ADC, REST_ADC, REST_MAX_ADC};

    fn observation(now_ms: u32, throttle_raw: u16) -> Observation {
        Observation {
            now_ms,
            throttle: ThrottleObservation::from_raw(throttle_raw),
            cadence_demand_percent: 0,
            brake_active: false,
            environment_dc_limit_counts: Some(crate::config::RIDE_DC_BUS_TRANSIENT_LIMIT_COUNTS),
            phase_current_limit_counts: RIDE_PHASE_CURRENT_LIMIT_COUNTS,
            hall_valid: true,
            current_valid: true,
            fault_flags: 0,
            safety_events: 0,
            phase_current_trips: 0,
            hall_sequence: 0,
            hall_progress: 0,
            hall_interval_us: 10_000,
            electrical_rpm: 0,
        }
    }

    fn arm(controller: &mut RideController) {
        assert_eq!(
            controller.update(observation(0, REST_ADC)).stage,
            Stage::Ready
        );
    }

    #[test]
    fn boot_high_cannot_start_until_a_valid_rest_sample() {
        let mut controller = RideController::new(0);
        assert_eq!(controller.update(observation(0, FULL_ADC)), Command::OFF);
        assert_eq!(
            controller.update(observation(1, REST_ADC)).stage,
            Stage::Ready
        );
        let command = controller.update(observation(2, FULL_ADC));
        assert!(command.energize);
        assert_eq!(
            command.target_q_counts,
            -(RIDE_PHASE_CURRENT_LIMIT_COUNTS as i16)
        );
    }

    #[test]
    fn qualified_cadence_scales_the_existing_ride_envelope() {
        let mut controller = RideController::new(0);
        arm(&mut controller);
        let mut pedaling = observation(1, REST_ADC);
        pedaling.cadence_demand_percent = 44;
        let command = controller.update(pedaling);
        assert!(command.energize);
        assert_eq!(
            command.target_q_counts,
            Demand::from_percent(44)
                .unwrap()
                .negative_q_target(RIDE_PHASE_CURRENT_LIMIT_COUNTS)
        );
        assert_eq!(command.stage, Stage::AwaitingFirstEdge);
    }

    #[test]
    fn active_cadence_cannot_arm_at_boot_and_throttle_can_take_over() {
        let mut blocked = RideController::new(0);
        let mut already_pedaling = observation(0, REST_ADC);
        already_pedaling.cadence_demand_percent = 44;
        assert_eq!(blocked.update(already_pedaling), Command::OFF);

        let mut controller = RideController::new(0);
        arm(&mut controller);
        let mut pedaling = observation(1, REST_ADC);
        pedaling.cadence_demand_percent = 44;
        assert!(controller.update(pedaling).energize);

        let throttle_takeover = observation(2, FULL_ADC);
        let command = controller.update(throttle_takeover);
        assert!(command.energize);
        assert_eq!(
            command.target_q_counts,
            -(RIDE_PHASE_CURRENT_LIMIT_COUNTS as i16)
        );
    }

    #[test]
    fn ride_uses_the_larger_of_throttle_and_cadence_percent() {
        let mut controller = RideController::new(0);
        arm(&mut controller);
        let half_throttle_raw = REST_MAX_ADC + (FULL_ADC - REST_MAX_ADC) / 2;
        let mut mixed = observation(1, half_throttle_raw);
        mixed.cadence_demand_percent = 44;
        assert_eq!(
            controller.update(mixed).target_q_counts,
            Demand::from_percent(50)
                .unwrap()
                .negative_q_target(RIDE_PHASE_CURRENT_LIMIT_COUNTS)
        );

        mixed.now_ms = 2;
        mixed.cadence_demand_percent = 60;
        assert_eq!(
            controller.update(mixed).target_q_counts,
            Demand::from_percent(60)
                .unwrap()
                .negative_q_target(RIDE_PHASE_CURRENT_LIMIT_COUNTS)
        );
    }

    #[test]
    fn brake_and_safety_events_remove_authority_and_require_rest() {
        for brake in [false, true] {
            let mut controller = RideController::new(0);
            arm(&mut controller);
            assert!(controller.update(observation(1, FULL_ADC)).energize);
            let mut stopped = observation(2, FULL_ADC);
            stopped.brake_active = brake;
            stopped.safety_events = u32::from(!brake);
            assert_eq!(controller.update(stopped), Command::OFF);
            assert_eq!(controller.update(observation(3, FULL_ADC)), Command::OFF);
        }
    }

    #[test]
    fn a_latched_fault_can_only_be_acknowledged_at_valid_throttle_rest() {
        let mut controller = RideController::new(0);
        arm(&mut controller);

        let mut faulted = observation(1, FULL_ADC);
        faulted.fault_flags = 1;
        let command = controller.update(faulted);
        assert_eq!(command, Command::OFF);
        assert!(!command.acknowledge_faults);

        faulted.now_ms = 2;
        assert!(!controller.update(faulted).acknowledge_faults);

        faulted.now_ms = 3;
        faulted.throttle = ThrottleObservation::invalid_acquisition(REST_ADC);
        assert!(!controller.update(faulted).acknowledge_faults);

        faulted.now_ms = 4;
        faulted.throttle = ThrottleObservation::from_raw(REST_ADC);
        let acknowledgement = controller.update(faulted);
        assert!(!acknowledgement.energize);
        assert!(acknowledgement.acknowledge_faults);
        assert_eq!(acknowledgement.stage, Stage::Disarmed);

        let ready = controller.update(observation(5, REST_ADC));
        assert!(!ready.acknowledge_faults);
        assert_eq!(ready.stage, Stage::Ready);
    }

    #[test]
    fn thermal_phase_ceiling_scales_ride_demand() {
        let mut controller = RideController::new(0);
        arm(&mut controller);
        let mut warm = observation(1, FULL_ADC);
        warm.phase_current_limit_counts = RIDE_PHASE_CURRENT_LIMIT_COUNTS / 2;
        let command = controller.update(warm);
        assert!(command.energize);
        assert_eq!(
            command.target_q_counts,
            -((RIDE_PHASE_CURRENT_LIMIT_COUNTS / 2) as i16)
        );
        assert_eq!(
            command.dc_current_limit_counts,
            crate::config::RIDE_DC_BUS_TRANSIENT_LIMIT_COUNTS
        );
        assert_eq!(
            command.phase_current_limit_counts,
            RIDE_PHASE_CURRENT_LIMIT_COUNTS / 2
        );
    }

    #[test]
    fn zero_phase_ceiling_removes_authority() {
        let mut controller = RideController::new(0);
        let mut cold = observation(0, REST_ADC);
        cold.phase_current_limit_counts = 0;
        assert_eq!(controller.update(cold), Command::OFF);

        arm(&mut controller);
        let mut hot = observation(1, FULL_ADC);
        hot.phase_current_limit_counts = 0;
        assert_eq!(controller.update(hot), Command::OFF);
    }

    fn reach_tracking(controller: &mut RideController, now_ms: u32, trips: u32) {
        let mut start = observation(now_ms, FULL_ADC);
        start.phase_current_trips = trips;
        assert!(controller.update(start).energize);
        let mut tracking = start;
        tracking.now_ms = now_ms + 1;
        tracking.hall_sequence = 12;
        tracking.hall_progress = 12;
        assert_eq!(controller.update(tracking).stage, Stage::Tracking);
    }

    #[test]
    fn repeated_power_faults_stop_acknowledgement_until_reset() {
        let mut controller = RideController::new(0);
        arm(&mut controller);
        for attempt in 1..=u32::from(RIDE.consecutive_fault_limit) {
            assert!(!controller.fault_lockout());
            let mut faulted = observation(attempt * 10, REST_ADC);
            faulted.fault_flags = crate::safety::fault_flag::HARDWARE_BREAK;
            for tick in 0..3 {
                faulted.now_ms += tick;
                let command = controller.update(faulted);
                let locked = attempt == u32::from(RIDE.consecutive_fault_limit);
                assert_eq!(command.acknowledge_faults, !locked);
            }
            controller.update(observation(attempt * 10 + 5, REST_ADC));
        }
        assert!(controller.fault_lockout());
        assert_eq!(controller.update(observation(100, REST_ADC)), Command::OFF);
        assert_eq!(controller.update(observation(101, FULL_ADC)), Command::OFF);
    }

    #[test]
    fn phase_overcurrent_trips_count_toward_the_lockout() {
        let mut controller = RideController::new(0);
        arm(&mut controller);
        for trips in 1..=u32::from(RIDE.consecutive_fault_limit) {
            let mut start = observation(trips * 10, FULL_ADC);
            start.safety_events = trips - 1;
            start.phase_current_trips = trips - 1;
            assert!(controller.update(start).energize);
            let mut tripped = start;
            tripped.now_ms += 1;
            tripped.safety_events = trips;
            tripped.phase_current_trips = trips;
            assert_eq!(controller.update(tripped), Command::OFF);
            tripped.now_ms += 1;
            tripped.throttle = ThrottleObservation::from_raw(REST_ADC);
            controller.update(tripped);
            tripped.now_ms += 1;
            let rearmed = controller.update(tripped);
            let locked = trips == u32::from(RIDE.consecutive_fault_limit);
            assert_eq!(rearmed.stage == Stage::Ready, !locked);
        }
        assert!(controller.fault_lockout());
    }

    #[test]
    fn a_phase_trip_and_its_adjacent_hardware_break_count_once_per_arming() {
        let mut controller = RideController::new(0);
        arm(&mut controller);
        for attempt in 1..=u32::from(RIDE.consecutive_fault_limit) {
            let base = attempt * 10;
            let mut start = observation(base, FULL_ADC);
            start.safety_events = attempt - 1;
            start.phase_current_trips = attempt - 1;
            assert!(controller.update(start).energize);
            let mut tripped = start;
            tripped.now_ms = base + 1;
            tripped.safety_events = attempt;
            tripped.phase_current_trips = attempt;
            assert_eq!(controller.update(tripped), Command::OFF);
            let mut latched = tripped;
            latched.now_ms = base + 2;
            latched.fault_flags = crate::safety::fault_flag::HARDWARE_BREAK;
            controller.update(latched);
            let locked = attempt == u32::from(RIDE.consecutive_fault_limit);
            assert_eq!(controller.fault_lockout(), locked);
            latched.now_ms = base + 3;
            latched.throttle = ThrottleObservation::from_raw(REST_ADC);
            assert_eq!(controller.update(latched).acknowledge_faults, !locked);
            let mut rest = tripped;
            rest.now_ms = base + 4;
            rest.throttle = ThrottleObservation::from_raw(REST_ADC);
            assert_eq!(controller.update(rest).stage == Stage::Ready, !locked);
        }
        assert!(controller.fault_lockout());
    }

    #[test]
    fn normal_tracking_clears_the_consecutive_fault_count() {
        let mut controller = RideController::new(0);
        arm(&mut controller);
        for cycle in 0..4_u32 {
            for fault in 0..u32::from(RIDE.consecutive_fault_limit) - 1 {
                let mut faulted = observation(cycle * 100 + fault * 10, REST_ADC);
                faulted.fault_flags = crate::safety::fault_flag::PWM_RANGE;
                assert!(controller.update(faulted).acknowledge_faults);
                controller.update(observation(cycle * 100 + fault * 10 + 1, REST_ADC));
            }
            reach_tracking(&mut controller, cycle * 100 + 50, 0);
            controller.update(observation(cycle * 100 + 60, REST_ADC));
            controller.update(observation(cycle * 100 + 61, REST_ADC));
        }
        assert!(!controller.fault_lockout());
    }

    #[test]
    fn hall_and_timing_faults_do_not_count_toward_the_lockout() {
        let mut controller = RideController::new(0);
        arm(&mut controller);
        for attempt in 0..10 {
            let mut faulted = observation(attempt * 10, REST_ADC);
            faulted.fault_flags =
                crate::safety::fault_flag::HALL_CAPTURE | crate::safety::fault_flag::CONTROL_TIMING;
            assert!(controller.update(faulted).acknowledge_faults);
            controller.update(observation(attempt * 10 + 1, REST_ADC));
        }
        assert!(!controller.fault_lockout());
    }

    #[test]
    fn startup_requires_twelve_net_forward_transitions() {
        let mut controller = RideController::new(0);
        arm(&mut controller);
        assert_eq!(
            controller.update(observation(1, FULL_ADC)).stage,
            Stage::AwaitingFirstEdge
        );
        for progress in 1..RIDE.startup_forward_hall_transitions {
            let mut edge = observation(1 + progress as u32, FULL_ADC);
            edge.hall_sequence = progress as u32;
            edge.hall_progress = progress;
            assert_eq!(controller.update(edge).stage, Stage::StartupTracking);
        }
        let mut final_edge = observation(20, FULL_ADC);
        final_edge.hall_sequence = 12;
        final_edge.hall_progress = 12;
        assert_eq!(controller.update(final_edge).stage, Stage::Tracking);
    }

    #[test]
    fn dynamic_hall_deadlines_fail_closed_after_the_first_edge() {
        let mut controller = RideController::new(0);
        arm(&mut controller);
        controller.update(observation(1, FULL_ADC));
        let mut edge_at_deadline = observation(501, FULL_ADC);
        edge_at_deadline.hall_sequence = 1;
        edge_at_deadline.hall_progress = 1;
        assert_eq!(
            controller.update(edge_at_deadline).stage,
            Stage::StartupTracking
        );

        let mut controller = RideController::new(0);
        arm(&mut controller);
        controller.update(observation(1, FULL_ADC));
        let mut edge = observation(10, FULL_ADC);
        edge.hall_sequence = 1;
        edge.hall_progress = 1;
        edge.hall_interval_us = 75_000;
        assert!(controller.update(edge).energize);
        let mut second_edge = observation(20, FULL_ADC);
        second_edge.hall_sequence = 2;
        second_edge.hall_progress = 2;
        second_edge.hall_interval_us = 75_000;
        assert!(controller.update(second_edge).energize);
        let mut overdue = observation(170, FULL_ADC);
        overdue.hall_sequence = 2;
        overdue.hall_progress = 2;
        assert_eq!(controller.update(overdue), Command::OFF);

        let mut controller = RideController::new(0);
        arm(&mut controller);
        controller.update(observation(1, FULL_ADC));
        let mut first_edge = observation(100, FULL_ADC);
        first_edge.hall_sequence = 1;
        first_edge.hall_progress = 1;
        assert!(controller.update(first_edge).energize);
        let mut first_edge_overdue = observation(600, FULL_ADC);
        first_edge_overdue.hall_sequence = 1;
        first_edge_overdue.hall_progress = 1;
        assert_eq!(controller.update(first_edge_overdue), Command::OFF);
    }

    #[test]
    fn rollback_can_reverse_within_the_initial_sector_before_the_first_edge() {
        let mut controller = RideController::new(0);
        arm(&mut controller);

        let mut rolling_backward = observation(1, FULL_ADC);
        rolling_backward.electrical_rpm = -1_000;
        assert_eq!(
            controller.update(rolling_backward).stage,
            Stage::AwaitingFirstEdge
        );

        let mut turning_around = observation(600, FULL_ADC);
        turning_around.electrical_rpm = -500;
        let command = controller.update(turning_around);
        assert!(command.energize);
        assert_eq!(command.stage, Stage::AwaitingFirstEdge);

        let mut first_forward_edge = observation(900, FULL_ADC);
        first_forward_edge.hall_sequence = 1;
        first_forward_edge.hall_progress = 1;
        first_forward_edge.electrical_rpm = 500;
        let command = controller.update(first_forward_edge);
        assert!(command.energize);
        assert_eq!(command.stage, Stage::StartupTracking);
    }

    #[test]
    fn awaiting_the_first_edge_still_obeys_the_absolute_startup_deadline() {
        let mut controller = RideController::new(0);
        arm(&mut controller);

        let mut rolling_backward = observation(1, FULL_ADC);
        rolling_backward.electrical_rpm = -1_000;
        assert!(controller.update(rolling_backward).energize);

        let mut startup_overdue = observation(2_001, FULL_ADC);
        startup_overdue.electrical_rpm = -100;
        assert_eq!(controller.update(startup_overdue), Command::OFF);
    }

    #[test]
    fn reverse_roll_start_keeps_authority_until_forward_progress_resumes() {
        let mut controller = RideController::new(0);
        arm(&mut controller);
        assert!(controller.update(observation(1, FULL_ADC)).energize);

        let mut reverse_edge = observation(100, FULL_ADC);
        reverse_edge.hall_sequence = 1;
        reverse_edge.hall_progress = -1;
        assert_eq!(
            controller.update(reverse_edge).stage,
            Stage::StartupTracking
        );

        let mut turning_around = observation(601, FULL_ADC);
        turning_around.hall_sequence = 1;
        turning_around.hall_progress = -1;
        assert_eq!(
            controller.update(turning_around).stage,
            Stage::StartupTracking
        );

        let mut recovered_start = observation(700, FULL_ADC);
        recovered_start.hall_sequence = 2;
        recovered_start.hall_progress = 0;
        recovered_start.hall_interval_us = 50_000;
        assert_eq!(
            controller.update(recovered_start).stage,
            Stage::StartupTracking
        );

        let mut forward_edge_overdue = observation(801, FULL_ADC);
        forward_edge_overdue.hall_sequence = 2;
        forward_edge_overdue.hall_progress = 0;
        assert_eq!(controller.update(forward_edge_overdue), Command::OFF);
    }

    #[test]
    fn reverse_roll_start_still_obeys_the_absolute_startup_deadline() {
        let mut controller = RideController::new(0);
        arm(&mut controller);
        controller.update(observation(1, FULL_ADC));

        let mut reverse_edge = observation(100, FULL_ADC);
        reverse_edge.hall_sequence = 1;
        reverse_edge.hall_progress = -1;
        assert!(controller.update(reverse_edge).energize);

        let mut startup_overdue = observation(2_001, FULL_ADC);
        startup_overdue.hall_sequence = 1;
        startup_overdue.hall_progress = -1;
        assert_eq!(controller.update(startup_overdue), Command::OFF);
    }

    #[test]
    fn tracking_uses_zero_current_coast_only_above_safe_hall_entry_speed() {
        let mut controller = RideController::new(0);
        arm(&mut controller);
        controller.update(observation(1, FULL_ADC));
        let mut tracking = observation(20, FULL_ADC);
        tracking.hall_sequence = 12;
        tracking.hall_progress = 12;
        tracking.electrical_rpm = 3_000;
        assert_eq!(controller.update(tracking).stage, Stage::Tracking);

        let mut coast = observation(21, REST_ADC);
        coast.hall_sequence = 12;
        coast.hall_progress = 12;
        coast.electrical_rpm = 3_000;
        let command = controller.update(coast);
        assert!(command.energize);
        assert_eq!(command.target_q_counts, 0);
        assert_eq!(command.stage, Stage::Coasting);

        let mut stopped = coast;
        stopped.now_ms = 22;
        stopped.electrical_rpm = 2_500;
        assert_eq!(controller.update(stopped), Command::OFF);
    }

    #[test]
    fn high_speed_brake_uses_energized_zero_current_coast() {
        let mut controller = RideController::new(0);
        arm(&mut controller);
        controller.update(observation(1, FULL_ADC));
        let mut tracking = observation(20, FULL_ADC);
        tracking.hall_sequence = 12;
        tracking.hall_progress = 12;
        tracking.electrical_rpm = 10_000;
        assert_eq!(controller.update(tracking).stage, Stage::Tracking);

        let mut braking = tracking;
        braking.now_ms = 21;
        braking.brake_active = true;
        let command = controller.update(braking);
        assert!(command.energize);
        assert_eq!(command.target_q_counts, 0);
        assert_eq!(command.stage, Stage::Coasting);

        braking.now_ms = 22;
        braking.electrical_rpm = RIDE.maximum_entry_electrical_rpm as i32;
        assert_eq!(controller.update(braking), Command::OFF);
    }

    #[test]
    fn high_speed_safety_loss_still_disarms_immediately() {
        let mut controller = RideController::new(0);
        arm(&mut controller);
        controller.update(observation(1, FULL_ADC));
        let mut tracking = observation(20, FULL_ADC);
        tracking.hall_sequence = 12;
        tracking.hall_progress = 12;
        tracking.electrical_rpm = 10_000;
        assert_eq!(controller.update(tracking).stage, Stage::Tracking);

        tracking.now_ms = 21;
        tracking.brake_active = true;
        tracking.safety_events = 1;
        assert_eq!(controller.update(tracking), Command::OFF);
    }

    #[test]
    fn hall_timeout_is_twice_the_sector_interval_and_bounded() {
        assert_eq!(hall_timeout_ms(1), 100);
        assert_eq!(hall_timeout_ms(50_000), 100);
        assert_eq!(hall_timeout_ms(100_000), 200);
        assert_eq!(hall_timeout_ms(300_000), 500);
    }
}
