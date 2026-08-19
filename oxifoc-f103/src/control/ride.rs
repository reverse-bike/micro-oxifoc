//! Local-throttle ride authority and Hall-progress lifecycle.

use crate::config::RIDE_PHASE_CURRENT_LIMIT_COUNTS;
use crate::sensors::throttle::Observation as ThrottleObservation;

pub const OUTPUT_LEASE_CYCLES: u32 = 160;
pub const STARTUP_HALL_TIMEOUT_MS: u32 = 500;
pub const STARTUP_TIMEOUT_MS: u32 = 2_000;
pub const STARTUP_FORWARD_TRANSITIONS: i32 = 12;
pub const MINIMUM_HALL_TIMEOUT_MS: u32 = 100;
pub const MAXIMUM_HALL_TIMEOUT_MS: u32 = 500;
pub const HALL_ENTRY_MAXIMUM_ERPM: u32 = 2_500;

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
    pub brake_active: bool,
    pub environment_dc_limit_counts: Option<u16>,
    pub hall_valid: bool,
    pub current_valid: bool,
    pub fault_flags: u32,
    pub safety_events: u32,
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
    pub stage: Stage,
}

impl Command {
    pub const OFF: Self = Self {
        energize: false,
        acknowledge_faults: false,
        target_q_counts: 0,
        dc_current_limit_counts: 0,
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
}

impl RideController {
    pub const fn new(initial_safety_events: u32) -> Self {
        Self {
            state: State::Disarmed,
            observed_safety_events: initial_safety_events,
        }
    }

    pub const fn stage(&self) -> Stage {
        match self.state {
            State::Disarmed => Stage::Disarmed,
            State::Ready => Stage::Ready,
            State::Active(run) => run.stage,
        }
    }

    pub fn update(&mut self, observation: Observation) -> Command {
        let safety_event = observation.safety_events != self.observed_safety_events;
        self.observed_safety_events = observation.safety_events;
        if observation.fault_flags != 0 {
            self.state = State::Disarmed;
            return Command {
                acknowledge_faults: observation.throttle.is_valid()
                    && observation.throttle.is_at_rest(),
                ..Command::OFF
            };
        }
        let environment_limit = observation.environment_dc_limit_counts.unwrap_or(0);
        let safety_ready = !safety_event
            && observation.hall_valid
            && observation.current_valid
            && environment_limit != 0;
        if observation.brake_active || !observation.throttle.is_valid() || !safety_ready {
            self.state = State::Disarmed;
            return Command::OFF;
        }

        match self.state {
            State::Disarmed => {
                if observation.throttle.is_at_rest() {
                    self.state = State::Ready;
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
        let Some(demand) = observation.throttle.demand() else {
            if !observation.throttle.is_at_rest() {
                self.state = State::Disarmed;
                return Command::OFF;
            }
            return Command {
                stage: Stage::Ready,
                ..Command::OFF
            };
        };
        if observation.electrical_rpm.unsigned_abs() > HALL_ENTRY_MAXIMUM_ERPM {
            return Command {
                stage: Stage::Ready,
                ..Command::OFF
            };
        }

        let run = Run {
            stage: Stage::AwaitingFirstEdge,
            startup_deadline_ms: observation.now_ms.wrapping_add(STARTUP_TIMEOUT_MS),
            hall_deadline_ms: observation.now_ms.wrapping_add(STARTUP_HALL_TIMEOUT_MS),
            last_hall_sequence: observation.hall_sequence,
            starting_hall_progress: observation.hall_progress,
        };
        self.state = State::Active(run);
        active_command(
            demand.negative_q_target(RIDE_PHASE_CURRENT_LIMIT_COUNTS),
            environment_limit,
            run.stage,
        )
    }

    fn update_active(
        &mut self,
        mut run: Run,
        observation: Observation,
        environment_limit: u16,
    ) -> Command {
        let demand = observation.throttle.demand();
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
                STARTUP_HALL_TIMEOUT_MS
            } else {
                hall_timeout_ms(observation.hall_interval_us)
            });
            if first_edge {
                run.stage = Stage::StartupTracking;
            }
            if matches!(run.stage, Stage::StartupTracking)
                && startup_progress >= STARTUP_FORWARD_TRANSITIONS
            {
                run.stage = Stage::Tracking;
            }
        }

        let timed_out = match run.stage {
            Stage::AwaitingFirstEdge => {
                deadline_reached(observation.now_ms, run.hall_deadline_ms)
                    || deadline_reached(observation.now_ms, run.startup_deadline_ms)
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
                if observation.electrical_rpm.unsigned_abs() <= HALL_ENTRY_MAXIMUM_ERPM {
                    self.state = State::Disarmed;
                    Command::OFF
                } else {
                    run.stage = Stage::Coasting;
                    self.state = State::Active(run);
                    active_command(0, environment_limit, Stage::Coasting)
                }
            }
            (Stage::Coasting, None) => {
                if observation.electrical_rpm.unsigned_abs() <= HALL_ENTRY_MAXIMUM_ERPM {
                    self.state = State::Disarmed;
                    Command::OFF
                } else {
                    self.state = State::Active(run);
                    active_command(0, environment_limit, Stage::Coasting)
                }
            }
            (Stage::Coasting, Some(demand)) => {
                run.stage = Stage::Tracking;
                self.state = State::Active(run);
                active_command(
                    demand.negative_q_target(RIDE_PHASE_CURRENT_LIMIT_COUNTS),
                    environment_limit,
                    Stage::Tracking,
                )
            }
            (_, Some(demand)) => {
                self.state = State::Active(run);
                active_command(
                    demand.negative_q_target(RIDE_PHASE_CURRENT_LIMIT_COUNTS),
                    environment_limit,
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

fn active_command(target_q_counts: i16, environment_limit: u16, stage: Stage) -> Command {
    Command {
        energize: true,
        acknowledge_faults: false,
        target_q_counts,
        dc_current_limit_counts: environment_limit,
        stage,
    }
}

pub const fn hall_timeout_ms(interval_us: u32) -> u32 {
    let interval_ms = interval_us.saturating_add(999) / 1_000;
    let timeout = interval_ms.saturating_mul(2);
    if timeout < MINIMUM_HALL_TIMEOUT_MS {
        MINIMUM_HALL_TIMEOUT_MS
    } else if timeout > MAXIMUM_HALL_TIMEOUT_MS {
        MAXIMUM_HALL_TIMEOUT_MS
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
    use crate::sensors::throttle::{FULL_ADC, REST_ADC};

    fn observation(now_ms: u32, throttle_raw: u16) -> Observation {
        Observation {
            now_ms,
            throttle: ThrottleObservation::from_raw(throttle_raw),
            brake_active: false,
            environment_dc_limit_counts: Some(crate::config::RIDE_DC_BUS_CURRENT_LIMIT_COUNTS),
            hall_valid: true,
            current_valid: true,
            fault_flags: 0,
            safety_events: 0,
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
        assert_eq!(command.target_q_counts, -838);
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
    fn startup_requires_twelve_net_forward_transitions() {
        let mut controller = RideController::new(0);
        arm(&mut controller);
        assert_eq!(
            controller.update(observation(1, FULL_ADC)).stage,
            Stage::AwaitingFirstEdge
        );
        for progress in 1..STARTUP_FORWARD_TRANSITIONS {
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
    fn startup_and_dynamic_hall_deadlines_fail_closed() {
        let mut controller = RideController::new(0);
        arm(&mut controller);
        assert!(controller.update(observation(1, FULL_ADC)).energize);
        assert!(controller.update(observation(500, FULL_ADC)).energize);
        assert_eq!(controller.update(observation(501, FULL_ADC)), Command::OFF);

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
    fn hall_timeout_is_twice_the_sector_interval_and_bounded() {
        assert_eq!(hall_timeout_ms(1), 100);
        assert_eq!(hall_timeout_ms(50_000), 100);
        assert_eq!(hall_timeout_ms(100_000), 200);
        assert_eq!(hall_timeout_ms(300_000), 500);
    }
}
