//! Selection of the one calibration routine driven by the control loop.

use super::flux_linkage::FluxLinkageCalibration;
use super::hall_calibration::HallCalibration;
use super::inductance::InductanceCalibration;
use super::resistance::ResistanceCalibration;
use super::types::{Actuation, Failure, Routine};
use core::sync::atomic::{AtomicU8, Ordering};

/// Hand-off of one routine start from foreground to the control loop.
///
/// The control loop takes the request with an atomic swap, so a foreground
/// `cancel` either removes the pending start or finds it already taken.
pub struct RoutineRequest(AtomicU8);

impl RoutineRequest {
    pub const fn new() -> Self {
        Self(AtomicU8::new(Routine::None as u8))
    }

    pub fn request(&self, routine: Routine) {
        self.0.store(routine as u8, Ordering::Release);
    }

    /// Withdraws a start the control loop has not taken; returns whether one
    /// was pending.
    pub fn cancel(&self) -> bool {
        self.0.swap(Routine::None as u8, Ordering::AcqRel) != Routine::None as u8
    }

    pub fn take(&self) -> Routine {
        match self.0.swap(Routine::None as u8, Ordering::AcqRel) {
            1 => Routine::Resistance,
            2 => Routine::Inductance,
            3 => Routine::FluxLinkage,
            4 => Routine::Hall,
            _ => Routine::None,
        }
    }
}

impl Default for RoutineRequest {
    fn default() -> Self {
        Self::new()
    }
}

/// Every routine keeps its own result; `routine` names the one last started.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Sequences {
    pub routine: Routine,
    pub resistance: ResistanceCalibration,
    pub inductance: InductanceCalibration,
    pub flux_linkage: FluxLinkageCalibration,
    pub hall: HallCalibration,
}

impl Sequences {
    pub const fn new() -> Self {
        Self {
            routine: Routine::None,
            resistance: ResistanceCalibration::new(),
            inductance: InductanceCalibration::new(),
            flux_linkage: FluxLinkageCalibration::new(),
            hall: HallCalibration::new(),
        }
    }

    /// Starts `routine` from the results of the routines it depends on.
    pub fn start(&mut self, routine: Routine) {
        let resistance_uv_per_count = self.resistance.result().effective_uv_per_count;
        match routine {
            Routine::Resistance => self.resistance.start(),
            Routine::Inductance => self.inductance.start(resistance_uv_per_count),
            Routine::FluxLinkage => {
                let inductance = self.inductance.result();
                self.flux_linkage.start(
                    resistance_uv_per_count,
                    inductance.inductance_d_nwb_per_count,
                    inductance.inductance_q_nwb_per_count,
                );
            }
            Routine::Hall => self.hall.start(),
            Routine::None => return,
        }
        self.routine = routine;
    }

    /// Fails the routine in progress and reports whether there was one.
    /// Finished routines keep their state, failure, and result.
    pub fn fail_active(&mut self, failure: Failure) -> bool {
        if !self.active() {
            return false;
        }
        match self.routine {
            Routine::Resistance => self.resistance.fail(failure),
            Routine::Inductance => self.inductance.fail(failure),
            Routine::FluxLinkage => self.flux_linkage.fail(failure),
            Routine::Hall => self.hall.fail(failure),
            Routine::None => {}
        }
        true
    }

    pub const fn active(&self) -> bool {
        match self.routine {
            Routine::Resistance => self.resistance.active(),
            Routine::Inductance => self.inductance.active(),
            Routine::FluxLinkage => self.flux_linkage.active(),
            Routine::Hall => self.hall.active(),
            Routine::None => false,
        }
    }

    pub const fn state(&self) -> u8 {
        match self.routine {
            Routine::Resistance => self.resistance.state() as u8,
            Routine::Inductance => self.inductance.state() as u8,
            Routine::FluxLinkage => self.flux_linkage.state() as u8,
            Routine::Hall => self.hall.state() as u8,
            Routine::None => 0,
        }
    }

    pub const fn failure(&self) -> Failure {
        match self.routine {
            Routine::Resistance => self.resistance.failure(),
            Routine::Inductance => self.inductance.failure(),
            Routine::FluxLinkage => self.flux_linkage.failure(),
            Routine::Hall => self.hall.failure(),
            Routine::None => Failure::None,
        }
    }

    pub fn progress(&self) -> u16 {
        match self.routine {
            Routine::Resistance => self.resistance.sample_progress(),
            Routine::Inductance => self.inductance.pulse_progress(),
            Routine::FluxLinkage => self.flux_linkage.progress(),
            Routine::Hall => self.hall.progress(),
            Routine::None => 0,
        }
    }

    pub fn actuation(&self) -> Actuation {
        match self.routine {
            Routine::Resistance if self.resistance.energizing() => Actuation::Current {
                angle: 0,
                direct_counts: self.resistance.target_counts(),
                quadrature_counts: 0,
            },
            Routine::Resistance | Routine::None => Actuation::Off,
            Routine::Inductance => self.inductance.actuation(),
            Routine::FluxLinkage => self.flux_linkage.actuation(),
            Routine::Hall => self.hall.actuation(),
        }
    }

    pub const fn electrical_rpm(&self) -> i32 {
        match self.routine {
            Routine::FluxLinkage => self.flux_linkage.electrical_rpm(),
            Routine::None | Routine::Resistance | Routine::Inductance | Routine::Hall => 0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::calibration::resistance::{Sample, State as ResistanceState};

    fn completed_resistance() -> Sequences {
        let mut sequences = Sequences::new();
        sequences.start(Routine::Resistance);
        while sequences.active() {
            let current = sequences.resistance.target_counts();
            sequences.resistance.tick(Sample {
                measured_d_counts: current,
                applied_d_tick_bits: (17 + i32::from(current) / 5) << 16,
                bus_voltage_mv: 45_000,
            });
        }
        assert_eq!(sequences.resistance.state(), ResistanceState::Complete);
        sequences
    }

    #[test]
    fn idle_failure_leaves_every_routine_untouched() {
        let mut sequences = Sequences::new();
        assert!(!sequences.fail_active(Failure::HardwareFault));
        assert_eq!(sequences, Sequences::new());
    }

    #[test]
    fn failure_after_completion_preserves_the_result() {
        let mut sequences = completed_resistance();
        let completed = sequences;
        assert!(!sequences.fail_active(Failure::HardwareFault));
        assert_eq!(sequences, completed);
        assert_eq!(sequences.failure(), Failure::None);
        assert_eq!(sequences.state(), ResistanceState::Complete as u8);
    }

    #[test]
    fn failure_is_recorded_once_on_the_active_routine() {
        let mut sequences = completed_resistance();
        sequences.start(Routine::Inductance);
        assert!(sequences.active());
        assert!(sequences.fail_active(Failure::HardwareFault));
        assert!(!sequences.fail_active(Failure::Stopped));
        assert_eq!(sequences.failure(), Failure::HardwareFault);
        assert_eq!(sequences.resistance.state(), ResistanceState::Complete);
        assert_eq!(sequences.resistance.failure(), Failure::None);
    }

    #[test]
    fn routines_start_from_their_prerequisite_results() {
        let mut sequences = Sequences::new();
        sequences.start(Routine::FluxLinkage);
        assert_eq!(sequences.routine, Routine::FluxLinkage);
        assert!(!sequences.active());
        assert_eq!(sequences.failure(), Failure::MissingPrerequisite);

        let mut sequences = completed_resistance();
        sequences.start(Routine::Inductance);
        assert!(sequences.active());
    }

    #[test]
    fn a_request_is_taken_once() {
        let request = RoutineRequest::new();
        request.request(Routine::Hall);
        assert_eq!(request.take(), Routine::Hall);
        assert_eq!(request.take(), Routine::None);
    }

    #[test]
    fn a_cancelled_request_never_starts() {
        let request = RoutineRequest::new();
        request.request(Routine::Inductance);
        assert!(request.cancel());
        assert_eq!(request.take(), Routine::None);
        assert!(!request.cancel());
    }
}
