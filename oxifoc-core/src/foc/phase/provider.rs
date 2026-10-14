//! Numeric contract between a phase source and the synchronous FOC loop.

use crate::foc::control_types::AlphaBeta;
use crate::foc::phase::PhaseSource;
use crate::foc::trig::Turns;

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct PhaseEstimate {
    pub angle: Turns,
    pub electrical_rpm: i32,
    pub trustworthy: bool,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct PhaseInput {
    /// Stationary-frame voltage that was physically applied while the current
    /// sample was developing.
    pub applied_voltage: AlphaBeta,
    /// Simultaneously sampled stationary-frame phase current.
    pub measured_current: AlphaBeta,
    pub control_period_ns: u32,
}

impl PhaseInput {
    pub const fn new(
        applied_voltage: AlphaBeta,
        measured_current: AlphaBeta,
        control_period_ns: u32,
    ) -> Self {
        Self {
            applied_voltage,
            measured_current,
            control_period_ns,
        }
    }
}

pub trait PhaseProvider {
    fn source(&self) -> PhaseSource;

    /// Estimate used by a periodic control loop.
    ///
    /// Stateful providers can use the control period to constrain estimator
    /// motion between samples.
    fn estimate_for_control(
        &mut self,
        elapsed_since_observation_us: u32,
        control_period_ns: u32,
    ) -> Option<PhaseEstimate>;

    fn update(&mut self, _input: &PhaseInput) {}
}
