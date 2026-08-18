//! Numeric contract between a phase strategy and the shared FOC loop.

use crate::foc::angle::Turns;
use crate::foc::control_types::Dq;
use crate::foc::numeric::{Fixed, Scalar};
use crate::foc::phase::PhaseStrategy;

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct ControlPhaseEstimate<A = Turns> {
    pub angle: A,
    pub electrical_rpm: i32,
    pub trustworthy: bool,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct ControlPhaseInput<N: Scalar = Fixed, A = Turns> {
    pub applied_voltage: Dq<N>,
    pub measured_current: Dq<N>,
    pub electrical_angle: A,
    pub control_period_ns: u32,
}

pub trait ControlPhaseProvider<N: Scalar = Fixed> {
    type Angle: Copy;

    fn source(&self) -> PhaseStrategy;

    fn estimate(
        &self,
        elapsed_since_observation_us: u32,
    ) -> Option<ControlPhaseEstimate<Self::Angle>>;

    fn update(&mut self, _input: &ControlPhaseInput<N, Self::Angle>) {}

    fn injection(&self) -> Dq<N> {
        Dq::default()
    }

    fn request_source(&mut self, _source: PhaseStrategy) -> bool {
        false
    }
}
