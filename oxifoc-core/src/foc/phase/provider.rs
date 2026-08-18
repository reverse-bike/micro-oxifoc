//! Numeric contract between a phase source and the synchronous FOC loop.

use crate::foc::control_types::Dq;
use crate::foc::numeric::{Fixed, Scalar};
use crate::foc::phase::PhaseSource;
use crate::foc::trig::Turns;

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct PhaseEstimate<A = Turns> {
    pub angle: A,
    pub electrical_rpm: i32,
    pub trustworthy: bool,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct PhaseInput<N: Scalar = Fixed, A = Turns> {
    pub applied_voltage: Dq<N>,
    pub measured_current: Dq<N>,
    pub electrical_angle: A,
    pub control_period_ns: u32,
}

pub trait PhaseProvider<N: Scalar = Fixed> {
    type Angle: Copy;

    fn source(&self) -> PhaseSource;

    fn estimate(&self, elapsed_since_observation_us: u32) -> Option<PhaseEstimate<Self::Angle>>;

    /// Estimate used by a periodic control loop.
    ///
    /// Stateful providers can use the control period to constrain estimator
    /// motion between samples. The default preserves the immutable estimate
    /// for providers that do not need rate limiting.
    fn estimate_for_control(
        &mut self,
        elapsed_since_observation_us: u32,
        _control_period_ns: u32,
    ) -> Option<PhaseEstimate<Self::Angle>> {
        self.estimate(elapsed_since_observation_us)
    }

    fn update(&mut self, _input: &PhaseInput<N, Self::Angle>) {}

    fn injection(&self) -> Dq<N> {
        Dq::default()
    }

    fn request_source(&mut self, _source: PhaseSource) -> bool {
        false
    }
}
