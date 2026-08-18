//! Phase-source ownership and selection for the synchronous controller.
//!
//! The manager remains the boundary between a concrete angle sensor and the
//! FOC loop. The F103 image currently installs only a Hall provider; the
//! source enum and validation errors keep the observer, encoder, HFI, manual,
//! and open-loop extension points explicit without linking unused estimators.

use core::marker::PhantomData;

use crate::foc::numeric::{Fixed, Scalar};

use super::{PhaseEstimate, PhaseInput, PhaseProvider, PhaseSource, PhaseSourceError};

/// Owns the configured phase providers and exposes the selected source to the
/// current loop.
///
/// Only the Hall slot is populated in the present F103 firmware. Additional
/// fixed-point providers belong here when they are ported; they should not be
/// embedded in the Hall sensor or the device interrupt.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PhaseManager<H, N: Scalar = Fixed> {
    hall: H,
    source: PhaseSource,
    numeric: PhantomData<N>,
}

impl<H, N: Scalar> PhaseManager<H, N> {
    pub const fn with_hall(hall: H) -> Self {
        Self {
            hall,
            source: PhaseSource::Hall,
            numeric: PhantomData,
        }
    }

    pub const fn source(&self) -> PhaseSource {
        self.source
    }

    pub const fn hall(&self) -> &H {
        &self.hall
    }

    pub fn hall_mut(&mut self) -> &mut H {
        &mut self.hall
    }

    /// Select an available phase source.
    ///
    /// Hall is the only installed provider today. Rejections distinguish the
    /// missing provider class so future observer experiments can add their
    /// state to this manager without changing the FOC/device boundary.
    pub fn set_source(&mut self, source: PhaseSource) -> Result<(), PhaseSourceError> {
        let result = match source {
            PhaseSource::Hall => Ok(()),
            PhaseSource::Encoder | PhaseSource::EncoderToObserver { .. } => {
                Err(PhaseSourceError::EncoderNotAvailable)
            }
            PhaseSource::Observer | PhaseSource::HallToObserver { .. } => {
                Err(PhaseSourceError::ObserverNotConfigured)
            }
            PhaseSource::Hfi
            | PhaseSource::HfiToObserver { .. }
            | PhaseSource::HfiToObserverVolts { .. }
            | PhaseSource::HfiToHall { .. }
            | PhaseSource::HfiToEncoder { .. } => Err(PhaseSourceError::HfiNotConfigured),
            PhaseSource::Manual | PhaseSource::OpenLoop => {
                Err(PhaseSourceError::ManualSourceNotConfigured)
            }
        };
        if result.is_ok() {
            self.source = source;
        }
        result
    }
}

impl<H, N> PhaseProvider<N> for PhaseManager<H, N>
where
    H: PhaseProvider<N>,
    N: Scalar,
{
    type Angle = H::Angle;

    fn source(&self) -> PhaseSource {
        self.source
    }

    fn estimate(&self, elapsed_since_observation_us: u32) -> Option<PhaseEstimate<Self::Angle>> {
        self.hall.estimate(elapsed_since_observation_us)
    }

    fn estimate_for_control(
        &mut self,
        elapsed_since_observation_us: u32,
        control_period_ns: u32,
    ) -> Option<PhaseEstimate<Self::Angle>> {
        self.hall
            .estimate_for_control(elapsed_since_observation_us, control_period_ns)
    }

    fn update(&mut self, input: &PhaseInput<N, Self::Angle>) {
        self.hall.update(input);
    }

    fn injection(&self) -> crate::foc::Dq<N> {
        self.hall.injection()
    }

    fn request_source(&mut self, source: PhaseSource) -> bool {
        self.set_source(source).is_ok()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::foc::trig::Turns;

    #[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
    struct TestHall;

    impl PhaseProvider for TestHall {
        type Angle = Turns;

        fn source(&self) -> PhaseSource {
            PhaseSource::Hall
        }

        fn estimate(&self, _elapsed_since_observation_us: u32) -> Option<PhaseEstimate<Turns>> {
            Some(PhaseEstimate {
                angle: 0x1234_0000,
                electrical_rpm: -900,
                trustworthy: true,
            })
        }
    }

    #[test]
    fn hall_provider_remains_behind_the_phase_manager() {
        let manager = PhaseManager::<TestHall>::with_hall(TestHall);
        assert_eq!(manager.source(), PhaseSource::Hall);
        assert_eq!(manager.estimate(10).unwrap().angle, 0x1234_0000);
    }

    #[test]
    fn unavailable_sources_are_rejected_without_changing_the_active_source() {
        let mut manager = PhaseManager::<TestHall>::with_hall(TestHall);
        assert_eq!(
            manager.set_source(PhaseSource::Observer),
            Err(PhaseSourceError::ObserverNotConfigured)
        );
        assert_eq!(manager.source(), PhaseSource::Hall);
    }
}
