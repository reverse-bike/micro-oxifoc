//! Phase-source selection for the fixed-point synchronous controller.

use crate::foc::Fixed;

/// Electrical-angle source and any crossover parameters it owns.
///
/// Speeds use signed electrical RPM and voltage/confidence values use Q16.16.
/// Keeping the strategy parameters in the source preserves OxiFOC's phase
/// manager contract without requiring floating-point values in the F103 image.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum PhaseSource {
    #[default]
    Hall,
    Encoder,
    Observer,
    Hfi,
    HallToObserver {
        blend_low_erpm: i32,
        blend_high_erpm: i32,
    },
    EncoderToObserver {
        blend_low_erpm: i32,
        blend_high_erpm: i32,
    },
    HfiToObserver {
        minimum_erpm: i32,
        minimum_confidence: Fixed,
    },
    HfiToObserverVolts {
        toggle_voltage: Fixed,
        minimum_confidence: Fixed,
    },
    HfiToHall {
        switch_erpm: i32,
    },
    HfiToEncoder {
        switch_erpm: i32,
    },
    Manual,
    OpenLoop,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PhaseSourceError {
    HallNotAvailable,
    EncoderNotAvailable,
    ObserverNotConfigured,
    HfiNotConfigured,
    ManualSourceNotConfigured,
}

impl PhaseSource {
    pub const fn requires_hall(self) -> bool {
        matches!(
            self,
            Self::Hall | Self::HallToObserver { .. } | Self::HfiToHall { .. }
        )
    }

    pub const fn requires_encoder(self) -> bool {
        matches!(
            self,
            Self::Encoder | Self::EncoderToObserver { .. } | Self::HfiToEncoder { .. }
        )
    }

    pub const fn requires_observer(self) -> bool {
        matches!(
            self,
            Self::Observer
                | Self::HallToObserver { .. }
                | Self::EncoderToObserver { .. }
                | Self::HfiToObserver { .. }
                | Self::HfiToObserverVolts { .. }
        )
    }

    pub const fn requires_hfi(self) -> bool {
        matches!(
            self,
            Self::Hfi
                | Self::HfiToObserver { .. }
                | Self::HfiToObserverVolts { .. }
                | Self::HfiToHall { .. }
                | Self::HfiToEncoder { .. }
        )
    }

    pub const fn is_manual(self) -> bool {
        matches!(self, Self::Manual | Self::OpenLoop)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn experimental_sources_retain_their_sensor_requirements() {
        let hall_observer = PhaseSource::HallToObserver {
            blend_low_erpm: 3_000,
            blend_high_erpm: 6_000,
        };
        let encoder_observer = PhaseSource::EncoderToObserver {
            blend_low_erpm: 3_000,
            blend_high_erpm: 6_000,
        };
        let hfi_observer = PhaseSource::HfiToObserver {
            minimum_erpm: 1_000,
            minimum_confidence: Fixed::ratio(4, 5),
        };
        let hfi_voltage = PhaseSource::HfiToObserverVolts {
            toggle_voltage: Fixed::from_integer(3),
            minimum_confidence: Fixed::ratio(4, 5),
        };

        assert!(PhaseSource::Hall.requires_hall());
        assert!(hall_observer.requires_hall());
        assert!(hall_observer.requires_observer());
        assert!(encoder_observer.requires_encoder());
        assert!(encoder_observer.requires_observer());
        assert!(hfi_observer.requires_hfi());
        assert!(hfi_observer.requires_observer());
        assert!(hfi_voltage.requires_hfi());
        assert!(hfi_voltage.requires_observer());
        assert!(PhaseSource::HfiToEncoder { switch_erpm: 1_000 }.requires_encoder());
        assert!(PhaseSource::Manual.is_manual());
        assert!(PhaseSource::OpenLoop.is_manual());
        assert!(!PhaseSource::OpenLoop.requires_hall());
    }
}
