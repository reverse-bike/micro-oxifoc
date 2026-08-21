//! Phase-source selection for the synchronous controller.

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum PhaseSource {
    #[default]
    Hall,
    Observer,
    HallToObserver {
        blend_low_erpm: i32,
        blend_high_erpm: i32,
    },
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PhaseSourceError {
    ObserverNotConfigured,
}

impl PhaseSource {
    pub const fn requires_hall(self) -> bool {
        matches!(self, Self::Hall | Self::HallToObserver { .. })
    }

    pub const fn requires_observer(self) -> bool {
        matches!(self, Self::Observer | Self::HallToObserver { .. })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sources_report_their_provider_requirements() {
        let crossover = PhaseSource::HallToObserver {
            blend_low_erpm: 3_000,
            blend_high_erpm: 6_000,
        };

        assert!(PhaseSource::Hall.requires_hall());
        assert!(!PhaseSource::Hall.requires_observer());
        assert!(!PhaseSource::Observer.requires_hall());
        assert!(PhaseSource::Observer.requires_observer());
        assert!(crossover.requires_hall());
        assert!(crossover.requires_observer());
    }
}
