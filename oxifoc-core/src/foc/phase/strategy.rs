//! Phase-strategy identities available to every numeric backend.

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum PhaseStrategy {
    #[default]
    Hall,
    Encoder,
    Observer,
    Hfi,
    HallToObserver,
    EncoderToObserver,
    HfiToObserver,
    HfiToObserverVolts,
    HfiToHall,
    HfiToEncoder,
    Manual,
    OpenLoop,
}

impl PhaseStrategy {
    pub const fn requires_hall(self) -> bool {
        matches!(self, Self::Hall | Self::HallToObserver | Self::HfiToHall)
    }

    pub const fn requires_encoder(self) -> bool {
        matches!(
            self,
            Self::Encoder | Self::EncoderToObserver | Self::HfiToEncoder
        )
    }

    pub const fn requires_observer(self) -> bool {
        matches!(
            self,
            Self::Observer
                | Self::HallToObserver
                | Self::EncoderToObserver
                | Self::HfiToObserver
                | Self::HfiToObserverVolts
        )
    }

    pub const fn requires_hfi(self) -> bool {
        matches!(
            self,
            Self::Hfi
                | Self::HfiToObserver
                | Self::HfiToObserverVolts
                | Self::HfiToHall
                | Self::HfiToEncoder
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
        assert!(PhaseStrategy::Hall.requires_hall());
        assert!(PhaseStrategy::HallToObserver.requires_hall());
        assert!(PhaseStrategy::HallToObserver.requires_observer());
        assert!(PhaseStrategy::EncoderToObserver.requires_encoder());
        assert!(PhaseStrategy::EncoderToObserver.requires_observer());
        assert!(PhaseStrategy::HfiToObserver.requires_hfi());
        assert!(PhaseStrategy::HfiToObserver.requires_observer());
        assert!(PhaseStrategy::HfiToObserverVolts.requires_hfi());
        assert!(PhaseStrategy::HfiToObserverVolts.requires_observer());
        assert!(PhaseStrategy::HfiToEncoder.requires_encoder());
        assert!(PhaseStrategy::Manual.is_manual());
        assert!(PhaseStrategy::OpenLoop.is_manual());
        assert!(!PhaseStrategy::OpenLoop.requires_hall());
    }
}
