use crate::config::RIDE_PHASE_CURRENT_LIMIT_COUNTS;

pub const PLAUSIBLE_LOW_ADC: u16 = 250;
pub const REST_ADC: u16 = 726;
pub const REST_MAX_ADC: u16 = 850;
pub const FULL_ADC: u16 = 3_252;
pub const PLAUSIBLE_HIGH_ADC: u16 = 3_750;
pub const MAXIMUM_DEMAND_PERCENT: u8 = 100;

const _: () = assert!(PLAUSIBLE_LOW_ADC <= REST_ADC);
const _: () = assert!(REST_ADC < REST_MAX_ADC);
const _: () = assert!(REST_MAX_ADC < FULL_ADC);
const _: () = assert!(FULL_ADC <= PLAUSIBLE_HIGH_ADC);
const _: () = assert!(PLAUSIBLE_HIGH_ADC <= 4_095);

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Observation {
    raw: u16,
    state: State,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum State {
    Invalid,
    AtRest,
    Demand(Demand),
}

impl Observation {
    pub const INVALID_ZERO: Self = Self::invalid_acquisition(0);

    pub const fn invalid_acquisition(raw: u16) -> Self {
        Self {
            raw,
            state: State::Invalid,
        }
    }

    pub fn from_raw(raw: u16) -> Self {
        if !(PLAUSIBLE_LOW_ADC..=PLAUSIBLE_HIGH_ADC).contains(&raw) {
            return Self::invalid_acquisition(raw);
        }
        if raw <= REST_MAX_ADC {
            return Self {
                raw,
                state: State::AtRest,
            };
        }

        let span = u32::from(FULL_ADC - REST_MAX_ADC);
        let above_rest = u32::from(raw.min(FULL_ADC) - REST_MAX_ADC);
        let percent = above_rest
            .saturating_mul(u32::from(MAXIMUM_DEMAND_PERCENT))
            .saturating_add(span - 1)
            / span;
        Self {
            raw,
            state: State::Demand(Demand(percent.min(u32::from(MAXIMUM_DEMAND_PERCENT)) as u8)),
        }
    }

    pub const fn raw(self) -> u16 {
        self.raw
    }

    pub const fn demand(self) -> Option<Demand> {
        match self.state {
            State::Demand(demand) => Some(demand),
            State::Invalid | State::AtRest => None,
        }
    }

    pub const fn demand_percent(self) -> u8 {
        match self.demand() {
            Some(demand) => demand.percent(),
            None => 0,
        }
    }

    pub const fn is_at_rest(self) -> bool {
        matches!(self.state, State::AtRest)
    }

    pub const fn is_valid(self) -> bool {
        !matches!(self.state, State::Invalid)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Demand(u8);

impl Demand {
    pub const fn from_percent(percent: u8) -> Option<Self> {
        if percent == 0 {
            None
        } else if percent > MAXIMUM_DEMAND_PERCENT {
            Some(Self(MAXIMUM_DEMAND_PERCENT))
        } else {
            Some(Self(percent))
        }
    }

    pub const fn percent(self) -> u8 {
        self.0
    }

    pub fn negative_q_target(self, phase_current_limit_counts: u16) -> i16 {
        let limit = phase_current_limit_counts.min(RIDE_PHASE_CURRENT_LIMIT_COUNTS);
        let scaled = u32::from(self.0)
            .saturating_mul(u32::from(limit))
            .saturating_add(u32::from(MAXIMUM_DEMAND_PERCENT - 1))
            / u32::from(MAXIMUM_DEMAND_PERCENT);
        -(scaled.min(i16::MAX as u32) as i16)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ArmingGate {
    armed: bool,
}

impl ArmingGate {
    pub const fn new() -> Self {
        Self { armed: false }
    }

    pub const fn armed(self) -> bool {
        self.armed
    }

    pub fn update(
        &mut self,
        throttle: Observation,
        brake_active: bool,
        safety_ready: bool,
    ) -> Option<Demand> {
        if brake_active || !safety_ready || !throttle.is_valid() {
            self.armed = false;
            return None;
        }
        if throttle.is_at_rest() {
            self.armed = true;
            return None;
        }
        if self.armed { throttle.demand() } else { None }
    }
}

impl Default for ArmingGate {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn calibration_boundaries_are_classified() {
        assert!(!Observation::from_raw(PLAUSIBLE_LOW_ADC - 1).is_valid());
        assert!(Observation::from_raw(PLAUSIBLE_LOW_ADC).is_at_rest());
        assert!(Observation::from_raw(REST_MAX_ADC).is_at_rest());
        assert!(Observation::from_raw(REST_MAX_ADC + 1).demand().is_some());
        assert_eq!(
            Observation::from_raw(FULL_ADC).demand_percent(),
            MAXIMUM_DEMAND_PERCENT
        );
        assert_eq!(
            Observation::from_raw(PLAUSIBLE_HIGH_ADC).demand_percent(),
            MAXIMUM_DEMAND_PERCENT
        );
        assert!(!Observation::from_raw(PLAUSIBLE_HIGH_ADC + 1).is_valid());
    }

    #[test]
    fn demand_is_monotonic_and_bounded() {
        let mut previous = 0;
        for raw in REST_MAX_ADC + 1..=PLAUSIBLE_HIGH_ADC {
            let demand = Observation::from_raw(raw).demand_percent();
            assert!(demand >= previous);
            assert!(demand <= MAXIMUM_DEMAND_PERCENT);
            previous = demand;
        }
    }

    #[test]
    fn full_throttle_uses_confirmed_forward_sign_and_current_scale() {
        let demand = Observation::from_raw(FULL_ADC).demand().unwrap();
        assert_eq!(demand.negative_q_target(880), -880);
        assert_eq!(demand.negative_q_target(60), -60);
        assert_eq!(demand.negative_q_target(900), -880);
    }

    #[test]
    fn boot_high_cannot_arm_until_throttle_returns_to_rest() {
        let mut gate = ArmingGate::new();
        let demand = Observation::from_raw(2_000);
        assert_eq!(gate.update(demand, false, true), None);
        assert!(!gate.armed());
        assert_eq!(
            gate.update(Observation::from_raw(REST_ADC), false, true),
            None
        );
        assert!(gate.armed());
        assert_eq!(gate.update(demand, false, true), demand.demand());
    }

    #[test]
    fn brake_invalid_input_and_safety_loss_require_rearming_at_rest() {
        for (observation, brake, ready) in [
            (Observation::from_raw(2_000), true, true),
            (Observation::invalid_acquisition(REST_ADC), false, true),
            (Observation::from_raw(2_000), false, false),
        ] {
            let mut gate = ArmingGate { armed: true };
            assert_eq!(gate.update(observation, brake, ready), None);
            assert!(!gate.armed());
            assert_eq!(gate.update(Observation::from_raw(2_000), false, true), None);
        }
    }
}
