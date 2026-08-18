//! Wheel-pulse qualification for stock speed, distance, and updater safety.

pub const DISTANCE_PER_PULSE_MM: u16 = 312;
pub const MINIMUM_INTERVAL_US: u32 = 1_000;

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Capture {
    pub primed: bool,
    pub quiet: bool,
    pub interval_us: u32,
    pub pulse_count: u32,
    pub capture_overruns: u16,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum State {
    #[default]
    Uninitialized,
    Quiet,
    AwaitingSecondPulse,
    Tracking {
        speed_tenths_kph: u16,
    },
    Implausible,
}

impl State {
    pub const fn speed_tenths_kph(self) -> u16 {
        match self {
            Self::Tracking { speed_tenths_kph } => speed_tenths_kph,
            Self::Uninitialized | Self::Quiet | Self::AwaitingSecondPulse | Self::Implausible => 0,
        }
    }

    pub const fn safe_for_update(self) -> bool {
        matches!(self, Self::Quiet)
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Estimator {
    last_pulse_count: u32,
    last_capture_overruns: u16,
    state: State,
}

impl Estimator {
    pub const fn new() -> Self {
        Self {
            last_pulse_count: 0,
            last_capture_overruns: 0,
            state: State::Uninitialized,
        }
    }

    pub fn update(&mut self, capture: Capture) -> State {
        if capture.quiet {
            self.last_capture_overruns = capture.capture_overruns;
            self.last_pulse_count = capture.pulse_count;
            self.state = State::Quiet;
            return self.state;
        }
        if !capture.primed {
            self.state = State::Uninitialized;
            return self.state;
        }
        if capture.capture_overruns != self.last_capture_overruns {
            self.last_capture_overruns = capture.capture_overruns;
            self.last_pulse_count = capture.pulse_count;
            self.state = State::Implausible;
            return self.state;
        }
        if capture.pulse_count == self.last_pulse_count {
            return self.state;
        }
        self.last_pulse_count = capture.pulse_count;
        if capture.interval_us == 0 {
            self.state = State::AwaitingSecondPulse;
            return self.state;
        }
        if capture.interval_us < MINIMUM_INTERVAL_US {
            self.state = State::Implausible;
            return self.state;
        }
        let speed = u32::from(DISTANCE_PER_PULSE_MM)
            .saturating_mul(36_000)
            .checked_div(capture.interval_us)
            .unwrap_or_default()
            .min(u32::from(u16::MAX)) as u16;
        self.state = State::Tracking {
            speed_tenths_kph: speed,
        };
        self.state
    }

    pub const fn state(self) -> State {
        self.state
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct DistanceCounter {
    last_pulse_count: u32,
    distance_remainder_mm: u32,
    value: u32,
}

impl DistanceCounter {
    pub const fn new() -> Self {
        Self {
            last_pulse_count: 0,
            distance_remainder_mm: 0,
            value: 0,
        }
    }

    pub fn update(&mut self, pulse_count: u32) -> u32 {
        let new_pulses = pulse_count.wrapping_sub(self.last_pulse_count);
        self.last_pulse_count = pulse_count;
        let accumulated = new_pulses
            .saturating_mul(u32::from(DISTANCE_PER_PULSE_MM))
            .saturating_add(self.distance_remainder_mm);
        self.value = self.value.wrapping_add(accumulated / 100_000);
        self.distance_remainder_mm = accumulated % 100_000;
        self.value
    }

    pub const fn value(self) -> u32 {
        self.value
    }
}

pub const fn extended_capture_timestamp(
    overflow_epoch: u32,
    capture: u16,
    update_pending: bool,
) -> u32 {
    let epoch = if update_pending && capture < 0x8000 {
        overflow_epoch.wrapping_add(1)
    } else {
        overflow_epoch
    };
    epoch.wrapping_shl(16) | capture as u32
}

pub const fn qualified_interval_us(
    previous: u32,
    current: u32,
    was_primed: bool,
    was_quiet: bool,
) -> u32 {
    if was_primed && !was_quiet {
        current.wrapping_sub(previous)
    } else {
        0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn captured_period_converts_to_stock_speed_units() {
        let mut estimator = Estimator::new();
        let state = estimator.update(Capture {
            primed: true,
            interval_us: 166_667,
            pulse_count: 2,
            ..Capture::default()
        });
        assert_eq!(state.speed_tenths_kph(), 67);
    }

    #[test]
    fn update_requires_observed_quiet_and_two_new_pulses() {
        let mut estimator = Estimator::new();
        assert!(!estimator.state().safe_for_update());
        assert_eq!(
            estimator.update(Capture {
                quiet: true,
                ..Capture::default()
            }),
            State::Quiet
        );
        assert!(estimator.state().safe_for_update());
        assert_eq!(
            estimator.update(Capture {
                primed: true,
                pulse_count: 1,
                ..Capture::default()
            }),
            State::AwaitingSecondPulse
        );
        assert!(!estimator.state().safe_for_update());
    }

    #[test]
    fn overcapture_and_impossibly_short_intervals_are_explicit() {
        let mut estimator = Estimator::new();
        assert_eq!(
            estimator.update(Capture {
                primed: true,
                interval_us: 20_000,
                pulse_count: 2,
                capture_overruns: 1,
                ..Capture::default()
            }),
            State::Implausible
        );
        assert_eq!(
            estimator.update(Capture {
                primed: true,
                interval_us: MINIMUM_INTERVAL_US - 1,
                pulse_count: 3,
                capture_overruns: 1,
                ..Capture::default()
            }),
            State::Implausible
        );
    }

    #[test]
    fn distance_accumulates_fractional_stock_units() {
        let mut distance = DistanceCounter::new();
        assert_eq!(distance.update(320), 0);
        assert_eq!(distance.update(321), 1);
        assert_eq!(distance.update(641), 1);
        assert_eq!(distance.update(642), 2);
    }

    #[test]
    fn timestamp_extension_resolves_capture_overflow_order() {
        assert_eq!(extended_capture_timestamp(7, 0xff00, true), 0x0007_ff00);
        assert_eq!(extended_capture_timestamp(7, 0x0010, true), 0x0008_0010);
        assert_eq!(
            qualified_interval_us(0xffff_ff00, 0x0000_0100, true, false),
            0x200
        );
        assert_eq!(qualified_interval_us(1_000, 900_000, true, true), 0);
    }
}
