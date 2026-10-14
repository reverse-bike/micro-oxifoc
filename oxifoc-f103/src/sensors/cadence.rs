//! Qualification and speed measurement for the stock 24-pulse cadence sensor.

use crate::config::BIKE_CONFIG;

pub const DEBOUNCE_SAMPLES: u8 = BIKE_CONFIG.cadence.debounce_milliseconds;
pub const FAST_TIMEOUT_THRESHOLD_TENTHS_KPH: u16 =
    BIKE_CONFIG.cadence.fast_timeout_threshold_kph * 10;
pub const STOCK_COMMAND_MAXIMUM: u16 = 2_500;
pub const STOCK_COMMAND_NUMERATOR: u32 = 41_074;
pub const STOCK_COMMAND_OFFSET: u32 = 67;

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum State {
    #[default]
    Inactive,
    Priming,
    Active,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Observation {
    pub state: State,
    pub rpm: u16,
    pub demand_percent: u8,
    pub raw_pin_low: bool,
    pub pin_low: bool,
}

impl Observation {
    pub const INACTIVE: Self = Self {
        state: State::Inactive,
        rpm: 0,
        demand_percent: 0,
        raw_pin_low: false,
        pin_low: false,
    };

    pub const fn is_active(self) -> bool {
        matches!(self.state, State::Active)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Tracker {
    debounced_low: bool,
    matching_samples: u8,
    phase_started_ms: u32,
    last_pulse_ms: u32,
    previous_low_ms: u16,
    qualifying_pulses: u8,
    rpm: u16,
    demand_percent: u8,
}

impl Tracker {
    pub const fn new() -> Self {
        Self {
            debounced_low: false,
            matching_samples: 0,
            phase_started_ms: 0,
            last_pulse_ms: 0,
            previous_low_ms: 0,
            qualifying_pulses: 0,
            rpm: 0,
            demand_percent: 0,
        }
    }

    pub fn update(
        &mut self,
        now_ms: u32,
        pin_low: bool,
        vehicle_speed_tenths_kph: u16,
    ) -> Observation {
        if pin_low == self.debounced_low {
            self.matching_samples = 0;
        } else {
            self.matching_samples = self.matching_samples.saturating_add(1);
            if self.matching_samples >= DEBOUNCE_SAMPLES {
                let previous_low = self.debounced_low;
                let phase_duration_ms = now_ms.wrapping_sub(self.phase_started_ms);
                self.debounced_low = pin_low;
                self.matching_samples = 0;
                self.phase_started_ms = now_ms;
                if previous_low {
                    self.observe_low_phase(now_ms, phase_duration_ms);
                }
            }
        }

        let timeout_ms = if vehicle_speed_tenths_kph <= FAST_TIMEOUT_THRESHOLD_TENTHS_KPH {
            BIKE_CONFIG.cadence.slow_timeout_milliseconds
        } else {
            BIKE_CONFIG.cadence.fast_timeout_milliseconds
        };
        if now_ms.wrapping_sub(self.phase_started_ms) > timeout_ms {
            self.reset_qualification();
        }

        Observation {
            state: if self.qualifying_pulses >= BIKE_CONFIG.cadence.qualifying_pulses {
                State::Active
            } else if self.qualifying_pulses != 0 {
                State::Priming
            } else {
                State::Inactive
            },
            rpm: self.rpm,
            demand_percent: if self.qualifying_pulses >= BIKE_CONFIG.cadence.qualifying_pulses {
                self.demand_percent
            } else {
                0
            },
            raw_pin_low: pin_low,
            pin_low: self.debounced_low,
        }
    }

    fn observe_low_phase(&mut self, now_ms: u32, duration_ms: u32) {
        if duration_ms > BIKE_CONFIG.cadence.maximum_low_milliseconds {
            self.reset_qualification();
            return;
        }

        let low_ms =
            duration_ms.max(u32::from(BIKE_CONFIG.cadence.full_demand_low_milliseconds)) as u16;
        if self.qualifying_pulses != 0 {
            let denominator = now_ms
                .wrapping_sub(self.last_pulse_ms)
                .saturating_mul(BIKE_CONFIG.cadence.pulses_per_revolution);
            if denominator != 0 {
                self.rpm = 60_000_u32
                    .checked_div(denominator)
                    .unwrap_or_default()
                    .min(u32::from(u16::MAX)) as u16;
            }
            let average_low_ms = (u32::from(self.previous_low_ms) + u32::from(low_ms)) / 2;
            self.demand_percent = demand_percent_from_low_ms(average_low_ms as u16);
        }
        self.last_pulse_ms = now_ms;
        self.previous_low_ms = low_ms;
        self.qualifying_pulses = self.qualifying_pulses.saturating_add(1);
        if self.qualifying_pulses >= BIKE_CONFIG.cadence.qualifying_pulses {
            self.qualifying_pulses = BIKE_CONFIG.cadence.qualifying_pulses;
        }
    }

    fn reset_qualification(&mut self) {
        self.qualifying_pulses = 0;
        self.previous_low_ms = 0;
        self.rpm = 0;
        self.demand_percent = 0;
    }
}

pub const fn demand_percent_from_low_ms(low_ms: u16) -> u8 {
    if low_ms == 0 || low_ms as u32 > BIKE_CONFIG.cadence.maximum_low_milliseconds {
        return 0;
    }
    let command = if low_ms <= BIKE_CONFIG.cadence.full_demand_low_milliseconds {
        STOCK_COMMAND_MAXIMUM as u32
    } else {
        let derived = STOCK_COMMAND_NUMERATOR / low_ms as u32 + STOCK_COMMAND_OFFSET;
        if derived > STOCK_COMMAND_MAXIMUM as u32 {
            STOCK_COMMAND_MAXIMUM as u32
        } else {
            derived
        }
    };
    ((command + 12) / 25) as u8
}

impl Default for Tracker {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_phase(
        tracker: &mut Tracker,
        now_ms: &mut u32,
        low: bool,
        samples: u32,
        speed_tenths_kph: u16,
    ) -> Observation {
        let mut observation = Observation::default();
        for _ in 0..samples {
            *now_ms = now_ms.wrapping_add(1);
            observation = tracker.update(*now_ms, low, speed_tenths_kph);
        }
        observation
    }

    fn pulse(tracker: &mut Tracker, now_ms: &mut u32, low_ms: u32, high_ms: u32) -> Observation {
        sample_phase(tracker, now_ms, true, low_ms, 0);
        sample_phase(tracker, now_ms, false, high_ms, 0)
    }

    #[test]
    fn three_debounced_pulses_are_required_for_assist() {
        let mut tracker = Tracker::new();
        let mut now = 0;
        assert_eq!(pulse(&mut tracker, &mut now, 40, 40).state, State::Priming);
        assert_eq!(pulse(&mut tracker, &mut now, 40, 40).state, State::Priming);
        let observation = pulse(&mut tracker, &mut now, 40, 40);
        assert!(observation.is_active());
        assert_eq!(observation.rpm, 31);
        assert_eq!(observation.demand_percent, 44);
    }

    #[test]
    fn stock_low_width_curve_maps_to_a_bounded_percentage() {
        assert_eq!(demand_percent_from_low_ms(0), 0);
        assert_eq!(demand_percent_from_low_ms(400), 7);
        assert_eq!(demand_percent_from_low_ms(125), 16);
        assert_eq!(demand_percent_from_low_ms(63), 29);
        assert_eq!(demand_percent_from_low_ms(42), 42);
        assert_eq!(demand_percent_from_low_ms(31), 56);
        assert_eq!(demand_percent_from_low_ms(21), 81);
        assert_eq!(demand_percent_from_low_ms(17), 99);
        assert_eq!(demand_percent_from_low_ms(16), 100);
        assert_eq!(demand_percent_from_low_ms(8), 100);
        assert_eq!(demand_percent_from_low_ms(401), 0);
    }

    #[test]
    fn bounce_is_rejected_and_a_stuck_low_phase_resets_qualification() {
        let mut tracker = Tracker::new();
        let mut now = 0;
        for _ in 0..8 {
            sample_phase(&mut tracker, &mut now, true, 3, 0);
            sample_phase(&mut tracker, &mut now, false, 3, 0);
        }
        assert_eq!(tracker.update(now, false, 0).state, State::Inactive);

        for _ in 0..3 {
            pulse(&mut tracker, &mut now, 8, 12);
        }
        assert!(tracker.update(now, false, 0).is_active());
        assert!(!pulse(&mut tracker, &mut now, 410, 40).is_active());
        assert_eq!(tracker.update(now, false, 0).demand_percent, 0);
    }

    #[test]
    fn timeout_shortens_above_twenty_kilometers_per_hour() {
        let mut slow = Tracker::new();
        let mut fast = Tracker::new();
        let mut slow_now = 0;
        let mut fast_now = 0;
        for _ in 0..3 {
            pulse(&mut slow, &mut slow_now, 40, 40);
            pulse(&mut fast, &mut fast_now, 40, 40);
        }
        assert!(sample_phase(&mut slow, &mut slow_now, false, 301, 200).is_active());
        assert!(!sample_phase(&mut fast, &mut fast_now, false, 301, 201).is_active());
        assert!(!sample_phase(&mut slow, &mut slow_now, false, 200, 200).is_active());
    }
}
