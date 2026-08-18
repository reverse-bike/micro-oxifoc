//! Slow fractional current-offset tracking while the inverter is undriven.

const FRACTION_BITS: u32 = 16;
const FILTER_SHIFT: u32 = 17;
const ROUNDING: u32 = 1 << (FRACTION_BITS - 1);
const MAX_ESTIMATE: u32 = 4_095 << FRACTION_BITS;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CurrentOffsetTracker {
    phase_a: u32,
    phase_b: u32,
}

impl CurrentOffsetTracker {
    pub const fn new(phase_a: u16, phase_b: u16) -> Self {
        Self {
            phase_a: (phase_a as u32) << FRACTION_BITS,
            phase_b: (phase_b as u32) << FRACTION_BITS,
        }
    }

    pub fn observe_undriven(&mut self, raw_a: u16, raw_b: u16) {
        self.phase_a = observe(self.phase_a, raw_a);
        self.phase_b = observe(self.phase_b, raw_b);
    }

    pub const fn offsets(self) -> (u16, u16) {
        (sample(self.phase_a), sample(self.phase_b))
    }
}

fn observe(estimate: u32, raw: u16) -> u32 {
    let target = i32::from(raw) << FRACTION_BITS;
    let estimate = estimate.min(i32::MAX as u32) as i32;
    let delta = target.saturating_sub(estimate);
    let correction = if delta >= 0 {
        delta.saturating_add((1 << FILTER_SHIFT) - 1) >> FILTER_SHIFT
    } else {
        delta >> FILTER_SHIFT
    };
    estimate
        .saturating_add(correction)
        .max(0)
        .min(MAX_ESTIMATE as i32) as u32
}

const fn sample(estimate: u32) -> u16 {
    let rounded = estimate.saturating_add(ROUNDING) >> FRACTION_BITS;
    if rounded > 4_095 {
        4_095
    } else {
        rounded as u16
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tracking_is_slow_and_retains_fractional_progress() {
        let mut tracker = CurrentOffsetTracker::new(2_000, 2_001);
        tracker.observe_undriven(2_100, 1_900);
        assert_eq!(tracker.offsets(), (2_000, 2_001));
        for _ in 0..(1 << FILTER_SHIFT) {
            tracker.observe_undriven(2_100, 1_900);
        }
        let (phase_a, phase_b) = tracker.offsets();
        assert!((2_060..2_100).contains(&phase_a));
        assert!((1_900..1_940).contains(&phase_b));
    }
}
