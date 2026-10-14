//! Slow fractional current-offset tracking while the inverter is undriven.

const FRACTION_BITS: u32 = 16;
const FILTER_SHIFT: u32 = 17;
const ROUNDING: u32 = 1 << (FRACTION_BITS - 1);

/// Bounds slow offset drift to a window around the boot calibration. Raw
/// undriven samples outside that window are counted up, in-window samples count
/// down, and the offsets are reported invalid from the moment the count reaches
/// its limit until it has decayed back to zero.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CurrentOffsetTracker {
    phase_a: u32,
    phase_b: u32,
    boot_a: u16,
    boot_b: u16,
    tolerance: u16,
    deviation_limit: u16,
    deviation_samples: u16,
    deviated: bool,
}

impl CurrentOffsetTracker {
    pub const fn new(phase_a: u16, phase_b: u16, tolerance: u16, deviation_limit: u16) -> Self {
        Self {
            phase_a: (phase_a as u32) << FRACTION_BITS,
            phase_b: (phase_b as u32) << FRACTION_BITS,
            boot_a: adc_sample(phase_a),
            boot_b: adc_sample(phase_b),
            tolerance,
            deviation_limit,
            deviation_samples: 0,
            deviated: false,
        }
    }

    pub fn observe_undriven(&mut self, raw_a: u16, raw_b: u16) {
        self.phase_a = observe(self.phase_a, raw_a, self.boot_a, self.tolerance);
        self.phase_b = observe(self.phase_b, raw_b, self.boot_b, self.tolerance);
        if raw_a.abs_diff(self.boot_a) > self.tolerance
            || raw_b.abs_diff(self.boot_b) > self.tolerance
        {
            self.deviation_samples = self
                .deviation_samples
                .saturating_add(1)
                .min(self.deviation_limit);
            if self.deviation_samples == self.deviation_limit {
                self.deviated = true;
            }
        } else if self.deviation_samples == 0 {
            self.deviated = false;
        } else {
            self.deviation_samples -= 1;
        }
    }

    pub const fn offsets(self) -> (u16, u16) {
        (sample(self.phase_a), sample(self.phase_b))
    }

    pub const fn offsets_valid(&self) -> bool {
        !self.deviated
    }
}

fn observe(estimate: u32, raw: u16, boot: u16, tolerance: u16) -> u32 {
    let target = i32::from(raw) << FRACTION_BITS;
    let estimate = estimate.min(i32::MAX as u32) as i32;
    let delta = target.saturating_sub(estimate);
    let correction = if delta >= 0 {
        delta.saturating_add((1 << FILTER_SHIFT) - 1) >> FILTER_SHIFT
    } else {
        delta >> FILTER_SHIFT
    };
    // The window is capped at the 12-bit ADC range before shifting, so a large
    // tolerance cannot wrap; `max`/`min` cannot panic even on an empty window.
    let minimum = i32::from(boot.saturating_sub(tolerance)) << FRACTION_BITS;
    let maximum = ((u32::from(boot) + u32::from(tolerance)).min(4_095) as i32) << FRACTION_BITS;
    estimate
        .saturating_add(correction)
        .max(minimum)
        .min(maximum) as u32
}

const fn adc_sample(value: u16) -> u16 {
    if value > 4_095 { 4_095 } else { value }
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
        let mut tracker = CurrentOffsetTracker::new(2_000, 2_001, 128, 1_600);
        tracker.observe_undriven(2_100, 1_900);
        assert_eq!(tracker.offsets(), (2_000, 2_001));
        for _ in 0..(1 << FILTER_SHIFT) {
            tracker.observe_undriven(2_100, 1_900);
        }
        let (phase_a, phase_b) = tracker.offsets();
        assert!((2_060..2_100).contains(&phase_a));
        assert!((1_900..1_940).contains(&phase_b));
    }

    #[test]
    fn tracked_offsets_stay_within_the_boot_tolerance() {
        let mut tracker = CurrentOffsetTracker::new(2_000, 300, 128, 4);
        for _ in 0..(8 << FILTER_SHIFT) {
            tracker.observe_undriven(4_095, 0);
        }
        assert_eq!(tracker.offsets(), (2_128, 172));

        let mut edge = CurrentOffsetTracker::new(4_000, 50, 128, 4);
        for _ in 0..(8 << FILTER_SHIFT) {
            edge.observe_undriven(4_095, 0);
        }
        assert_eq!(edge.offsets(), (4_095, 0));
    }

    #[test]
    fn unbounded_tolerance_spans_the_full_adc_range() {
        let mut tracker = CurrentOffsetTracker::new(2_000, 4_095, u16::MAX, 4);
        for _ in 0..(8 << FILTER_SHIFT) {
            tracker.observe_undriven(4_095, 0);
        }
        assert_eq!(tracker.offsets(), (4_095, 0));
        assert!(tracker.offsets_valid());
    }

    #[test]
    fn sustained_out_of_tolerance_samples_invalidate_until_they_decay() {
        let mut tracker = CurrentOffsetTracker::new(2_000, 2_000, 128, 4);
        assert!(tracker.offsets_valid());
        tracker.observe_undriven(2_128, 1_872);
        assert!(tracker.offsets_valid());
        for _ in 0..3 {
            tracker.observe_undriven(2_129, 2_000);
            assert!(tracker.offsets_valid());
        }
        tracker.observe_undriven(2_000, 1_871);
        assert!(!tracker.offsets_valid());
        for _ in 0..4 {
            tracker.observe_undriven(2_000, 2_000);
            assert!(!tracker.offsets_valid());
        }
        tracker.observe_undriven(2_000, 2_000);
        assert!(tracker.offsets_valid());
    }
}
