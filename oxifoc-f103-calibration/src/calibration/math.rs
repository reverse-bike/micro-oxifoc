//! Integer helpers shared by the calibration sequences.

pub use oxifoc_core::foc::numeric::{integer_sqrt_u64, saturating_i64_to_i32};

/// Divides and rounds half away from zero.
pub fn div_round(value: i64, divisor: i64) -> i64 {
    if value < 0 {
        (value - divisor / 2) / divisor
    } else {
        (value + divisor / 2) / divisor
    }
}

pub fn saturating_i64_to_i16(value: i64) -> i16 {
    value.clamp(i64::from(i16::MIN), i64::from(i16::MAX)) as i16
}

/// Linear ramp from `start` to `end`, holding `end` once `cycle` reaches
/// `duration`.
pub fn ramp_i32(start: i32, end: i32, cycle: u32, duration: u32) -> i32 {
    let value = i64::from(start)
        + (i64::from(end) - i64::from(start)) * i64::from(cycle.min(duration))
            / i64::from(duration);
    saturating_i64_to_i32(value)
}

pub fn ramp_i16(start: i16, end: i16, cycle: u32, duration: u32) -> i16 {
    // The ramp stays between its i16 endpoints.
    ramp_i32(i32::from(start), i32::from(end), cycle, duration) as i16
}

/// A held current is settled within 30% of its target, or two counts.
pub fn settled(measured: i16, target: i16) -> bool {
    (i32::from(measured) - i32::from(target)).unsigned_abs()
        <= (i32::from(target).unsigned_abs() * 3 / 10).max(2)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn division_rounds_half_away_from_zero() {
        assert_eq!(div_round(5, 2), 3);
        assert_eq!(div_round(-5, 2), -3);
        assert_eq!(div_round(4, 3), 1);
        assert_eq!(div_round(-4, 3), -1);
    }

    #[test]
    fn ramps_hold_their_end_point() {
        assert_eq!(ramp_i16(0, 100, 0, 10), 0);
        assert_eq!(ramp_i16(0, 100, 5, 10), 50);
        assert_eq!(ramp_i16(100, -100, 50, 10), -100);
        assert_eq!(ramp_i32(i32::MAX, i32::MIN, 1, 2), 0);
    }

    #[test]
    fn settling_tolerance_is_thirty_percent_with_a_two_count_floor() {
        assert!(settled(70, 100));
        assert!(!settled(69, 100));
        assert!(settled(-2, 0));
        assert!(!settled(3, 0));
        assert!(settled(-130, -100));
    }
}
