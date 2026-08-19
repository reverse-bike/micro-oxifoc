//! Calibrated Hall sensor and electrical-angle estimator in Q0.32 turns.

use crate::foc::numeric::Fixed;
use crate::foc::phase::{PhaseEstimate, PhaseProvider, PhaseSource};
use crate::foc::trig::Turns;

const MIN_EDGE_INTERVAL_US: u32 = 100;
const STALE_SECTOR_MULTIPLE: u32 = 8;
const RATE_LIMIT_MIN_ERPM: u32 = 500;
const HALF_MICROSECOND_NS: u32 = 500;
const TAU_NUMERATOR: u32 = 710;
const TAU_DENOMINATOR: u32 = 113;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct HallGeometry {
    electrical_states: [u8; 6],
    boundaries_q16: [u16; 6],
    sectors_by_raw: [HallSector; 8],
    positive_angle_direction: i8,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
struct HallSector {
    boundary_q16: u16,
    width_q16: u16,
    next_raw: u8,
    previous_raw: u8,
}

impl HallGeometry {
    pub const fn new(
        electrical_states: [u8; 6],
        boundaries_q16: [u16; 6],
        positive_angle_direction: i8,
    ) -> Self {
        assert!(
            positive_angle_direction == -1 || positive_angle_direction == 1,
            "positive Hall direction must be -1 or 1"
        );
        let mut index = 0;
        while index < electrical_states.len() {
            assert!(
                electrical_states[index] > 0 && electrical_states[index] < 7,
                "Hall states must be three-bit values excluding 0 and 7"
            );
            let mut previous = 0;
            while previous < index {
                assert!(
                    electrical_states[previous] != electrical_states[index],
                    "Hall states must be unique"
                );
                previous += 1;
            }
            if index > 0 {
                assert!(
                    boundaries_q16[index] > boundaries_q16[index - 1],
                    "Hall boundaries must be ordered within one Q0.16 turn"
                );
            }
            index += 1;
        }
        let mut sectors_by_raw = [HallSector {
            boundary_q16: 0,
            width_q16: 0,
            next_raw: 0,
            previous_raw: 0,
        }; 8];
        index = 0;
        while index < electrical_states.len() {
            let next_index = if index + 1 == electrical_states.len() {
                0
            } else {
                index + 1
            };
            let previous_index = if index == 0 {
                electrical_states.len() - 1
            } else {
                index - 1
            };
            sectors_by_raw[electrical_states[index] as usize] = HallSector {
                boundary_q16: boundaries_q16[index],
                width_q16: boundaries_q16[next_index].wrapping_sub(boundaries_q16[index]),
                next_raw: electrical_states[next_index],
                previous_raw: electrical_states[previous_index],
            };
            index += 1;
        }
        Self {
            electrical_states,
            boundaries_q16,
            sectors_by_raw,
            positive_angle_direction,
        }
    }

    pub const fn electrical_states(&self) -> &[u8; 6] {
        &self.electrical_states
    }

    pub const fn boundaries_q16(&self) -> &[u16; 6] {
        &self.boundaries_q16
    }

    pub const fn raw_is_valid(&self, raw: u8) -> bool {
        self.sector(raw).is_some()
    }

    pub const fn calibrated_center(&self, raw: u8) -> u16 {
        match self.sector(raw) {
            Some(sector) => sector.boundary_q16.wrapping_add(sector.width_q16 / 2),
            None => 0,
        }
    }

    const fn sector(&self, raw: u8) -> Option<HallSector> {
        if raw as usize >= self.sectors_by_raw.len() {
            return None;
        }
        let sector = self.sectors_by_raw[raw as usize];
        if sector.width_q16 == 0 {
            None
        } else {
            Some(sector)
        }
    }

    const fn signed_motion_direction(&self, angle_direction: i8) -> i8 {
        angle_direction.saturating_mul(self.positive_angle_direction)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HallError {
    InvalidState,
    InvalidTransition,
    EdgeTooFast,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct HallSensor {
    geometry: &'static HallGeometry,
    raw: u8,
    base_angle_q16: u16,
    measured_width_q16: u16,
    entered_width_q16: u16,
    entered_duration_us: u32,
    measured_interval_us: u32,
    direction: i8,
    next_interval_since_run_start: bool,
    rate_limited_angle: Turns,
    unlimited_angle: Turns,
    control_period_ns: u32,
    control_step_q16: u32,
    actuation_advance: Fixed,
    valid: bool,
}

impl HallSensor {
    pub const fn new(geometry: &'static HallGeometry) -> Self {
        Self {
            geometry,
            raw: 0,
            base_angle_q16: 0,
            measured_width_q16: 0,
            entered_width_q16: 0,
            entered_duration_us: 0,
            measured_interval_us: 0,
            direction: 0,
            next_interval_since_run_start: false,
            rate_limited_angle: 0,
            unlimited_angle: 0,
            control_period_ns: 0,
            control_step_q16: 0,
            actuation_advance: Fixed::ZERO,
            valid: false,
        }
    }

    pub fn seed(&mut self, raw: u8) -> Result<(), HallError> {
        let Some(sector) = self.geometry.sector(raw) else {
            self.valid = false;
            return Err(HallError::InvalidState);
        };
        self.raw = raw;
        self.base_angle_q16 = sector.boundary_q16.wrapping_add(sector.width_q16 / 2);
        self.measured_width_q16 = sector.width_q16;
        self.entered_width_q16 = sector.width_q16;
        self.entered_duration_us = 0;
        self.measured_interval_us = 0;
        self.direction = 0;
        self.next_interval_since_run_start = false;
        self.rate_limited_angle = u32::from(self.base_angle_q16) << 16;
        self.unlimited_angle = self.rate_limited_angle;
        self.control_step_q16 = 0;
        self.actuation_advance = Fixed::ZERO;
        self.valid = true;
        Ok(())
    }

    /// Marks the next captured interval as beginning at the stationary sector
    /// center when the ride lease starts, rather than at an earlier idle edge.
    pub fn discard_next_interval(&mut self) {
        self.next_interval_since_run_start = true;
    }

    pub fn update_edge(&mut self, raw: u8, interval_us: u32) -> Result<(), HallError> {
        let Some(entered_sector) = self.geometry.sector(raw) else {
            self.valid = false;
            return Err(HallError::InvalidState);
        };
        if !self.valid {
            return self.seed(raw);
        }
        if raw == self.raw {
            return Ok(());
        }

        let previous_raw = self.raw;
        let Some(previous_sector) = self.geometry.sector(previous_raw) else {
            self.valid = false;
            return Err(HallError::InvalidState);
        };
        let previous_angle_q16 = self.base_angle_q16;
        let previous_width = previous_sector.width_q16;
        let previous_direction = self.direction;
        let interval_since_run_start = self.next_interval_since_run_start;
        self.next_interval_since_run_start = false;
        let direction = if raw == previous_sector.previous_raw {
            -1
        } else if raw == previous_sector.next_raw {
            1
        } else {
            self.valid = false;
            return Err(HallError::InvalidTransition);
        };
        let boundary_q16 = if direction < 0 {
            entered_sector
                .boundary_q16
                .wrapping_add(entered_sector.width_q16)
        } else {
            entered_sector.boundary_q16
        };
        let (measured_interval_us, measured_width_q16) = if interval_since_run_start {
            if previous_direction == direction && self.measured_interval_us != 0 {
                (self.measured_interval_us, self.measured_width_q16)
            } else if previous_direction == 0 {
                let partial_width = if direction < 0 {
                    previous_angle_q16.wrapping_sub(boundary_q16)
                } else {
                    boundary_q16.wrapping_sub(previous_angle_q16)
                };
                if partial_width == 0 {
                    self.valid = false;
                    return Err(HallError::InvalidTransition);
                }
                let equivalent = scale_interval(interval_us, previous_width, partial_width);
                (equivalent, previous_width)
            } else {
                self.valid = false;
                return Err(HallError::InvalidTransition);
            }
        } else {
            (interval_us, previous_width)
        };
        if (previous_direction != 0 || interval_since_run_start)
            && measured_interval_us < MIN_EDGE_INTERVAL_US
        {
            self.valid = false;
            return Err(HallError::EdgeTooFast);
        }

        self.raw = raw;
        self.base_angle_q16 = boundary_q16;
        self.measured_width_q16 = measured_width_q16;
        self.entered_width_q16 = entered_sector.width_q16;
        if previous_direction == 0 && !interval_since_run_start {
            self.entered_duration_us = 0;
            self.measured_interval_us = 0;
        } else {
            self.measured_interval_us = measured_interval_us;
            self.entered_duration_us = scale_interval(
                self.measured_interval_us,
                self.entered_width_q16,
                self.measured_width_q16,
            )
            .max(1);
        }
        self.direction = direction;
        self.refresh_control_rate(self.control_period_ns);
        Ok(())
    }

    pub fn angle(&self, edge_age_us: u32) -> Option<Turns> {
        if !self.valid {
            return None;
        }
        if self.direction == 0 || self.entered_duration_us == 0 {
            return Some(u32::from(self.base_angle_q16) << 16);
        }

        let elapsed = edge_age_us;
        if elapsed
            > self
                .entered_duration_us
                .saturating_mul(STALE_SECTOR_MULTIPLE)
        {
            return None;
        }
        let scale_shift = (32 - self.entered_duration_us.leading_zeros()).saturating_sub(16);
        let scaled_elapsed = elapsed.min(self.entered_duration_us) >> scale_shift;
        let scaled_sector = self.entered_duration_us >> scale_shift;
        let fraction_q16 = (scaled_elapsed << 16) / scaled_sector.max(1);
        let advance_q16 = ((u32::from(self.entered_width_q16) * fraction_q16) >> 16) as u16;
        let angle_q16 = if self.direction < 0 {
            self.base_angle_q16.wrapping_sub(advance_q16)
        } else {
            self.base_angle_q16.wrapping_add(advance_q16)
        };
        Some(u32::from(angle_q16) << 16)
    }

    pub const fn is_valid(&self) -> bool {
        self.valid
    }

    pub const fn raw_state(&self) -> u8 {
        self.raw
    }

    pub const fn sector_interval_us(&self) -> u32 {
        self.measured_interval_us
    }

    pub const fn entered_duration_us(&self) -> u32 {
        self.entered_duration_us
    }

    /// Unconstrained Hall interpolation target from the latest control
    /// estimate. This remains separate from the angle actually supplied to
    /// the current loop so transition corrections can be diagnosed.
    pub const fn unlimited_angle(&self) -> Turns {
        self.unlimited_angle
    }

    /// Signed electrical-angle travel expected during one control period,
    /// expressed as Q16.16 radians for output-only actuation compensation.
    pub const fn actuation_advance(&self) -> Fixed {
        self.actuation_advance
    }

    pub fn electrical_rpm(&self) -> i32 {
        if self.direction == 0 || self.measured_interval_us == 0 {
            return 0;
        }
        let magnitude = self.electrical_rpm_magnitude();
        if self.direction < 0 {
            magnitude
        } else {
            -magnitude
        }
    }

    /// Signed speed in the calibrated electrical-angle coordinate.
    ///
    /// [`Self::electrical_rpm`] follows the configured physical forward
    /// direction for vehicle policy. Phase estimators instead need velocity
    /// to carry the same sign as the Q0.32 angle they integrate.
    pub fn angle_electrical_rpm(&self) -> i32 {
        let magnitude = self.electrical_rpm_magnitude();
        if self.direction < 0 {
            -magnitude
        } else {
            magnitude
        }
    }

    pub const fn physical_direction(&self) -> i8 {
        self.geometry.signed_motion_direction(self.direction)
    }

    /// Sign of motion in the electrical-angle coordinate used by the Park
    /// transforms: `-1` decreases Q0.32 turns, `1` increases it.
    pub const fn angle_direction(&self) -> i8 {
        self.direction
    }

    pub const fn is_stationary(&self) -> bool {
        self.direction == 0 || self.measured_interval_us == 0
    }

    fn electrical_rpm_magnitude(&self) -> i32 {
        if self.direction == 0 || self.measured_interval_us == 0 {
            return 0;
        }
        let rpm_q8 =
            u32::from(self.measured_width_q16).saturating_mul(234_375) / self.measured_interval_us;
        (rpm_q8 / 256).min(i32::MAX as u32) as i32
    }
}

/// Computes `interval * numerator / denominator` exactly with saturating
/// `u32` output while keeping division on the Cortex-M3's native word size.
#[inline(never)]
fn scale_interval(interval: u32, numerator: u16, denominator: u16) -> u32 {
    let denominator = u32::from(denominator);
    if denominator == 0 {
        return 0;
    }
    let numerator = u32::from(numerator);
    let quotient = interval / denominator;
    let remainder = interval - quotient * denominator;
    quotient
        .saturating_mul(numerator)
        .saturating_add(remainder * numerator / denominator)
}

impl PhaseProvider<Fixed> for HallSensor {
    type Angle = Turns;

    fn source(&self) -> PhaseSource {
        PhaseSource::Hall
    }

    fn estimate(&self, elapsed_since_edge_us: u32) -> Option<PhaseEstimate<Turns>> {
        self.angle(elapsed_since_edge_us)
            .map(|angle| PhaseEstimate {
                angle,
                electrical_rpm: self.angle_electrical_rpm(),
                trustworthy: self.is_valid(),
            })
    }

    fn estimate_for_control(
        &mut self,
        elapsed_since_edge_us: u32,
        control_period_ns: u32,
    ) -> Option<PhaseEstimate<Turns>> {
        if control_period_ns != self.control_period_ns {
            self.refresh_control_rate(control_period_ns);
        }
        let target = self.angle(elapsed_since_edge_us)?;
        self.unlimited_angle = target;
        let electrical_rpm = self.angle_electrical_rpm();
        if electrical_rpm.unsigned_abs() < RATE_LIMIT_MIN_ERPM || self.measured_interval_us == 0 {
            self.rate_limited_angle = target;
        } else {
            let maximum_step = self.maximum_control_step().clamp(1, i32::MAX as u32) as i32;
            let desired_step = target.wrapping_sub(self.rate_limited_angle) as i32;
            let limited_step = desired_step.clamp(-maximum_step, maximum_step);
            self.rate_limited_angle = self.rate_limited_angle.wrapping_add(limited_step as u32);
        }
        Some(PhaseEstimate {
            angle: self.rate_limited_angle,
            electrical_rpm,
            trustworthy: self.is_valid(),
        })
    }
}

impl HallSensor {
    fn refresh_control_rate(&mut self, control_period_ns: u32) {
        self.control_period_ns = control_period_ns;
        if self.direction == 0 || self.measured_interval_us == 0 || control_period_ns == 0 {
            self.control_step_q16 = 0;
            self.actuation_advance = Fixed::ZERO;
            return;
        }
        let half_microseconds =
            control_period_ns.saturating_add(HALF_MICROSECOND_NS / 2) / HALF_MICROSECOND_NS;
        self.control_step_q16 = u32::from(self.measured_width_q16)
            .saturating_mul(half_microseconds)
            / self.measured_interval_us.saturating_mul(2).max(1);
        let radians_q16 = self.control_step_q16.saturating_mul(TAU_NUMERATOR) / TAU_DENOMINATOR;
        let magnitude = radians_q16.min(i32::MAX as u32) as i32;
        self.actuation_advance = Fixed::from_bits(if self.direction < 0 {
            magnitude.saturating_neg()
        } else {
            magnitude
        });
    }

    fn maximum_control_step(&self) -> u32 {
        let rate_limited_step_q16 = self
            .control_step_q16
            .saturating_add(self.control_step_q16 / 2);
        if rate_limited_step_q16 >= 1 << 15 {
            i32::MAX as u32
        } else {
            rate_limited_step_q16 << 16
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    static TEST_GEOMETRY: HallGeometry = HallGeometry::new(
        [5, 1, 3, 2, 6, 4],
        [5_699, 16_526, 26_499, 37_754, 49_151, 59_124],
        -1,
    );

    fn tracker() -> HallSensor {
        HallSensor::new(&TEST_GEOMETRY)
    }

    #[test]
    fn stationary_seed_uses_the_recovered_sector_center() {
        let mut hall = tracker();
        hall.seed(5).unwrap();
        assert_eq!(hall.angle(40_000), Some(u32::from(11_112_u16) << 16));
    }

    #[test]
    fn physical_forward_sequence_decreases_calibrated_angle() {
        let mut hall = tracker();
        hall.seed(5).unwrap();
        hall.update_edge(4, 1_000).unwrap();
        hall.update_edge(6, 1_000).unwrap();
        let edge = hall.angle(0).unwrap();
        let halfway = hall.angle(500).unwrap();
        assert!(edge.wrapping_sub(halfway) > 0);
        assert!(hall.electrical_rpm() > 0);
        assert!(hall.angle_electrical_rpm() < 0);
        assert_eq!(
            hall.estimate(500).unwrap().electrical_rpm,
            hall.angle_electrical_rpm()
        );
        for raw in [2, 3, 1, 5] {
            hall.update_edge(raw, 1_000).unwrap();
        }
        assert_eq!(hall.raw_state(), 5);
        assert!(hall.is_valid());
    }

    #[test]
    fn physical_reverse_sequence_increases_calibrated_angle() {
        let mut hall = tracker();
        hall.seed(5).unwrap();
        hall.update_edge(1, 1_000).unwrap();
        hall.update_edge(3, 1_000).unwrap();
        assert!(
            hall.angle(500)
                .unwrap()
                .wrapping_sub(hall.angle(0).unwrap())
                > 0
        );
        assert!(hall.electrical_rpm() < 0);
        assert!(hall.angle_electrical_rpm() > 0);
    }

    #[test]
    fn invalid_transition_revokes_angle() {
        let mut hall = tracker();
        hall.seed(5).unwrap();
        assert_eq!(hall.update_edge(2, 100), Err(HallError::InvalidTransition));
        assert_eq!(hall.angle(100), None);
    }

    #[test]
    fn moving_estimate_eventually_becomes_stale() {
        let mut hall = tracker();
        hall.seed(5).unwrap();
        hall.update_edge(4, 1_000).unwrap();
        hall.update_edge(6, 1_000).unwrap();
        let stale_after = hall
            .entered_duration_us()
            .saturating_mul(STALE_SECTOR_MULTIPLE);
        assert!(hall.angle(stale_after).is_some());
        assert_eq!(hall.angle(stale_after + 1), None);
    }

    #[test]
    fn unequal_sectors_reach_the_entered_boundary_without_stopping_early() {
        let mut hall = tracker();
        hall.seed(5).unwrap();
        hall.update_edge(4, 1_000).unwrap();
        hall.update_edge(6, 1_000).unwrap();
        let end = hall.angle(hall.entered_duration_us()).unwrap();
        assert_eq!(
            end,
            u32::from(TEST_GEOMETRY.sector(6).unwrap().boundary_q16) << 16
        );
    }

    #[test]
    fn control_estimate_limits_an_early_edge_correction() {
        const CONTROL_PERIOD_NS: u32 = 62_500;

        let mut hall = tracker();
        hall.seed(5).unwrap();
        hall.update_edge(4, 1_000).unwrap();
        let _ = hall.estimate_for_control(0, CONTROL_PERIOD_NS).unwrap();
        hall.update_edge(6, 1_000).unwrap();
        for edge_age_us in [0, 63, 125, 188, 250, 313, 375, 438, 500] {
            let _ = hall
                .estimate_for_control(edge_age_us, CONTROL_PERIOD_NS)
                .unwrap();
        }
        let before = hall
            .estimate_for_control(500, CONTROL_PERIOD_NS)
            .unwrap()
            .angle;

        hall.update_edge(2, 500).unwrap();
        let unlimited = hall.angle(0).unwrap();
        let limited = hall
            .estimate_for_control(0, CONTROL_PERIOD_NS)
            .unwrap()
            .angle;
        let limited_step = limited.wrapping_sub(before) as i32;
        let unlimited_step = unlimited.wrapping_sub(before) as i32;

        assert!(limited_step.unsigned_abs() < unlimited_step.unsigned_abs());
        assert_eq!(hall.unlimited_angle(), unlimited);
    }

    #[test]
    fn low_speed_control_estimate_preserves_the_unlimited_angle() {
        let mut hall = tracker();
        hall.seed(5).unwrap();
        hall.update_edge(4, 200_000).unwrap();
        hall.update_edge(6, 200_000).unwrap();

        let unlimited = hall.angle(50_000).unwrap();
        let limited = hall.estimate_for_control(50_000, 62_500).unwrap().angle;

        assert_eq!(limited, unlimited);
    }

    #[test]
    fn actuation_advance_follows_the_signed_angle_motion() {
        let mut hall = tracker();
        hall.seed(5).unwrap();
        hall.update_edge(4, 1_000).unwrap();
        hall.update_edge(6, 1_000).unwrap();

        let _ = hall.estimate_for_control(0, 62_500).unwrap();
        let advance = hall.actuation_advance();
        assert!(advance.to_bits() < 0);
        assert!((3_500..=5_000).contains(&advance.to_bits().unsigned_abs()));
    }

    #[test]
    fn first_run_edge_derives_a_complete_interval_from_the_sector_center() {
        let mut hall = tracker();
        hall.seed(5).unwrap();
        hall.discard_next_interval();
        hall.update_edge(4, 5_000).unwrap();

        assert!((9_900..=10_100).contains(&hall.sector_interval_us()));
        assert!((950..=1_050).contains(&hall.electrical_rpm()));
        assert!(hall.angle(5_000).unwrap() != hall.angle(0).unwrap());
    }

    #[test]
    fn interval_scaling_matches_the_wide_reference_without_overflow() {
        let intervals = [0, 1, 999, 65_535, 1_000_000, u32::MAX];
        let numerators = [1, 9_973, 11_397, u16::MAX];
        let denominators = [1, 5_699, 9_973, u16::MAX];

        for interval in intervals {
            for numerator in numerators {
                for denominator in denominators {
                    let expected = (u64::from(interval) * u64::from(numerator)
                        / u64::from(denominator))
                    .min(u64::from(u32::MAX)) as u32;
                    assert_eq!(
                        scale_interval(interval, numerator, denominator),
                        expected,
                        "interval={interval} numerator={numerator} denominator={denominator}"
                    );
                }
            }
        }
        assert_eq!(scale_interval(1_000, 5_699, 0), 0);
    }

    #[test]
    fn geometry_uses_the_supplied_sequence_and_boundaries() {
        assert_eq!(TEST_GEOMETRY.electrical_states(), &[5, 1, 3, 2, 6, 4]);
        assert_eq!(
            TEST_GEOMETRY.boundaries_q16(),
            &[5_699, 16_526, 26_499, 37_754, 49_151, 59_124]
        );
        assert_eq!(TEST_GEOMETRY.calibrated_center(5), 11_112);
    }

    #[test]
    fn geometry_indexes_calibration_directly_by_raw_state() {
        let sector = TEST_GEOMETRY.sector(5).unwrap();
        assert_eq!(sector.boundary_q16, 5_699);
        assert_eq!(sector.width_q16, 10_827);
        assert_eq!(sector.next_raw, 1);
        assert_eq!(sector.previous_raw, 4);

        let wrapping_sector = TEST_GEOMETRY.sector(4).unwrap();
        assert_eq!(wrapping_sector.boundary_q16, 59_124);
        assert_eq!(wrapping_sector.width_q16, 12_111);
        assert_eq!(wrapping_sector.next_raw, 5);
        assert_eq!(wrapping_sector.previous_raw, 6);

        assert!(TEST_GEOMETRY.sector(0).is_none());
        assert!(TEST_GEOMETRY.sector(7).is_none());
        assert!(TEST_GEOMETRY.sector(8).is_none());
    }
}
