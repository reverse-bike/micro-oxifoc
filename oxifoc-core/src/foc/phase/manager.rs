//! Phase-source ownership, validation, and crossover for synchronous FOC.
//!
//! The manager is the boundary OxiFOC uses between physical angle sensors,
//! software observers, and the motor driver. The F103 configuration installs a
//! Hall provider plus [`BackEmfObserver`], selecting the established
//! Hall-to-observer blend strategy without putting estimator policy in the
//! interrupt or Hall implementation.

use crate::foc::numeric::Fixed;

use super::observer::signed_angle_difference;
use super::{
    BackEmfObserver, PhaseEstimate, PhaseInput, PhaseProvider, PhaseSource, PhaseSourceError,
};
use crate::foc::trig::Turns;

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct ObserverDiagnostics {
    pub configured: bool,
    pub ready: bool,
    pub active: bool,
    pub blend: Fixed,
    pub electrical_rpm: i32,
    /// Observer minus Hall as a signed Q0.32 turn difference.
    pub hall_error_q32: i32,
    pub confidence: Fixed,
    pub flux_magnitude_mwb: Fixed,
    pub bemf_q_v: Fixed,
    pub phase_error_filtered_q32: u32,
    pub validity_progress: u8,
}

/// Owns the installed Hall sensor and optional back-EMF estimator.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PhaseManager<H> {
    hall: H,
    observer: Option<BackEmfObserver>,
    source: PhaseSource,
    observer_seed_requested: bool,
    last_observer_blend: Fixed,
    last_observer_hall_error_q32: i32,
}

impl<H> PhaseManager<H> {
    pub const fn with_hall(hall: H) -> Self {
        Self {
            hall,
            observer: None,
            source: PhaseSource::Hall,
            observer_seed_requested: false,
            last_observer_blend: Fixed::ZERO,
            last_observer_hall_error_q32: 0,
        }
    }

    pub const fn source(&self) -> PhaseSource {
        self.source
    }

    pub const fn hall(&self) -> &H {
        &self.hall
    }

    pub fn hall_mut(&mut self) -> &mut H {
        &mut self.hall
    }

    pub const fn observer(&self) -> Option<&BackEmfObserver> {
        self.observer.as_ref()
    }

    pub fn observer_mut(&mut self) -> Option<&mut BackEmfObserver> {
        self.observer.as_mut()
    }

    pub fn set_observer(&mut self, observer: BackEmfObserver) {
        self.observer = Some(observer);
    }

    pub fn reset_observer(&mut self) {
        if let Some(observer) = &mut self.observer {
            observer.reset();
        }
        self.last_observer_blend = Fixed::ZERO;
        self.last_observer_hall_error_q32 = 0;
    }

    /// Seed the estimator from the next Hall estimate before source blending.
    pub fn request_observer_seed(&mut self) {
        self.observer_seed_requested = true;
    }

    pub fn observer_diagnostics(&self) -> ObserverDiagnostics {
        let Some(observer) = self.observer.as_ref() else {
            return ObserverDiagnostics::default();
        };
        ObserverDiagnostics {
            configured: true,
            ready: observer.is_ready(),
            active: self.last_observer_blend > Fixed::ZERO,
            blend: self.last_observer_blend,
            electrical_rpm: observer.electrical_rpm(),
            hall_error_q32: self.last_observer_hall_error_q32,
            confidence: observer.confidence(),
            flux_magnitude_mwb: observer.flux_magnitude_mwb(),
            bemf_q_v: observer.bemf_q_filtered_v(),
            phase_error_filtered_q32: observer.phase_error_filtered_q32(),
            validity_progress: observer.validity_progress(),
        }
    }

    pub fn set_source(&mut self, source: PhaseSource) -> Result<(), PhaseSourceError> {
        let result = match source {
            PhaseSource::Hall => Ok(()),
            PhaseSource::Observer | PhaseSource::HallToObserver { .. } => {
                if self.observer.is_some() {
                    Ok(())
                } else {
                    Err(PhaseSourceError::ObserverNotConfigured)
                }
            }
            PhaseSource::Encoder | PhaseSource::EncoderToObserver { .. } => {
                Err(PhaseSourceError::EncoderNotAvailable)
            }
            PhaseSource::Hfi
            | PhaseSource::HfiToObserver { .. }
            | PhaseSource::HfiToObserverVolts { .. }
            | PhaseSource::HfiToHall { .. }
            | PhaseSource::HfiToEncoder { .. } => Err(PhaseSourceError::HfiNotConfigured),
            PhaseSource::Manual | PhaseSource::OpenLoop => {
                Err(PhaseSourceError::ManualSourceNotConfigured)
            }
        };
        if result.is_ok() {
            self.source = source;
            self.last_observer_blend = Fixed::ZERO;
            self.last_observer_hall_error_q32 = 0;
        }
        result
    }

    fn observer_estimate(&self) -> Option<PhaseEstimate<Turns>> {
        let observer = self.observer.as_ref()?;
        Some(PhaseEstimate {
            angle: observer.phase(),
            electrical_rpm: observer.electrical_rpm(),
            trustworthy: observer.is_ready(),
        })
    }

    fn select_estimate(
        &mut self,
        hall: Option<PhaseEstimate<Turns>>,
    ) -> Option<PhaseEstimate<Turns>> {
        if self.observer_seed_requested {
            if let (Some(sensor), Some(observer)) = (hall, &mut self.observer) {
                observer.seed(sensor.angle, sensor.electrical_rpm);
            }
            self.observer_seed_requested = false;
        }

        let observer = self.observer_estimate();
        let mut blend = Fixed::ZERO;
        let mut hall_error_q32 = match (hall, observer) {
            (Some(sensor), Some(estimate)) => signed_angle_difference(estimate.angle, sensor.angle),
            _ => 0,
        };
        let output = match self.source {
            PhaseSource::Hall => hall,
            PhaseSource::Observer => match observer {
                Some(estimate) if estimate.trustworthy => {
                    blend = Fixed::ONE;
                    Some(estimate)
                }
                _ => None,
            },
            PhaseSource::HallToObserver {
                blend_low_erpm,
                blend_high_erpm,
            } => match (hall, observer) {
                (Some(sensor), Some(estimate)) if estimate.trustworthy => {
                    blend = crossover_blend(
                        sensor
                            .electrical_rpm
                            .unsigned_abs()
                            .max(estimate.electrical_rpm.unsigned_abs()),
                        blend_low_erpm,
                        blend_high_erpm,
                    );
                    let error = signed_angle_difference(estimate.angle, sensor.angle);
                    // OxiFOC's half-turn ambiguity guard: while Hall retains
                    // any authority, a >90-degree disagreement is a reseed,
                    // never a command to blend through zero torque.
                    if blend < Fixed::ONE && error.unsigned_abs() > 0x4000_0000 {
                        if let Some(observer) = &mut self.observer {
                            observer.seed(sensor.angle, sensor.electrical_rpm);
                        }
                        blend = Fixed::ZERO;
                        hall_error_q32 = 0;
                        Some(sensor)
                    } else if blend == Fixed::ZERO {
                        Some(sensor)
                    } else if blend == Fixed::ONE {
                        Some(estimate)
                    } else {
                        Some(blend_estimates(sensor, estimate, blend))
                    }
                }
                (Some(sensor), _) => Some(sensor),
                (None, Some(estimate)) if estimate.trustworthy => {
                    blend = Fixed::ONE;
                    Some(estimate)
                }
                _ => None,
            },
            _ => hall,
        };
        self.last_observer_blend = blend;
        self.last_observer_hall_error_q32 = hall_error_q32;
        output
    }
}

impl<H> PhaseProvider for PhaseManager<H>
where
    H: PhaseProvider<Fixed, Angle = Turns>,
{
    type Angle = Turns;

    fn source(&self) -> PhaseSource {
        self.source
    }

    fn estimate(&self, elapsed_since_observation_us: u32) -> Option<PhaseEstimate<Turns>> {
        let hall = self.hall.estimate(elapsed_since_observation_us);
        match self.source {
            PhaseSource::Hall => hall,
            PhaseSource::Observer => self
                .observer_estimate()
                .filter(|estimate| estimate.trustworthy),
            PhaseSource::HallToObserver {
                blend_low_erpm,
                blend_high_erpm,
            } => match (hall, self.observer_estimate()) {
                (Some(sensor), Some(observer)) if observer.trustworthy => {
                    let blend = crossover_blend(
                        sensor
                            .electrical_rpm
                            .unsigned_abs()
                            .max(observer.electrical_rpm.unsigned_abs()),
                        blend_low_erpm,
                        blend_high_erpm,
                    );
                    if blend == Fixed::ZERO {
                        Some(sensor)
                    } else if blend == Fixed::ONE {
                        Some(observer)
                    } else {
                        Some(blend_estimates(sensor, observer, blend))
                    }
                }
                (Some(sensor), _) => Some(sensor),
                (None, Some(observer)) if observer.trustworthy => Some(observer),
                _ => None,
            },
            _ => hall,
        }
    }

    fn estimate_for_control(
        &mut self,
        elapsed_since_observation_us: u32,
        control_period_ns: u32,
    ) -> Option<PhaseEstimate<Turns>> {
        let hall = self
            .hall
            .estimate_for_control(elapsed_since_observation_us, control_period_ns);
        self.select_estimate(hall)
    }

    fn update(&mut self, input: &PhaseInput) {
        self.hall.update(input);
        if let Some(observer) = &mut self.observer {
            observer.update(input);
        }
    }

    fn injection(&self) -> crate::foc::Dq {
        self.hall.injection()
    }

    fn request_source(&mut self, source: PhaseSource) -> bool {
        self.set_source(source).is_ok()
    }
}

fn crossover_blend(speed_erpm: u32, low_erpm: i32, high_erpm: i32) -> Fixed {
    let low = low_erpm.max(0) as u32;
    let high = high_erpm.max(low_erpm.max(0) + 1) as u32;
    if speed_erpm <= low {
        Fixed::ZERO
    } else if speed_erpm >= high {
        Fixed::ONE
    } else {
        ratio_u32_q16(speed_erpm - low, high - low)
    }
}

/// Return an unsigned ratio in Q16.16 using the Cortex-M3's native divider.
/// The operands are scaled together when the Q16 numerator would exceed a
/// `u32`; the F103 crossover band needs no scaling and remains bit-exact.
fn ratio_u32_q16(numerator: u32, denominator: u32) -> Fixed {
    if denominator == 0 || numerator >= denominator {
        return Fixed::ONE;
    }
    let bit_length = 32 - denominator.leading_zeros();
    let shift = bit_length.saturating_sub(16);
    let scaled_numerator = numerator >> shift;
    let scaled_denominator = (denominator >> shift).max(1);
    Fixed::from_bits(
        (scaled_numerator.saturating_mul(1 << 16) / scaled_denominator)
            .min(Fixed::ONE.to_bits() as u32) as i32,
    )
}

fn blend_estimates(
    sensor: PhaseEstimate<Turns>,
    observer: PhaseEstimate<Turns>,
    blend: Fixed,
) -> PhaseEstimate<Turns> {
    let angle_error = signed_angle_difference(observer.angle, sensor.angle);
    let blended_angle = sensor
        .angle
        .wrapping_add(scale_i32(angle_error, blend) as u32);
    let velocity_error = observer
        .electrical_rpm
        .saturating_sub(sensor.electrical_rpm);
    PhaseEstimate {
        angle: blended_angle,
        electrical_rpm: sensor
            .electrical_rpm
            .saturating_add(scale_i32(velocity_error, blend)),
        trustworthy: sensor.trustworthy || observer.trustworthy,
    }
}

fn scale_i32(value: i32, scale: Fixed) -> i32 {
    ((i64::from(value) * i64::from(scale.to_bits())) >> 16)
        .clamp(i64::from(i32::MIN), i64::from(i32::MAX)) as i32
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
    struct TestHall {
        angle: Turns,
        electrical_rpm: i32,
        valid: bool,
    }

    impl PhaseProvider for TestHall {
        type Angle = Turns;

        fn source(&self) -> PhaseSource {
            PhaseSource::Hall
        }

        fn estimate(&self, _elapsed_since_observation_us: u32) -> Option<PhaseEstimate<Turns>> {
            self.valid.then_some(PhaseEstimate {
                angle: self.angle,
                electrical_rpm: self.electrical_rpm,
                trustworthy: true,
            })
        }
    }

    fn observer() -> BackEmfObserver {
        BackEmfObserver::new(
            Fixed::ratio(884, 10_000),
            Fixed::ratio(39, 1_000),
            Fixed::ratio(122, 10),
            16_000,
        )
    }

    #[test]
    fn unavailable_sources_are_rejected_without_changing_source() {
        let hall = TestHall {
            valid: true,
            ..TestHall::default()
        };
        let mut manager = PhaseManager::with_hall(hall);
        assert_eq!(
            manager.set_source(PhaseSource::Observer),
            Err(PhaseSourceError::ObserverNotConfigured)
        );
        assert_eq!(manager.source(), PhaseSource::Hall);
    }

    #[test]
    fn hall_to_observer_blends_across_the_configured_erpm_band() {
        let hall = TestHall {
            angle: 0x1000_0000,
            electrical_rpm: 4_500,
            valid: true,
        };
        let mut observer = observer();
        observer.seed(0x2000_0000, 4_500);
        let mut manager = PhaseManager::with_hall(hall);
        manager.set_observer(observer);
        manager
            .set_source(PhaseSource::HallToObserver {
                blend_low_erpm: 3_000,
                blend_high_erpm: 6_000,
            })
            .unwrap();

        let estimate = manager.estimate_for_control(0, 62_500).unwrap();
        assert!((estimate.angle.wrapping_sub(0x1800_0000) as i32).unsigned_abs() < 0x1_0000);
        assert_eq!(manager.observer_diagnostics().blend, Fixed::ratio(1, 2));
    }

    #[test]
    fn crossover_ratio_preserves_every_f103_blend_step() {
        for numerator in 0..=3_000 {
            assert_eq!(
                ratio_u32_q16(numerator, 3_000),
                Fixed::ratio(numerator as i32, 3_000),
                "numerator={numerator}",
            );
        }
    }

    #[test]
    fn full_speed_crossover_uses_the_observer() {
        let hall = TestHall {
            angle: 0x1000_0000,
            electrical_rpm: 7_000,
            valid: true,
        };
        let mut observer = observer();
        observer.seed(0x2000_0000, 7_000);
        let mut manager = PhaseManager::with_hall(hall);
        manager.set_observer(observer);
        manager
            .set_source(PhaseSource::HallToObserver {
                blend_low_erpm: 3_000,
                blend_high_erpm: 6_000,
            })
            .unwrap();

        assert_eq!(
            manager.estimate_for_control(0, 62_500).unwrap().angle,
            0x2000_0000
        );
        assert_eq!(manager.observer_diagnostics().blend, Fixed::ONE);
        assert!(manager.observer_diagnostics().active);
    }

    #[test]
    fn half_turn_disagreement_reseeds_instead_of_blending() {
        let hall = TestHall {
            angle: 0x1000_0000,
            electrical_rpm: 4_500,
            valid: true,
        };
        let mut observer = observer();
        observer.seed(0x9000_0000, 4_500);
        let mut manager = PhaseManager::with_hall(hall);
        manager.set_observer(observer);
        manager
            .set_source(PhaseSource::HallToObserver {
                blend_low_erpm: 3_000,
                blend_high_erpm: 6_000,
            })
            .unwrap();

        assert_eq!(
            manager.estimate_for_control(0, 62_500).unwrap().angle,
            hall.angle
        );
        assert_eq!(manager.observer().unwrap().phase(), hall.angle);
        assert_eq!(manager.observer_diagnostics().blend, Fixed::ZERO);
    }

    #[test]
    fn requested_seed_uses_the_next_rate_limited_hall_estimate() {
        let hall = TestHall {
            angle: 0x3000_0000,
            electrical_rpm: -5_000,
            valid: true,
        };
        let mut manager = PhaseManager::with_hall(hall);
        manager.set_observer(observer());
        manager.request_observer_seed();
        let _ = manager.estimate_for_control(0, 62_500);
        assert_eq!(manager.observer().unwrap().phase(), hall.angle);
        assert!((manager.observer().unwrap().electrical_rpm() + 5_000).unsigned_abs() < 2);
    }
}
