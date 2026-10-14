//! Space-vector PWM in timer-tick voltage units.

use super::control_types::{AlphaBeta, PwmDuty};
use super::numeric::Fixed;
use super::transforms::inverse_clarke;

/// Convert αβ phase-voltage values expressed in timer ticks into compares.
///
/// Min/max zero-sequence injection shifts the three phase voltages so the
/// largest and smallest sit symmetrically about `neutral`, which yields the
/// same compares as six-sector space-vector modulation without a sector
/// search. The phases are formed at twice their voltage so `√3·β` is the only
/// rounded product, and each compare is built at four times its tick value so
/// a single floor (every in-range compare is positive) quantizes it; the
/// quadrupled neutral fits Q16.16 for `neutral` below 8,192 ticks.
///
/// The caller supplies the conversion from physical voltage to timer ticks.
/// `phase_limit_ticks` reserves the required bootstrap, dead-time,
/// and ADC sampling margin around `neutral`.
pub fn space_vector_pwm_ticks(voltage: AlphaBeta, neutral: u16, phase_limit_ticks: u16) -> PwmDuty {
    let (a, b, c) = inverse_clarke(voltage.alpha + voltage.alpha, voltage.beta + voltage.beta);
    let largest = a.max(b).max(c);
    let smallest = a.min(b).min(c);
    let offset = Fixed::from_integer(i32::from(neutral) * 4) - largest - smallest;
    let compare = |doubled_phase: Fixed| {
        let quadrupled = offset + doubled_phase + doubled_phase;
        limited_compare(quadrupled.integer() >> 2, neutral, phase_limit_ticks)
    };

    PwmDuty {
        a: compare(a),
        b: compare(b),
        c: compare(c),
    }
}

fn limited_compare(ticks: i32, neutral: u16, phase_limit_ticks: u16) -> u16 {
    let lower = neutral.saturating_sub(phase_limit_ticks);
    let upper = neutral.saturating_add(phase_limit_ticks);
    ticks.clamp(i32::from(lower), i32::from(upper)) as u16
}

#[cfg(test)]
mod tests {
    use super::*;

    const NEUTRAL: u16 = 1_125;
    const PHASE_LIMIT: u16 = 1_085;

    #[test]
    fn fixed_modulation_respects_the_sampling_margin() {
        for (alpha, beta) in [(1_250, 0), (0, 1_250), (-884, 884)] {
            let duty = space_vector_pwm_ticks(
                AlphaBeta {
                    alpha: Fixed::from_integer(alpha),
                    beta: Fixed::from_integer(beta),
                },
                NEUTRAL,
                PHASE_LIMIT,
            );
            for compare in duty.as_array() {
                assert!(compare.abs_diff(NEUTRAL) <= PHASE_LIMIT);
            }
        }
    }

    #[test]
    fn zero_vector_is_centered() {
        assert_eq!(
            space_vector_pwm_ticks(
                AlphaBeta {
                    alpha: Fixed::ZERO,
                    beta: Fixed::ZERO,
                },
                NEUTRAL,
                PHASE_LIMIT,
            ),
            PwmDuty {
                a: NEUTRAL,
                b: NEUTRAL,
                c: NEUTRAL,
            }
        );
    }

    /// Real-valued min/max injection, truncated toward zero like the
    /// fixed-point path.
    fn reference_compares(alpha: f64, beta: f64) -> [u16; 3] {
        const SQRT_3: f64 = 1.732_050_807_568_877_2;
        let phases = [
            alpha,
            (-alpha + SQRT_3 * beta) / 2.0,
            (-alpha - SQRT_3 * beta) / 2.0,
        ];
        let largest = phases.into_iter().fold(f64::MIN, f64::max);
        let smallest = phases.into_iter().fold(f64::MAX, f64::min);
        let midpoint = (largest + smallest) / 2.0;
        let lower = f64::from(NEUTRAL - PHASE_LIMIT);
        let upper = f64::from(NEUTRAL + PHASE_LIMIT);
        phases.map(|phase| {
            (f64::from(NEUTRAL) + phase - midpoint)
                .trunc()
                .clamp(lower, upper) as u16
        })
    }

    #[test]
    fn modulation_matches_the_real_valued_reference_within_one_tick() {
        for alpha_ticks in (-1_400_i32..=1_400).step_by(13) {
            for beta_ticks in (-1_400_i32..=1_400).step_by(13) {
                for fraction in [0, 1, 21_845, 32_768, 65_535] {
                    let alpha = Fixed::from_bits((alpha_ticks << 16) + fraction);
                    let beta = Fixed::from_bits((beta_ticks << 16) - fraction);
                    let duty =
                        space_vector_pwm_ticks(AlphaBeta { alpha, beta }, NEUTRAL, PHASE_LIMIT);
                    let expected = reference_compares(
                        f64::from(alpha.to_bits()) / 65_536.0,
                        f64::from(beta.to_bits()) / 65_536.0,
                    );
                    for (compare, reference) in duty.as_array().into_iter().zip(expected) {
                        assert!(
                            compare.abs_diff(reference) <= 1,
                            "alpha {alpha:?} beta {beta:?}: {compare} vs {reference}"
                        );
                    }
                }
            }
        }
    }
}
