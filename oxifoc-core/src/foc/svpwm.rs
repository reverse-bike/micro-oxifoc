//! Sector-based space-vector PWM in timer-tick voltage units.

use super::control_types::{AlphaBeta, PwmDuty};
use super::numeric::Scalar;

/// Convert αβ phase-voltage values expressed in timer ticks into compares.
///
/// The caller supplies the conversion from physical voltage to timer ticks.
/// `phase_limit_ticks` reserves the required bootstrap, dead-time,
/// and ADC sampling margin around `neutral`.
pub fn space_vector_pwm_ticks<N: Scalar>(
    voltage: AlphaBeta<N>,
    neutral: u16,
    phase_limit_ticks: u16,
) -> PwmDuty {
    let alpha = voltage.alpha;
    let beta = voltage.beta;
    let full_period = N::from_i32(i32::from(neutral) * 2);

    let (phase_a, phase_b, phase_c) = match get_sector(alpha, beta) {
        1 => {
            let t1 = (alpha - N::INV_SQRT_3 * beta) * N::THREE_HALVES;
            let t2 = N::SQRT_3 * beta;
            let phase_a = (full_period + t1 + t2) * N::HALF;
            let phase_b = phase_a - t1;
            let phase_c = phase_b - t2;
            (phase_a, phase_b, phase_c)
        }
        2 => {
            let t2 = (alpha + N::INV_SQRT_3 * beta) * N::THREE_HALVES;
            let t3 = (-alpha + N::INV_SQRT_3 * beta) * N::THREE_HALVES;
            let phase_b = (full_period + t2 + t3) * N::HALF;
            let phase_a = phase_b - t3;
            let phase_c = phase_a - t2;
            (phase_a, phase_b, phase_c)
        }
        3 => {
            let t3 = N::SQRT_3 * beta;
            let t4 = (-alpha - N::INV_SQRT_3 * beta) * N::THREE_HALVES;
            let phase_b = (full_period + t3 + t4) * N::HALF;
            let phase_c = phase_b - t3;
            let phase_a = phase_c - t4;
            (phase_a, phase_b, phase_c)
        }
        4 => {
            let t4 = (-alpha + N::INV_SQRT_3 * beta) * N::THREE_HALVES;
            let t5 = -(N::SQRT_3 * beta);
            let phase_c = (full_period + t4 + t5) * N::HALF;
            let phase_b = phase_c - t5;
            let phase_a = phase_b - t4;
            (phase_a, phase_b, phase_c)
        }
        5 => {
            let t5 = (-alpha - N::INV_SQRT_3 * beta) * N::THREE_HALVES;
            let t6 = (alpha - N::INV_SQRT_3 * beta) * N::THREE_HALVES;
            let phase_c = (full_period + t5 + t6) * N::HALF;
            let phase_a = phase_c - t5;
            let phase_b = phase_a - t6;
            (phase_a, phase_b, phase_c)
        }
        _ => {
            let t6 = -(N::SQRT_3 * beta);
            let t1 = (alpha + N::INV_SQRT_3 * beta) * N::THREE_HALVES;
            let phase_a = (full_period + t6 + t1) * N::HALF;
            let phase_c = phase_a - t1;
            let phase_b = phase_c - t6;
            (phase_a, phase_b, phase_c)
        }
    };

    PwmDuty {
        a: limited_compare(phase_a, neutral, phase_limit_ticks),
        b: limited_compare(phase_b, neutral, phase_limit_ticks),
        c: limited_compare(phase_c, neutral, phase_limit_ticks),
    }
}

/// Geometric sector for an αβ vector.
pub fn get_sector<N: Scalar>(alpha: N, beta: N) -> u8 {
    if beta >= N::ZERO {
        if alpha >= N::ZERO {
            if N::INV_SQRT_3 * beta > alpha { 2 } else { 1 }
        } else if -(N::INV_SQRT_3 * beta) > alpha {
            3
        } else {
            2
        }
    } else if alpha >= N::ZERO {
        if -(N::INV_SQRT_3 * beta) > alpha {
            5
        } else {
            6
        }
    } else if N::INV_SQRT_3 * beta > alpha {
        4
    } else {
        5
    }
}

pub trait TickModulator<N: Scalar> {
    fn to_duties(voltage: AlphaBeta<N>, neutral: u16, phase_limit_ticks: u16) -> PwmDuty;
}

#[derive(Clone, Copy, Debug, Default)]
pub struct SvpwmTickModulator;

impl<N: Scalar> TickModulator<N> for SvpwmTickModulator {
    #[inline]
    fn to_duties(voltage: AlphaBeta<N>, neutral: u16, phase_limit_ticks: u16) -> PwmDuty {
        space_vector_pwm_ticks(voltage, neutral, phase_limit_ticks)
    }
}

fn limited_compare<N: Scalar>(value: N, neutral: u16, phase_limit_ticks: u16) -> u16 {
    let lower = neutral.saturating_sub(phase_limit_ticks);
    let upper = neutral.saturating_add(phase_limit_ticks);
    value
        .trunc_to_i32()
        .clamp(i32::from(lower), i32::from(upper)) as u16
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::foc::numeric::Fixed;

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
}
