//! Clarke and Park transforms for the fixed-point control path.

use super::numeric::Scalar;

/// Convert two measured phase currents into the stationary αβ frame.
#[inline]
pub fn clarke<N: Scalar>(phase_a: N, phase_b: N) -> (N, N) {
    (phase_a, (phase_a + phase_b + phase_b) * N::INV_SQRT_3)
}

/// Convert a stationary αβ vector back to the three phase frame.
#[inline]
pub fn inverse_clarke<N: Scalar>(alpha: N, beta: N) -> (N, N, N) {
    let scaled_beta = N::SQRT_3 * beta;
    (
        alpha,
        (-alpha + scaled_beta) * N::HALF,
        (-alpha - scaled_beta) * N::HALF,
    )
}

/// Rotate a stationary αβ vector into the rotor-aligned dq frame.
#[inline]
pub fn park<N: Scalar>(alpha: N, beta: N, sin_theta: N, cos_theta: N) -> (N, N) {
    (
        cos_theta * alpha + sin_theta * beta,
        cos_theta * beta - sin_theta * alpha,
    )
}

/// Rotate a rotor-aligned dq vector back into the stationary αβ frame.
#[inline]
pub fn inverse_park<N: Scalar>(d: N, q: N, sin_theta: N, cos_theta: N) -> (N, N) {
    (cos_theta * d - sin_theta * q, sin_theta * d + cos_theta * q)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::foc::numeric::Fixed;

    #[test]
    fn fixed_clarke_keeps_adc_count_accuracy() {
        let (alpha, beta) = clarke(Fixed::from_integer(1_000), Fixed::from_integer(-500));
        assert_eq!(alpha.integer(), 1_000);
        assert!(beta.integer().unsigned_abs() <= 1);
    }

    #[test]
    fn fixed_park_round_trip_is_within_one_count() {
        let alpha = Fixed::from_integer(480);
        let beta = Fixed::from_integer(-320);
        let sin = Fixed::ratio(3, 5);
        let cos = Fixed::ratio(4, 5);
        let (d, q) = park(alpha, beta, sin, cos);
        let (roundtrip_alpha, roundtrip_beta) = inverse_park(d, q, sin, cos);
        assert!((roundtrip_alpha.integer() - alpha.integer()).unsigned_abs() <= 1);
        assert!((roundtrip_beta.integer() - beta.integer()).unsigned_abs() <= 1);
    }
}
