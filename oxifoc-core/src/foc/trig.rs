//! Electrical-angle representation and trigonometry backends.

use super::numeric::{Fixed, Scalar};

/// Q0.32 turns: one complete electrical revolution is `u32::MAX + 1`.
pub type Turns = u32;

pub trait SinCos<N: Scalar> {
    type Angle: Copy;

    fn sin_cos(angle: Self::Angle) -> (N, N);
}

/// Software integer CORDIC for Q0.32-turn angles.
#[derive(Clone, Copy, Debug, Default)]
pub struct CordicSinCos;

impl SinCos<Fixed> for CordicSinCos {
    type Angle = Turns;

    #[inline]
    fn sin_cos(angle: Self::Angle) -> (Fixed, Fixed) {
        let (sin, cos) = sin_cos_q31(angle);
        (Fixed::from_q31(sin), Fixed::from_q31(cos))
    }
}

/// Software floating-point sine/cosine for radian angles.
#[cfg(feature = "algorithms")]
#[derive(Clone, Copy, Debug, Default)]
pub struct LibmSinCos;

#[cfg(feature = "algorithms")]
impl SinCos<f32> for LibmSinCos {
    type Angle = f32;

    #[inline]
    fn sin_cos(angle: Self::Angle) -> (f32, f32) {
        (libm::sinf(angle), libm::cosf(angle))
    }
}

const CORDIC_GAIN_INV_Q30: i32 = 652_032_874;
const CORDIC_ATAN_TURNS: [i32; 16] = [
    536_870_912,
    316_933_406,
    167_458_907,
    85_004_756,
    42_667_331,
    21_354_465,
    10_679_838,
    5_340_191,
    2_670_129,
    1_335_085,
    667_544,
    333_772,
    166_886,
    83_443,
    41_722,
    20_861,
];

/// 16-iteration integer CORDIC returning Q1.31 sine and cosine.
#[inline]
pub fn sin_cos_q31(angle: Turns) -> (i32, i32) {
    let quadrant = angle >> 30;
    let mut z = (angle & 0x3fff_ffff) as i32;
    let mut x = CORDIC_GAIN_INV_Q30;
    let mut y = 0_i32;

    for (shift, atan) in CORDIC_ATAN_TURNS.iter().enumerate() {
        let x_shifted = x >> shift;
        let y_shifted = y >> shift;
        if z >= 0 {
            x -= y_shifted;
            y += x_shifted;
            z -= atan;
        } else {
            x += y_shifted;
            y -= x_shifted;
            z += atan;
        }
    }

    let sin = y << 1;
    let cos = x << 1;
    match quadrant {
        0 => (sin, cos),
        1 => (cos, -sin),
        2 => (-sin, -cos),
        _ => (-cos, sin),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cordic_cardinal_angles_are_correct() {
        let cases = [
            (0, 0, i32::MAX),
            (0x4000_0000, i32::MAX, 0),
            (0x8000_0000, 0, -i32::MAX),
            (0xc000_0000, -i32::MAX, 0),
        ];
        for (angle, expected_sin, expected_cos) in cases {
            let (sin, cos) = sin_cos_q31(angle);
            assert!((sin - expected_sin).unsigned_abs() < 100_000);
            assert!((cos - expected_cos).unsigned_abs() < 100_000);
        }
    }

    #[cfg(feature = "algorithms")]
    #[test]
    fn fixed_and_float_backends_agree_around_a_turn() {
        for eighth in 0..8_u32 {
            let turns = eighth << 29;
            let radians = eighth as f32 * core::f32::consts::FRAC_PI_4;
            let (fixed_sin, fixed_cos) = CordicSinCos::sin_cos(turns);
            let (float_sin, float_cos) = LibmSinCos::sin_cos(radians);
            let fixed_sin = fixed_sin.to_bits() as f32 / 65_536.0;
            let fixed_cos = fixed_cos.to_bits() as f32 / 65_536.0;
            assert!((fixed_sin - float_sin).abs() < 0.000_2);
            assert!((fixed_cos - float_cos).abs() < 0.000_2);
        }
    }
}
