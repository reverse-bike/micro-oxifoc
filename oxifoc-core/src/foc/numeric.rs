//! Scalar backends used by the shared FOC algorithms.
//!
//! The control code is generic over [`Scalar`]. `f32` is the reference and
//! experimentation backend; [`Fixed`] is a signed Q16.16 representation for
//! processors without an FPU.

use core::fmt::Debug;
use core::ops::{Add, Mul, Neg, Sub};

const FRACTION_BITS: u32 = 16;
const SCALE: i64 = 1_i64 << FRACTION_BITS;

/// Arithmetic required by the Clarke/Park, PI, limiting, and SVPWM paths.
///
/// Implementations define their overflow and quantization behavior. The fixed
/// backend saturates arithmetic and truncates actuator commands toward zero;
/// the floating-point backend keeps its fractional output.
pub trait Scalar:
    Copy
    + Debug
    + Default
    + PartialEq
    + PartialOrd
    + Add<Output = Self>
    + Sub<Output = Self>
    + Mul<Output = Self>
    + Neg<Output = Self>
{
    const ZERO: Self;
    const ONE: Self;
    const HALF: Self;
    const THREE_HALVES: Self;
    const TWO_THIRDS: Self;
    const INV_SQRT_3: Self;
    const TWO_INV_SQRT_3: Self;
    const SQRT_3: Self;

    fn from_i32(value: i32) -> Self;
    fn from_ratio(numerator: i32, denominator: i32) -> Self;
    fn trunc_to_i32(self) -> i32;
    fn trunc(self) -> Self;
    fn sqrt(self) -> Self;
    fn abs(self) -> Self;
    fn abs_ceil_u32(self) -> u32;
    fn circular_remaining(limit: Self, direct: Self) -> Self;

    #[inline]
    fn min(self, other: Self) -> Self {
        if self < other { self } else { other }
    }

    #[inline]
    fn max(self, other: Self) -> Self {
        if self > other { self } else { other }
    }

    #[inline]
    fn clamp(self, minimum: Self, maximum: Self) -> Self {
        self.max(minimum).min(maximum)
    }
}

/// Signed Q16.16 scalar.
#[derive(Clone, Copy, Debug, Default, Eq, Ord, PartialEq, PartialOrd)]
#[repr(transparent)]
pub struct Fixed(i32);

impl Fixed {
    pub const ZERO: Self = Self(0);
    pub const ONE: Self = Self(1_i32 << FRACTION_BITS);

    pub const fn from_bits(bits: i32) -> Self {
        Self(bits)
    }

    pub const fn to_bits(self) -> i32 {
        self.0
    }

    pub const fn from_integer(value: i32) -> Self {
        Self(saturating_i64_to_i32((value as i64) << FRACTION_BITS))
    }

    pub const fn ratio(numerator: i32, denominator: i32) -> Self {
        assert!(
            denominator != 0,
            "fixed-point ratio denominator must be nonzero"
        );
        Self(saturating_i64_to_i32(
            (numerator as i64 * SCALE) / denominator as i64,
        ))
    }

    pub const fn integer(self) -> i32 {
        self.0 / (1_i32 << FRACTION_BITS)
    }

    pub const fn from_q31(value: i32) -> Self {
        Self(value / (1_i32 << 15))
    }
}

impl Add for Fixed {
    type Output = Self;

    #[inline]
    fn add(self, rhs: Self) -> Self::Output {
        Self(self.0.saturating_add(rhs.0))
    }
}

impl Sub for Fixed {
    type Output = Self;

    #[inline]
    fn sub(self, rhs: Self) -> Self::Output {
        Self(self.0.saturating_sub(rhs.0))
    }
}

impl Mul for Fixed {
    type Output = Self;

    #[inline]
    fn mul(self, rhs: Self) -> Self::Output {
        Self(saturating_i64_to_i32(
            (i64::from(self.0) * i64::from(rhs.0)) / SCALE,
        ))
    }
}

impl Neg for Fixed {
    type Output = Self;

    #[inline]
    fn neg(self) -> Self::Output {
        Self(self.0.saturating_neg())
    }
}

impl Scalar for Fixed {
    const ZERO: Self = Self::ZERO;
    const ONE: Self = Self::ONE;
    const HALF: Self = Self(32_768);
    const THREE_HALVES: Self = Self(98_304);
    const TWO_THIRDS: Self = Self(43_690);
    const INV_SQRT_3: Self = Self(37_837);
    const TWO_INV_SQRT_3: Self = Self(75_674);
    const SQRT_3: Self = Self(113_512);

    #[inline]
    fn from_i32(value: i32) -> Self {
        Self::from_integer(value)
    }

    #[inline]
    fn from_ratio(numerator: i32, denominator: i32) -> Self {
        Self::ratio(numerator, denominator)
    }

    #[inline]
    fn trunc_to_i32(self) -> i32 {
        self.integer()
    }

    #[inline]
    fn trunc(self) -> Self {
        Self::from_integer(self.integer())
    }

    #[inline]
    fn sqrt(self) -> Self {
        if self.0 <= 0 {
            return Self::ZERO;
        }
        let radicand = self.0 as u64 * SCALE as u64;
        Self(integer_sqrt_u64(radicand).min(i32::MAX as u64) as i32)
    }

    #[inline]
    fn abs(self) -> Self {
        Self(self.0.saturating_abs())
    }

    #[inline]
    fn abs_ceil_u32(self) -> u32 {
        self.0
            .unsigned_abs()
            .saturating_add((1_u32 << FRACTION_BITS) - 1)
            >> FRACTION_BITS
    }

    #[inline]
    fn circular_remaining(limit: Self, direct: Self) -> Self {
        let limit = i64::from(limit.0.max(0));
        let direct = i64::from(direct.0.saturating_abs()).min(limit);
        let remaining = (limit * limit).saturating_sub(direct * direct) as u64;
        Self(integer_sqrt_u64(remaining).min(i32::MAX as u64) as i32)
    }
}

#[cfg(feature = "algorithms")]
impl Scalar for f32 {
    const ZERO: Self = 0.0;
    const ONE: Self = 1.0;
    const HALF: Self = 0.5;
    const THREE_HALVES: Self = 1.5;
    const TWO_THIRDS: Self = 2.0 / 3.0;
    const INV_SQRT_3: Self = 0.577_350_26;
    const TWO_INV_SQRT_3: Self = 1.154_700_5;
    const SQRT_3: Self = 1.732_050_8;

    #[inline]
    fn from_i32(value: i32) -> Self {
        value as Self
    }

    #[inline]
    fn from_ratio(numerator: i32, denominator: i32) -> Self {
        numerator as Self / denominator as Self
    }

    #[inline]
    fn trunc_to_i32(self) -> i32 {
        self as i32
    }

    #[inline]
    fn trunc(self) -> Self {
        self
    }

    #[inline]
    fn sqrt(self) -> Self {
        libm::sqrtf(self.max(0.0))
    }

    #[inline]
    fn abs(self) -> Self {
        Self::abs(self)
    }

    #[inline]
    fn abs_ceil_u32(self) -> u32 {
        libm::ceilf(Self::abs(self)).max(0.0) as u32
    }

    #[inline]
    fn circular_remaining(limit: Self, direct: Self) -> Self {
        libm::sqrtf((limit * limit - direct * direct).max(0.0))
    }
}

const fn saturating_i64_to_i32(value: i64) -> i32 {
    if value > i32::MAX as i64 {
        i32::MAX
    } else if value < i32::MIN as i64 {
        i32::MIN
    } else {
        value as i32
    }
}

fn integer_sqrt_u64(mut radicand: u64) -> u64 {
    let mut result = 0_u64;
    let mut bit = 1_u64 << 62;
    while bit > radicand {
        bit >>= 2;
    }
    while bit != 0 {
        if radicand >= result + bit {
            radicand -= result + bit;
            result = (result >> 1) + bit;
        } else {
            result >>= 1;
        }
        bit >>= 2;
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fixed_multiplication_keeps_fractional_precision() {
        let result = Fixed::ratio(3, 2) * Fixed::ratio(-7, 4);
        assert_eq!(result.integer(), -2);
        assert!((result.to_bits() + 172_032).unsigned_abs() <= 1);
    }

    #[test]
    fn fixed_square_root_uses_the_same_scale() {
        assert_eq!(Fixed::from_integer(9).sqrt(), Fixed::from_integer(3));
        assert!((Fixed::from_integer(2).sqrt().to_bits() - 92_681).unsigned_abs() <= 1);
    }

    #[test]
    fn actuator_quantization_is_toward_zero() {
        assert_eq!(Fixed::ratio(19, 10).trunc(), Fixed::from_integer(1));
        assert_eq!(Fixed::ratio(-19, 10).trunc(), Fixed::from_integer(-1));
    }
}
