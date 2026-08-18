//! Frame and PWM values shared by every numeric backend.

use super::numeric::{Fixed, Scalar};

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct AlphaBeta<N: Scalar = Fixed> {
    pub alpha: N,
    pub beta: N,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Dq<N: Scalar = Fixed> {
    pub d: N,
    pub q: N,
}

impl<N: Scalar> Dq<N> {
    pub const fn new(d: N, q: N) -> Self {
        Self { d, q }
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct PwmDuty {
    pub a: u16,
    pub b: u16,
    pub c: u16,
}

impl PwmDuty {
    pub const fn as_array(self) -> [u16; 3] {
        [self.a, self.b, self.c]
    }
}
