//! Frame and PWM values shared by the fixed-point control path.

use super::numeric::Fixed;

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct AlphaBeta {
    pub alpha: Fixed,
    pub beta: Fixed,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Dq {
    pub d: Fixed,
    pub q: Fixed,
}

impl Dq {
    pub const fn new(d: Fixed, q: Fixed) -> Self {
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
