//! Cycle-counted slew limiting for a quadrature-current request.

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct QuadratureTargetRamp {
    target: i32,
    elapsed_cycles: u8,
    cycles_per_step: u8,
    counts_per_step: i32,
}

impl QuadratureTargetRamp {
    pub const fn new(cycles_per_step: u8, counts_per_step: i32) -> Self {
        Self {
            target: 0,
            elapsed_cycles: 0,
            cycles_per_step: if cycles_per_step == 0 {
                1
            } else {
                cycles_per_step
            },
            counts_per_step: if counts_per_step <= 0 {
                1
            } else {
                counts_per_step
            },
        }
    }

    pub fn next(&mut self, requested: i32) -> i32 {
        if self.target < requested {
            self.target = requested;
            self.elapsed_cycles = 0;
        }
        let target = self.target;
        self.elapsed_cycles = self.elapsed_cycles.saturating_add(1);
        if self.elapsed_cycles >= self.cycles_per_step {
            self.elapsed_cycles = 0;
            if self.target > requested {
                self.target = self
                    .target
                    .saturating_sub(self.counts_per_step)
                    .max(requested);
            }
        }
        target
    }

    pub fn reset(&mut self) {
        self.target = 0;
        self.elapsed_cycles = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn torque_increase_is_slew_limited_and_reduction_is_immediate() {
        let mut ramp = QuadratureTargetRamp::new(16, 4);
        for expected in [0, -4, -8, -12] {
            assert_eq!(ramp.next(-100), expected);
            for _ in 1..16 {
                assert_eq!(ramp.next(-100), expected);
            }
        }
        assert_eq!(ramp.next(-2), -2);
    }

    #[test]
    fn reset_preserves_the_configured_slew_rate() {
        let mut ramp = QuadratureTargetRamp::new(3, 7);
        for _ in 0..4 {
            let _ = ramp.next(-100);
        }
        ramp.reset();
        assert_eq!(ramp.next(-100), 0);
        assert_eq!(ramp.next(-100), 0);
        assert_eq!(ramp.next(-100), 0);
        assert_eq!(ramp.next(-100), -7);
    }
}
