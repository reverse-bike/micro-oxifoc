//! Per-control-period PI controller with external saturation back-calculation.

use super::numeric::{Fixed, Scalar};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PIController<N: Scalar = Fixed> {
    proportional_gain: N,
    integral_gain_per_cycle: N,
    integral: N,
    previous_error: N,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct PIUpdate<N: Scalar> {
    pub next_integral: N,
    pub raw_output: N,
    error: N,
}

impl<N: Scalar> PIController<N> {
    pub const fn new(proportional_gain: N, integral_gain_per_cycle: N) -> Self {
        Self {
            proportional_gain,
            integral_gain_per_cycle,
            integral: N::ZERO,
            previous_error: N::ZERO,
        }
    }

    pub fn reset(&mut self) {
        self.integral = N::ZERO;
        self.previous_error = N::ZERO;
    }

    pub const fn integral(&self) -> N {
        self.integral
    }

    pub(crate) fn prepare_update(&self, target: N, measurement: N) -> PIUpdate<N> {
        let error = target - measurement;
        let proportional = (error * self.proportional_gain).trunc();
        let trapezoidal_error = (error + self.previous_error) * N::HALF;
        let next_integral = self.integral + trapezoidal_error * self.integral_gain_per_cycle;
        PIUpdate {
            next_integral,
            raw_output: proportional + next_integral.trunc(),
            error,
        }
    }

    pub(crate) fn apply_back_calculation(&mut self, update: PIUpdate<N>, output_error: N) {
        self.integral = update.next_integral + output_error;
        self.previous_error = update.error;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixed_pi() -> PIController<Fixed> {
        PIController::new(Fixed::ratio(512, 1_024), Fixed::ratio(605, 16_384))
    }

    #[test]
    fn configured_fixed_gains_preserve_the_integer_controller_response() {
        let pi = fixed_pi();
        let first = pi.prepare_update(Fixed::from_integer(480), Fixed::ZERO);
        assert_eq!(first.raw_output.integer(), 248);
        assert_eq!(first.next_integral.to_bits(), 580_800);
    }

    #[test]
    fn back_calculation_tracks_the_applied_output() {
        let mut pi = fixed_pi();
        let update = pi.prepare_update(Fixed::from_integer(480), Fixed::ZERO);
        pi.apply_back_calculation(update, Fixed::from_integer(-100));
        assert_eq!(pi.integral().integer(), -91);
    }
}
