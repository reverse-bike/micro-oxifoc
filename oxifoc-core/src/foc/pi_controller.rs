//! Per-control-period PI controller with external saturation back-calculation.

use super::numeric::Fixed;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PIController {
    proportional_gain: Fixed,
    integral_gain_per_cycle: Fixed,
    integral: Fixed,
    previous_error: Fixed,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct PIUpdate {
    pub next_integral: Fixed,
    pub raw_output: Fixed,
    error: Fixed,
}

impl PIController {
    pub const fn new(proportional_gain: Fixed, integral_gain_per_cycle: Fixed) -> Self {
        Self {
            proportional_gain,
            integral_gain_per_cycle,
            integral: Fixed::ZERO,
            previous_error: Fixed::ZERO,
        }
    }

    pub fn reset(&mut self) {
        self.integral = Fixed::ZERO;
        self.previous_error = Fixed::ZERO;
    }

    pub const fn integral(&self) -> Fixed {
        self.integral
    }

    pub(crate) fn prepare_update(&self, target: Fixed, measurement: Fixed) -> PIUpdate {
        let error = target - measurement;
        let proportional = (error * self.proportional_gain).trunc();
        let trapezoidal_error = (error + self.previous_error) * Fixed::HALF;
        let next_integral = self.integral + trapezoidal_error * self.integral_gain_per_cycle;
        PIUpdate {
            next_integral,
            raw_output: proportional + next_integral.trunc(),
            error,
        }
    }

    pub(crate) fn apply_back_calculation(&mut self, update: PIUpdate, output_error: Fixed) {
        self.integral = update.next_integral + output_error;
        self.previous_error = update.error;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixed_pi() -> PIController {
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
