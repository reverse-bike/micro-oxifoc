#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct DebouncedActiveLow {
    required_samples: u8,
    low_samples: u8,
    high_samples: u8,
    active: bool,
}

impl DebouncedActiveLow {
    pub const fn new(required_samples: u8) -> Self {
        Self {
            required_samples: if required_samples == 0 {
                1
            } else {
                required_samples
            },
            low_samples: 0,
            high_samples: 0,
            active: false,
        }
    }

    pub const fn active(self) -> bool {
        self.active
    }

    pub fn update(&mut self, pin_is_low: bool) -> bool {
        if pin_is_low {
            self.high_samples = 0;
            self.low_samples = self.low_samples.saturating_add(1);
            if self.low_samples >= self.required_samples {
                self.active = true;
            }
        } else {
            self.low_samples = 0;
            self.high_samples = self.high_samples.saturating_add(1);
            if self.high_samples >= self.required_samples {
                self.active = false;
            }
        }
        self.active
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn active_low_input_requires_four_matching_samples() {
        let mut input = DebouncedActiveLow::new(4);
        for _ in 0..3 {
            assert!(!input.update(true));
        }
        assert!(input.update(true));
        for _ in 0..3 {
            assert!(input.update(false));
        }
        assert!(!input.update(false));
    }

    #[test]
    fn opposite_sample_restarts_the_debounce_window() {
        let mut input = DebouncedActiveLow::new(4);
        assert!(!input.update(true));
        assert!(!input.update(true));
        assert!(!input.update(false));
        for expected in [false, false, false, true] {
            assert_eq!(input.update(true), expected);
        }
    }
}
