//! Last-resort gate shutdown, watchdogs, and reset forensics.

pub mod reset_flag {
    pub const PIN: u8 = 1;
    pub const POWER: u8 = 1 << 1;
    pub const SOFTWARE: u8 = 1 << 2;
    pub const INDEPENDENT_WATCHDOG: u8 = 1 << 3;
    pub const WINDOW_WATCHDOG: u8 = 1 << 4;
    pub const LOW_POWER: u8 = 1 << 5;
}

pub mod fatal_reason {
    pub const NONE: u8 = 0;
    pub const PANIC: u8 = 1;
    pub const HARD_FAULT: u8 = 2;
    pub const NON_MASKABLE_INTERRUPT: u8 = 3;
    pub const DEFAULT_HANDLER: u8 = 4;
}

pub mod checkpoint {
    pub const IDLE: u8 = 0;
    pub const ENTERED: u8 = 1;
    pub const CURRENT_SAMPLED: u8 = 2;
    pub const PHASE_ESTIMATED: u8 = 3;
    pub const DRIVER_COMPLETE: u8 = 4;
    pub const PWM_WRITTEN: u8 = 5;
    pub const COMPLETE: u8 = 6;
}

pub mod pwm_failure_cause {
    pub const NONE: u8 = 0;
    pub const COMPARE_RANGE: u8 = 1;
    pub const BREAK_ACTIVE: u8 = 2;
    pub const FAULT_LATCHED: u8 = 3;
    pub const POWER_STAGE_DISABLED: u8 = 4;
    pub const POST_PIN_CONFIGURATION: u8 = 5;
    pub const ENABLE_READBACK: u8 = 6;
    pub const POST_ENABLE: u8 = 7;
}

pub mod pwm_pin_flag {
    pub const BREAK_ACTIVE: u8 = 1;
    pub const POWER_STAGE_DISABLED: u8 = 1 << 1;
    pub const CLOCK_SECURITY_FAILURE: u8 = 1 << 2;
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct BootDiagnostics {
    pub reset_flags: u8,
    pub retained_context_valid: bool,
    pub fatal_reason: u8,
    pub checkpoint: u8,
    pub detail: i16,
    pub control_cycle: u32,
    pub last_control_cycles: u32,
    pub maximum_control_cycles: u32,
    pub program_counter: u32,
    pub link_register: u32,
    pub pwm_failure: PwmFailureContext,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct PwmFailureContext {
    words: [u32; 4],
}

impl PwmFailureContext {
    pub const EMPTY: Self = Self { words: [0; 4] };

    pub fn new(
        cause: u8,
        fault_flags: u8,
        pin_flags: u8,
        timer_status: u16,
        timer_bdtr: u16,
        timer_ccer: u16,
        compares: [u16; 3],
    ) -> Self {
        Self {
            words: [
                u32::from_le_bytes([cause, fault_flags, pin_flags, 0]),
                u32::from(timer_status) | (u32::from(timer_bdtr) << 16),
                u32::from(timer_ccer) | (u32::from(compares[0]) << 16),
                u32::from(compares[1]) | (u32::from(compares[2]) << 16),
            ],
        }
    }

    pub const fn cause(self) -> u8 {
        self.words[0] as u8
    }

    pub const fn words(self) -> [u32; 4] {
        self.words
    }

    #[cfg(test)]
    fn decoded(self) -> (u8, u8, u8, u16, u16, u16, [u16; 3]) {
        let header = self.words[0].to_le_bytes();
        let timer = self.words[1].to_le_bytes();
        let ccer_a = self.words[2].to_le_bytes();
        let b_c = self.words[3].to_le_bytes();
        (
            header[0],
            header[1],
            header[2],
            u16::from_le_bytes([timer[0], timer[1]]),
            u16::from_le_bytes([timer[2], timer[3]]),
            u16::from_le_bytes([ccer_a[0], ccer_a[1]]),
            [
                u16::from_le_bytes([ccer_a[2], ccer_a[3]]),
                u16::from_le_bytes([b_c[0], b_c[1]]),
                u16::from_le_bytes([b_c[2], b_c[3]]),
            ],
        )
    }
}

#[cfg(any(feature = "board", test))]
mod retained;
#[cfg(any(feature = "board", test))]
use retained::*;

pub const fn watchdog_progressed(
    previous_control_cycles: u32,
    control_cycles: u32,
    previous_injected_samples: u32,
    injected_samples: u32,
    latched_safe_off: bool,
) -> bool {
    control_cycles != previous_control_cycles
        && (injected_samples != previous_injected_samples || latched_safe_off)
}

#[cfg(feature = "board")]
mod runtime;
#[cfg(feature = "board")]
pub use runtime::*;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reset_flags_are_compacted_without_losing_a_cause() {
        assert_eq!(
            compact_reset_flags(
                RCC_RESET_PIN
                    | RCC_RESET_POWER
                    | RCC_RESET_SOFTWARE
                    | RCC_RESET_INDEPENDENT_WATCHDOG
                    | RCC_RESET_WINDOW_WATCHDOG
                    | RCC_RESET_LOW_POWER
            ),
            0x3f
        );
    }

    #[test]
    fn pwm_failure_context_has_a_lossless_four_word_retained_form() {
        let context =
            PwmFailureContext::new(6, 0x0b, 3, 0x0081, 0x9d19, 0x1ddd, [22, 1_125, 2_228]);
        assert_eq!(
            context.decoded(),
            (6, 0x0b, 3, 0x0081, 0x9d19, 0x1ddd, [22, 1_125, 2_228])
        );
        assert_eq!(core::mem::size_of::<RetainedContext>(), 56);
    }

    #[test]
    fn pwm_failure_capture_is_first_failure_wins_until_rearmed() {
        assert!(pwm_failure_slot_is_empty(
            PwmFailureContext::EMPTY.words()[0]
        ));
        assert!(!pwm_failure_slot_is_empty(
            PwmFailureContext::new(2, 1, 5, 0, 0, 0, [0; 3]).words()[0]
        ));
    }

    #[test]
    fn a_latched_safe_off_fault_keeps_the_watchdog_alive_without_adc_progress() {
        assert!(watchdog_progressed(100, 101, 200, 201, false));
        assert!(!watchdog_progressed(100, 101, 200, 200, false));
        assert!(watchdog_progressed(100, 101, 200, 200, true));
        assert!(!watchdog_progressed(100, 100, 200, 201, true));
    }

    #[test]
    fn valid_context_is_reported_only_after_a_watchdog_reset() {
        let retained = RetainedContext {
            fatal_reason: u32::from(fatal_reason::HARD_FAULT),
            checkpoint: u32::from(checkpoint::DRIVER_COMPLETE),
            detail: -3,
            control_cycle: 0x1234_5678,
            last_control_cycles: 4_321,
            maximum_control_cycles: 4_498,
            program_counter: 0x0800_9abc,
            link_register: 0xffff_fff9,
            pwm_failure: PwmFailureContext::new(
                6,
                1,
                3,
                0x0081,
                0x9d19,
                0x1ddd,
                [22, 1_125, 2_228],
            ),
            ..RetainedContext::active()
        };
        let watchdog = boot_diagnostics(RCC_RESET_INDEPENDENT_WATCHDOG, retained);
        assert!(watchdog.retained_context_valid);
        assert_eq!(watchdog.fatal_reason, fatal_reason::HARD_FAULT);
        assert_eq!(watchdog.checkpoint, checkpoint::DRIVER_COMPLETE);
        assert_eq!(watchdog.control_cycle, 0x1234_5678);
        assert_eq!(watchdog.program_counter, 0x0800_9abc);
        assert_eq!(
            watchdog.pwm_failure,
            PwmFailureContext::new(6, 1, 3, 0x0081, 0x9d19, 0x1ddd, [22, 1_125, 2_228],)
        );

        let power = boot_diagnostics(RCC_RESET_POWER, retained);
        assert!(!power.retained_context_valid);
        assert_eq!(power.fatal_reason, fatal_reason::NONE);
        assert_eq!(power.pwm_failure, PwmFailureContext::default());
    }

    #[test]
    fn corrupt_retained_signature_is_never_reported() {
        let retained = RetainedContext {
            magic_inverse: RETAINED_MAGIC,
            fatal_reason: u32::from(fatal_reason::HARD_FAULT),
            ..RetainedContext::active()
        };
        let diagnostics = boot_diagnostics(RCC_RESET_WINDOW_WATCHDOG, retained);
        assert!(!diagnostics.retained_context_valid);
        assert_eq!(diagnostics.program_counter, 0);
    }
}
