//! Encoding and validation of watchdog-retained reset diagnostics.

use super::{
    BootDiagnostics, PwmFailureContext, checkpoint, fatal_reason, pwm_failure_cause, reset_flag,
};

pub(super) const RCC_RESET_PIN: u32 = 1 << 26;
pub(super) const RCC_RESET_POWER: u32 = 1 << 27;
pub(super) const RCC_RESET_SOFTWARE: u32 = 1 << 28;
pub(super) const RCC_RESET_INDEPENDENT_WATCHDOG: u32 = 1 << 29;
pub(super) const RCC_RESET_WINDOW_WATCHDOG: u32 = 1 << 30;
pub(super) const RCC_RESET_LOW_POWER: u32 = 1 << 31;
pub(super) const RETAINED_MAGIC: u32 = 0x4f58_4652;

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub(super) struct RetainedContext {
    pub(super) magic: u32,
    pub(super) magic_inverse: u32,
    pub(super) fatal_reason: u32,
    pub(super) checkpoint: u32,
    pub(super) detail: i32,
    pub(super) control_cycle: u32,
    pub(super) last_control_cycles: u32,
    pub(super) maximum_control_cycles: u32,
    pub(super) program_counter: u32,
    pub(super) link_register: u32,
    pub(super) pwm_failure: PwmFailureContext,
}

impl RetainedContext {
    pub(super) const fn active() -> Self {
        Self {
            magic: RETAINED_MAGIC,
            magic_inverse: !RETAINED_MAGIC,
            fatal_reason: fatal_reason::NONE as u32,
            checkpoint: checkpoint::IDLE as u32,
            detail: 0,
            control_cycle: 0,
            last_control_cycles: 0,
            maximum_control_cycles: 0,
            program_counter: 0,
            link_register: 0,
            pwm_failure: PwmFailureContext::EMPTY,
        }
    }

    pub(super) const fn is_valid(self) -> bool {
        self.magic == RETAINED_MAGIC && self.magic_inverse == !RETAINED_MAGIC
    }
}

pub(super) fn compact_reset_flags(raw: u32) -> u8 {
    (u8::from(raw & RCC_RESET_PIN != 0) * reset_flag::PIN)
        | (u8::from(raw & RCC_RESET_POWER != 0) * reset_flag::POWER)
        | (u8::from(raw & RCC_RESET_SOFTWARE != 0) * reset_flag::SOFTWARE)
        | (u8::from(raw & RCC_RESET_INDEPENDENT_WATCHDOG != 0) * reset_flag::INDEPENDENT_WATCHDOG)
        | (u8::from(raw & RCC_RESET_WINDOW_WATCHDOG != 0) * reset_flag::WINDOW_WATCHDOG)
        | (u8::from(raw & RCC_RESET_LOW_POWER != 0) * reset_flag::LOW_POWER)
}

pub(super) const fn pwm_failure_slot_is_empty(header: u32) -> bool {
    header as u8 == pwm_failure_cause::NONE
}

pub(super) fn boot_diagnostics(raw_reset_flags: u32, retained: RetainedContext) -> BootDiagnostics {
    let reset_flags = compact_reset_flags(raw_reset_flags);
    let watchdog_reset =
        reset_flags & (reset_flag::INDEPENDENT_WATCHDOG | reset_flag::WINDOW_WATCHDOG) != 0;
    if watchdog_reset && retained.is_valid() {
        BootDiagnostics {
            reset_flags,
            retained_context_valid: true,
            fatal_reason: retained.fatal_reason as u8,
            checkpoint: retained.checkpoint as u8,
            detail: retained.detail.clamp(i16::MIN as i32, i16::MAX as i32) as i16,
            control_cycle: retained.control_cycle,
            last_control_cycles: retained.last_control_cycles,
            maximum_control_cycles: retained.maximum_control_cycles,
            program_counter: retained.program_counter,
            link_register: retained.link_register,
            pwm_failure: retained.pwm_failure,
        }
    } else {
        BootDiagnostics {
            reset_flags,
            ..BootDiagnostics::default()
        }
    }
}
