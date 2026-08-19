//! Last-resort gate shutdown, watchdogs, and reset forensics.

#[cfg(feature = "firmware")]
use core::hint::spin_loop;
#[cfg(feature = "firmware")]
use core::mem::MaybeUninit;
#[cfg(feature = "firmware")]
use core::ptr::{read_volatile, write_volatile};
#[cfg(feature = "firmware")]
use core::sync::atomic::{AtomicBool, AtomicI32, AtomicU8, AtomicU32, Ordering};
#[cfg(feature = "firmware")]
use cortex_m_rt::{ExceptionFrame, exception};

#[cfg(any(feature = "firmware", test))]
const RCC_RESET_PIN: u32 = 1 << 26;
#[cfg(any(feature = "firmware", test))]
const RCC_RESET_POWER: u32 = 1 << 27;
#[cfg(any(feature = "firmware", test))]
const RCC_RESET_SOFTWARE: u32 = 1 << 28;
#[cfg(any(feature = "firmware", test))]
const RCC_RESET_INDEPENDENT_WATCHDOG: u32 = 1 << 29;
#[cfg(any(feature = "firmware", test))]
const RCC_RESET_WINDOW_WATCHDOG: u32 = 1 << 30;
#[cfg(any(feature = "firmware", test))]
const RCC_RESET_LOW_POWER: u32 = 1 << 31;
#[cfg(any(feature = "firmware", test))]
const RETAINED_MAGIC: u32 = 0x4f58_4652;

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
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[cfg(any(feature = "firmware", test))]
struct RetainedContext {
    magic: u32,
    magic_inverse: u32,
    fatal_reason: u32,
    checkpoint: u32,
    detail: i32,
    control_cycle: u32,
    last_control_cycles: u32,
    maximum_control_cycles: u32,
    program_counter: u32,
    link_register: u32,
}

#[cfg(any(feature = "firmware", test))]
impl RetainedContext {
    const fn active() -> Self {
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
        }
    }

    const fn is_valid(self) -> bool {
        self.magic == RETAINED_MAGIC && self.magic_inverse == !RETAINED_MAGIC
    }
}

#[cfg(any(feature = "firmware", test))]
fn compact_reset_flags(raw: u32) -> u8 {
    (u8::from(raw & RCC_RESET_PIN != 0) * reset_flag::PIN)
        | (u8::from(raw & RCC_RESET_POWER != 0) * reset_flag::POWER)
        | (u8::from(raw & RCC_RESET_SOFTWARE != 0) * reset_flag::SOFTWARE)
        | (u8::from(raw & RCC_RESET_INDEPENDENT_WATCHDOG != 0) * reset_flag::INDEPENDENT_WATCHDOG)
        | (u8::from(raw & RCC_RESET_WINDOW_WATCHDOG != 0) * reset_flag::WINDOW_WATCHDOG)
        | (u8::from(raw & RCC_RESET_LOW_POWER != 0) * reset_flag::LOW_POWER)
}

#[cfg(any(feature = "firmware", test))]
fn boot_diagnostics(raw_reset_flags: u32, retained: RetainedContext) -> BootDiagnostics {
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
        }
    } else {
        BootDiagnostics {
            reset_flags,
            ..BootDiagnostics::default()
        }
    }
}

#[cfg(feature = "firmware")]
const RCC_APB1ENR: *mut u32 = 0x4002_101c as *mut u32;
#[cfg(feature = "firmware")]
const RCC_APB1ENR_WWDGEN: u32 = 1 << 11;
#[cfg(feature = "firmware")]
const RCC_CSR: *mut u32 = 0x4002_1024 as *mut u32;
#[cfg(feature = "firmware")]
const RCC_CLEAR_RESET_FLAGS: u32 = 1 << 24;

#[cfg(feature = "firmware")]
const WWDG_CR: *mut u32 = 0x4000_2c00 as *mut u32;
#[cfg(feature = "firmware")]
const WWDG_CFR: *mut u32 = 0x4000_2c04 as *mut u32;
#[cfg(feature = "firmware")]
const WWDG_ENABLE: u32 = 1 << 7;
#[cfg(feature = "firmware")]
const WWDG_PRESCALER_DIVIDE_8: u32 = 3 << 7;
#[cfg(feature = "firmware")]
const WWDG_COUNTER_MAXIMUM: u32 = 0x7f;
#[cfg(feature = "firmware")]
const WWDG_RESET_THRESHOLD: u32 = 0x40;

#[cfg(feature = "firmware")]
const IWDG_KR: *mut u32 = 0x4000_3000 as *mut u32;
#[cfg(feature = "firmware")]
const IWDG_PR: *mut u32 = 0x4000_3004 as *mut u32;
#[cfg(feature = "firmware")]
const IWDG_RLR: *mut u32 = 0x4000_3008 as *mut u32;
#[cfg(feature = "firmware")]
const IWDG_SR: *const u32 = 0x4000_300c as *const u32;
#[cfg(feature = "firmware")]
const IWDG_KEY_START: u32 = 0xcccc;
#[cfg(feature = "firmware")]
const IWDG_KEY_UNLOCK: u32 = 0x5555;
#[cfg(feature = "firmware")]
const IWDG_KEY_FEED: u32 = 0xaaaa;
#[cfg(feature = "firmware")]
const IWDG_PRESCALER_DIVIDE_16: u32 = 2;
#[cfg(feature = "firmware")]
const IWDG_RELOAD_100MS_NOMINAL: u32 = 249;
#[cfg(feature = "firmware")]
const IWDG_PRESCALER_AND_RELOAD_UPDATING: u32 = 0x3;

#[cfg(feature = "firmware")]
const TIMER_UPDATES_PER_WINDOW_FEED: u32 = 256;
#[cfg(feature = "firmware")]
const SYNCHRONIZATION_WAIT_ITERATIONS: u32 = 100_000;

#[cfg(feature = "firmware")]
static STARTED: AtomicBool = AtomicBool::new(false);
#[cfg(feature = "firmware")]
static TIMER_UPDATES_SINCE_FEED: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "firmware")]
static BOOT_RESET_FLAGS: AtomicU8 = AtomicU8::new(0);
#[cfg(feature = "firmware")]
static BOOT_CONTEXT_VALID: AtomicBool = AtomicBool::new(false);
#[cfg(feature = "firmware")]
static BOOT_FATAL_REASON: AtomicU8 = AtomicU8::new(0);
#[cfg(feature = "firmware")]
static BOOT_CHECKPOINT: AtomicU8 = AtomicU8::new(0);
#[cfg(feature = "firmware")]
static BOOT_DETAIL: AtomicI32 = AtomicI32::new(0);
#[cfg(feature = "firmware")]
static BOOT_CONTROL_CYCLE: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "firmware")]
static BOOT_LAST_CONTROL_CYCLES: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "firmware")]
static BOOT_MAXIMUM_CONTROL_CYCLES: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "firmware")]
static BOOT_PROGRAM_COUNTER: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "firmware")]
static BOOT_LINK_REGISTER: AtomicU32 = AtomicU32::new(0);

#[cfg(feature = "firmware")]
#[unsafe(link_section = ".retained.reset_forensics")]
// `memory.x` keeps this above the bootloader and application stack ranges.
static mut RETAINED_CONTEXT: MaybeUninit<RetainedContext> = MaybeUninit::uninit();

#[cfg(feature = "firmware")]
fn retained_context_ptr() -> *mut RetainedContext {
    core::ptr::addr_of_mut!(RETAINED_CONTEXT).cast::<RetainedContext>()
}

/// Captures and clears RCC reset flags before peripheral initialization, then
/// starts a fresh bootloader-safe retained record for the current boot.
#[cfg(feature = "firmware")]
pub fn capture_boot_diagnostics() {
    // SAFETY: RCC_CSR is the F103 reset-status register. The retained section
    // contains only integer fields, so every previous SRAM bit pattern is a
    // valid value to inspect before checking its two-word signature.
    let diagnostics = unsafe {
        let raw_reset_flags = read_volatile(RCC_CSR);
        let retained = read_volatile(retained_context_ptr());
        let diagnostics = boot_diagnostics(raw_reset_flags, retained);
        write_volatile(retained_context_ptr(), RetainedContext::active());
        write_volatile(RCC_CSR, raw_reset_flags | RCC_CLEAR_RESET_FLAGS);
        diagnostics
    };
    BOOT_RESET_FLAGS.store(diagnostics.reset_flags, Ordering::Relaxed);
    BOOT_CONTEXT_VALID.store(diagnostics.retained_context_valid, Ordering::Relaxed);
    BOOT_FATAL_REASON.store(diagnostics.fatal_reason, Ordering::Relaxed);
    BOOT_CHECKPOINT.store(diagnostics.checkpoint, Ordering::Relaxed);
    BOOT_DETAIL.store(i32::from(diagnostics.detail), Ordering::Relaxed);
    BOOT_CONTROL_CYCLE.store(diagnostics.control_cycle, Ordering::Relaxed);
    BOOT_LAST_CONTROL_CYCLES.store(diagnostics.last_control_cycles, Ordering::Relaxed);
    BOOT_MAXIMUM_CONTROL_CYCLES.store(diagnostics.maximum_control_cycles, Ordering::Relaxed);
    BOOT_PROGRAM_COUNTER.store(diagnostics.program_counter, Ordering::Relaxed);
    BOOT_LINK_REGISTER.store(diagnostics.link_register, Ordering::Relaxed);
}

#[cfg(feature = "firmware")]
pub fn boot_diagnostics_snapshot() -> BootDiagnostics {
    BootDiagnostics {
        reset_flags: BOOT_RESET_FLAGS.load(Ordering::Relaxed),
        retained_context_valid: BOOT_CONTEXT_VALID.load(Ordering::Relaxed),
        fatal_reason: BOOT_FATAL_REASON.load(Ordering::Relaxed),
        checkpoint: BOOT_CHECKPOINT.load(Ordering::Relaxed),
        detail: BOOT_DETAIL
            .load(Ordering::Relaxed)
            .clamp(i32::from(i16::MIN), i32::from(i16::MAX)) as i16,
        control_cycle: BOOT_CONTROL_CYCLE.load(Ordering::Relaxed),
        last_control_cycles: BOOT_LAST_CONTROL_CYCLES.load(Ordering::Relaxed),
        maximum_control_cycles: BOOT_MAXIMUM_CONTROL_CYCLES.load(Ordering::Relaxed),
        program_counter: BOOT_PROGRAM_COUNTER.load(Ordering::Relaxed),
        link_register: BOOT_LINK_REGISTER.load(Ordering::Relaxed),
    }
}

#[cfg(feature = "firmware")]
pub fn record_control_cycle(control_cycle: u32) {
    // SAFETY: these aligned u32 fields have one writer, TIM1_UP.
    unsafe {
        write_volatile(
            core::ptr::addr_of_mut!((*retained_context_ptr()).control_cycle),
            control_cycle,
        );
        write_volatile(
            core::ptr::addr_of_mut!((*retained_context_ptr()).checkpoint),
            u32::from(checkpoint::ENTERED),
        );
    }
}

#[cfg(feature = "firmware")]
pub fn record_control_checkpoint(value: u8) {
    // SAFETY: this aligned u32 field has one writer, TIM1_UP.
    unsafe {
        write_volatile(
            core::ptr::addr_of_mut!((*retained_context_ptr()).checkpoint),
            u32::from(value),
        );
    }
}

#[cfg(feature = "firmware")]
pub fn record_control_timing(last: u32, maximum: u32) {
    // SAFETY: these aligned u32 fields have one writer, TIM1_UP.
    unsafe {
        write_volatile(
            core::ptr::addr_of_mut!((*retained_context_ptr()).last_control_cycles),
            last,
        );
        write_volatile(
            core::ptr::addr_of_mut!((*retained_context_ptr()).maximum_control_cycles),
            maximum,
        );
        write_volatile(
            core::ptr::addr_of_mut!((*retained_context_ptr()).checkpoint),
            u32::from(checkpoint::COMPLETE),
        );
    }
}

#[cfg(feature = "firmware")]
pub fn record_safety_loss(reason: u8) {
    // When no exception supersedes it, detail describes the final control
    // safety-loss reason from the existing page-12 enumeration.
    unsafe {
        write_volatile(
            core::ptr::addr_of_mut!((*retained_context_ptr()).detail),
            i32::from(reason),
        );
    }
}

#[cfg(feature = "firmware")]
fn record_fatal(reason: u8, detail: i16, program_counter: u32, link_register: u32) {
    cortex_m::interrupt::disable();
    // SAFETY: interrupts are disabled for this bounded set of aligned
    // volatile writes, so the record cannot mix two fatal handlers.
    unsafe {
        write_volatile(
            core::ptr::addr_of_mut!((*retained_context_ptr()).fatal_reason),
            u32::from(reason),
        );
        write_volatile(
            core::ptr::addr_of_mut!((*retained_context_ptr()).detail),
            i32::from(detail),
        );
        write_volatile(
            core::ptr::addr_of_mut!((*retained_context_ptr()).program_counter),
            program_counter,
        );
        write_volatile(
            core::ptr::addr_of_mut!((*retained_context_ptr()).link_register),
            link_register,
        );
    }
}

/// Starts a roughly 58 ms PCLK1 window watchdog and a nominal 100 ms
/// LSI-clocked independent watchdog. The former is refreshed only by TIM1;
/// the latter is refreshed by foreground code only after observing both TIM1
/// and injected-current progress.
#[cfg(feature = "firmware")]
pub fn start() {
    // SAFETY: called once after clock and control-loop initialization. Both
    // watchdog register blocks and the RCC enable register are fixed by RM0008.
    unsafe {
        write_volatile(RCC_APB1ENR, read_volatile(RCC_APB1ENR) | RCC_APB1ENR_WWDGEN);
        write_volatile(WWDG_CFR, WWDG_PRESCALER_DIVIDE_8 | WWDG_COUNTER_MAXIMUM);

        write_volatile(IWDG_KR, IWDG_KEY_START);
        write_volatile(IWDG_KR, IWDG_KEY_UNLOCK);
        write_volatile(IWDG_PR, IWDG_PRESCALER_DIVIDE_16);
        write_volatile(IWDG_RLR, IWDG_RELOAD_100MS_NOMINAL);
        for _ in 0..SYNCHRONIZATION_WAIT_ITERATIONS {
            if read_volatile(IWDG_SR) & IWDG_PRESCALER_AND_RELOAD_UPDATING == 0 {
                break;
            }
            spin_loop();
        }
        feed_independent();
        write_volatile(WWDG_CR, WWDG_ENABLE | WWDG_COUNTER_MAXIMUM);
    }
    TIMER_UPDATES_SINCE_FEED.store(0, Ordering::Relaxed);
    STARTED.store(true, Ordering::Release);
}

/// Records a real TIM1 update-handler entry and periodically refreshes WWDG.
#[cfg(feature = "firmware")]
pub fn timer_update_entered() {
    if !STARTED.load(Ordering::Acquire) {
        return;
    }
    let updates = TIMER_UPDATES_SINCE_FEED
        .fetch_add(1, Ordering::Relaxed)
        .wrapping_add(1);
    if updates >= TIMER_UPDATES_PER_WINDOW_FEED {
        TIMER_UPDATES_SINCE_FEED.store(0, Ordering::Relaxed);
        feed_window();
    }
}

/// Refreshes IWDG after the caller has independently verified foreground,
/// TIM1-update, and injected-ADC progress.
#[cfg(feature = "firmware")]
pub fn feed_main_loop() {
    if STARTED.load(Ordering::Acquire) {
        // SAFETY: the key register accepts this value at any time after start.
        unsafe { feed_independent() };
    }
}

#[cfg(feature = "firmware")]
fn feed_window() {
    // SAFETY: read/write access to the enabled WWDG register block. Refreshing
    // outside its legal counter range is deliberately skipped.
    unsafe {
        let counter = read_volatile(WWDG_CR) & WWDG_COUNTER_MAXIMUM;
        if (WWDG_RESET_THRESHOLD..WWDG_COUNTER_MAXIMUM).contains(&counter) {
            write_volatile(WWDG_CR, WWDG_ENABLE | WWDG_COUNTER_MAXIMUM);
        }
    }
}

#[cfg(feature = "firmware")]
unsafe fn feed_independent() {
    // SAFETY: caller has established the IWDG register block.
    unsafe { write_volatile(IWDG_KR, IWDG_KEY_FEED) }
}

#[cfg(feature = "firmware")]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    record_fatal(fatal_reason::PANIC, 0, 0, 0);
    fatal_shutdown()
}

#[cfg(feature = "firmware")]
#[exception]
unsafe fn HardFault(frame: &ExceptionFrame) -> ! {
    record_fatal(fatal_reason::HARD_FAULT, 0, frame.pc(), frame.lr());
    fatal_shutdown()
}

#[cfg(feature = "firmware")]
#[exception]
unsafe fn NonMaskableInt() -> ! {
    record_fatal(fatal_reason::NON_MASKABLE_INTERRUPT, 0, 0, 0);
    fatal_shutdown()
}

#[cfg(feature = "firmware")]
#[exception]
unsafe fn DefaultHandler(irqn: i16) {
    record_fatal(fatal_reason::DEFAULT_HANDLER, irqn, 0, 0);
    fatal_shutdown()
}

#[cfg(feature = "firmware")]
fn fatal_shutdown() -> ! {
    cortex_m::interrupt::disable();
    crate::hardware::peripherals::emergency_shutdown();
    loop {
        cortex_m::asm::wfi();
    }
}

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
            ..RetainedContext::active()
        };
        let watchdog = boot_diagnostics(RCC_RESET_INDEPENDENT_WATCHDOG, retained);
        assert!(watchdog.retained_context_valid);
        assert_eq!(watchdog.fatal_reason, fatal_reason::HARD_FAULT);
        assert_eq!(watchdog.checkpoint, checkpoint::DRIVER_COMPLETE);
        assert_eq!(watchdog.control_cycle, 0x1234_5678);
        assert_eq!(watchdog.program_counter, 0x0800_9abc);

        let power = boot_diagnostics(RCC_RESET_POWER, retained);
        assert!(!power.retained_context_valid);
        assert_eq!(power.fatal_reason, fatal_reason::NONE);
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
