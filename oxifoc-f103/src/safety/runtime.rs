//! STM32F103 shutdown, watchdog, and reset-forensics runtime.

use core::cell::UnsafeCell;
use core::hint::spin_loop;
use core::mem::MaybeUninit;
use core::ptr::{read_volatile, write_volatile};
use core::sync::atomic::{AtomicBool, AtomicI32, AtomicU8, AtomicU32, Ordering};

use cortex_m_rt::{ExceptionFrame, exception};

use super::*;
use crate::hardware::system::RCC_APB1ENR;

const RCC_APB1ENR_WWDGEN: u32 = 1 << 11;
const RCC_CSR: *mut u32 = 0x4002_1024 as *mut u32;
const RCC_CLEAR_RESET_FLAGS: u32 = 1 << 24;

const WWDG_CR: *mut u32 = 0x4000_2c00 as *mut u32;
const WWDG_CFR: *mut u32 = 0x4000_2c04 as *mut u32;
const WWDG_ENABLE: u32 = 1 << 7;
const WWDG_PRESCALER_DIVIDE_8: u32 = 3 << 7;
const WWDG_COUNTER_MAXIMUM: u32 = 0x7f;
const WWDG_RESET_THRESHOLD: u32 = 0x40;

const IWDG_KR: *mut u32 = 0x4000_3000 as *mut u32;
const IWDG_PR: *mut u32 = 0x4000_3004 as *mut u32;
const IWDG_RLR: *mut u32 = 0x4000_3008 as *mut u32;
const IWDG_SR: *const u32 = 0x4000_300c as *const u32;
const IWDG_KEY_START: u32 = 0xcccc;
const IWDG_KEY_UNLOCK: u32 = 0x5555;
const IWDG_KEY_FEED: u32 = 0xaaaa;
const IWDG_PRESCALER_DIVIDE_16: u32 = 2;
const IWDG_RELOAD_100MS_NOMINAL: u32 = 249;
const IWDG_PRESCALER_AND_RELOAD_UPDATING: u32 = 0x3;

const TIMER_UPDATES_PER_WINDOW_FEED: u32 = 256;
const SYNCHRONIZATION_WAIT_ITERATIONS: u32 = 100_000;

static STARTED: AtomicBool = AtomicBool::new(false);
static TIMER_UPDATES_SINCE_FEED: AtomicU32 = AtomicU32::new(0);
static BOOT_RESET_FLAGS: AtomicU8 = AtomicU8::new(0);
static BOOT_CONTEXT_VALID: AtomicBool = AtomicBool::new(false);
static BOOT_FATAL_REASON: AtomicU8 = AtomicU8::new(0);
static BOOT_CHECKPOINT: AtomicU8 = AtomicU8::new(0);
static BOOT_DETAIL: AtomicI32 = AtomicI32::new(0);
static BOOT_CONTROL_CYCLE: AtomicU32 = AtomicU32::new(0);
static BOOT_LAST_CONTROL_CYCLES: AtomicU32 = AtomicU32::new(0);
static BOOT_MAXIMUM_CONTROL_CYCLES: AtomicU32 = AtomicU32::new(0);
static BOOT_PROGRAM_COUNTER: AtomicU32 = AtomicU32::new(0);
static BOOT_LINK_REGISTER: AtomicU32 = AtomicU32::new(0);
static BOOT_PWM_FAILURE_0: AtomicU32 = AtomicU32::new(0);
static BOOT_PWM_FAILURE_1: AtomicU32 = AtomicU32::new(0);
static BOOT_PWM_FAILURE_2: AtomicU32 = AtomicU32::new(0);
static BOOT_PWM_FAILURE_3: AtomicU32 = AtomicU32::new(0);
struct PwmFailureCell(UnsafeCell<PwmFailureContext>);

// SAFETY: writes occur before interrupts start or in the non-nesting TIM1
// handlers; foreground reads mask interrupts around the complete copy.
unsafe impl Sync for PwmFailureCell {}

static LATEST_PWM_FAILURE: PwmFailureCell =
    PwmFailureCell(UnsafeCell::new(PwmFailureContext::EMPTY));

#[unsafe(link_section = ".retained.reset_forensics")]
// `memory.x` keeps this above the bootloader and application stack ranges.
static mut RETAINED_CONTEXT: MaybeUninit<RetainedContext> = MaybeUninit::uninit();

fn retained_context_ptr() -> *mut RetainedContext {
    core::ptr::addr_of_mut!(RETAINED_CONTEXT).cast::<RetainedContext>()
}

/// Captures and clears RCC reset flags before peripheral initialization, then
/// starts a fresh bootloader-safe retained record for the current boot.
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
    let pwm_failure = diagnostics.pwm_failure.words();
    BOOT_PWM_FAILURE_0.store(pwm_failure[0], Ordering::Relaxed);
    BOOT_PWM_FAILURE_1.store(pwm_failure[1], Ordering::Relaxed);
    BOOT_PWM_FAILURE_2.store(pwm_failure[2], Ordering::Relaxed);
    BOOT_PWM_FAILURE_3.store(pwm_failure[3], Ordering::Relaxed);
    store_latest_pwm_failure(pwm_failure);
}

fn store_latest_pwm_failure(words: [u32; 4]) {
    // SAFETY: callers satisfy PwmFailureCell's writer exclusion contract.
    unsafe { *LATEST_PWM_FAILURE.0.get() = PwmFailureContext { words } }
}

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
        pwm_failure: PwmFailureContext {
            words: [
                BOOT_PWM_FAILURE_0.load(Ordering::Relaxed),
                BOOT_PWM_FAILURE_1.load(Ordering::Relaxed),
                BOOT_PWM_FAILURE_2.load(Ordering::Relaxed),
                BOOT_PWM_FAILURE_3.load(Ordering::Relaxed),
            ],
        },
    }
}

pub fn latest_pwm_failure_snapshot() -> PwmFailureContext {
    cortex_m::interrupt::free(|_| {
        // SAFETY: interrupts remain masked for the complete copy.
        unsafe { *LATEST_PWM_FAILURE.0.get() }
    })
}

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

pub fn record_control_checkpoint(value: u8) {
    // SAFETY: this aligned u32 field has one writer, TIM1_UP.
    unsafe {
        write_volatile(
            core::ptr::addr_of_mut!((*retained_context_ptr()).checkpoint),
            u32::from(value),
        );
    }
}

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

pub fn record_pwm_failure(context: PwmFailureContext) {
    // SAFETY: TIM1_UP and TIM1_BRK run at the same priority and cannot preempt
    // one another. The cause word is written last and commits the complete
    // first failure of the current fault episode.
    let words = context.words();
    unsafe {
        let destination =
            core::ptr::addr_of_mut!((*retained_context_ptr()).pwm_failure).cast::<u32>();
        if !pwm_failure_slot_is_empty(read_volatile(destination)) {
            return;
        }
        write_volatile(destination.add(1), words[1]);
        write_volatile(destination.add(2), words[2]);
        write_volatile(destination.add(3), words[3]);
        write_volatile(destination, words[0]);
    }
    store_latest_pwm_failure(words);
}

pub(crate) fn clear_current_pwm_failure() {
    // SAFETY: the hardware fault acknowledgement calls this with interrupts
    // masked while every motor channel is disabled.
    unsafe {
        let destination =
            core::ptr::addr_of_mut!((*retained_context_ptr()).pwm_failure).cast::<u32>();
        write_volatile(destination, 0);
        write_volatile(destination.add(1), 0);
        write_volatile(destination.add(2), 0);
        write_volatile(destination.add(3), 0);
    }
}

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
pub fn feed_main_loop() {
    if STARTED.load(Ordering::Acquire) {
        // SAFETY: the key register accepts this value at any time after start.
        unsafe { feed_independent() };
    }
}

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

unsafe fn feed_independent() {
    // SAFETY: caller has established the IWDG register block.
    unsafe { write_volatile(IWDG_KR, IWDG_KEY_FEED) }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    record_fatal(fatal_reason::PANIC, 0, 0, 0);
    fatal_shutdown()
}

#[exception]
unsafe fn HardFault(frame: &ExceptionFrame) -> ! {
    record_fatal(fatal_reason::HARD_FAULT, 0, frame.pc(), frame.lr());
    fatal_shutdown()
}

#[exception]
unsafe fn NonMaskableInt() -> ! {
    record_fatal(fatal_reason::NON_MASKABLE_INTERRUPT, 0, 0, 0);
    fatal_shutdown()
}

#[exception]
unsafe fn DefaultHandler(irqn: i16) {
    record_fatal(fatal_reason::DEFAULT_HANDLER, irqn, 0, 0);
    fatal_shutdown()
}

fn fatal_shutdown() -> ! {
    cortex_m::interrupt::disable();
    crate::hardware::peripherals::emergency_shutdown();
    loop {
        cortex_m::asm::wfi();
    }
}
