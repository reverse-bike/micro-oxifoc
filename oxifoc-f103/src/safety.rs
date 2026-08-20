//! Last-resort gate shutdown, watchdogs, and reset forensics.

#[cfg(feature = "board")]
use core::cell::UnsafeCell;
#[cfg(feature = "board")]
use core::hint::spin_loop;
#[cfg(feature = "board")]
use core::mem::MaybeUninit;
#[cfg(feature = "board")]
use core::ptr::{read_volatile, write_volatile};
#[cfg(feature = "board")]
use core::sync::atomic::{AtomicBool, AtomicI32, AtomicU8, AtomicU32, Ordering};
#[cfg(feature = "board")]
use cortex_m_rt::{ExceptionFrame, exception};

#[cfg(any(feature = "board", test))]
const RCC_RESET_PIN: u32 = 1 << 26;
#[cfg(any(feature = "board", test))]
const RCC_RESET_POWER: u32 = 1 << 27;
#[cfg(any(feature = "board", test))]
const RCC_RESET_SOFTWARE: u32 = 1 << 28;
#[cfg(any(feature = "board", test))]
const RCC_RESET_INDEPENDENT_WATCHDOG: u32 = 1 << 29;
#[cfg(any(feature = "board", test))]
const RCC_RESET_WINDOW_WATCHDOG: u32 = 1 << 30;
#[cfg(any(feature = "board", test))]
const RCC_RESET_LOW_POWER: u32 = 1 << 31;
#[cfg(any(feature = "board", test))]
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

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[cfg(any(feature = "board", test))]
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
    pwm_failure: PwmFailureContext,
}

#[cfg(any(feature = "board", test))]
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
            pwm_failure: PwmFailureContext::EMPTY,
        }
    }

    const fn is_valid(self) -> bool {
        self.magic == RETAINED_MAGIC && self.magic_inverse == !RETAINED_MAGIC
    }
}

#[cfg(any(feature = "board", test))]
fn compact_reset_flags(raw: u32) -> u8 {
    (u8::from(raw & RCC_RESET_PIN != 0) * reset_flag::PIN)
        | (u8::from(raw & RCC_RESET_POWER != 0) * reset_flag::POWER)
        | (u8::from(raw & RCC_RESET_SOFTWARE != 0) * reset_flag::SOFTWARE)
        | (u8::from(raw & RCC_RESET_INDEPENDENT_WATCHDOG != 0) * reset_flag::INDEPENDENT_WATCHDOG)
        | (u8::from(raw & RCC_RESET_WINDOW_WATCHDOG != 0) * reset_flag::WINDOW_WATCHDOG)
        | (u8::from(raw & RCC_RESET_LOW_POWER != 0) * reset_flag::LOW_POWER)
}

#[cfg(any(feature = "board", test))]
const fn pwm_failure_slot_is_empty(header: u32) -> bool {
    header as u8 == pwm_failure_cause::NONE
}

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

#[cfg(any(feature = "board", test))]
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
            pwm_failure: retained.pwm_failure,
        }
    } else {
        BootDiagnostics {
            reset_flags,
            ..BootDiagnostics::default()
        }
    }
}

#[cfg(feature = "board")]
const RCC_APB1ENR: *mut u32 = 0x4002_101c as *mut u32;
#[cfg(feature = "board")]
const RCC_APB1ENR_WWDGEN: u32 = 1 << 11;
#[cfg(feature = "board")]
const RCC_CSR: *mut u32 = 0x4002_1024 as *mut u32;
#[cfg(feature = "board")]
const RCC_CLEAR_RESET_FLAGS: u32 = 1 << 24;

#[cfg(feature = "board")]
const WWDG_CR: *mut u32 = 0x4000_2c00 as *mut u32;
#[cfg(feature = "board")]
const WWDG_CFR: *mut u32 = 0x4000_2c04 as *mut u32;
#[cfg(feature = "board")]
const WWDG_ENABLE: u32 = 1 << 7;
#[cfg(feature = "board")]
const WWDG_PRESCALER_DIVIDE_8: u32 = 3 << 7;
#[cfg(feature = "board")]
const WWDG_COUNTER_MAXIMUM: u32 = 0x7f;
#[cfg(feature = "board")]
const WWDG_RESET_THRESHOLD: u32 = 0x40;

#[cfg(feature = "board")]
const IWDG_KR: *mut u32 = 0x4000_3000 as *mut u32;
#[cfg(feature = "board")]
const IWDG_PR: *mut u32 = 0x4000_3004 as *mut u32;
#[cfg(feature = "board")]
const IWDG_RLR: *mut u32 = 0x4000_3008 as *mut u32;
#[cfg(feature = "board")]
const IWDG_SR: *const u32 = 0x4000_300c as *const u32;
#[cfg(feature = "board")]
const IWDG_KEY_START: u32 = 0xcccc;
#[cfg(feature = "board")]
const IWDG_KEY_UNLOCK: u32 = 0x5555;
#[cfg(feature = "board")]
const IWDG_KEY_FEED: u32 = 0xaaaa;
#[cfg(feature = "board")]
const IWDG_PRESCALER_DIVIDE_16: u32 = 2;
#[cfg(feature = "board")]
const IWDG_RELOAD_100MS_NOMINAL: u32 = 249;
#[cfg(feature = "board")]
const IWDG_PRESCALER_AND_RELOAD_UPDATING: u32 = 0x3;

#[cfg(feature = "board")]
const TIMER_UPDATES_PER_WINDOW_FEED: u32 = 256;
#[cfg(feature = "board")]
const SYNCHRONIZATION_WAIT_ITERATIONS: u32 = 100_000;

#[cfg(feature = "board")]
static STARTED: AtomicBool = AtomicBool::new(false);
#[cfg(feature = "board")]
static TIMER_UPDATES_SINCE_FEED: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "board")]
static BOOT_RESET_FLAGS: AtomicU8 = AtomicU8::new(0);
#[cfg(feature = "board")]
static BOOT_CONTEXT_VALID: AtomicBool = AtomicBool::new(false);
#[cfg(feature = "board")]
static BOOT_FATAL_REASON: AtomicU8 = AtomicU8::new(0);
#[cfg(feature = "board")]
static BOOT_CHECKPOINT: AtomicU8 = AtomicU8::new(0);
#[cfg(feature = "board")]
static BOOT_DETAIL: AtomicI32 = AtomicI32::new(0);
#[cfg(feature = "board")]
static BOOT_CONTROL_CYCLE: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "board")]
static BOOT_LAST_CONTROL_CYCLES: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "board")]
static BOOT_MAXIMUM_CONTROL_CYCLES: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "board")]
static BOOT_PROGRAM_COUNTER: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "board")]
static BOOT_LINK_REGISTER: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "board")]
static BOOT_PWM_FAILURE_0: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "board")]
static BOOT_PWM_FAILURE_1: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "board")]
static BOOT_PWM_FAILURE_2: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "board")]
static BOOT_PWM_FAILURE_3: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "board")]
struct PwmFailureCell(UnsafeCell<PwmFailureContext>);

#[cfg(feature = "board")]
// SAFETY: writes occur before interrupts start or in the non-nesting TIM1
// handlers; foreground reads mask interrupts around the complete copy.
unsafe impl Sync for PwmFailureCell {}

#[cfg(feature = "board")]
static LATEST_PWM_FAILURE: PwmFailureCell =
    PwmFailureCell(UnsafeCell::new(PwmFailureContext::EMPTY));

#[cfg(feature = "board")]
#[unsafe(link_section = ".retained.reset_forensics")]
// `memory.x` keeps this above the bootloader and application stack ranges.
static mut RETAINED_CONTEXT: MaybeUninit<RetainedContext> = MaybeUninit::uninit();

#[cfg(feature = "board")]
fn retained_context_ptr() -> *mut RetainedContext {
    core::ptr::addr_of_mut!(RETAINED_CONTEXT).cast::<RetainedContext>()
}

/// Captures and clears RCC reset flags before peripheral initialization, then
/// starts a fresh bootloader-safe retained record for the current boot.
#[cfg(feature = "board")]
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

#[cfg(feature = "board")]
fn store_latest_pwm_failure(words: [u32; 4]) {
    // SAFETY: callers satisfy PwmFailureCell's writer exclusion contract.
    unsafe { *LATEST_PWM_FAILURE.0.get() = PwmFailureContext { words } }
}

#[cfg(feature = "board")]
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

#[cfg(feature = "board")]
pub fn latest_pwm_failure_snapshot() -> PwmFailureContext {
    cortex_m::interrupt::free(|_| {
        // SAFETY: interrupts remain masked for the complete copy.
        unsafe { *LATEST_PWM_FAILURE.0.get() }
    })
}

#[cfg(feature = "board")]
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

#[cfg(feature = "board")]
pub fn record_control_checkpoint(value: u8) {
    // SAFETY: this aligned u32 field has one writer, TIM1_UP.
    unsafe {
        write_volatile(
            core::ptr::addr_of_mut!((*retained_context_ptr()).checkpoint),
            u32::from(value),
        );
    }
}

#[cfg(feature = "board")]
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

#[cfg(feature = "board")]
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

#[cfg(feature = "board")]
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

#[cfg(feature = "board")]
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

#[cfg(feature = "board")]
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
#[cfg(feature = "board")]
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
#[cfg(feature = "board")]
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
#[cfg(feature = "board")]
pub fn feed_main_loop() {
    if STARTED.load(Ordering::Acquire) {
        // SAFETY: the key register accepts this value at any time after start.
        unsafe { feed_independent() };
    }
}

#[cfg(feature = "board")]
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

#[cfg(feature = "board")]
unsafe fn feed_independent() {
    // SAFETY: caller has established the IWDG register block.
    unsafe { write_volatile(IWDG_KR, IWDG_KEY_FEED) }
}

#[cfg(feature = "board")]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    record_fatal(fatal_reason::PANIC, 0, 0, 0);
    fatal_shutdown()
}

#[cfg(feature = "board")]
#[exception]
unsafe fn HardFault(frame: &ExceptionFrame) -> ! {
    record_fatal(fatal_reason::HARD_FAULT, 0, frame.pc(), frame.lr());
    fatal_shutdown()
}

#[cfg(feature = "board")]
#[exception]
unsafe fn NonMaskableInt() -> ! {
    record_fatal(fatal_reason::NON_MASKABLE_INTERRUPT, 0, 0, 0);
    fatal_shutdown()
}

#[cfg(feature = "board")]
#[exception]
unsafe fn DefaultHandler(irqn: i16) {
    record_fatal(fatal_reason::DEFAULT_HANDLER, irqn, 0, 0);
    fatal_shutdown()
}

#[cfg(feature = "board")]
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
