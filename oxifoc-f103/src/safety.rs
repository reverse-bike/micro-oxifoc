//! Last-resort gate shutdown and progress-gated hardware watchdogs.

use core::hint::spin_loop;
use core::ptr::{read_volatile, write_volatile};
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use cortex_m_rt::{ExceptionFrame, exception};

const RCC_APB1ENR: *mut u32 = 0x4002_101c as *mut u32;
const RCC_APB1ENR_WWDGEN: u32 = 1 << 11;

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
    fatal_shutdown()
}

#[exception]
unsafe fn HardFault(_frame: &ExceptionFrame) -> ! {
    fatal_shutdown()
}

#[exception]
unsafe fn NonMaskableInt() -> ! {
    fatal_shutdown()
}

#[exception]
unsafe fn DefaultHandler(_irqn: i16) {
    fatal_shutdown()
}

fn fatal_shutdown() -> ! {
    cortex_m::interrupt::disable();
    crate::hardware::peripherals::emergency_shutdown();
    loop {
        cortex_m::asm::wfi();
    }
}
