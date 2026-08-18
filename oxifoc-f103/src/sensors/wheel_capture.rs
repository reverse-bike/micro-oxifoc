//! TIM3 capture for the PB4 wheel sensor.

use core::ptr::{read_volatile, write_volatile};
use core::sync::atomic::{AtomicU16, AtomicU32, Ordering, compiler_fence};

use super::wheel::{Capture, extended_capture_timestamp, qualified_interval_us};
use stm32f1::stm32f103::interrupt;

const RCC_APB1ENR: *mut u32 = 0x4002_101c as *mut u32;
const RCC_APB1RSTR: *mut u32 = 0x4002_1010 as *mut u32;
const RCC_APB2ENR: *mut u32 = 0x4002_1018 as *mut u32;
const RCC_APB1ENR_TIM3EN: u32 = 1 << 1;
const RCC_APB2ENR_AFIOEN: u32 = 1;
const RCC_APB2ENR_IOPBEN: u32 = 1 << 3;

const AFIO_MAPR: *mut u32 = 0x4001_0004 as *mut u32;
const GPIOB_CRL: *mut u32 = 0x4001_0c00 as *mut u32;
const GPIOB_BSRR: *mut u32 = 0x4001_0c10 as *mut u32;
const TIM3_BASE: usize = 0x4000_0400;
const TIM3_CR1: *mut u32 = TIM3_BASE as *mut u32;
const TIM3_DIER: *mut u32 = (TIM3_BASE + 0x0c) as *mut u32;
const TIM3_SR: *mut u32 = (TIM3_BASE + 0x10) as *mut u32;
const TIM3_EGR: *mut u32 = (TIM3_BASE + 0x14) as *mut u32;
const TIM3_CCMR1: *mut u32 = (TIM3_BASE + 0x18) as *mut u32;
const TIM3_CCER: *mut u32 = (TIM3_BASE + 0x20) as *mut u32;
const TIM3_CNT: *mut u32 = (TIM3_BASE + 0x24) as *mut u32;
const TIM3_PSC: *mut u32 = (TIM3_BASE + 0x28) as *mut u32;
const TIM3_ARR: *mut u32 = (TIM3_BASE + 0x2c) as *mut u32;
const TIM3_CCR1: *const u32 = (TIM3_BASE + 0x34) as *const u32;

const TIM3_PARTIAL_REMAP: u32 = 2 << 10;
const TIM3_REMAP_MASK: u32 = 3 << 10;
const SWJ_CONFIGURATION_MASK: u32 = 7 << 24;
const SWJ_DISABLED: u32 = 4 << 24;
const WHEEL_PIN: u32 = 4;
const GPIO_INPUT_PULL: u32 = 0x8;
const TIM3_UIF: u32 = 1;
const TIM3_CC1IF: u32 = 1 << 1;
const TIM3_CC1OF: u32 = 1 << 9;
const TIM3_QUIET_OVERFLOWS: u32 = 13;

const FLAG_INITIALIZED: u32 = 1;
const FLAG_PRIMED: u32 = 1 << 1;
const FLAG_QUIET: u32 = 1 << 2;

static SNAPSHOT_SEQUENCE: AtomicU32 = AtomicU32::new(0);
static SNAPSHOT_FLAGS: AtomicU32 = AtomicU32::new(0);
static INTERVAL_US: AtomicU32 = AtomicU32::new(0);
static PULSE_COUNT: AtomicU32 = AtomicU32::new(0);
static CAPTURE_OVERRUNS: AtomicU16 = AtomicU16::new(0);
static OVERFLOW_EPOCH: AtomicU32 = AtomicU32::new(0);
static QUIET_OVERFLOWS: AtomicU32 = AtomicU32::new(0);
static LAST_CAPTURE_TIMESTAMP: AtomicU32 = AtomicU32::new(0);

pub fn initialize() {
    // SAFETY: one-time TIM3, AFIO, and PB4 setup before the interrupt is
    // unmasked. The partial remap routes CH1 to PB4.
    unsafe {
        write_volatile(
            RCC_APB2ENR,
            read_volatile(RCC_APB2ENR) | RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPBEN,
        );
        write_volatile(RCC_APB1ENR, read_volatile(RCC_APB1ENR) | RCC_APB1ENR_TIM3EN);
        write_volatile(
            RCC_APB1RSTR,
            read_volatile(RCC_APB1RSTR) | RCC_APB1ENR_TIM3EN,
        );
        write_volatile(
            RCC_APB1RSTR,
            read_volatile(RCC_APB1RSTR) & !RCC_APB1ENR_TIM3EN,
        );
        write_volatile(
            AFIO_MAPR,
            (read_volatile(AFIO_MAPR) & !(TIM3_REMAP_MASK | SWJ_CONFIGURATION_MASK))
                | TIM3_PARTIAL_REMAP
                | SWJ_DISABLED,
        );
        let shift = WHEEL_PIN * 4;
        write_volatile(
            GPIOB_CRL,
            (read_volatile(GPIOB_CRL) & !(0xf << shift)) | (GPIO_INPUT_PULL << shift),
        );
        write_volatile(GPIOB_BSRR, 1 << WHEEL_PIN);

        write_volatile(TIM3_CR1, 0);
        write_volatile(TIM3_PSC, 71);
        write_volatile(TIM3_ARR, u32::from(u16::MAX));
        write_volatile(TIM3_CNT, 0);
        write_volatile(TIM3_EGR, 1);
        write_volatile(TIM3_CCMR1, 1 | (7 << 4));
        write_volatile(TIM3_CCER, 1 | (1 << 1));
        write_volatile(TIM3_SR, 0);
        write_volatile(TIM3_DIER, TIM3_UIF | TIM3_CC1IF);

        OVERFLOW_EPOCH.store(0, Ordering::Relaxed);
        QUIET_OVERFLOWS.store(0, Ordering::Relaxed);
        LAST_CAPTURE_TIMESTAMP.store(0, Ordering::Relaxed);
        publish(FLAG_INITIALIZED, 0, 0, 0);
        set_interrupt_priority(29, 0x50);
        cortex_m::peripheral::NVIC::unmask(stm32f1::stm32f103::Interrupt::TIM3);
        write_volatile(TIM3_CR1, 1);
    }
}

pub fn snapshot() -> Capture {
    loop {
        let before = SNAPSHOT_SEQUENCE.load(Ordering::Acquire);
        if before & 1 != 0 {
            core::hint::spin_loop();
            continue;
        }
        let flags = SNAPSHOT_FLAGS.load(Ordering::Relaxed);
        let snapshot = Capture {
            primed: flags & FLAG_PRIMED != 0,
            quiet: flags & FLAG_QUIET != 0,
            interval_us: INTERVAL_US.load(Ordering::Relaxed),
            pulse_count: PULSE_COUNT.load(Ordering::Relaxed),
            capture_overruns: CAPTURE_OVERRUNS.load(Ordering::Relaxed),
        };
        compiler_fence(Ordering::Acquire);
        if before == SNAPSHOT_SEQUENCE.load(Ordering::Acquire) {
            return snapshot;
        }
    }
}

#[interrupt]
fn TIM3() {
    // SAFETY: TIM3 is owned by this capture module.
    unsafe {
        let status = read_volatile(TIM3_SR);
        let mut handled = 0;
        let update_pending = status & TIM3_UIF != 0;
        let overflow_epoch = OVERFLOW_EPOCH.load(Ordering::Relaxed);
        if status & TIM3_CC1OF != 0 {
            CAPTURE_OVERRUNS.fetch_add(1, Ordering::Relaxed);
            handled |= TIM3_CC1OF;
        }
        if status & TIM3_CC1IF != 0 {
            let capture = (read_volatile(TIM3_CCR1) & u32::from(u16::MAX)) as u16;
            let timestamp = extended_capture_timestamp(overflow_epoch, capture, update_pending);
            let prior_flags = SNAPSHOT_FLAGS.load(Ordering::Relaxed);
            let pulse_count = PULSE_COUNT.load(Ordering::Relaxed).wrapping_add(1);
            let interval = qualified_interval_us(
                LAST_CAPTURE_TIMESTAMP.load(Ordering::Relaxed),
                timestamp,
                prior_flags & FLAG_PRIMED != 0,
                prior_flags & FLAG_QUIET != 0,
            );
            LAST_CAPTURE_TIMESTAMP.store(timestamp, Ordering::Relaxed);
            QUIET_OVERFLOWS.store(
                u32::from(update_pending && capture >= 0x8000),
                Ordering::Relaxed,
            );
            publish(
                FLAG_INITIALIZED | FLAG_PRIMED,
                interval,
                pulse_count,
                CAPTURE_OVERRUNS.load(Ordering::Relaxed),
            );
            handled |= TIM3_CC1IF;
        } else if update_pending {
            let previous = QUIET_OVERFLOWS.load(Ordering::Relaxed);
            let quiet = previous.saturating_add(1).min(TIM3_QUIET_OVERFLOWS);
            QUIET_OVERFLOWS.store(quiet, Ordering::Relaxed);
            if previous < TIM3_QUIET_OVERFLOWS && quiet == TIM3_QUIET_OVERFLOWS {
                publish(
                    FLAG_INITIALIZED | FLAG_QUIET,
                    0,
                    PULSE_COUNT.load(Ordering::Relaxed),
                    CAPTURE_OVERRUNS.load(Ordering::Relaxed),
                );
            }
        }
        if update_pending {
            OVERFLOW_EPOCH.store(overflow_epoch.wrapping_add(1), Ordering::Relaxed);
            handled |= TIM3_UIF;
        }
        if handled != 0 {
            write_volatile(TIM3_SR, !handled);
        }
    }
}

fn publish(flags: u32, interval_us: u32, pulse_count: u32, capture_overruns: u16) {
    SNAPSHOT_SEQUENCE.fetch_add(1, Ordering::AcqRel);
    compiler_fence(Ordering::Release);
    SNAPSHOT_FLAGS.store(flags, Ordering::Relaxed);
    INTERVAL_US.store(interval_us, Ordering::Relaxed);
    PULSE_COUNT.store(pulse_count, Ordering::Relaxed);
    CAPTURE_OVERRUNS.store(capture_overruns, Ordering::Relaxed);
    compiler_fence(Ordering::Release);
    SNAPSHOT_SEQUENCE.fetch_add(1, Ordering::Release);
}

unsafe fn set_interrupt_priority(interrupt_number: usize, priority: u8) {
    const NVIC_IPR: *mut u8 = 0xe000_e400 as *mut u8;
    // SAFETY: each external interrupt owns one byte in NVIC IPR.
    unsafe { write_volatile(NVIC_IPR.add(interrupt_number), priority) }
}
