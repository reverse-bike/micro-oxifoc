//! STM32F103 RCC, AFIO, and NVIC registers shared by the peripheral drivers.

use core::ptr::{read_volatile, write_volatile};

use crate::config::afio_mapr_with_remap;

pub const RCC_APB2RSTR: *mut u32 = 0x4002_100c as *mut u32;
pub const RCC_APB1RSTR: *mut u32 = 0x4002_1010 as *mut u32;
pub const RCC_AHBENR: *mut u32 = 0x4002_1014 as *mut u32;
pub const RCC_APB2ENR: *mut u32 = 0x4002_1018 as *mut u32;
pub const RCC_APB1ENR: *mut u32 = 0x4002_101c as *mut u32;
pub const RCC_APB2ENR_AFIOEN: u32 = 1;

const AFIO_MAPR: *mut u32 = 0x4001_0004 as *mut u32;
const NVIC_IPR: *mut u8 = 0xe000_e400 as *mut u8;

/// Selects one AFIO MAPR remap field. Every write also restates the
/// firmware's SWJ_CFG selection because that field reads back undefined.
pub unsafe fn remap_afio(field_mask: u32, field_bits: u32) {
    // SAFETY: the caller has exclusive startup access to RCC and AFIO.
    unsafe {
        write_volatile(RCC_APB2ENR, read_volatile(RCC_APB2ENR) | RCC_APB2ENR_AFIOEN);
        write_volatile(
            AFIO_MAPR,
            afio_mapr_with_remap(read_volatile(AFIO_MAPR), field_mask, field_bits),
        );
    }
}

pub unsafe fn set_interrupt_priority(interrupt_number: usize, priority: u8) {
    // SAFETY: each external interrupt owns one byte in NVIC IPR.
    unsafe { write_volatile(NVIC_IPR.add(interrupt_number), priority) }
}
