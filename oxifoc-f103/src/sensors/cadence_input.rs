//! PB9 input for the stock active-low cadence sensor.

use core::ptr::{read_volatile, write_volatile};

use crate::config;

const RCC_APB2ENR: *mut u32 = 0x4002_1018 as *mut u32;
const RCC_APB2ENR_IOPBEN: u32 = 1 << 3;
const GPIOB_CRH: *mut u32 = 0x4001_0c04 as *mut u32;
const GPIOB_IDR: *const u32 = 0x4001_0c08 as *const u32;
const GPIOB_BSRR: *mut u32 = 0x4001_0c10 as *mut u32;
const GPIO_INPUT_PULL: u32 = 0x8;
const PIN: u32 = config::CADENCE_GPIO_PIN as u32;

const _: () = assert!(config::CADENCE_GPIO_PORT == b'B');
const _: () = assert!(PIN >= 8 && PIN < 16);

pub fn initialize() {
    // SAFETY: one-time PB9 input setup before interrupts are unmasked.
    unsafe {
        write_volatile(RCC_APB2ENR, read_volatile(RCC_APB2ENR) | RCC_APB2ENR_IOPBEN);
        write_volatile(GPIOB_BSRR, 1 << PIN);
        let shift = (PIN - 8) * 4;
        write_volatile(
            GPIOB_CRH,
            (read_volatile(GPIOB_CRH) & !(0xf << shift)) | (GPIO_INPUT_PULL << shift),
        );
    }
}

pub fn pin_is_low() -> bool {
    // SAFETY: GPIOB IDR is a read-only peripheral register.
    unsafe { read_volatile(GPIOB_IDR) & (1 << PIN) == 0 }
}
