//! Minimal, fail-closed STM32F103 register bring-up.
//!
//! No motor output is configured here. The first operation enables GPIOA and
//! drives PA2 high, which disables the active-low power stage before TIM1,
//! ADC, or Hall peripherals are touched.

use core::ptr::{read_volatile, write_volatile};
use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU32, Ordering};
use stm32f1::stm32f103::interrupt;

const RCC_APB2ENR: *mut u32 = 0x4002_1018 as *mut u32;
const RCC_APB2RSTR: *mut u32 = 0x4002_100c as *mut u32;
const RCC_CR: *mut u32 = 0x4002_1000 as *mut u32;
const RCC_CFGR: *mut u32 = 0x4002_1004 as *mut u32;
const RCC_APB1ENR: *mut u32 = 0x4002_101c as *mut u32;
const RCC_APB1RSTR: *mut u32 = 0x4002_1010 as *mut u32;
const FLASH_ACR: *mut u32 = 0x4002_2000 as *mut u32;
const SCB_VTOR: *mut u32 = 0xe000_ed08 as *mut u32;
const NVIC_IPR: *mut u8 = 0xe000_e400 as *mut u8;
const CORE_DEBUG_DEMCR: *mut u32 = 0xe000_edfc as *mut u32;
const DWT_CTRL: *mut u32 = 0xe000_1000 as *mut u32;
const DWT_CYCCNT: *mut u32 = 0xe000_1004 as *mut u32;
const CORE_DEBUG_TRACE_ENABLE: u32 = 1 << 24;
const DWT_CYCLE_COUNTER_ENABLE: u32 = 1;

const RCC_CR_HSION: u32 = 1;
const RCC_CR_HSIRDY: u32 = 1 << 1;
const RCC_CR_HSEON: u32 = 1 << 16;
const RCC_CR_HSERDY: u32 = 1 << 17;
const RCC_CR_CSSON: u32 = 1 << 19;
const RCC_CR_PLLON: u32 = 1 << 24;
const RCC_CR_PLLRDY: u32 = 1 << 25;
const RCC_CFGR_CLOCK_MASK: u32 = 0x003f_fcf3;
const RCC_CFGR_HSE_PLL_X9_72MHZ: u32 = (0b100 << 8) | (0b10 << 14) | (1 << 16) | (0b0111 << 18);
const RCC_CFGR_SWITCH_HSI: u32 = 0;
const RCC_CFGR_SWITCH_PLL: u32 = 0b10;
const RCC_CFGR_STATUS_HSI: u32 = 0;
const RCC_CFGR_STATUS_PLL: u32 = 0b10 << 2;
const FLASH_ACR_72MHZ: u32 = (1 << 4) | 0b010;
const CLOCK_WAIT_ITERATIONS: u32 = 1_000_000;

const RCC_APB2ENR_IOPAEN: u32 = 1 << 2;
const RCC_APB2ENR_AFIOEN: u32 = 1;
const RCC_APB2ENR_IOPBEN: u32 = 1 << 3;
const RCC_APB2ENR_IOPCEN: u32 = 1 << 4;
const RCC_APB2ENR_ADC1EN: u32 = 1 << 9;
const RCC_APB2ENR_ADC2EN: u32 = 1 << 10;
const RCC_APB2ENR_TIM1EN: u32 = 1 << 11;
const RCC_APB1ENR_TIM2EN: u32 = 1;

const GPIOA_CRL: *mut u32 = 0x4001_0800 as *mut u32;
const GPIOA_CRH: *mut u32 = 0x4001_0804 as *mut u32;
const GPIOA_IDR: *const u32 = 0x4001_0808 as *const u32;
const GPIOA_ODR: *const u32 = 0x4001_080c as *const u32;
const GPIOA_BSRR: *mut u32 = 0x4001_0810 as *mut u32;
const AFIO_MAPR: *mut u32 = 0x4001_0004 as *mut u32;
const GPIOB_CRL: *mut u32 = 0x4001_0c00 as *mut u32;
const GPIOB_CRH: *mut u32 = 0x4001_0c04 as *mut u32;
const GPIOB_IDR: *const u32 = 0x4001_0c08 as *const u32;
const GPIOB_BSRR: *mut u32 = 0x4001_0c10 as *mut u32;
const GPIOC_CRL: *mut u32 = 0x4001_1000 as *mut u32;
const PA2_CONFIG_MASK: u32 = 0b1111 << 8;
const PA2_OUTPUT_PUSH_PULL_2MHZ: u32 = 0b0010 << 8;
const PA2_SET: u32 = 1 << 2;

const TIM1_CR1: *mut u32 = 0x4001_2c00 as *mut u32;
const TIM1_CR2: *mut u32 = 0x4001_2c04 as *mut u32;
const TIM1_DIER: *mut u32 = 0x4001_2c0c as *mut u32;
const TIM1_SR: *mut u32 = 0x4001_2c10 as *mut u32;
const TIM1_EGR: *mut u32 = 0x4001_2c14 as *mut u32;
const TIM1_CCMR1: *mut u32 = 0x4001_2c18 as *mut u32;
const TIM1_CCMR2: *mut u32 = 0x4001_2c1c as *mut u32;
const TIM1_CCER: *mut u32 = 0x4001_2c20 as *mut u32;
const TIM1_CNT: *mut u32 = 0x4001_2c24 as *mut u32;
const TIM1_PSC: *mut u32 = 0x4001_2c28 as *mut u32;
const TIM1_ARR: *mut u32 = 0x4001_2c2c as *mut u32;
const TIM1_RCR: *mut u32 = 0x4001_2c30 as *mut u32;
const TIM1_CCR1: *mut u32 = 0x4001_2c34 as *mut u32;
const TIM1_CCR2: *mut u32 = 0x4001_2c38 as *mut u32;
const TIM1_CCR3: *mut u32 = 0x4001_2c3c as *mut u32;
const TIM1_CCR4: *mut u32 = 0x4001_2c40 as *mut u32;
const TIM1_BDTR: *mut u32 = 0x4001_2c44 as *mut u32;

const PWM_PIN_AF_PUSH_PULL_50MHZ: u32 = 0b1011;
const BREAK_PIN_INPUT_PULL: u32 = 0b1000;
const HALL_PIN_INPUT_FLOATING: u32 = 0b0100;
const TIM1_CENTER_ALIGNED_MODE_3: u32 = 0b11 << 5;
const TIM1_AUTO_RELOAD_PRELOAD: u32 = 1 << 7;
const TIM1_CLOCK_DIVISION_2: u32 = 0b01 << 8;
const TIM1_DIRECTION_DOWN: u32 = 1 << 4;
const TIM1_COMPLEMENTARY_IDLE_STATES: u32 = 0x2a00;
const TIM1_PASSIVE_CCER: u32 = 0x1888;
const TIM1_ACTIVE_CCER: u32 = 0x1ddd;
const TIM1_OFF_STATE_RUN: u32 = 1 << 11;
const TIM1_OFF_STATE_IDLE: u32 = 1 << 10;
const TIM1_LOCK_LEVEL_1: u32 = 1 << 8;
const TIM1_BREAK_ENABLE: u32 = 1 << 12;
const TIM1_MAIN_OUTPUT_ENABLE: u32 = 1 << 15;

const ADC1_BASE: usize = 0x4001_2400;
const ADC2_BASE: usize = 0x4001_2800;
const ADC_SR_OFFSET: usize = 0x00;
const ADC_CR1_OFFSET: usize = 0x04;
const ADC_CR2_OFFSET: usize = 0x08;
const ADC_SMPR1_OFFSET: usize = 0x0c;
const ADC_JSQR_OFFSET: usize = 0x38;
const ADC_JDR1_OFFSET: usize = 0x3c;
const ADC_ADON: u32 = 1;
const ADC_CAL: u32 = 1 << 2;
const ADC_RSTCAL: u32 = 1 << 3;
const ADC_JEOC: u32 = 1 << 2;
const ADC_JEXTSEL_MASK: u32 = 0b111 << 12;
const ADC_JEXTSEL_TIM1_CC4: u32 = 0b001 << 12;
const ADC_JEXTSEL_SOFTWARE: u32 = 0b111 << 12;
const ADC_JEXTTRIG: u32 = 1 << 15;
const ADC_JSWSTART: u32 = 1 << 21;
const ADC_CALIBRATION_SAMPLES: u32 = 256;
const ADC_WAIT_ITERATIONS: u32 = 100_000;

const TIM2_CR1: *mut u32 = 0x4000_0000 as *mut u32;
const TIM2_CR2: *mut u32 = 0x4000_0004 as *mut u32;
const TIM2_SMCR: *mut u32 = 0x4000_0008 as *mut u32;
const TIM2_DIER: *mut u32 = 0x4000_000c as *mut u32;
const TIM2_SR: *mut u32 = 0x4000_0010 as *mut u32;
const TIM2_EGR: *mut u32 = 0x4000_0014 as *mut u32;
const TIM2_CCMR1: *mut u32 = 0x4000_0018 as *mut u32;
const TIM2_CCER: *mut u32 = 0x4000_0020 as *mut u32;
const TIM2_CNT: *const u32 = 0x4000_0024 as *const u32;
const TIM2_PSC: *mut u32 = 0x4000_0028 as *mut u32;
const TIM2_ARR: *mut u32 = 0x4000_002c as *mut u32;
const TIM2_CCR1: *const u32 = 0x4000_0034 as *const u32;
const TIM2_CCR2: *const u32 = 0x4000_0038 as *const u32;
const TIM2_UIF: u32 = 1;
const TIM2_CC1IF: u32 = 1 << 1;
const TIM2_CC2IF: u32 = 1 << 2;
const TIM2_CC1OF: u32 = 1 << 9;
const TIM2_CC2OF: u32 = 1 << 10;
const TIM2_UIE: u32 = 1;
const TIM2_CC1IE: u32 = 1 << 1;
const TIM2_CC2IE: u32 = 1 << 2;
const TIM2_CAPTURE_FLAGS: u32 = TIM2_CC1IF | TIM2_CC2IF;
const TIM2_OVERCAPTURE_FLAGS: u32 = TIM2_CC1OF | TIM2_CC2OF;
const TIM2_UPDATE_REQUEST_OVERFLOW_ONLY: u32 = 1 << 2;
const TIM2_HALL_XOR_ENABLE: u32 = 1 << 7;
const TIM2_SLAVE_RESET_MODE: u32 = 0b100;
const TIM2_TRIGGER_TI1_EDGE_DETECTOR: u32 = 0b100 << 4;
const TIM2_MASTER_SLAVE_MODE_ENABLE: u32 = 1 << 7;
const TIM2_CAPTURE_1_INPUT_FILTERED: u32 = 0b01 | (0b1111 << 4);
const TIM2_CAPTURE_2_INPUT_FILTERED_INDIRECT: u32 = (0b10 << 8) | (0b1111 << 12);
const TIM2_CAPTURE_1_ENABLE: u32 = 1;
const TIM2_CAPTURE_2_ENABLE_FALLING: u32 = (1 << 4) | (1 << 5);
const TIM2_TICKS_PER_OVERFLOW: u32 = 65_536;

static HALL_RAW: AtomicU8 = AtomicU8::new(0);
static HALL_INTERVAL_US: AtomicU32 = AtomicU32::new(0);
static HALL_SEQUENCE: AtomicU32 = AtomicU32::new(0);
static TIM2_OVERFLOWS: AtomicU32 = AtomicU32::new(0);
static HALL_CONFIGURED: AtomicBool = AtomicBool::new(false);
static FAULT_FLAGS: AtomicU32 = AtomicU32::new(0);

pub const FAULT_HARDWARE_BREAK: u32 = 1;
pub const FAULT_PWM_RANGE: u32 = 1 << 1;
pub const FAULT_HALL_CAPTURE: u32 = 1 << 2;
pub const FAULT_CONTROL_TIMING: u32 = 1 << 3;

/// Establishes the hardware invariant that the active-low power stage is
/// disabled. This is idempotent and must precede every other peripheral init.
pub fn disable_power_stage() {
    // SAFETY: these fixed addresses are the STM32F103 RCC and GPIOA registers;
    // this function runs once during reset before interrupts are enabled.
    unsafe {
        write_volatile(RCC_APB2ENR, read_volatile(RCC_APB2ENR) | RCC_APB2ENR_IOPAEN);
        let crl = read_volatile(GPIOA_CRL);
        write_volatile(
            GPIOA_CRL,
            (crl & !PA2_CONFIG_MASK) | PA2_OUTPUT_PUSH_PULL_2MHZ,
        );
        write_volatile(GPIOA_BSRR, PA2_SET);
    }
}

pub fn select_application_vector_table() {
    // SAFETY: VTOR is word-accessible on Cortex-M3. The address is aligned to
    // the 128-byte architectural requirement and matches the linker origin.
    unsafe { write_volatile(SCB_VTOR, crate::config::APPLICATION_FLASH_ORIGIN) }
}

/// Immediately removes every software-controlled route to gate drive. This is
/// safe before initialization and from any exception context.
pub fn emergency_shutdown() {
    // SAFETY: atomic GPIO set plus idempotent timer register masking. Hardware
    // break independently clears MOE before software reaches this function.
    unsafe {
        configure_inert_motor_pins();
        write_volatile(GPIOA_BSRR, PA2_SET);
        write_volatile(
            GPIOA_CRL,
            (read_volatile(GPIOA_CRL) & !PA2_CONFIG_MASK) | PA2_OUTPUT_PUSH_PULL_2MHZ,
        );
        write_volatile(
            TIM1_BDTR,
            read_volatile(TIM1_BDTR) & !TIM1_MAIN_OUTPUT_ENABLE,
        );
        write_volatile(TIM1_CCER, TIM1_PASSIVE_CCER);
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ClockError {
    HsiTimeout,
    HseTimeout,
    HsiSwitchTimeout,
    PllDisableTimeout,
    PllTimeout,
    SwitchTimeout,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AdcError {
    ResetCalibrationTimeout,
    CalibrationTimeout,
    ConversionTimeout,
    CalibrationInvalid,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CurrentOffsets {
    pub phase_a: u16,
    pub phase_b: u16,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PhaseCurrentSample {
    pub raw_a: u16,
    pub raw_b: u16,
    pub phase_a: i16,
    pub phase_b: i16,
    pub phase_c: i16,
}

impl PhaseCurrentSample {
    pub fn exceeds_limit(self, limit: u16) -> bool {
        self.phase_a
            .unsigned_abs()
            .max(self.phase_b.unsigned_abs())
            .max(self.phase_c.unsigned_abs())
            > limit
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CurrentSampleError {
    NotReady,
    OutOfRange,
}

pub fn configure_72mhz_clock() -> Result<(), ClockError> {
    // SAFETY: startup-time exclusive access to RCC and FLASH registers. The
    // resident bootloader may hand over with PLL already selected, so first
    // establish HSI as a known temporary system clock before changing PLL or
    // bus prescaler fields.
    unsafe {
        write_volatile(FLASH_ACR, FLASH_ACR_72MHZ);
        write_volatile(RCC_CR, read_volatile(RCC_CR) | RCC_CR_HSION | RCC_CR_HSEON);
        wait_for_bits(RCC_CR, RCC_CR_HSIRDY, RCC_CR_HSIRDY).map_err(|()| ClockError::HsiTimeout)?;
        wait_for_bits(RCC_CR, RCC_CR_HSERDY, RCC_CR_HSERDY).map_err(|()| ClockError::HseTimeout)?;

        write_volatile(
            RCC_CFGR,
            (read_volatile(RCC_CFGR) & !0b11) | RCC_CFGR_SWITCH_HSI,
        );
        wait_for_bits(RCC_CFGR, 0b11 << 2, RCC_CFGR_STATUS_HSI)
            .map_err(|()| ClockError::HsiSwitchTimeout)?;

        write_volatile(
            RCC_CR,
            read_volatile(RCC_CR) & !(RCC_CR_PLLON | RCC_CR_CSSON),
        );
        wait_for_bits(RCC_CR, RCC_CR_PLLRDY, 0).map_err(|()| ClockError::PllDisableTimeout)?;

        let cfgr = read_volatile(RCC_CFGR) & !RCC_CFGR_CLOCK_MASK;
        write_volatile(
            RCC_CFGR,
            cfgr | RCC_CFGR_HSE_PLL_X9_72MHZ | RCC_CFGR_SWITCH_HSI,
        );
        write_volatile(RCC_CR, read_volatile(RCC_CR) | RCC_CR_PLLON);
        wait_for_bits(RCC_CR, RCC_CR_PLLRDY, RCC_CR_PLLRDY).map_err(|()| ClockError::PllTimeout)?;

        write_volatile(
            RCC_CFGR,
            (read_volatile(RCC_CFGR) & !0b11) | RCC_CFGR_SWITCH_PLL,
        );
        wait_for_bits(RCC_CFGR, 0b11 << 2, RCC_CFGR_STATUS_PLL)
            .map_err(|()| ClockError::SwitchTimeout)?;
        write_volatile(RCC_CR, read_volatile(RCC_CR) | RCC_CR_CSSON);
    }
    Ok(())
}

/// Configures inert motor pins, hardware break, and TIM1's waveform registers.
/// Motor channel enables remain clear and the counter remains stopped.
pub fn configure_tim1_passive() {
    // SAFETY: called once before interrupts or timer start.
    unsafe {
        write_volatile(
            RCC_APB2ENR,
            read_volatile(RCC_APB2ENR)
                | RCC_APB2ENR_AFIOEN
                | RCC_APB2ENR_IOPAEN
                | RCC_APB2ENR_IOPBEN
                | RCC_APB2ENR_TIM1EN,
        );
        write_volatile(
            RCC_APB2RSTR,
            read_volatile(RCC_APB2RSTR) | RCC_APB2ENR_TIM1EN,
        );
        write_volatile(
            RCC_APB2RSTR,
            read_volatile(RCC_APB2RSTR) & !RCC_APB2ENR_TIM1EN,
        );

        let mut gpiob_crh = read_volatile(GPIOB_CRH);
        gpiob_crh = (gpiob_crh & !(0b1111 << 16)) | (BREAK_PIN_INPUT_PULL << 16);
        write_volatile(GPIOB_CRH, gpiob_crh);
        write_volatile(GPIOB_BSRR, 1 << 12);
        configure_inert_motor_pins();

        write_volatile(
            TIM1_CR1,
            TIM1_CENTER_ALIGNED_MODE_3 | TIM1_AUTO_RELOAD_PRELOAD | TIM1_CLOCK_DIVISION_2,
        );
        write_volatile(TIM1_CR2, TIM1_COMPLEMENTARY_IDLE_STATES);
        write_volatile(TIM1_DIER, 0);
        write_volatile(TIM1_CCMR1, 0x6868);
        write_volatile(TIM1_CCMR2, 0x7868);
        write_volatile(TIM1_CCER, TIM1_PASSIVE_CCER);
        write_volatile(TIM1_PSC, 0);
        write_volatile(TIM1_ARR, u32::from(crate::config::PWM_ARR));
        write_volatile(TIM1_RCR, 0);
        write_volatile(TIM1_CCR1, u32::from(crate::config::PWM_NEUTRAL));
        write_volatile(TIM1_CCR2, u32::from(crate::config::PWM_NEUTRAL));
        write_volatile(TIM1_CCR3, u32::from(crate::config::PWM_NEUTRAL));
        write_volatile(TIM1_CCR4, u32::from(crate::config::PWM_SAMPLE_CC4));
        write_volatile(
            TIM1_BDTR,
            TIM1_OFF_STATE_RUN
                | TIM1_OFF_STATE_IDLE
                | TIM1_LOCK_LEVEL_1
                | TIM1_BREAK_ENABLE
                | u32::from(crate::config::PWM_DEAD_TIME_TICKS),
        );
        write_volatile(TIM1_EGR, 1);
        write_volatile(TIM1_SR, 0);
        write_volatile(TIM1_CNT, 0);
        // The validated board firmware keeps this active after passive timer
        // setup. Normal stops use TIM1 channel gating and inert GPIO modes;
        // fatal shutdown still raises PA2 independently.
        write_volatile(GPIOA_BSRR, 1 << 18);
    }
}

/// Configures TIM2's remapped XOR Hall interface. The slave controller resets
/// the 1 MHz counter at every TI1F_ED transition; CH1 and CH2 capture the
/// rising and falling XOR edges without software-timestamp latency.
pub fn configure_hall_capture() {
    // SAFETY: one-time GPIO/AFIO/TIM2 setup before the IRQ is unmasked.
    unsafe {
        write_volatile(RCC_APB1ENR, read_volatile(RCC_APB1ENR) | RCC_APB1ENR_TIM2EN);
        write_volatile(
            RCC_APB1RSTR,
            read_volatile(RCC_APB1RSTR) | RCC_APB1ENR_TIM2EN,
        );
        write_volatile(
            RCC_APB1RSTR,
            read_volatile(RCC_APB1RSTR) & !RCC_APB1ENR_TIM2EN,
        );
        write_volatile(
            RCC_APB2ENR,
            read_volatile(RCC_APB2ENR)
                | RCC_APB2ENR_AFIOEN
                | RCC_APB2ENR_IOPAEN
                | RCC_APB2ENR_IOPBEN,
        );

        let mapr = read_volatile(AFIO_MAPR) & !((0b111 << 24) | (0b11 << 8));
        write_volatile(AFIO_MAPR, mapr | (0b100 << 24) | (0b11 << 8));

        let gpioa = (read_volatile(GPIOA_CRH) & !(0b1111 << 28)) | (HALL_PIN_INPUT_FLOATING << 28);
        write_volatile(GPIOA_CRH, gpioa);
        let gpiob_crl =
            (read_volatile(GPIOB_CRL) & !(0b1111 << 12)) | (HALL_PIN_INPUT_FLOATING << 12);
        write_volatile(GPIOB_CRL, gpiob_crl);
        let gpiob_crh =
            (read_volatile(GPIOB_CRH) & !(0b1111 << 8)) | (HALL_PIN_INPUT_FLOATING << 8);
        write_volatile(GPIOB_CRH, gpiob_crh);

        write_volatile(TIM2_CR1, TIM2_UPDATE_REQUEST_OVERFLOW_ONLY);
        write_volatile(TIM2_CR2, TIM2_HALL_XOR_ENABLE);
        write_volatile(
            TIM2_SMCR,
            TIM2_SLAVE_RESET_MODE | TIM2_TRIGGER_TI1_EDGE_DETECTOR | TIM2_MASTER_SLAVE_MODE_ENABLE,
        );
        write_volatile(
            TIM2_CCMR1,
            TIM2_CAPTURE_1_INPUT_FILTERED | TIM2_CAPTURE_2_INPUT_FILTERED_INDIRECT,
        );
        write_volatile(
            TIM2_CCER,
            TIM2_CAPTURE_1_ENABLE | TIM2_CAPTURE_2_ENABLE_FALLING,
        );
        write_volatile(TIM2_PSC, u32::from(crate::config::HALL_TIMER_PRESCALER));
        write_volatile(TIM2_ARR, 0xffff);
        write_volatile(TIM2_EGR, 1);
        write_volatile(TIM2_SR, 0);
        write_volatile(TIM2_CNT as *mut u32, 0);
        TIM2_OVERFLOWS.store(0, Ordering::Relaxed);
        write_volatile(TIM2_DIER, TIM2_UIE | TIM2_CC1IE | TIM2_CC2IE);
        set_interrupt_priority(28, 0x20);
        HALL_CONFIGURED.store(true, Ordering::Release);
        write_volatile(TIM2_CR1, TIM2_UPDATE_REQUEST_OVERFLOW_ONLY | 1);
        cortex_m::peripheral::NVIC::unmask(stm32f1::stm32f103::Interrupt::TIM2);
    }
}

/// Returns a coherent edge sequence, raw Hall state, and extended microsecond
/// captured interval. A changed sequence means a new edge is available.
pub fn hall_edge_snapshot() -> (u32, u8, u32) {
    loop {
        let before = HALL_SEQUENCE.load(Ordering::Acquire);
        let raw = HALL_RAW.load(Ordering::Relaxed);
        let interval_us = HALL_INTERVAL_US.load(Ordering::Relaxed);
        let after = HALL_SEQUENCE.load(Ordering::Acquire);
        if before == after {
            return (after, raw, interval_us);
        }
    }
}

pub fn hall_edge_age_us() -> u32 {
    loop {
        let before = TIM2_OVERFLOWS.load(Ordering::Acquire);
        // SAFETY: TIM2 CNT is a read-only observation here.
        let low = unsafe { read_volatile(TIM2_CNT) } & 0xffff;
        let after = TIM2_OVERFLOWS.load(Ordering::Acquire);
        if before == after {
            return after
                .saturating_mul(TIM2_TICKS_PER_OVERFLOW)
                .saturating_add(low);
        }
    }
}

/// Restarts stationary Hall timing at ride entry. TIM1_UP calls this while it
/// owns motor state and runs above TIM2's interrupt priority.
pub fn restart_stationary_hall_interval() {
    // SAFETY: TIM1_UP cannot be preempted by TIM2. Clearing a capture that
    // raced exactly with ride entry is fail-closed by the live-Hall equality
    // checks on either side of this reset.
    unsafe {
        TIM2_OVERFLOWS.store(0, Ordering::Relaxed);
        write_volatile(TIM2_CNT as *mut u32, 0);
        write_volatile(
            TIM2_SR,
            !(TIM2_UIF | TIM2_CAPTURE_FLAGS | TIM2_OVERCAPTURE_FLAGS),
        );
        cortex_m::peripheral::NVIC::unpend(stm32f1::stm32f103::Interrupt::TIM2);
    }
}

pub fn start_tim1_control_loop() {
    // SAFETY: control state and ADC trigger have been configured first.
    unsafe {
        enable_cycle_counter();
        write_volatile(TIM1_RCR, 0);
        write_volatile(TIM1_EGR, 1);
        write_volatile(TIM1_SR, 0);
        write_volatile(TIM1_DIER, 1 | (1 << 7));
        set_interrupt_priority(24, 0);
        set_interrupt_priority(25, 0x10);
        cortex_m::peripheral::NVIC::unmask(stm32f1::stm32f103::Interrupt::TIM1_UP);
        cortex_m::peripheral::NVIC::unmask(stm32f1::stm32f103::Interrupt::TIM1_BRK);
        write_volatile(
            TIM1_BDTR,
            read_volatile(TIM1_BDTR) | TIM1_MAIN_OUTPUT_ENABLE,
        );
        write_volatile(TIM1_CR1, read_volatile(TIM1_CR1) | 1);
    }
}

#[inline]
pub fn enable_motor_outputs() -> bool {
    // SAFETY: duty preloads and Hall angle are established by the control ISR
    // before it reaches this function. BKIN can clear MOE asynchronously.
    if break_input_active() {
        latch_fault(FAULT_HARDWARE_BREAK);
        return false;
    }
    if fault_flags() != 0 {
        emergency_shutdown();
        return false;
    }
    // The gate-driver control is established once during passive setup so it
    // has settled before the first ride request.
    if unsafe { read_volatile(GPIOA_ODR) } & PA2_SET != 0 {
        return false;
    }
    unsafe {
        if read_volatile(TIM1_CCER) == TIM1_ACTIVE_CCER
            && read_volatile(TIM1_BDTR) & TIM1_MAIN_OUTPUT_ENABLE != 0
            && read_volatile(GPIOA_ODR) & PA2_SET == 0
        {
            return fault_flags() == 0 && !break_input_active();
        }
        configure_active_motor_pins();
    }
    if fault_flags() != 0 || break_input_active() {
        emergency_shutdown();
        return false;
    }
    let enabled = cortex_m::interrupt::free(|_| {
        if fault_flags() != 0 || break_input_active() {
            return false;
        }
        // SAFETY: interrupts are masked across the software enable sequence;
        // TIM1 break remains an asynchronous hardware path to clear MOE.
        unsafe {
            write_volatile(TIM1_CCER, TIM1_ACTIVE_CCER);
            write_volatile(
                TIM1_BDTR,
                read_volatile(TIM1_BDTR) | TIM1_MAIN_OUTPUT_ENABLE,
            );
            if read_volatile(TIM1_BDTR) & TIM1_MAIN_OUTPUT_ENABLE == 0
                || break_input_active()
                || read_volatile(GPIOA_ODR) & PA2_SET != 0
            {
                return false;
            }
            read_volatile(TIM1_CCER) == TIM1_ACTIVE_CCER
                && read_volatile(TIM1_BDTR) & TIM1_MAIN_OUTPUT_ENABLE != 0
                && read_volatile(GPIOA_ODR) & PA2_SET == 0
        }
    });
    if !enabled || fault_flags() != 0 || break_input_active() {
        emergency_shutdown();
        return false;
    }
    true
}

#[inline]
pub fn disable_motor_outputs() {
    // SAFETY: clearing MOE and motor-channel enables removes PWM before the
    // pins return to analog mode. PA2 remains in its validated run-time state
    // so a recoverable stop never cold-starts the gate driver.
    unsafe {
        write_volatile(
            TIM1_BDTR,
            read_volatile(TIM1_BDTR) & !TIM1_MAIN_OUTPUT_ENABLE,
        );
        write_volatile(TIM1_CCER, TIM1_PASSIVE_CCER);
        configure_inert_motor_pins();
    }
    if fault_flags() == 0 && !break_input_active() {
        // SAFETY: motor-channel enables remain clear and PA2 remains in its
        // settled active-low run-time state.
        // MOE is restored solely so the internal CC4 event continues to
        // trigger passive injected-current conversions.
        unsafe {
            write_volatile(
                TIM1_BDTR,
                read_volatile(TIM1_BDTR) | TIM1_MAIN_OUTPUT_ENABLE,
            );
        }
    }
}

pub fn motor_outputs_disabled() -> bool {
    // SAFETY: read-only inspection of all six motor-channel enable bits.
    unsafe { read_volatile(TIM1_CCER) & 0x0555 == 0 }
}

pub fn hall_is_quiet(required_ticks: u32) -> bool {
    !HALL_CONFIGURED.load(Ordering::Acquire) || hall_edge_age_us() >= required_ticks
}

pub fn fault_flags() -> u32 {
    FAULT_FLAGS.load(Ordering::Acquire)
}

pub fn break_input_active() -> bool {
    // SAFETY: read-only observation of the configured active-low BKIN pin.
    unsafe { read_volatile(GPIOB_IDR) & (1 << 12) == 0 }
}

fn latch_fault(fault: u32) {
    FAULT_FLAGS.fetch_or(fault, Ordering::Release);
    emergency_shutdown();
}

pub fn latch_hall_capture_fault() {
    latch_fault(FAULT_HALL_CAPTURE);
}

pub fn latch_control_timing_fault() {
    latch_fault(FAULT_CONTROL_TIMING);
}

#[inline]
pub fn cycle_count() -> u32 {
    // SAFETY: DWT CYCCNT is a read-only observation after startup enables it.
    unsafe { read_volatile(DWT_CYCCNT) }
}

pub fn configure_can_interrupt_priority() {
    // CAN servicing is intentionally below Hall capture and the FOC update.
    unsafe { set_interrupt_priority(20, 0x40) }
}

#[interrupt]
fn TIM1_BRK() {
    latch_fault(FAULT_HARDWARE_BREAK);
    // SAFETY: rc_w0 clear of BIF; disable repeat interrupts after latching off.
    unsafe {
        write_volatile(TIM1_SR, !(1 << 7));
        write_volatile(TIM1_DIER, read_volatile(TIM1_DIER) & !(1 << 7));
    }
}

#[inline]
pub fn clear_tim1_update_flag() {
    // SAFETY: rc_w0 TIM1 status register; writing ones preserves other flags.
    unsafe { write_volatile(TIM1_SR, !1) }
}

#[inline]
pub fn tim1_counting_down() -> bool {
    // SAFETY: read-only observation of TIM1's hardware-maintained DIR bit.
    unsafe { read_volatile(TIM1_CR1) & TIM1_DIRECTION_DOWN != 0 }
}

#[inline]
pub fn write_pwm_duties(duty: oxifoc_core::foc::PwmDuty) -> bool {
    let neutral = crate::config::PWM_NEUTRAL;
    if [duty.a, duty.b, duty.c].into_iter().any(|compare| {
        compare > crate::config::PWM_ARR
            || compare.abs_diff(neutral) > crate::config::FOC_HARD_PHASE_LIMIT_TICKS
    }) {
        latch_fault(FAULT_PWM_RANGE);
        return false;
    }
    // SAFETY: preload is enabled, so all three values transfer together at an
    // update event. The explicit range check preserves the stock-proven
    // current-sample window even if the modulator is changed later.
    unsafe {
        write_volatile(TIM1_CCR1, u32::from(duty.a));
        write_volatile(TIM1_CCR2, u32::from(duty.b));
        write_volatile(TIM1_CCR3, u32::from(duty.c));
    }
    true
}

#[inline]
pub fn write_pwm_neutral() {
    let _ = write_pwm_duties(oxifoc_core::foc::PwmDuty {
        a: crate::config::PWM_NEUTRAL,
        b: crate::config::PWM_NEUTRAL,
        c: crate::config::PWM_NEUTRAL,
    });
}

#[stm32f1::stm32f103::interrupt]
fn TIM2() {
    // SAFETY: TIM2 is owned by the Hall capture module.
    unsafe {
        let status = read_volatile(TIM2_SR);
        if status & TIM2_UIF != 0 {
            TIM2_OVERFLOWS.fetch_add(1, Ordering::Relaxed);
        }
        let invalid_capture = status & TIM2_OVERCAPTURE_FLAGS != 0
            || status & TIM2_CAPTURE_FLAGS == TIM2_CAPTURE_FLAGS;
        let captured = if status & TIM2_CC1IF != 0 {
            Some(read_volatile(TIM2_CCR1) & 0xffff)
        } else if status & TIM2_CC2IF != 0 {
            Some(read_volatile(TIM2_CCR2) & 0xffff)
        } else {
            None
        };
        write_volatile(
            TIM2_SR,
            !(status & (TIM2_UIF | TIM2_CAPTURE_FLAGS | TIM2_OVERCAPTURE_FLAGS)),
        );
        if invalid_capture {
            latch_fault(FAULT_HALL_CAPTURE);
            return;
        }
        if let Some(captured_ticks) = captured {
            let overflows = TIM2_OVERFLOWS.swap(0, Ordering::Relaxed);
            let interval_us = overflows
                .saturating_mul(TIM2_TICKS_PER_OVERFLOW)
                .saturating_add(captured_ticks);
            HALL_RAW.store(read_hall_pins(), Ordering::Relaxed);
            HALL_INTERVAL_US.store(interval_us, Ordering::Relaxed);
            HALL_SEQUENCE.fetch_add(1, Ordering::Release);
        }
    }
}

unsafe fn set_interrupt_priority(interrupt_number: usize, priority: u8) {
    // SAFETY: each external interrupt owns one byte in NVIC IPR.
    unsafe { write_volatile(NVIC_IPR.add(interrupt_number), priority) }
}

unsafe fn configure_inert_motor_pins() {
    unsafe {
        write_volatile(
            RCC_APB2ENR,
            read_volatile(RCC_APB2ENR) | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN,
        );
        let mut gpioa = read_volatile(GPIOA_CRH);
        for pin in 8..=10 {
            gpioa &= !(0xf << ((pin - 8) * 4));
        }
        write_volatile(GPIOA_CRH, gpioa);
        let mut gpiob = read_volatile(GPIOB_CRH);
        for pin in 13..=15 {
            gpiob &= !(0xf << ((pin - 8) * 4));
        }
        write_volatile(GPIOB_CRH, gpiob);
    }
}

unsafe fn configure_active_motor_pins() {
    unsafe {
        let mut gpioa = read_volatile(GPIOA_CRH);
        for pin in 8..=10 {
            let shift = (pin - 8) * 4;
            gpioa = (gpioa & !(0xf << shift)) | (PWM_PIN_AF_PUSH_PULL_50MHZ << shift);
        }
        write_volatile(GPIOA_CRH, gpioa);
        let mut gpiob = read_volatile(GPIOB_CRH);
        for pin in 13..=15 {
            let shift = (pin - 8) * 4;
            gpiob = (gpiob & !(0xf << shift)) | (PWM_PIN_AF_PUSH_PULL_50MHZ << shift);
        }
        write_volatile(GPIOB_CRH, gpiob);
    }
}

unsafe fn enable_cycle_counter() {
    // SAFETY: called once with TIM1_UP still masked. These are the Cortex-M3
    // CoreDebug and DWT registers defined by the architecture.
    unsafe {
        write_volatile(
            CORE_DEBUG_DEMCR,
            read_volatile(CORE_DEBUG_DEMCR) | CORE_DEBUG_TRACE_ENABLE,
        );
        write_volatile(DWT_CYCCNT, 0);
        write_volatile(DWT_CTRL, read_volatile(DWT_CTRL) | DWT_CYCLE_COUNTER_ENABLE);
    }
}

unsafe fn read_hall_pins() -> u8 {
    let a = (unsafe { read_volatile(GPIOA_IDR) } >> 15) & 1;
    let b = (unsafe { read_volatile(GPIOB_IDR) } >> 3) & 1;
    let c = (unsafe { read_volatile(GPIOB_IDR) } >> 10) & 1;
    (a | (b << 1) | (c << 2)) as u8
}

pub fn live_hall_state() -> u8 {
    // SAFETY: read-only observation of the three configured Hall GPIO inputs.
    unsafe { read_hall_pins() }
}

/// Configures ADC1 channel 11 and ADC2 channel 12, calibrates both ADCs, then
/// measures the midpoint offsets while the bridge is disabled. Hardware
/// triggering is selected only after the offset average is complete.
pub fn configure_and_calibrate_current_adcs() -> Result<CurrentOffsets, AdcError> {
    // SAFETY: startup-time exclusive access; all motor PWM pins and channel
    // enables are inert while the active-low gate-driver control settles.
    unsafe {
        write_volatile(
            RCC_APB2ENR,
            read_volatile(RCC_APB2ENR)
                | RCC_APB2ENR_IOPCEN
                | RCC_APB2ENR_ADC1EN
                | RCC_APB2ENR_ADC2EN,
        );
        write_volatile(
            RCC_APB2RSTR,
            read_volatile(RCC_APB2RSTR) | RCC_APB2ENR_ADC1EN | RCC_APB2ENR_ADC2EN,
        );
        write_volatile(
            RCC_APB2RSTR,
            read_volatile(RCC_APB2RSTR) & !(RCC_APB2ENR_ADC1EN | RCC_APB2ENR_ADC2EN),
        );
        let crl = read_volatile(GPIOC_CRL) & !((0b1111 << 4) | (0b1111 << 8));
        write_volatile(GPIOC_CRL, crl);

        configure_adc(ADC1_BASE, 11)?;
        configure_adc(ADC2_BASE, 12)?;

        let mut sum_a = 0_u32;
        let mut sum_b = 0_u32;
        let mut minimum_a = u16::MAX;
        let mut maximum_a = 0_u16;
        let mut minimum_b = u16::MAX;
        let mut maximum_b = 0_u16;
        for _ in 0..ADC_CALIBRATION_SAMPLES {
            start_injected_software(ADC1_BASE);
            start_injected_software(ADC2_BASE);
            wait_for_adc_conversion(ADC1_BASE)?;
            wait_for_adc_conversion(ADC2_BASE)?;
            let raw_a = read_injected(ADC1_BASE);
            let raw_b = read_injected(ADC2_BASE);
            sum_a += u32::from(raw_a);
            sum_b += u32::from(raw_b);
            minimum_a = minimum_a.min(raw_a);
            maximum_a = maximum_a.max(raw_a);
            minimum_b = minimum_b.min(raw_b);
            maximum_b = maximum_b.max(raw_b);
        }

        let phase_a = (sum_a / ADC_CALIBRATION_SAMPLES) as u16;
        let phase_b = (sum_b / ADC_CALIBRATION_SAMPLES) as u16;
        if !(257..3_840).contains(&phase_a)
            || !(257..3_840).contains(&phase_b)
            || maximum_a.saturating_sub(minimum_a) > 128
            || maximum_b.saturating_sub(minimum_b) > 128
        {
            return Err(AdcError::CalibrationInvalid);
        }
        select_injected_trigger(ADC1_BASE, ADC_JEXTSEL_TIM1_CC4);
        select_injected_trigger(ADC2_BASE, ADC_JEXTSEL_TIM1_CC4);
        Ok(CurrentOffsets { phase_a, phase_b })
    }
}

#[inline]
pub fn read_phase_currents(
    offsets: CurrentOffsets,
) -> Result<PhaseCurrentSample, CurrentSampleError> {
    // SAFETY: status and injected-data registers belong to the two ADCs that
    // are hardware-triggered together by TIM1 CC4.
    let (raw_a, raw_b) = unsafe {
        let status_a = read_volatile(reg(ADC1_BASE, ADC_SR_OFFSET));
        let status_b = read_volatile(reg(ADC2_BASE, ADC_SR_OFFSET));
        if status_a & ADC_JEOC == 0 || status_b & ADC_JEOC == 0 {
            write_volatile(reg(ADC1_BASE, ADC_SR_OFFSET), status_a & !ADC_JEOC);
            write_volatile(reg(ADC2_BASE, ADC_SR_OFFSET), status_b & !ADC_JEOC);
            return Err(CurrentSampleError::NotReady);
        }
        let raw_a = read_injected(ADC1_BASE);
        let raw_b = read_injected(ADC2_BASE);
        write_volatile(reg(ADC1_BASE, ADC_SR_OFFSET), status_a & !ADC_JEOC);
        write_volatile(reg(ADC2_BASE, ADC_SR_OFFSET), status_b & !ADC_JEOC);
        (raw_a, raw_b)
    };
    if raw_a > 4_095 || raw_b > 4_095 || offsets.phase_a > 4_095 || offsets.phase_b > 4_095 {
        return Err(CurrentSampleError::OutOfRange);
    }
    let phase_a = i32::from(raw_a) - i32::from(offsets.phase_a);
    let phase_b = i32::from(raw_b) - i32::from(offsets.phase_b);
    let phase_c = 0_i32.saturating_sub(phase_a).saturating_sub(phase_b);
    let sample = PhaseCurrentSample {
        raw_a,
        raw_b,
        phase_a: phase_a.clamp(i32::from(i16::MIN), i32::from(i16::MAX)) as i16,
        phase_b: phase_b.clamp(i32::from(i16::MIN), i32::from(i16::MAX)) as i16,
        phase_c: phase_c.clamp(i32::from(i16::MIN), i32::from(i16::MAX)) as i16,
    };
    Ok(sample)
}

unsafe fn configure_adc(base: usize, channel: u32) -> Result<(), AdcError> {
    unsafe {
        write_volatile(reg(base, ADC_CR1_OFFSET), 0);
        write_volatile(reg(base, ADC_SMPR1_OFFSET), 0b010 << ((channel - 10) * 3));
        write_volatile(reg(base, ADC_JSQR_OFFSET), channel << 15);
        write_volatile(
            reg(base, ADC_CR2_OFFSET),
            ADC_ADON | ADC_JEXTTRIG | ADC_JEXTSEL_SOFTWARE,
        );
        for _ in 0..1_000 {
            core::hint::spin_loop();
        }
        write_volatile(
            reg(base, ADC_CR2_OFFSET),
            read_volatile(reg(base, ADC_CR2_OFFSET)) | ADC_RSTCAL,
        );
        wait_for_clear(reg(base, ADC_CR2_OFFSET), ADC_RSTCAL)
            .map_err(|()| AdcError::ResetCalibrationTimeout)?;
        write_volatile(
            reg(base, ADC_CR2_OFFSET),
            read_volatile(reg(base, ADC_CR2_OFFSET)) | ADC_CAL,
        );
        wait_for_clear(reg(base, ADC_CR2_OFFSET), ADC_CAL)
            .map_err(|()| AdcError::CalibrationTimeout)
    }
}

unsafe fn start_injected_software(base: usize) {
    unsafe {
        write_volatile(
            reg(base, ADC_SR_OFFSET),
            read_volatile(reg(base, ADC_SR_OFFSET)) & !ADC_JEOC,
        );
        write_volatile(
            reg(base, ADC_CR2_OFFSET),
            read_volatile(reg(base, ADC_CR2_OFFSET)) | ADC_JSWSTART,
        );
    }
}

unsafe fn wait_for_adc_conversion(base: usize) -> Result<(), AdcError> {
    for _ in 0..ADC_WAIT_ITERATIONS {
        if unsafe { read_volatile(reg(base, ADC_SR_OFFSET)) } & ADC_JEOC != 0 {
            return Ok(());
        }
    }
    Err(AdcError::ConversionTimeout)
}

unsafe fn read_injected(base: usize) -> u16 {
    unsafe { (read_volatile(reg(base, ADC_JDR1_OFFSET)) & 0x0fff) as u16 }
}

unsafe fn select_injected_trigger(base: usize, trigger: u32) {
    unsafe {
        let cr2 = read_volatile(reg(base, ADC_CR2_OFFSET));
        write_volatile(
            reg(base, ADC_CR2_OFFSET),
            (cr2 & !ADC_JEXTSEL_MASK) | ADC_JEXTTRIG | trigger,
        );
        write_volatile(
            reg(base, ADC_SR_OFFSET),
            read_volatile(reg(base, ADC_SR_OFFSET)) & !ADC_JEOC,
        );
    }
}

unsafe fn wait_for_clear(register: *const u32, mask: u32) -> Result<(), ()> {
    for _ in 0..ADC_WAIT_ITERATIONS {
        if unsafe { read_volatile(register) } & mask == 0 {
            return Ok(());
        }
    }
    Err(())
}

const fn reg(base: usize, offset: usize) -> *mut u32 {
    (base + offset) as *mut u32
}

unsafe fn wait_for_bits(register: *const u32, mask: u32, expected: u32) -> Result<(), ()> {
    for _ in 0..CLOCK_WAIT_ITERATIONS {
        // SAFETY: the caller supplies an RCC register address.
        if unsafe { read_volatile(register) } & mask == expected {
            return Ok(());
        }
    }
    Err(())
}
