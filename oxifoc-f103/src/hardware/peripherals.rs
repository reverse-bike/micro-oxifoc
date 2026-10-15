//! Minimal, fail-closed STM32F103 register bring-up.
//!
//! No motor output is configured here. The first operation drives the
//! configured power-stage enable inactive before TIM1, ADC, or Hall
//! peripherals are touched.

use core::ptr::{read_volatile, write_volatile};
use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU32, Ordering};
use oxifoc_core::foc::PwmDuty;
use stm32f1::stm32f103::interrupt;

use crate::config::BIKE_CONFIG;
use crate::hardware::gpio;
use crate::hardware::system::{
    RCC_APB1ENR, RCC_APB1RSTR, RCC_APB2ENR, RCC_APB2RSTR, remap_afio, set_interrupt_priority,
};
use crate::safety::{PwmFailureContext, pwm_failure_cause as pwm_cause, pwm_pin_flag as pwm_pin};

const RCC_CR: *mut u32 = 0x4002_1000 as *mut u32;
const RCC_CFGR: *mut u32 = 0x4002_1004 as *mut u32;
const RCC_CIR: *const u32 = 0x4002_1008 as *const u32;
const FLASH_ACR: *mut u32 = 0x4002_2000 as *mut u32;
const SCB_VTOR: *mut u32 = 0xe000_ed08 as *mut u32;
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
const RCC_CIR_CLOCK_SECURITY_FAILURE: u32 = 1 << 7;
const RCC_CFGR_SW_MASK: u32 = 0b11;
const RCC_CFGR_HPRE_MASK: u32 = 0b1111 << 4;
const RCC_CFGR_PPRE1_MASK: u32 = 0b111 << 8;
const RCC_CFGR_PPRE2_MASK: u32 = 0b111 << 11;
const RCC_CFGR_ADCPRE_MASK: u32 = 0b11 << 14;
const RCC_CFGR_PLL_MASK: u32 = 0b11_1111 << 16;
/// Every writable field the clock bring-up owns: the clock switch, AHB/APB/ADC
/// prescalers, and the PLL source, HSE divider, and multiplier.
const RCC_CFGR_CLOCK_MASK: u32 = RCC_CFGR_SW_MASK
    | RCC_CFGR_HPRE_MASK
    | RCC_CFGR_PPRE1_MASK
    | RCC_CFGR_PPRE2_MASK
    | RCC_CFGR_ADCPRE_MASK
    | RCC_CFGR_PLL_MASK;
const RCC_CFGR_HSE_PLL_X9_72MHZ: u32 = (0b100 << 8) | (0b10 << 14) | (1 << 16) | (0b0111 << 18);
const RCC_CFGR_SWITCH_HSI: u32 = 0;
const RCC_CFGR_SWITCH_PLL: u32 = 0b10;
const RCC_CFGR_STATUS_HSI: u32 = 0;
const RCC_CFGR_STATUS_PLL: u32 = 0b10 << 2;
const _: () = assert!(RCC_CFGR_HSE_PLL_X9_72MHZ & !RCC_CFGR_CLOCK_MASK == 0);
const _: () = assert!(RCC_CFGR_SWITCH_PLL & !RCC_CFGR_CLOCK_MASK == 0);
const FLASH_ACR_72MHZ: u32 = (1 << 4) | 0b010;
const CLOCK_WAIT_ITERATIONS: u32 = 1_000_000;

const RCC_APB2ENR_ADC1EN: u32 = 1 << 9;
const RCC_APB2ENR_ADC2EN: u32 = 1 << 10;
const RCC_APB2ENR_TIM1EN: u32 = 1 << 11;
const RCC_APB1ENR_TIM2EN: u32 = 1;

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
const TIM1_BREAK_FLAG: u32 = 1 << 7;
const TIM1_BREAK_INTERRUPT_ENABLE: u32 = 1 << 7;

const ADC1_BASE: usize = 0x4001_2400;
const ADC2_BASE: usize = 0x4001_2800;
const ADC_SR_OFFSET: usize = 0x00;
const ADC_CR1_OFFSET: usize = 0x04;
const ADC_CR2_OFFSET: usize = 0x08;
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

pub const FAULT_HARDWARE_BREAK: u32 = crate::safety::fault_flag::HARDWARE_BREAK;
pub const FAULT_PWM_RANGE: u32 = crate::safety::fault_flag::PWM_RANGE;
pub const FAULT_HALL_CAPTURE: u32 = crate::safety::fault_flag::HALL_CAPTURE;
pub const FAULT_CONTROL_TIMING: u32 = crate::safety::fault_flag::CONTROL_TIMING;

/// Establishes the hardware invariant that the active-low power stage is
/// disabled. This is idempotent and must precede every other peripheral init.
pub fn disable_power_stage() {
    // SAFETY: this runs once during reset before interrupts are enabled.
    unsafe {
        gpio::configure_output(BIKE_CONFIG.pins.inverter.power_stage_enable, false);
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
        gpio::configure_output(BIKE_CONFIG.pins.inverter.power_stage_enable, false);
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
        write_volatile(RCC_APB2ENR, read_volatile(RCC_APB2ENR) | RCC_APB2ENR_TIM1EN);
        write_volatile(
            RCC_APB2RSTR,
            read_volatile(RCC_APB2RSTR) | RCC_APB2ENR_TIM1EN,
        );
        write_volatile(
            RCC_APB2RSTR,
            read_volatile(RCC_APB2RSTR) & !RCC_APB2ENR_TIM1EN,
        );

        remap_afio(
            crate::config::AFIO_MAPR_TIM1_REMAP_MASK,
            crate::config::tim1_remap_bits(),
        );
        gpio::configure_pull_up_input(BIKE_CONFIG.pins.inverter.hardware_break);
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
        // fatal shutdown still drives the power-stage enable inactive.
        gpio::set_output_active(BIKE_CONFIG.pins.inverter.power_stage_enable, true);
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
        remap_afio(
            crate::config::AFIO_MAPR_TIM2_REMAP_MASK,
            crate::config::tim2_remap_bits(),
        );
        for pin in BIKE_CONFIG.pins.hall {
            gpio::configure(pin, gpio::FLOATING_INPUT);
        }

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
        set_interrupt_priority(25, 0);
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
    if break_event_pending() {
        retain_current_pwm_failure(pwm_cause::BREAK_ACTIVE);
        latch_fault(FAULT_HARDWARE_BREAK);
        return false;
    }
    if fault_flags() != 0 {
        retain_current_pwm_failure(pwm_cause::FAULT_LATCHED);
        disable_motor_outputs();
        return false;
    }
    // The gate-driver control is established once during passive setup so it
    // has settled before the first ride request.
    if power_stage_is_disabled() {
        retain_current_pwm_failure(pwm_cause::POWER_STAGE_DISABLED);
        return false;
    }
    unsafe {
        if read_volatile(TIM1_CCER) == TIM1_ACTIVE_CCER
            && read_volatile(TIM1_BDTR) & TIM1_MAIN_OUTPUT_ENABLE != 0
            && !power_stage_is_disabled()
        {
            let enabled = fault_flags() == 0 && !break_event_pending();
            if !enabled {
                retain_current_pwm_failure(pwm_cause::POST_ENABLE);
            }
            return enabled;
        }
        configure_active_motor_pins();
    }
    if fault_flags() != 0 || break_event_pending() {
        retain_current_pwm_failure(pwm_cause::POST_PIN_CONFIGURATION);
        disable_motor_outputs();
        return false;
    }
    let enabled = cortex_m::interrupt::free(|_| {
        if fault_flags() != 0 || break_event_pending() {
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
                || break_event_pending()
                || power_stage_is_disabled()
            {
                return false;
            }
            read_volatile(TIM1_CCER) == TIM1_ACTIVE_CCER
                && read_volatile(TIM1_BDTR) & TIM1_MAIN_OUTPUT_ENABLE != 0
                && !power_stage_is_disabled()
        }
    });
    let post_enable_failure = fault_flags() != 0 || break_event_pending();
    if !enabled || post_enable_failure {
        retain_current_pwm_failure(if enabled {
            pwm_cause::POST_ENABLE
        } else {
            pwm_cause::ENABLE_READBACK
        });
        disable_motor_outputs();
        return false;
    }
    true
}

#[inline(never)]
fn current_pwm_duty() -> PwmDuty {
    // SAFETY: read-only inspection of the three TIM1 compare preloads.
    unsafe {
        PwmDuty {
            a: read_volatile(TIM1_CCR1) as u16,
            b: read_volatile(TIM1_CCR2) as u16,
            c: read_volatile(TIM1_CCR3) as u16,
        }
    }
}

#[inline(never)]
fn retain_current_pwm_failure(cause: u8) {
    retain_pwm_failure(cause, current_pwm_duty());
}

#[inline(never)]
fn retain_pwm_failure(cause: u8, duty: PwmDuty) {
    let break_active = break_input_active();
    let power_stage_disabled = power_stage_is_disabled();
    let clock_security_failed =
        unsafe { read_volatile(RCC_CIR) } & RCC_CIR_CLOCK_SECURITY_FAILURE != 0;
    // SAFETY: read-only inspection of TIM1 state at the exact rejected output
    // operation. All retained fields are intentionally narrowed to the
    // register widths implemented by STM32F103 TIM1.
    unsafe {
        crate::safety::record_pwm_failure(PwmFailureContext::new(
            cause,
            fault_flags() as u8,
            (u8::from(break_active) * pwm_pin::BREAK_ACTIVE)
                | (u8::from(power_stage_disabled) * pwm_pin::POWER_STAGE_DISABLED)
                | (u8::from(clock_security_failed) * pwm_pin::CLOCK_SECURITY_FAILURE),
            read_volatile(TIM1_SR) as u16,
            read_volatile(TIM1_BDTR) as u16,
            read_volatile(TIM1_CCER) as u16,
            duty.as_array(),
        ));
    }
}

#[inline]
pub fn disable_motor_outputs() {
    // SAFETY: clearing MOE and motor-channel enables removes PWM before the
    // pins return to analog mode. The power-stage enable remains in its
    // validated run-time state so a recoverable stop never cold-starts the
    // gate driver.
    unsafe {
        write_volatile(
            TIM1_BDTR,
            read_volatile(TIM1_BDTR) & !TIM1_MAIN_OUTPUT_ENABLE,
        );
        write_volatile(TIM1_CCER, TIM1_PASSIVE_CCER);
        configure_inert_motor_pins();
    }
    restore_passive_sampling();
}

fn restore_passive_sampling() {
    if !break_event_pending() {
        // SAFETY: all six motor-channel enables remain clear and their pins
        // are inert. MOE is used only to preserve the internal CC4 current-
        // sampling event while the fault latch keeps motor output disabled.
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
    unsafe { !gpio::read(BIKE_CONFIG.pins.inverter.hardware_break) }
}

fn power_stage_is_disabled() -> bool {
    let enable = BIKE_CONFIG.pins.inverter.power_stage_enable;
    // SAFETY: read-only observation of the configured output latch.
    unsafe { gpio::output_is_high(enable.pin) == enable.active_low }
}

fn break_event_pending() -> bool {
    break_input_active() || unsafe { read_volatile(TIM1_SR) } & TIM1_BREAK_FLAG != 0
}

fn latch_fault(fault: u32) {
    FAULT_FLAGS.fetch_or(fault, Ordering::Release);
    disable_motor_outputs();
}

/// Clears the latched faults only while they are exactly `expected_faults`,
/// the mask the caller observed and chose to acknowledge. A fault latched
/// since that observation leaves every latch in place and returns false.
pub fn acknowledge_faults(expected_faults: u32) -> bool {
    disable_motor_outputs();
    if fault_flags() == 0 {
        return true;
    }
    if break_input_active() || power_stage_is_disabled() {
        return false;
    }
    cortex_m::interrupt::free(|_| {
        let previous_faults = fault_flags();
        if break_input_active()
            || !crate::safety::fault_flag::acknowledgement_permitted(
                previous_faults,
                expected_faults,
            )
        {
            return false;
        }
        // SAFETY: motor outputs and interrupts are disabled while the stale
        // break flag is cleared and its interrupt is rearmed.
        unsafe {
            write_volatile(TIM1_SR, !TIM1_BREAK_FLAG);
            write_volatile(
                TIM1_DIER,
                read_volatile(TIM1_DIER) | TIM1_BREAK_INTERRUPT_ENABLE,
            );
        }
        FAULT_FLAGS.store(0, Ordering::Release);
        restore_passive_sampling();
        let ready = || unsafe {
            !break_input_active()
                && read_volatile(TIM1_SR) & TIM1_BREAK_FLAG == 0
                && read_volatile(TIM1_BDTR) & TIM1_MAIN_OUTPUT_ENABLE != 0
                && motor_outputs_disabled()
        };
        if !ready() {
            FAULT_FLAGS.store(previous_faults, Ordering::Release);
            disable_motor_outputs();
            return false;
        }
        crate::safety::clear_current_pwm_failure();
        if !ready() {
            retain_current_pwm_failure(pwm_cause::BREAK_ACTIVE);
            latch_fault(previous_faults | FAULT_HARDWARE_BREAK);
            return false;
        }
        true
    })
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

#[interrupt]
fn TIM1_BRK() {
    retain_current_pwm_failure(pwm_cause::BREAK_ACTIVE);
    latch_fault(FAULT_HARDWARE_BREAK);
    // SAFETY: rc_w0 clear of BIF; disable repeat interrupts after latching off.
    unsafe {
        write_volatile(TIM1_SR, !TIM1_BREAK_FLAG);
        write_volatile(
            TIM1_DIER,
            read_volatile(TIM1_DIER) & !TIM1_BREAK_INTERRUPT_ENABLE,
        );
    }
    restore_passive_sampling();
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
pub fn write_pwm_duties(duty: PwmDuty) -> bool {
    let neutral = crate::config::PWM_NEUTRAL;
    if [duty.a, duty.b, duty.c].into_iter().any(|compare| {
        compare > crate::config::PWM_ARR
            || compare.abs_diff(neutral) > crate::config::FOC_PHASE_LIMIT_TICKS
    }) {
        retain_pwm_failure(pwm_cause::COMPARE_RANGE, duty);
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
    let _ = write_pwm_duties(PwmDuty {
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
        // Each capture holds the time since the previous XOR edge, so two
        // pending captures together span both edges since the last one
        // published. They are published as one edge at the live Hall state;
        // a rising-and-falling pair on one line returns to the same state.
        let captured = match status & TIM2_CAPTURE_FLAGS {
            TIM2_CAPTURE_FLAGS => {
                Some((read_volatile(TIM2_CCR1) & 0xffff) + (read_volatile(TIM2_CCR2) & 0xffff))
            }
            TIM2_CC1IF => Some(read_volatile(TIM2_CCR1) & 0xffff),
            TIM2_CC2IF => Some(read_volatile(TIM2_CCR2) & 0xffff),
            _ => None,
        };
        write_volatile(
            TIM2_SR,
            !(status & (TIM2_UIF | TIM2_CAPTURE_FLAGS | TIM2_OVERCAPTURE_FLAGS)),
        );
        if let Some(captured_ticks) = captured {
            let overflows = TIM2_OVERFLOWS.swap(0, Ordering::Relaxed);
            let interval_us = overflows
                .saturating_mul(TIM2_TICKS_PER_OVERFLOW)
                .saturating_add(captured_ticks);
            HALL_RAW.store(read_hall_pins(), Ordering::Relaxed);
            HALL_INTERVAL_US.store(interval_us, Ordering::Relaxed);
            // An overcapture lost at least one edge and its interval. The
            // sequence gap makes the control pass resynchronise to the live
            // state near standstill or latch a Hall-capture fault in motion.
            let edges = if status & TIM2_OVERCAPTURE_FLAGS != 0 {
                2
            } else {
                1
            };
            HALL_SEQUENCE.fetch_add(edges, Ordering::Release);
        }
    }
}

unsafe fn configure_inert_motor_pins() {
    unsafe {
        for pin in BIKE_CONFIG
            .pins
            .inverter
            .high_side_pwm
            .into_iter()
            .chain(BIKE_CONFIG.pins.inverter.low_side_pwm)
        {
            gpio::configure(pin, gpio::ANALOG_INPUT);
        }
    }
}

unsafe fn configure_active_motor_pins() {
    unsafe {
        for pin in BIKE_CONFIG
            .pins
            .inverter
            .high_side_pwm
            .into_iter()
            .chain(BIKE_CONFIG.pins.inverter.low_side_pwm)
        {
            gpio::configure(pin, gpio::ALTERNATE_PUSH_PULL_50MHZ);
        }
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
    let hall = BIKE_CONFIG.pins.hall;
    u8::from(unsafe { gpio::read(hall[0]) })
        | (u8::from(unsafe { gpio::read(hall[1]) }) << 1)
        | (u8::from(unsafe { gpio::read(hall[2]) }) << 2)
}

pub fn live_hall_state() -> u8 {
    // SAFETY: read-only observation of the three configured Hall GPIO inputs.
    unsafe { read_hall_pins() }
}

/// Configures the two mapped injected-current channels, calibrates both ADCs,
/// then measures midpoint offsets while the bridge is disabled. Hardware
/// triggering is selected only after the offset average is complete.
pub fn configure_and_calibrate_current_adcs() -> Result<CurrentOffsets, AdcError> {
    // SAFETY: startup-time exclusive access; all motor PWM pins and channel
    // enables are inert while the active-low gate-driver control settles.
    unsafe {
        write_volatile(
            RCC_APB2ENR,
            read_volatile(RCC_APB2ENR) | RCC_APB2ENR_ADC1EN | RCC_APB2ENR_ADC2EN,
        );
        write_volatile(
            RCC_APB2RSTR,
            read_volatile(RCC_APB2RSTR) | RCC_APB2ENR_ADC1EN | RCC_APB2ENR_ADC2EN,
        );
        write_volatile(
            RCC_APB2RSTR,
            read_volatile(RCC_APB2RSTR) & !(RCC_APB2ENR_ADC1EN | RCC_APB2ENR_ADC2EN),
        );
        gpio::configure(BIKE_CONFIG.pins.phase_current[0].pin, gpio::ANALOG_INPUT);
        gpio::configure(BIKE_CONFIG.pins.phase_current[1].pin, gpio::ANALOG_INPUT);

        configure_adc(ADC1_BASE, BIKE_CONFIG.pins.phase_current[0].channel)?;
        configure_adc(ADC2_BASE, BIKE_CONFIG.pins.phase_current[1].channel)?;

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

unsafe fn configure_adc(base: usize, channel: u8) -> Result<(), AdcError> {
    let (sample_time_offset, sample_time_shift) = crate::config::adc_sample_time_field(channel);
    unsafe {
        write_volatile(reg(base, ADC_CR1_OFFSET), 0);
        write_volatile(reg(base, sample_time_offset), 0b010 << sample_time_shift);
        write_volatile(reg(base, ADC_JSQR_OFFSET), u32::from(channel) << 15);
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
