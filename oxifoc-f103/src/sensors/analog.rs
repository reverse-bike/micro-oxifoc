//! Passive local ride-input acquisition.

use core::cell::UnsafeCell;
use core::hint::spin_loop;
use core::ptr::{read_volatile, write_volatile};
use core::sync::atomic::{AtomicBool, AtomicU16, AtomicU32, Ordering};

use super::inputs::DebouncedActiveLow;
use super::throttle::Observation;

const RCC_AHBENR: *mut u32 = 0x4002_1014 as *mut u32;
const RCC_APB2ENR: *mut u32 = 0x4002_1018 as *mut u32;
const RCC_AHBENR_DMA1EN: u32 = 1;
const RCC_APB2ENR_IOPAEN: u32 = 1 << 2;
const RCC_APB2ENR_IOPBEN: u32 = 1 << 3;
const RCC_APB2ENR_IOPCEN: u32 = 1 << 4;

const GPIOA_CRL: *mut u32 = 0x4001_0800 as *mut u32;
const GPIOB_CRL: *mut u32 = 0x4001_0c00 as *mut u32;
const GPIOC_CRL: *mut u32 = 0x4001_1000 as *mut u32;
const GPIOC_IDR: *const u32 = 0x4001_1008 as *const u32;
const GPIOC_BSRR: *mut u32 = 0x4001_1010 as *mut u32;
const BRAKE_PIN: u32 = 4;
const GPIO_INPUT_PULL: u32 = 0x8;

const ADC1_BASE: usize = 0x4001_2400;
const ADC_CR1: *mut u32 = (ADC1_BASE + 0x04) as *mut u32;
const ADC_CR2: *mut u32 = (ADC1_BASE + 0x08) as *mut u32;
const ADC_SMPR1: *mut u32 = (ADC1_BASE + 0x0c) as *mut u32;
const ADC_SMPR2: *mut u32 = (ADC1_BASE + 0x10) as *mut u32;
const ADC_SQR1: *mut u32 = (ADC1_BASE + 0x2c) as *mut u32;
const ADC_SQR3: *mut u32 = (ADC1_BASE + 0x34) as *mut u32;
const ADC_DR: *mut u32 = (ADC1_BASE + 0x4c) as *mut u32;
const ADC_SCAN_MODE: u32 = 1 << 8;
const ADC_CONTINUOUS: u32 = 1 << 1;
const ADC_DMA_ENABLE: u32 = 1 << 8;
const ADC_REGULAR_TRIGGER_ENABLE: u32 = 1 << 20;
const ADC_REGULAR_SOFTWARE_TRIGGER: u32 = 7 << 17;
const ADC_SOFTWARE_START: u32 = 1 << 22;
const ADC_TEMPERATURE_SENSOR_ENABLE: u32 = 1 << 23;
const ADC_SAMPLE_TIME_239_5: u32 = 7;

const DMA1_BASE: usize = 0x4002_0000;
const DMA_ISR: *const u32 = DMA1_BASE as *const u32;
const DMA_IFCR: *mut u32 = (DMA1_BASE + 0x04) as *mut u32;
const DMA_CCR1: *mut u32 = (DMA1_BASE + 0x08) as *mut u32;
const DMA_CNDTR1: *mut u32 = (DMA1_BASE + 0x0c) as *mut u32;
const DMA_CPAR1: *mut u32 = (DMA1_BASE + 0x10) as *mut u32;
const DMA_CMAR1: *mut u32 = (DMA1_BASE + 0x14) as *mut u32;
const DMA_CHANNEL_ENABLE: u32 = 1;
const DMA_CIRCULAR: u32 = 1 << 5;
const DMA_MEMORY_INCREMENT: u32 = 1 << 7;
const DMA_PERIPHERAL_HALF_WORD: u32 = 1 << 8;
const DMA_MEMORY_HALF_WORD: u32 = 1 << 10;
const DMA_HIGH_PRIORITY: u32 = 2 << 12;
const DMA_TRANSFER_COMPLETE: u32 = 1 << 1;
const DMA_CLEAR_CHANNEL_1: u32 = 0x0f;

const SAMPLE_COUNT: usize = crate::config::REGULAR_ADC_CHANNELS.len();
const ANALOG_SAMPLE_PERIOD_MS: u32 = 10;
const BRAKE_DEBOUNCE_SAMPLES: u8 = 4;
const INITIAL_SCAN_WAIT: u32 = 100_000;

#[repr(align(4))]
struct DmaBuffer(UnsafeCell<[u16; SAMPLE_COUNT]>);

// SAFETY: DMA1 channel 1 is the only writer. Readers use aligned volatile
// half-word accesses after observing the transfer-complete flag.
unsafe impl Sync for DmaBuffer {}

static DMA_SAMPLES: DmaBuffer = DmaBuffer(UnsafeCell::new([0; SAMPLE_COUNT]));

static LATEST_SEQUENCE: AtomicU32 = AtomicU32::new(0);
static LATEST_VALID: AtomicBool = AtomicBool::new(false);
static LATEST_BRAKE: AtomicBool = AtomicBool::new(false);
static LATEST_THROTTLE: AtomicU16 = AtomicU16::new(0);
static LATEST_BUS_VOLTAGE: AtomicU16 = AtomicU16::new(0);
static LATEST_MOTOR_TEMPERATURE: AtomicU16 = AtomicU16::new(0);
static LATEST_CONTROLLER_TEMPERATURE: AtomicU16 = AtomicU16::new(0);
static LATEST_SAMPLED_AT_MS: AtomicU32 = AtomicU32::new(0);

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
struct AnalogReadings {
    valid: bool,
    motor_temperature: u16,
    throttle: u16,
    bus_voltage: u16,
    controller_temperature: u16,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Snapshot {
    pub analog_valid: bool,
    pub brake_active: bool,
    pub throttle: Observation,
    pub bus_voltage_adc: u16,
    pub motor_temperature_adc: u16,
    pub controller_temperature_adc: u16,
    pub sampled_at_ms: u32,
}

impl Default for Snapshot {
    fn default() -> Self {
        Self {
            analog_valid: false,
            brake_active: false,
            throttle: Observation::INVALID_ZERO,
            bus_voltage_adc: 0,
            motor_temperature_adc: 0,
            controller_temperature_adc: 0,
            sampled_at_ms: 0,
        }
    }
}

pub struct InputMonitor {
    adc_initialized: bool,
    brake: DebouncedActiveLow,
    last_digital_ms: u32,
    last_analog_ms: u32,
}

impl InputMonitor {
    pub fn initialize(now_ms: u32) -> Self {
        configure_input_pins();
        let adc_initialized = configure_regular_adc_dma();
        let mut monitor = Self {
            adc_initialized,
            brake: DebouncedActiveLow::new(BRAKE_DEBOUNCE_SAMPLES),
            last_digital_ms: now_ms.wrapping_sub(1),
            last_analog_ms: now_ms.wrapping_sub(ANALOG_SAMPLE_PERIOD_MS),
        };
        monitor.service(now_ms);
        monitor
    }

    pub fn service(&mut self, now_ms: u32) {
        if now_ms != self.last_digital_ms {
            self.brake.update(brake_pin_is_low());
            LATEST_BRAKE.store(self.brake.active(), Ordering::Relaxed);
            self.last_digital_ms = now_ms;
        }
        if now_ms.wrapping_sub(self.last_analog_ms) >= ANALOG_SAMPLE_PERIOD_MS {
            publish(sample_analog(self.adc_initialized), now_ms);
            self.last_analog_ms = now_ms;
        }
    }
}

pub fn latest() -> Snapshot {
    loop {
        let before = LATEST_SEQUENCE.load(Ordering::Acquire);
        if before & 1 != 0 {
            spin_loop();
            continue;
        }
        let valid = LATEST_VALID.load(Ordering::Relaxed);
        let raw = LATEST_THROTTLE.load(Ordering::Relaxed);
        let snapshot = Snapshot {
            analog_valid: valid,
            brake_active: LATEST_BRAKE.load(Ordering::Relaxed),
            throttle: if valid {
                Observation::from_raw(raw)
            } else {
                Observation::invalid_acquisition(raw)
            },
            bus_voltage_adc: LATEST_BUS_VOLTAGE.load(Ordering::Relaxed),
            motor_temperature_adc: LATEST_MOTOR_TEMPERATURE.load(Ordering::Relaxed),
            controller_temperature_adc: LATEST_CONTROLLER_TEMPERATURE.load(Ordering::Relaxed),
            sampled_at_ms: LATEST_SAMPLED_AT_MS.load(Ordering::Relaxed),
        };
        let after = LATEST_SEQUENCE.load(Ordering::Acquire);
        if before == after {
            return snapshot;
        }
    }
}

pub fn bus_voltage_mv(raw: u16) -> u32 {
    super::environment::bus_voltage_mv(raw)
}

fn publish(readings: AnalogReadings, now_ms: u32) {
    LATEST_SEQUENCE.fetch_add(1, Ordering::AcqRel);
    LATEST_THROTTLE.store(readings.throttle, Ordering::Relaxed);
    LATEST_BUS_VOLTAGE.store(readings.bus_voltage, Ordering::Relaxed);
    LATEST_MOTOR_TEMPERATURE.store(readings.motor_temperature, Ordering::Relaxed);
    LATEST_CONTROLLER_TEMPERATURE.store(readings.controller_temperature, Ordering::Relaxed);
    LATEST_SAMPLED_AT_MS.store(now_ms, Ordering::Relaxed);
    LATEST_VALID.store(readings.valid, Ordering::Relaxed);
    LATEST_SEQUENCE.fetch_add(1, Ordering::Release);
}

fn configure_input_pins() {
    // SAFETY: fixed STM32F103 GPIO/RCC registers, configured once at startup.
    unsafe {
        write_volatile(
            RCC_APB2ENR,
            read_volatile(RCC_APB2ENR)
                | RCC_APB2ENR_IOPAEN
                | RCC_APB2ENR_IOPBEN
                | RCC_APB2ENR_IOPCEN,
        );

        write_volatile(GPIOA_CRL, read_volatile(GPIOA_CRL) & !(0xf << (5 * 4)));
        write_volatile(GPIOB_CRL, read_volatile(GPIOB_CRL) & !0xf);
        let mut gpioc = read_volatile(GPIOC_CRL);
        gpioc &= !((0xf << (3 * 4)) | (0xf << (4 * 4)) | (0xf << (5 * 4)));
        gpioc |= GPIO_INPUT_PULL << (4 * 4);
        write_volatile(GPIOC_CRL, gpioc);
        write_volatile(GPIOC_BSRR, 1 << BRAKE_PIN);
    }
}

fn configure_regular_adc_dma() -> bool {
    // SAFETY: ADC1 is already enabled and calibrated by current-sense setup;
    // only its regular group and DMA1 channel 1 are added here.
    unsafe {
        write_volatile(RCC_AHBENR, read_volatile(RCC_AHBENR) | RCC_AHBENR_DMA1EN);

        write_volatile(DMA_CCR1, 0);
        write_volatile(DMA_IFCR, DMA_CLEAR_CHANNEL_1);
        write_volatile(DMA_CPAR1, ADC_DR as usize as u32);
        write_volatile(DMA_CMAR1, DMA_SAMPLES.0.get() as usize as u32);
        write_volatile(DMA_CNDTR1, SAMPLE_COUNT as u32);
        write_volatile(
            DMA_CCR1,
            DMA_CIRCULAR
                | DMA_MEMORY_INCREMENT
                | DMA_PERIPHERAL_HALF_WORD
                | DMA_MEMORY_HALF_WORD
                | DMA_HIGH_PRIORITY
                | DMA_CHANNEL_ENABLE,
        );

        write_volatile(ADC_CR1, read_volatile(ADC_CR1) | ADC_SCAN_MODE);
        set_sample_time(
            ADC_SMPR1,
            u32::from(crate::config::MOTOR_TEMPERATURE_ADC_CHANNEL - 10),
            ADC_SAMPLE_TIME_239_5,
        );
        set_sample_time(
            ADC_SMPR1,
            u32::from(crate::config::THROTTLE_ADC_CHANNEL - 10),
            ADC_SAMPLE_TIME_239_5,
        );
        set_sample_time(
            ADC_SMPR1,
            u32::from(crate::config::CONTROLLER_TEMPERATURE_ADC_CHANNEL - 10),
            ADC_SAMPLE_TIME_239_5,
        );
        set_sample_time(
            ADC_SMPR2,
            u32::from(crate::config::VBUS_ADC_CHANNEL),
            ADC_SAMPLE_TIME_239_5,
        );
        set_sample_time(
            ADC_SMPR2,
            u32::from(crate::config::UNUSED_TORQUE_ADC_CHANNEL),
            ADC_SAMPLE_TIME_239_5,
        );
        write_volatile(ADC_SQR1, crate::config::regular_adc_sequence_register_1());
        write_volatile(ADC_SQR3, crate::config::regular_adc_sequence_register_3());
        write_volatile(
            ADC_CR2,
            read_volatile(ADC_CR2)
                | ADC_CONTINUOUS
                | ADC_DMA_ENABLE
                | ADC_REGULAR_TRIGGER_ENABLE
                | ADC_REGULAR_SOFTWARE_TRIGGER
                | ADC_SOFTWARE_START
                | ADC_TEMPERATURE_SENSOR_ENABLE,
        );

        for _ in 0..INITIAL_SCAN_WAIT {
            if read_volatile(DMA_ISR) & DMA_TRANSFER_COMPLETE != 0 {
                return true;
            }
            spin_loop();
        }
    }
    false
}

unsafe fn set_sample_time(register: *mut u32, channel_index: u32, sample_time: u32) {
    let shift = channel_index * 3;
    // SAFETY: caller provides ADC1 SMPR1/2 and a field index within it.
    unsafe {
        let value = read_volatile(register);
        write_volatile(register, (value & !(7 << shift)) | (sample_time << shift));
    }
}

fn sample_analog(initialized: bool) -> AnalogReadings {
    if !initialized {
        return AnalogReadings::default();
    }
    // SAFETY: channel 1 remains configured for the fixed aligned buffer.
    unsafe {
        if read_volatile(DMA_ISR) & DMA_TRANSFER_COMPLETE == 0
            || read_volatile(DMA_CCR1) & DMA_CHANNEL_ENABLE == 0
        {
            return AnalogReadings::default();
        }
        write_volatile(DMA_IFCR, DMA_TRANSFER_COMPLETE);
        let samples = DMA_SAMPLES.0.get().cast::<u16>();
        let motor_temperature = read_volatile(samples);
        let throttle = read_volatile(samples.add(1));
        let bus_voltage = read_volatile(samples.add(2));
        let unused_torque = read_volatile(samples.add(3));
        let controller_temperature = read_volatile(samples.add(4));
        let valid = [
            motor_temperature,
            throttle,
            bus_voltage,
            unused_torque,
            controller_temperature,
        ]
        .into_iter()
        .all(|sample| sample <= 4_095);
        AnalogReadings {
            valid,
            motor_temperature,
            throttle,
            bus_voltage,
            controller_temperature,
        }
    }
}

fn brake_pin_is_low() -> bool {
    // SAFETY: read-only access to GPIOC input data.
    unsafe { read_volatile(GPIOC_IDR) & (1 << BRAKE_PIN) == 0 }
}
