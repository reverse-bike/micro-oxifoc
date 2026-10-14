//! Passive local ride-input acquisition.

use core::cell::UnsafeCell;
use core::hint::spin_loop;
use core::ptr::{read_volatile, write_volatile};

use crate::config::{ANALOG_SAMPLE_PERIOD_MS, BIKE_CONFIG, adc_sample_time_field};
use crate::hardware::gpio;
use crate::hardware::system::RCC_AHBENR;

use super::inputs::DebouncedActiveLow;
use super::throttle::Observation;

const RCC_AHBENR_DMA1EN: u32 = 1;

const ADC1_BASE: usize = 0x4001_2400;
const ADC_CR1: *mut u32 = (ADC1_BASE + 0x04) as *mut u32;
const ADC_CR2: *mut u32 = (ADC1_BASE + 0x08) as *mut u32;
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
const BRAKE_DEBOUNCE_SAMPLES: u8 = BIKE_CONFIG.ride.brake_debounce_milliseconds;
const INITIAL_SCAN_WAIT: u32 = 100_000;

#[repr(align(4))]
struct DmaBuffer(UnsafeCell<[u16; SAMPLE_COUNT]>);

// SAFETY: DMA1 channel 1 is the only writer. Readers use aligned volatile
// half-word accesses after observing the transfer-complete flag.
unsafe impl Sync for DmaBuffer {}

static DMA_SAMPLES: DmaBuffer = DmaBuffer(UnsafeCell::new([0; SAMPLE_COUNT]));

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

/// Foreground-owned input sampling. Its snapshot is read only by the
/// foreground loop that services it.
pub struct InputMonitor {
    adc_initialized: bool,
    brake: DebouncedActiveLow,
    last_digital_ms: u32,
    last_analog_ms: u32,
    latest: Snapshot,
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
            latest: Snapshot::default(),
        };
        monitor.service(now_ms);
        monitor
    }

    pub fn service(&mut self, now_ms: u32) {
        if now_ms != self.last_digital_ms {
            self.latest.brake_active = self.brake.update(brake_pin_is_low());
            self.last_digital_ms = now_ms;
        }
        if now_ms.wrapping_sub(self.last_analog_ms) >= ANALOG_SAMPLE_PERIOD_MS {
            self.sample_analog(now_ms);
            self.last_analog_ms = now_ms;
        }
    }

    pub const fn latest(&self) -> Snapshot {
        self.latest
    }

    fn sample_analog(&mut self, now_ms: u32) {
        let latest = &mut self.latest;
        latest.sampled_at_ms = now_ms;
        // SAFETY: channel 1 remains configured for the fixed aligned buffer.
        let dma_ready = self.adc_initialized
            && unsafe {
                read_volatile(DMA_ISR) & DMA_TRANSFER_COMPLETE != 0
                    && read_volatile(DMA_CCR1) & DMA_CHANNEL_ENABLE != 0
            };
        latest.analog_valid = dma_ready;
        if !dma_ready {
            latest.throttle = Observation::INVALID_ZERO;
            latest.bus_voltage_adc = 0;
            latest.motor_temperature_adc = 0;
            latest.controller_temperature_adc = 0;
            return;
        }
        // SAFETY: the DMA buffer is aligned and only written by DMA1 channel
        // 1 in right-aligned 12-bit regular-sequence order.
        unsafe {
            write_volatile(DMA_IFCR, DMA_TRANSFER_COMPLETE);
            let samples = DMA_SAMPLES.0.get().cast::<u16>();
            latest.motor_temperature_adc = read_volatile(samples);
            latest.throttle = Observation::from_raw(read_volatile(samples.add(1)));
            latest.bus_voltage_adc = read_volatile(samples.add(2));
            latest.controller_temperature_adc = read_volatile(samples.add(4));
        }
    }
}

pub fn bus_voltage_mv(raw: u16) -> u32 {
    super::environment::bus_voltage_mv(raw)
}

fn configure_input_pins() {
    // SAFETY: configured once at startup before the inputs are sampled.
    unsafe {
        for input in [
            BIKE_CONFIG.pins.motor_temperature,
            BIKE_CONFIG.pins.throttle,
            BIKE_CONFIG.pins.bus_voltage,
            BIKE_CONFIG.pins.auxiliary_analog,
        ] {
            gpio::configure(input.pin, gpio::ANALOG_INPUT);
        }
        gpio::configure_pull_up_input(BIKE_CONFIG.pins.brake);
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
        for channel in crate::config::REGULAR_ADC_CHANNELS {
            set_sample_time(channel, ADC_SAMPLE_TIME_239_5);
        }
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

unsafe fn set_sample_time(channel: u8, sample_time: u32) {
    let (offset, shift) = adc_sample_time_field(channel);
    let register = (ADC1_BASE + offset) as *mut u32;
    // SAFETY: the offset and shift select this channel's ADC1 SMPR field.
    unsafe {
        let value = read_volatile(register);
        write_volatile(register, (value & !(7 << shift)) | (sample_time << shift));
    }
}

fn brake_pin_is_low() -> bool {
    // SAFETY: read-only access to the configured GPIO input.
    unsafe { !gpio::read(BIKE_CONFIG.pins.brake) }
}
