//! Small STM32F103 GPIO layer driven by the configured pin map.

use core::ptr::{read_volatile, write_volatile};

use crate::config::{DigitalOutput, GpioPin, GpioPort};

const RCC_APB2ENR: *mut u32 = 0x4002_1018 as *mut u32;
const GPIOA_BASE: usize = 0x4001_0800;
const GPIO_PORT_STRIDE: usize = 0x400;
const CRL_OFFSET: usize = 0x00;
const CRH_OFFSET: usize = 0x04;
const IDR_OFFSET: usize = 0x08;
const ODR_OFFSET: usize = 0x0c;
const BSRR_OFFSET: usize = 0x10;

pub const ANALOG_INPUT: u32 = 0;
pub const FLOATING_INPUT: u32 = 0b0100;
pub const PULLED_INPUT: u32 = 0b1000;
pub const OUTPUT_PUSH_PULL_2MHZ: u32 = 0b0010;
pub const ALTERNATE_PUSH_PULL_50MHZ: u32 = 0b1011;

const fn port_base(port: GpioPort) -> usize {
    GPIOA_BASE + port as usize * GPIO_PORT_STRIDE
}

const fn port_clock_enable(port: GpioPort) -> u32 {
    1 << (2 + port as u32)
}

#[inline(never)]
pub unsafe fn enable(pin: GpioPin) {
    unsafe {
        write_volatile(
            RCC_APB2ENR,
            read_volatile(RCC_APB2ENR) | port_clock_enable(pin.port),
        );
    }
}

#[inline(never)]
pub unsafe fn configure(pin: GpioPin, configuration: u32) {
    unsafe {
        enable(pin);
        configure_enabled(pin, configuration);
    }
}

/// Configures an input with its STM32F1 pull resistor selecting the high rail.
#[inline(never)]
pub unsafe fn configure_pull_up_input(pin: GpioPin) {
    unsafe {
        enable(pin);
        write_level_enabled(pin, true);
        configure_enabled(pin, PULLED_INPUT);
    }
}

#[inline(never)]
pub unsafe fn set_output_active(output: DigitalOutput, active: bool) {
    // The output's port clock was enabled when the output was configured.
    unsafe { write_level_enabled(output.pin, active != output.active_low) };
}

#[inline(never)]
pub unsafe fn configure_output(output: DigitalOutput, active: bool) {
    unsafe {
        enable(output.pin);
        write_level_enabled(output.pin, active != output.active_low);
        configure_enabled(output.pin, OUTPUT_PUSH_PULL_2MHZ);
    }
}

#[inline(always)]
unsafe fn configure_enabled(pin: GpioPin, configuration: u32) {
    let (offset, index) = if pin.number < 8 {
        (CRL_OFFSET, pin.number)
    } else {
        (CRH_OFFSET, pin.number - 8)
    };
    let register = (port_base(pin.port) + offset) as *mut u32;
    let shift = u32::from(index) * 4;
    unsafe {
        write_volatile(
            register,
            (read_volatile(register) & !(0xf << shift)) | (configuration << shift),
        );
    }
}

#[inline(always)]
unsafe fn write_level_enabled(pin: GpioPin, high: bool) {
    let shift = if high { pin.number } else { pin.number + 16 };
    unsafe {
        write_volatile((port_base(pin.port) + BSRR_OFFSET) as *mut u32, 1 << shift);
    }
}

#[inline(always)]
pub unsafe fn read(pin: GpioPin) -> bool {
    unsafe {
        read_volatile((port_base(pin.port) + IDR_OFFSET) as *const u32) & (1 << pin.number) != 0
    }
}

#[inline(always)]
pub unsafe fn output_is_high(pin: GpioPin) -> bool {
    unsafe {
        read_volatile((port_base(pin.port) + ODR_OFFSET) as *const u32) & (1 << pin.number) != 0
    }
}
