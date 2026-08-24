#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};

use cortex_m::asm;
use cortex_m_rt::{entry, exception};
use oxifoc_f103_calibration::{app::CalibrationApp, control, transport};
#[allow(unused_imports, reason = "links the PAC interrupt-vector table")]
use stm32f1::stm32f103 as _;
use stm32f1::stm32f103::interrupt;

static MILLIS: AtomicU32 = AtomicU32::new(0);

#[entry]
fn main() -> ! {
    let mut app = CalibrationApp::new();
    app.initialize();
    loop {
        app.service(MILLIS.load(Ordering::Relaxed));
        asm::wfi();
    }
}

#[exception]
fn SysTick() {
    MILLIS.fetch_add(1, Ordering::Relaxed);
}

#[interrupt]
fn TIM1_UP() {
    control::timer_update_interrupt();
}

#[interrupt]
fn USB_LP_CAN_RX0() {
    transport::can::receive_interrupt();
}
