//! Poll-free bxCAN transport for the required stock-bike interface.

use crate::config::BIKE_CONFIG;
use crate::hardware::gpio;
use crate::hardware::system::{RCC_APB1ENR, RCC_APB1RSTR, remap_afio, set_interrupt_priority};
use crate::protocol::Frame;
use core::ptr::{read_volatile, write_volatile};
use core::sync::atomic::{Ordering, compiler_fence};

const RCC_APB1ENR_CANEN: u32 = 1 << 25;

const CAN_BASE: usize = 0x4000_6400;
const CAN_MCR: *mut u32 = CAN_BASE as *mut u32;
const CAN_MSR: *const u32 = (CAN_BASE + 0x04) as *const u32;
const CAN_TSR: *const u32 = (CAN_BASE + 0x08) as *const u32;
const CAN_RF0R: *mut u32 = (CAN_BASE + 0x0c) as *mut u32;
const CAN_IER: *mut u32 = (CAN_BASE + 0x14) as *mut u32;
const CAN_BTR: *mut u32 = (CAN_BASE + 0x1c) as *mut u32;
const CAN_RX0: usize = CAN_BASE + 0x1b0;
const CAN_FMR: *mut u32 = (CAN_BASE + 0x200) as *mut u32;
const CAN_FM1R: *mut u32 = (CAN_BASE + 0x204) as *mut u32;
const CAN_FS1R: *mut u32 = (CAN_BASE + 0x20c) as *mut u32;
const CAN_FFA1R: *mut u32 = (CAN_BASE + 0x214) as *mut u32;
const CAN_FA1R: *mut u32 = (CAN_BASE + 0x21c) as *mut u32;
const CAN_F0R1: *mut u32 = (CAN_BASE + 0x240) as *mut u32;
const CAN_F0R2: *mut u32 = (CAN_BASE + 0x244) as *mut u32;
const CAN_F1R1: *mut u32 = (CAN_BASE + 0x248) as *mut u32;
const CAN_F1R2: *mut u32 = (CAN_BASE + 0x24c) as *mut u32;

const CAN_INIT_TIMEOUT: u32 = 1_000_000;
const CAN_BTR_CONFIG: u32 = ((crate::config::CAN_BS2_TQ as u32 - 1) << 20)
    | ((crate::config::CAN_BS1_TQ as u32 - 1) << 16)
    | (crate::config::CAN_SJW_TQ as u32 - 1) << 24
    | (crate::config::CAN_PRESCALER as u32 - 1);
const CAN_RESET_REQUEST: u32 = 1;
const CAN_RESET_ACK: u32 = 1;
const CAN_FIFO_PENDING_INTERRUPT: u32 = 1 << 1;
const CAN_FIFO_RELEASE: u32 = 1 << 5;
const CAN_FIFO_OVERRUN: u32 = 1 << 4;
/// NVIC priority of the FIFO0 receive interrupt, the only interrupt that
/// transmits. It is below Hall capture, hardware break, and the FOC update.
pub const RECEIVE_INTERRUPT_PRIORITY: u8 = 0x40;

// BASEPRI 0 disables masking, so a zero priority could not guard transmit.
const _: () = assert!(RECEIVE_INTERRUPT_PRIORITY != 0);

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FilterConfig {
    Mask32 { identifier: u32, mask: u32 },
    List16 { identifiers: [u16; 8] },
}

pub struct BxCan;

impl BxCan {
    pub fn initialize(filters: FilterConfig) -> bool {
        // SAFETY: one-time peripheral setup before CAN IRQ is unmasked.
        unsafe {
            write_volatile(RCC_APB1ENR, read_volatile(RCC_APB1ENR) | RCC_APB1ENR_CANEN);
            write_volatile(
                RCC_APB1RSTR,
                read_volatile(RCC_APB1RSTR) | RCC_APB1ENR_CANEN,
            );
            write_volatile(
                RCC_APB1RSTR,
                read_volatile(RCC_APB1RSTR) & !RCC_APB1ENR_CANEN,
            );

            gpio::configure(BIKE_CONFIG.pins.can.rx, gpio::FLOATING_INPUT);
            gpio::configure(BIKE_CONFIG.pins.can.tx, gpio::ALTERNATE_PUSH_PULL_50MHZ);
            gpio::configure_output(BIKE_CONFIG.pins.can.transceiver_enable, true);
            remap_afio(
                crate::config::AFIO_MAPR_CAN_REMAP_MASK,
                crate::config::can_remap_bits(),
            );

            write_volatile(CAN_MCR, CAN_RESET_REQUEST);
            if !wait_msr(CAN_RESET_ACK, true) {
                return false;
            }
            write_volatile(CAN_MCR, CAN_RESET_REQUEST | (1 << 6) | (1 << 2));
            write_volatile(CAN_BTR, CAN_BTR_CONFIG);

            write_volatile(CAN_FMR, read_volatile(CAN_FMR) | 1);
            match filters {
                FilterConfig::Mask32 { identifier, mask } => {
                    write_volatile(CAN_FA1R, read_volatile(CAN_FA1R) & !1);
                    write_volatile(CAN_FM1R, read_volatile(CAN_FM1R) & !1);
                    write_volatile(CAN_FS1R, read_volatile(CAN_FS1R) | 1);
                    write_volatile(CAN_FFA1R, read_volatile(CAN_FFA1R) & !1);
                    write_volatile(CAN_F0R1, identifier);
                    write_volatile(CAN_F0R2, mask);
                    write_volatile(CAN_FA1R, read_volatile(CAN_FA1R) | 1);
                }
                FilterConfig::List16 { identifiers } => {
                    const BANKS: u32 = 0b11;
                    write_volatile(CAN_FA1R, read_volatile(CAN_FA1R) & !BANKS);
                    write_volatile(CAN_FM1R, read_volatile(CAN_FM1R) | BANKS);
                    write_volatile(CAN_FS1R, read_volatile(CAN_FS1R) & !BANKS);
                    write_volatile(CAN_FFA1R, read_volatile(CAN_FFA1R) & !BANKS);
                    write_volatile(CAN_F0R1, filter16_pair(identifiers[0], identifiers[1]));
                    write_volatile(CAN_F0R2, filter16_pair(identifiers[2], identifiers[3]));
                    write_volatile(CAN_F1R1, filter16_pair(identifiers[4], identifiers[5]));
                    write_volatile(CAN_F1R2, filter16_pair(identifiers[6], identifiers[7]));
                    write_volatile(CAN_FA1R, read_volatile(CAN_FA1R) | BANKS);
                }
            }
            write_volatile(CAN_FMR, read_volatile(CAN_FMR) & !1);

            write_volatile(CAN_IER, CAN_FIFO_PENDING_INTERRUPT);
            write_volatile(CAN_MCR, read_volatile(CAN_MCR) & !CAN_RESET_REQUEST);
            if !wait_msr(CAN_RESET_ACK, false) {
                return false;
            }
            set_interrupt_priority(20, RECEIVE_INTERRUPT_PRIORITY);
            cortex_m::peripheral::NVIC::unmask(stm32f1::stm32f103::Interrupt::USB_LP_CAN_RX0);
        }
        true
    }
}

pub fn transmit(frame: Frame) -> bool {
    BxCan::transmit(frame)
}

impl BxCan {
    pub fn transmit(frame: Frame) -> bool {
        // Raising BASEPRI to the receive-interrupt priority across selection
        // and fill keeps the RX interrupt and foreground telemetry from
        // selecting the same mailbox, so neither caller's frame is dropped by
        // contention, while TIM1 update/break and Hall capture still preempt.
        // BASEPRI_MAX only ever raises the mask, so a caller already running
        // under a stricter BASEPRI keeps it, and the previous value is
        // restored afterwards. Inside the receive interrupt the raise is a
        // no-op because execution is already at that priority.
        let previous_basepri = cortex_m::register::basepri::read();
        cortex_m::register::basepri_max::write(RECEIVE_INTERRUPT_PRIORITY);
        compiler_fence(Ordering::SeqCst);
        // SAFETY: mailbox ownership is claimed by setting TXRQ only after all
        // fields are written, and no other transmitter can preempt this
        // section while BASEPRI masks the receive interrupt.
        let sent = unsafe {
            if let Some(mailbox) = super::first_empty_mailbox(read_volatile(CAN_TSR)) {
                let base = CAN_BASE + 0x180 + mailbox * 0x10;
                write_volatile(base as *mut u32, u32::from(frame.id) << 21);
                write_volatile((base + 4) as *mut u32, u32::from(frame.len));
                write_volatile(
                    (base + 8) as *mut u32,
                    u32::from_le_bytes([
                        frame.data[0],
                        frame.data[1],
                        frame.data[2],
                        frame.data[3],
                    ]),
                );
                write_volatile(
                    (base + 12) as *mut u32,
                    u32::from_le_bytes([
                        frame.data[4],
                        frame.data[5],
                        frame.data[6],
                        frame.data[7],
                    ]),
                );
                write_volatile(base as *mut u32, (u32::from(frame.id) << 21) | 1);
                true
            } else {
                false
            }
        };
        compiler_fence(Ordering::SeqCst);
        // SAFETY: restores the mask that was active on entry, which is never
        // lower than the caller's own masking requirement.
        unsafe { cortex_m::register::basepri::write(previous_basepri) };
        sent
    }
}

impl BxCan {
    pub fn drain_fifo0(mut receive: impl FnMut(Frame)) -> bool {
        let mut overrun = false;
        // SAFETY: FIFO0 is exclusively drained by this interrupt.
        unsafe {
            let fifo = read_volatile(CAN_RF0R);
            if fifo & CAN_FIFO_OVERRUN != 0 {
                overrun = true;
                write_volatile(CAN_RF0R, CAN_FIFO_OVERRUN);
            }
            while read_volatile(CAN_RF0R) & 3 != 0 {
                let identifier = read_volatile(CAN_RX0 as *const u32);
                let len = (read_volatile((CAN_RX0 + 4) as *const u32) & 0xf).min(8) as u8;
                let low = read_volatile((CAN_RX0 + 8) as *const u32).to_le_bytes();
                let high = read_volatile((CAN_RX0 + 12) as *const u32).to_le_bytes();
                if identifier & 6 == 0 {
                    let frame = Frame::new(
                        (identifier >> 21) as u16,
                        len,
                        [
                            low[0], low[1], low[2], low[3], high[0], high[1], high[2], high[3],
                        ],
                    );
                    receive(frame);
                }
                write_volatile(CAN_RF0R, CAN_FIFO_RELEASE);
            }
        }
        overrun
    }
}

const fn filter16_pair(low_id: u16, high_id: u16) -> u32 {
    ((low_id as u32) << 5) | ((high_id as u32) << 21)
}

unsafe fn wait_msr(mask: u32, set: bool) -> bool {
    for _ in 0..CAN_INIT_TIMEOUT {
        let present = unsafe { read_volatile(CAN_MSR) } & mask != 0;
        if present == set {
            return true;
        }
    }
    false
}
