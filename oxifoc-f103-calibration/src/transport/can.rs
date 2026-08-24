//! Calibration policy layered on the shared bxCAN peripheral driver.

use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicBool, Ordering};

use oxifoc_f103::{
    hardware::peripherals,
    protocol::{self as bike_protocol, Frame},
    transport::can::{BxCan, FilterConfig},
};

use crate::protocol;

struct ReceivedFrameCell(UnsafeCell<Frame>);

// SAFETY: the CAN receive interrupt is the only writer. Foreground copies the
// frame with interrupts masked after observing RX_PENDING.
unsafe impl Sync for ReceivedFrameCell {}

static RECEIVED_FRAME: ReceivedFrameCell =
    ReceivedFrameCell(UnsafeCell::new(Frame::new(0, 0, [0; 8])));
static RX_PENDING: AtomicBool = AtomicBool::new(false);
static RX_FIFO_OVERRUN_SEEN: AtomicBool = AtomicBool::new(false);
static COMMAND_QUEUE_DROP_SEEN: AtomicBool = AtomicBool::new(false);

pub fn initialize() -> bool {
    BxCan::initialize(FilterConfig::List16 {
        identifiers: [
            protocol::COMMAND_ID,
            0x67f,
            0x210,
            0x211,
            0x212,
            0x212,
            0x212,
            0x212,
        ],
    })
}

pub fn receive_interrupt() {
    let overrun = BxCan::drain_fifo0(|frame| {
        if frame.len == 0 {
            if let Some(response) = bike_protocol::identity_response(frame.id) {
                let _ = BxCan::transmit(response);
            }
        } else if bike_protocol::is_updater_reset(frame) {
            // The calibration image is recovery firmware. The exact updater
            // request must remove every gate-drive path and reset regardless
            // of foreground or calibration state.
            peripherals::emergency_shutdown();
            cortex_m::peripheral::SCB::sys_reset();
        } else if frame.id == protocol::COMMAND_ID {
            retain_received_frame(frame);
        }
    });
    if overrun {
        RX_FIFO_OVERRUN_SEEN.store(true, Ordering::Release);
    }
}

pub fn take_received_frame() -> Option<Frame> {
    cortex_m::interrupt::free(|_| {
        if !RX_PENDING.swap(false, Ordering::AcqRel) {
            return None;
        }
        // SAFETY: the CAN interrupt cannot preempt this critical section, and
        // RX_PENDING prevents it from replacing an unread frame.
        Some(unsafe { *RECEIVED_FRAME.0.get() })
    })
}

pub fn rx_fifo_overrun_seen() -> bool {
    RX_FIFO_OVERRUN_SEEN.load(Ordering::Acquire)
}

pub fn command_queue_drop_seen() -> bool {
    COMMAND_QUEUE_DROP_SEEN.load(Ordering::Acquire)
}

fn retain_received_frame(frame: Frame) {
    let stop = frame.len >= 4
        && u32::from_le_bytes([frame.data[0], frame.data[1], frame.data[2], frame.data[3]])
            == u32::from_le_bytes(*b"STOP");
    if RX_PENDING.load(Ordering::Acquire) && !stop {
        COMMAND_QUEUE_DROP_SEEN.store(true, Ordering::Release);
        return;
    }
    // SAFETY: this function runs only in the CAN receive interrupt. The
    // release store publishes the complete copy to foreground.
    unsafe { *RECEIVED_FRAME.0.get() = frame };
    RX_PENDING.store(true, Ordering::Release);
}

pub fn transmit(frame: Frame) -> bool {
    BxCan::transmit(frame)
}
