//! Synchronous transports used by the recovered controller.

#[cfg(feature = "board")]
pub mod can;

#[cfg(feature = "firmware")]
mod ride;
#[cfg(feature = "firmware")]
pub use ride::{RideCan, RuntimeTelemetry, receive_interrupt};

/// Lowest empty bxCAN transmit mailbox from the CAN_TSR TME0..TME2 bits.
#[cfg(any(feature = "board", test))]
pub(crate) const fn first_empty_mailbox(transmit_status: u32) -> Option<usize> {
    let empty = transmit_status >> 26;
    if empty & 1 != 0 {
        Some(0)
    } else if empty & 2 != 0 {
        Some(1)
    } else if empty & 4 != 0 {
        Some(2)
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::first_empty_mailbox;

    #[test]
    fn transmit_selects_the_lowest_empty_mailbox_from_tsr() {
        assert_eq!(first_empty_mailbox(0), None);
        assert_eq!(first_empty_mailbox(0b111 << 26), Some(0));
        assert_eq!(first_empty_mailbox(0b110 << 26), Some(1));
        assert_eq!(first_empty_mailbox(0b100 << 26), Some(2));
        assert_eq!(first_empty_mailbox(!(0b111 << 26)), None);
    }
}
