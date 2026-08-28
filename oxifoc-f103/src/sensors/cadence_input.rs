//! Active-low cadence input selected by the bike pin map.

use crate::config::BIKE_CONFIG;
use crate::hardware::gpio;

pub fn initialize() {
    // SAFETY: one-time input setup before it is sampled.
    unsafe {
        gpio::configure_pull_up_input(BIKE_CONFIG.pins.cadence);
    }
}

pub fn pin_is_low() -> bool {
    // SAFETY: read-only observation of the configured input.
    unsafe { !gpio::read(BIKE_CONFIG.pins.cadence) }
}
