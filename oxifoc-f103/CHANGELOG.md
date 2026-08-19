# Changelog

The F103 crate patch version identifies a CAN-flashed application image. Before
every real `flash-f103 --yes` invocation, bump the crate version and add an
entry here. Validation builds and rebuilding an unchanged image do not create a
new version.

## 0.1.8 - 2026-08-18

Changes:

- Added early RCC reset-cause capture and a signature-validated retained
  context in a dedicated NOLOAD region above the resident bootloader's SRAM
  clear/stack
  range and the application's runtime stack. Watchdog resets now preserve the
  last control checkpoint, control-cycle number, whole-handler timing, fatal
  exception class, exception detail, stacked PC, and stacked LR.
- Added reset-summary and crash-context telemetry on pages 22 and 23 and
  advanced the project telemetry schema to 6. Power and pin resets deliberately
  ignore retained SRAM context so stale data cannot be mistaken for a crash.
- Moved ISR-produced flags, live measurements, counters, maxima, observer
  diagnostics, and the coherent maximum-|d| event into the existing
  interrupt-owned `ControlState`. Foreground snapshots copy that state in one
  short critical section; only command mailboxes remain atomic across the
  foreground/interrupt boundary.

Reason: the loaded 0.1.7 run felt good and held the observer through 56.3 km/h,
but reached 4,334 of 4,500 cycles, accumulated 2,509 timing warnings, and
restarted once without a user power cycle. Post-reset telemetry could not
distinguish an IWDG/WWDG reset from a fatal exception or identify the last
completed control stage. The retained record makes the next occurrence
actionable, while removing diagnostic read-modify-write atomics recovers
control-loop margin without changing OxiFOC's control algorithm.

## 0.1.7 - 2026-08-18

Changes:

- Replaced repeated six-state Hall geometry scans with a const-built table
  indexed directly by the raw three-bit Hall value. Each entry caches its
  calibrated boundary, sector width, and adjacent raw states, matching the
  lookup-table structure of OxiFOC's original Hall implementation.
- Changed telemetry page 21 to retain the complete TIM1-entry-through-phase-
  selection maximum alongside the FOC/observer driver maximum, and advanced
  the project telemetry schema to 5. Page 6 remains the whole-handler timing.

Reason: the loaded 0.1.6 capture still restarted six times and reached 4,561
cycles against the 4,500-cycle period. Its retained maxima showed 964 cycles
in Hall-transition handling and 2,445 in the driver, with 122 warning passes in
one 0.75-second interval. The two blocks plus the fixed interrupt work only
crossed the deadline when combined. Direct Hall metadata removes the avoidable
edge-cycle scan cost while the revised timing boundary accounts for all work
before the driver on the next capture.

## 0.1.6 - 2026-08-18

Changes:

- Replaced the Hall transition's two 64-bit interval-ratio divisions with an
  exact saturating quotient/remainder calculation using the Cortex-M3's native
  32-bit divider.
- Added an exact full-observer endpoint fast path to `PhaseManager`, avoiding
  unnecessary interpolation arithmetic without changing the crossover result.
- Added retained Hall-edge and driver-step maximum timings on telemetry page
  21 and advanced the project telemetry schema to 4.
- Made a timing overrun publish the already-disabled output state immediately,
  preserving `ControlTiming` as the single root safety-loss cause instead of
  letting the next interrupt replace it with a generic hardware-fault event.

Reason: the loaded 0.1.5 capture reached 4,503 cycles against the exact 4,500-
cycle period and restarted six times. During its sustained observer interval,
the warning rate was 865 per second while measured speed predicted 847 Hall
edges per second, identifying the combined Hall-transition/observer cycle as
the over-budget path. This revision keeps the observer active while removing
the avoidable M3 arithmetic and makes the next hardware capture report the
remaining cost by subsystem.

## 0.1.5 - 2026-08-18

Changes:

- Ported OxiFOC's MXLEMMING active-flux integrator, PLL, confidence, and
  back-EMF validity gates to the canonical fixed-point `phase/observer.rs`.
- Configured `PhaseManager` for an active Hall-to-observer blend from 3,000 to
  6,000 electrical RPM, including the established readiness gate, Hall seed,
  half-turn ambiguity guard, and Hall fallback while the observer is not ready.
- Fed the observer causally paired previous-cycle stationary voltage and live
  current, using the measured 88.4 mOhm, 39 uH, and 12.2 mWb motor model, the
  160 mA/ADC-count analog conversion, and live bus-voltage scaling.
- Separated physical vehicle direction from calibrated angle-coordinate speed
  so a Hall seed gives the observer the correct PLL sign.
- Added the most recent safety-loss cause to page 12 and observer crossover,
  flux, PLL, and back-EMF diagnostics on pages 19 and 20 (telemetry schema 3).
- Changed the F103 release optimization from `z` to `s` and placed the
  interrupt-owned control state in zero-initialized RAM, leaving 984 bytes in
  the 26,200-byte application region.

Reason: exercise OxiFOC's proven sensor-to-observer architecture directly at
high speed, rather than maintaining a parallel or shadow estimator path, while
keeping the complete observer inside the F103 flash budget and designing its
hot path for the 16 kHz control-loop timing envelope.

## 0.1.4 - 2026-08-18

Changes:

- Moved fixed-point current-command, supply-current, and measured-overcurrent
  handling into the canonical `motor/foc_driver.rs` module.
- Made the F103 control state own one `FocDriver<PhaseManager<HallSensor>>`;
  the driver now owns the controller, phase provider, current limits, and
  filtered q-modulation state.
- Applied the final current circle and DC-side clamp inside the driver after
  the ride target ramp, matching OxiFOC's control-step ownership.
- Removed the parallel `foc/current_limits.rs` module and the inactive
  floating-point driver implementation. The `oxifoc-original` tag retains the
  original source for reference.

Reason: current limiting is part of OxiFOC's motor-driver sequencing, not an
independent FOC math module. Keeping it in the driver preserves the proven
ownership boundary while the F103 port replaces numeric representation and
platform I/O without inventing a second architecture.

## 0.1.3 - 2026-08-18

Changes:

- Replaced saturated fixed-point voltage-vector normalization's iterative
  64-bit square root with a dynamically scaled 32-bit integer root.
- Retained OxiFOC's circular dq limit, common direction-preserving scale, and
  coordinated anti-windup without adding a speed-dependent control path.

Reason: the 0.1.2 top-speed capture repeatedly drove the 16 kHz interrupt to
its complete 4,500-cycle budget and showed restart signatures. The expensive
path ran whenever the voltage vector saturated. Conservatively rounding the
scaled magnitude keeps the applied vector inside the same circle while using
the Cortex-M3's bounded 32-bit arithmetic.

## 0.1.2 - 2026-08-18

Changes:

- Restored OxiFOC's d-priority circular current-command envelope, filtered
  q-modulation supply-current clamp, and measured dq overcurrent trip.
- Restored direction-preserving circular voltage limiting, coordinated
  anti-windup, and trapezoidal PI integration.
- Documented that recovered stock firmware supplies hardware and motor
  constants only; regulation, limiting, estimation, and modulation follow
  OxiFOC.

Reason: remove F103-specific limiting behavior that had diverged from the
original OxiFOC control architecture. This image was flashed before its crate
version was advanced, so its page-18 telemetry incorrectly reports 0.1.1; the
ride capture is identified externally as 0.1.2.

## 0.1.1 - 2026-08-18

Changes:

- Raised the projected DC-side ride limit from 400 to 480 current counts while
  retaining the 838-count phase-current and 1,344-count trip envelopes.
- Rate-limited fixed-point Hall angle corrections above 500 electrical RPM to
  1.5 times the measured per-cycle angle travel.
- Added one-control-period output-only actuation advance; sampled current still
  uses the measurement-time Hall angle.
- Added coherent maximum-|d| event telemetry on pages 14--17 and crate
  version/schema telemetry on page 18.
- Consolidated the fixed-point controller, PI, Hall sensor, trigonometry, and
  phase-provider code into OxiFOC's canonical modules. The F103 ISR now uses
  `PhaseManager<HallSensor>` instead of a parallel compact phase stack.

Reason: the 400-count loaded run was stable but left battery-current headroom,
while its retained |d| maximum came from brief angle-transition events rather
than a steady Hall offset. This image increases acceleration authority,
smooths those estimator corrections, compensates the known PWM pipeline delay,
records enough same-cycle context to evaluate the result, and keeps future
observer work on the original OxiFOC architecture rather than an F103-only
fork of the algorithms.

## 0.1.0 - 2026-08-18

Changes: initial flashable STM32F103 fork with fixed-point Hall-only FOC, local
throttle and safety handling, stock-bike CAN compatibility, resident-bootloader
DFU, and commissioning telemetry through page 13. Hardware commissioning
progressed through the 400-count DC-side and 1,273-tick voltage-vector image.

Reason: establish a compact, synchronous, hardware-specific baseline derived
from OxiFOC while preserving the stock controller's bootloader and bike-facing
CAN behavior.
