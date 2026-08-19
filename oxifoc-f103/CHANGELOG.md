# Changelog

The F103 crate patch version identifies a CAN-flashed application image. Before
every real `flash-f103 --yes` invocation, bump the crate version and add an
entry here. Validation builds and rebuilding an unchanged image do not create a
new version.

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
