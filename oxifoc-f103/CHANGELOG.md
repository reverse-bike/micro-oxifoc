# Changelog

The F103 crate version identifies a CAN-flashed application image. New features
bump the minor version; bug fixes to an existing feature bump the patch version.
Before every real `flash-f103 --yes` invocation, make exactly one appropriate
bump and add an entry here. Validation builds and rebuilding an unchanged image
do not create a new version.

## 0.8.0 - 2026-08-23

Changes:

- Restored the stock active-low PB9 cadence input as a local ride-demand
  source. The 1 kHz tracker uses four-sample edge debounce, requires three
  qualified pulses, rejects low phases longer than 400 ms, and expires after
  500 ms at or below 20 km/h or 300 ms above it.
- Routed qualified cadence through the existing ride owner, target ramp,
  phase-current circle, battery-current envelope, brake handling, Hall startup
  lifecycle, and fault protections. Throttle travel and the recovered stock
  cadence low-width command are each normalized to 0--100 percent; the larger
  request drives the unchanged phase-current target, allowing either input to
  take over without cycling the power stage.
- Restored stock cadence reporting in `0x203.b0..b1` as RPM multiplied by 40.
  Project page 8 now exposes cadence demand and qualification plus
  raw/debounced PB9 levels; project telemetry moves to schema 15.

Reason: the controller has a directly connected 24-pulse cadence sensor, but
the ride application only accepted throttle demand. The installed display
transmits zero in the stock assist-percentage field, so applying that field
would leave pedal assist permanently disabled. Converting its measured pulse
width into a bounded local demand restores proportional pedal assist without
making CAN a torque source or bypassing any existing current, brake, startup,
or safety envelope.

## 0.7.2 - 2026-08-23

Changes:

- Replaced the shared crate's ride-versus-calibration feature branches with
  additive `board` and `firmware` capabilities. Ride and calibration retain
  separate packages and optimization profiles, while each binary now owns its
  TIM1 and CAN interrupt entry points.
- Split bxCAN register access from application policy. The shared driver takes
  a receive callback; ride owns stock-bike commands and telemetry, while the
  calibration image owns its command queue and unconditional emergency updater
  reset path.
- Moved foreground state into `RideApp`, `RideCan`, and `CalibrationApp`
  instances, and split safety into public diagnostics, retained-record
  encoding, and hardware watchdog/runtime modules.
- Configured Rust Analyzer from the workspace root so both additive firmware
  feature sets can be checked together without contradictory IDE expansion.

Reason: application mode was encoded through many item-level feature gates,
which made the firmware difficult to navigate and let Cargo feature unification
produce misleading IDE errors. The new module and ownership boundaries preserve
the synchronous control, safety, CAN updater, and telemetry behavior while
reducing the ride payload from 25,752 to 25,632 bytes.

## 0.7.1 - 2026-08-23

Changes:

- Slowed transient-current recovery from one quarter to one eighth of unused
  continuous-current headroom. A fully exhausted bucket now requires about 27
  seconds at zero drive current to restore the 65 A allowance; the 7-second
  fresh-bucket duration and 44 A continuous limit are unchanged.
- Added the remaining full-transient-equivalent time in milliseconds to page
  11 bytes 1--2. Schema 14 reports this foreground-owned state without adding
  any work to the 16 kHz current-control interrupt.

Reason: the 0.7.0 loaded ride made three high-current pulls and then triggered
the battery's protection at 113.06 seconds. BMS current fell from 53.7 A to
zero, battery-side voltage rebounded, the controller DC link decayed, and the
next boot reported a pin/power reset; no controller fault or current trip was
present. The firmware still exposed its 640-count transient allowance with no
recovery flag immediately before shutdown. Replaying the BMS current against
the one-eighth recovery model exhausts the bucket at about 110.4 seconds,
allowing the existing 431-count continuous clamp to act before the pack opens.

## 0.7.0 - 2026-08-23

Changes:

- Split the 1 kHz ride/safety snapshot from the complete CAN diagnostic
  snapshot. The frequent foreground path now masks interrupts only long enough
  to copy Hall state, safety progress, and the measured/applied dq values used
  by the local battery envelope.
- Kept target, measured, requested, and applied dq values in their native fixed-
  point representation inside the interrupt-owned state. Integer scaling for
  CAN and the foreground battery monitor now happens after interrupts are
  unmasked instead of on every 16 kHz control pass.
- Kept observer health telemetry but moved its scaling into the foreground.
  Raw observer capture is deferred from a scheduled Hall-edge cycle to the next
  quiet control cycle so diagnostic work cannot stack with Hall transition
  handling.
- Removed the saturated timing-warning counter, dynamic q-limit and PWM-span
  telemetry, and cumulative q-error/PWM-span peaks. Schema 13 retains applied
  dq voltage, phase and direct-current peaks, field-weakening request data,
  observer health, and all fault/reset evidence.
- Wrote validated PWM compares before optional diagnostic bookkeeping and
  compiled `oxifoc-core` at release `opt-level = 2`; the firmware wrapper and
  protocol remain size-optimized. The 6,000-eRPM entry, 1,209-tick threshold,
  200-count weakening ceiling, ramp rates, and every current/fault envelope are
  unchanged.

Reason: the fault-free loaded 0.6.1 ride held field weakening for five episodes
and reached 55.7 km/h, but the worst control pass consumed 4,351 of the 4,500
available cycles. The per-page CAN cadence is about 1.25 Hz while observer
diagnostics were formatted at 62.5 Hz and live commissioning values were
formatted at the full 16 kHz rate. Retaining raw control state preserves the
same evidence and local battery protection while removing diagnostic work from
the deadline-critical path. The resulting release payload is 25,644 bytes, and
the ARM `TIM1_UP` body shrinks from 0x928 to 0x840 bytes before hardware timing
validation.

## 0.6.1 - 2026-08-22

Changes:

- Replaced the Q16.16 64-bit restoring square root in the d-priority current
  circle with a conservative scaled-u32 calculation. At the configured
  880-count circle its quantization is 1/32 count, and exhaustive fixed-domain
  tests verify that the returned q limit never crosses the requested circle.
- Removed a duplicate current-circle calculation from the driver's combined
  current and bus-limit path. A nonzero d target now computes the q-axis circle
  limit once and reuses it for the terminal-power clamp.
- Restored the previous q-only bus-limit fast path whenever target d current is
  zero. The complete `Vd * Id + Vq * Iq` projection remains active whenever a
  requested or field-weakening d target is nonzero.
- Changed the field-weakening voltage-headroom comparison to conservative
  whole-PWM-tick squared arithmetic and skip that comparison entirely below
  the entry speed. The 6,000-eRPM entry, 95-percent voltage threshold,
  200-count ceiling, and ramp rates are unchanged.

Reason: the first 0.6.0 ride recorded four `ControlTiming` safety events and no
hardware-break, phase-overcurrent, or battery-disconnect event. Whole-handler
time reached 4,620 cycles against the 4,500-cycle 16 kHz deadline, versus a
4,144-cycle maximum in the preceding firmware. The expensive current-circle
path had been dormant while the requested d current was zero; field weakening
made it execute twice per control pass. This patch keeps the same circular
current and full-dq battery envelopes while removing that real-time regression.

## 0.6.0 - 2026-08-22

Changes:

- Added OxiFOC's documented MESC-V2 field-weakening path to the synchronous
  motor driver. Above 6,000 eRPM, motoring voltage demand beyond 95 percent of
  the 1,273-tick circle ramps the d-current target negative; restored voltage
  headroom ramps it back toward zero. The first hardware envelope is limited
  to 200 current counts, reaching that limit in 50 ms and releasing over 200
  ms.
- Kept field weakening inside the existing d-priority 880-count current
  circle. At the 200-count weakening ceiling, at least 856 counts of q-current
  authority remain. The existing radial voltage limiter and `NoDecoupling`
  specialization are unchanged so the first ride isolates saturation feedback
  from voltage-allocation and feedforward changes.
- Extended the 16 kHz bus-current clamp from `Vq * Iq` to the complete target
  terminal-power projection `Vd * Id + Vq * Iq`. The clamp still uses the 2 ms
  filtered applied voltage and the selected discharge/regen bound, while the
  slower transient budget continues using measured dq terminal power.
- Made an ordinary high-speed brake request enter the existing energized,
  zero-q coasting state. This lets negative d current unwind only as voltage
  headroom returns instead of disabling the bridge above the unweakened base
  speed. Faults, invalid sensors, expired authority, and environmental loss
  still stop immediately.
- Replaced the four-page maximum-|d| event capture with page 14 field-
  weakening telemetry: active/at-limit/voltage-limit flags, target d current,
  and requested dq voltage. Removed page 21's temporary split-ISR timing and
  observer-acquisition diagnostics; page 6 retains whole-handler timing and
  pages 12--13 and 22--25 retain fault evidence. Project telemetry schema is
  now 12.

Reason: loaded rides reach the voltage circle at high speed while retaining
phase-current and battery-current capacity. Saturation feedback spends a
bounded amount of that phase-current envelope on negative d current only when
the controller has no voltage headroom, avoiding a model-derived speed table.
The 20 A initial ceiling reduces the calibrated effective flux by an estimated
11 percent and leaves nearly all low-speed q authority, while the full-dq bus
projection prevents its copper power from bypassing the 44 A continuous/65 A
transient battery envelope.

## 0.5.1 - 2026-08-22

Changes:

- Changed only the transient-budget measurement from final target q current
  times applied q voltage to measured motor-terminal power
  `Vd * Id + Vq * Iq`. The foreground converts that complete dq dot product to
  projected DC-current counts with the existing `3 / (2 * PWM_PERIOD)` scale.
- Kept OxiFOC's target-based, filtered-q-modulation bus clamp unchanged. It
  remains the conservative instantaneous command bound, while the slower
  battery budget now accounts for energy the motor actually received.
- Kept negative terminal power outside the discharge budget, preserving
  rollback-plugging behavior, and added regressions for q-axis motoring,
  d-axis power, charging, and zero-current cases.

Reason: the first 0.5.0 ride matched the BMS while the current loop tracked:
63.4 A median local transient projection versus 64.2 A from the BMS, and the
continuous state held 44.0--44.7 A. At the voltage ceiling, however, target q
remained around 760--790 counts while measured q fell toward 350--600 counts.
The target-based budget therefore continued accounting roughly 64 A while the
BMS current fell through approximately 51--40 A. It exhausted three times;
the same envelope replayed from BMS current did not exhaust. The measured dq
terminal-power projection independently fit the ride at 0.099 A/count median
against the configured 0.100 A/count, so it is the correct local,
BMS-independent quantity for accumulated battery use.

## 0.5.0 - 2026-08-22

Changes:

- Replaced the single 392-count projected DC-current ceiling with OxiFOC's
  existing runtime bus-limit path driven by a stateful battery envelope. The
  continuous limit is 431 counts, calibrated to approximately 44 A at the BMS,
  and the transient limit is 640 counts, calibrated to approximately 65 A.
- Added an excess-current budget equivalent to seven seconds at the full
  transient limit. Current above the continuous limit consumes the budget in
  proportion to its excess; headroom below the continuous limit restores it at
  one-quarter rate.
- Made an exhausted budget hold the continuous limit until it is fully
  recovered. This prevents a one-sample recovery from alternating the bus
  limit between continuous and transient values.
- Derived budget consumption from the final q-current target and applied
  q-voltage using the same `Ibus = Iq * modq` projection as OxiFOC's bus clamp.
  Rollback plugging has the opposite voltage/current sign and therefore does
  not consume the discharge budget.
- Added page-9 derating bit `0x04` for the continuous-recovery state and bumped
  project telemetry schema to 11. Existing thermal, undervoltage, phase-circle,
  measured-current, software-trip, and TIM1 break protections are unchanged.

Reason: a stock-bike ride reached 65.7 A briefly but only began reducing drive
after battery current remained above 43.5 A for roughly seven seconds. That
behavior indicates a battery with separate peak and continuous capability, not
the single approximately 40 A ceiling previously imposed by this port. The new
policy uses those observations as constants while preserving OxiFOC's original
modulation-derived bus-current enforcement instead of adding a speed table or
a second clamp inside the current loop.

## 0.4.0 - 2026-08-22

Changes:

- Restored the stock headlight command path. Display frame `0x300.b2` now
  controls a runtime boolean light state; zero disables it and any nonzero
  value enables it.
- Mirrored that state into `0x201.b4` bit 1 and the `0x64A` digital-profile
  bits 3 and 13. Local brake status continues to occupy its independent stock
  bits, and neither display traffic nor light state participates in ride
  authorization.
- Kept the complete foreground CAN telemetry service out of the application
  loop's inline body and size-optimized that 20 Hz path. The 16 kHz current
  loop and interrupt handlers retain their existing optimization policy.
- Added protocol regressions for command decoding and both stock status
  mirrors.

Reason: the installed electronics actuate the headlight from the controller's
stock CAN status mirror; there is no controller lamp GPIO to drive. The port
already produced both status frames but ignored `0x300`, leaving their
headlight bits permanently clear. Isolating the non-real-time CAN service
recovers enough flash for the feature without altering FOC timing or the
project telemetry schema.

## 0.3.0 - 2026-08-20

Changes:

- Replaced the stock-derived Hall boundaries with the circular consensus from
  four complete bidirectional calibration runs. The raw-state order and motor
  direction remain `[5, 1, 3, 2, 6, 4]` and `-1`; the calibrated Q0.16
  boundaries are now `[5484, 15416, 26478, 38207, 49768, 60875]`.
- Added a regression that pins the ride firmware to the calibrated geometry.
  Motor-model coefficients, observer crossover, current limits, protection
  thresholds, and CAN telemetry schema and layout are unchanged.

Reason: the four-run consensus found only a +1.35-degree global phase offset,
but repeatable sector-shape errors reached 7.54 electrical degrees at a sector
center and 9.62 degrees at a boundary. The calibrated geometry removes those
local errors so Hall-only and Hall-to-observer operation use the measured motor
geometry instead of the stock approximation. The worst raw-state-center spread
between runs was 5.17 degrees, while the largest corrections were concentrated
in the repeatable raw-state 6 and 4 sectors.

## 0.2.1 - 2026-08-20

Changes:

- Made a live Hall-to-observer phase reseed preserve the observer's previous
  alpha/beta current sample, matching OxiFOC's `force_phase` behavior and
  preventing the next update from interpreting the complete loaded current as
  a one-cycle current change.
- Made the explicitly requested Hall seed start a fresh observer epoch by
  resetting immediately before seeding. This clears current history frozen
  while PWM output was stopped without changing energized crossover reseeds.
- Added separate regressions for an 80 A live reseed and an 80 A stopped-
  current carryover followed by a requested seed.

Reason: the fixed-point observer's `seed` zeroed its previous-current fields,
unlike OxiFOC's floating-point observer. Its next active-flux update therefore
injected `-L * i_previous` into a freshly seeded flux vector. With the fitted
75 uH inductance this artificial term is 3.0 mWb at 40 A and 6.0 mWb at 80 A,
or approximately 22 and 45 percent of the configured 13.4 mWb rotor flux. A
direct seed now preserves causal current continuity. Requested ride-start
seeds remain cold starts because this port does not update the observer while
PWM is stopped, so their reset-and-seed operation clears any pre-stop sample
atomically inside the phase manager.

## 0.2.0 - 2026-08-20

Changes:

- Raised the single OxiFOC phase-current command ceiling from 838 to 880 ADC
  counts, nominally 83.8 to 88.0 A. The existing q-modulation-derived bus
  clamp remains the only speed-dependent reduction; no vehicle-speed current
  table was added.
- Calibrated the projected DC-side limit from 400 to 392 counts. Based on the
  loaded 0.1.18 and 0.1.19 pulls, this should move the BMS reading from
  approximately 40.6--41.1 A to 39.8--40.3 A while preserving the maximum
  usable continuous battery envelope. Thermal and undervoltage derating still
  reduce this same limit.
- Made an unavailable combined phase estimate coast at zero output and retain
  its opportunity to recover instead of latching a `PhaseEstimate` safety
  event. Invalid Hall states, skipped or implausibly fast transitions, capture
  loss, measured overcurrent, TIM1 break, and every hardware fault remain
  latching protections.
- Retained the 1,344-count software phase-current trip, leaving more than 1.5
  times the new commanded-current ceiling, and updated the compile-time and
  unit-test envelope checks accordingly.

Reason: the 0.1.19 log recorded one safety event at 92.26 s, at zero vehicle
speed and battery current, immediately after the output became inactive. It
had no internal fault, current trip, TIM1 break, BMS disconnect, or reset. A
review against the `oxifoc-original` tag confirmed that OxiFOC gates q current
to zero while phase trust is unavailable and resumes when the source recovers;
the F103 `PhaseEstimate` latch was a port divergence. The same ride's clean
full-power pulls tracked q current within 42 counts, held loaded |d| to 48
counts, and reached only 881 phase counts while the static 838-count ceiling
was active. Above approximately 23--25 km/h the modulation-derived bus limit,
not phase current or voltage, governed torque and produced 40.6--41.1 A at the
BMS. The new envelope therefore adds five percent low-speed torque, retains
OxiFOC's automatic electrical-power crossover, and corrects the projection
for measured inverter/input losses rather than asking the battery for more
than approximately 40 A.

## 0.1.19 - 2026-08-20

Changes:

- Allowed the existing stationary Hall recovery path to restore a missing
  phase estimate while a ride lease is active, provided the PWM outputs are
  physically disabled. Recovery still requires five consecutive samples of a
  valid, unchanged live Hall state before reseeding the calibrated sector
  center; active PWM, invalid Hall states, transitions, and all existing
  current and hardware-fault checks remain fail-closed.
- Added a regression test that keeps phase recovery conditional on both a
  missing estimate and disabled PWM, independent of ride-command state.

Reason: the 0.1.18 ride log recorded one `PhaseEstimate` safety loss at
51.92 s while creeping at approximately 3.1 km/h. It had no internal fault,
current trip, TIM1 break, BMS disconnect, or reset. The nearest control sample
already reported inactive output while throttle was being reapplied, but the
recovery guard rejected the safe live-Hall reseed solely because the new ride
lease was active. That made a stale low-speed estimate latch a safety event
before PWM could be enabled. Recovery is now governed by the physical output
state rather than the requested lease, closing that re-entry gap without
weakening energized-operation protection. Current limits, FOC behavior,
observer policy, Hall geometry, CAN layout, and telemetry schema are unchanged.

## 0.1.18 - 2026-08-20

Changes:

- Made an adjacent in-run Hall direction reversal retain its valid raw state
  and new direction while discarding the edge-to-edge speed interval. The
  commutation estimate holds the new calibrated sector center with zero speed
  until the next same-direction edge supplies a complete sector interval.
- Preserved the center-to-boundary interval reconstruction for the first edge
  after ride entry. Invalid and skipped Hall transitions remain fail-closed,
  and a reversal edge below the existing 100 us minimum is still rejected.

Reason: the 0.1.17 rollback log recorded three `PhaseEstimate` safety losses
without an internal fault, current trip, or TIM1 break. Immediately before the
first loss, the 40-count regen budget left the full 838-count phase limit and
measured q current tracked its -807-count target at -806 counts, confirming
that plugging authority was restored. The retained reversal event instead
changed Hall direction while its interval collapsed from 30,303 us to 759 us,
activated angle rate limiting with a 43.6-degree correction, and captured
d=-743/q=-655 counts. Those two edges recrossed the same boundary around the
turnaround; treating their time as a full-sector traversal fabricated a high
speed and expired the eight-sector Hall stale window within milliseconds while
the observer was unavailable. Discarding only that non-physical interval keeps
the low-speed Hall estimate valid through the turnaround without weakening the
transition or edge-rate checks. Bus-current limits, FOC gains, observer,
Hall geometry, CAN layout, and telemetry schema remain unchanged.

## 0.1.17 - 2026-08-20

Changes:

- Kept ride authority active while awaiting the first Hall edge, using the
  valid seeded sector-center estimate for commutation. A rollback can now
  decelerate and reverse inside its initial Hall sector without a transition.
- Retained the two-second absolute startup deadline before the first edge and
  every existing dynamic Hall deadline after the first edge. Invalid Hall
  states, skipped transitions, implausibly fast edges, current limits, and
  hardware faults remain fail-closed.
- Added a 40-count projected DC-side charge-current budget, approximately 4 A,
  so forward torque can pass through the brief regenerative plugging region
  while reversing a rollback. At 66 filtered q-voltage ticks the budget leaves
  the complete 838-count phase-current circle available; faster rollback is
  progressively limited rather than reduced immediately to zero torque.

Reason: 0.1.16 covered the no-edge interval around zero speed only after a
backward transition had moved the ride policy into startup tracking. A slight
rollback can instead reverse before crossing a Hall boundary, leaving the
policy in `AwaitingFirstEdge`; its independent 500 ms deadline then disabled
otherwise valid OxiFOC Hall commutation and required throttle release to rearm.
The original Hall path treats its seeded sector center as a usable low-speed
phase estimate, so the ride policy now preserves that authority until the
existing absolute startup bound.

The F103 port also configured OxiFOC's battery charge-current limit to zero.
While rolling backward, the filtered q voltage initially opposes the requested
forward q current, selecting that zero regen bound and clamping the target to
zero before the PI controller could establish plugging torque. The zero target
then preserved the opposing voltage sign, so the clamp self-released only after
the output reset or the wheel moved forward. A small real charge-current budget
restores OxiFOC's two-sided bus-current behavior without adding rider-commanded
regenerative braking. The 400-count discharge limit, phase-current limits,
observer, Hall geometry, CAN layout, and telemetry schema remain unchanged.

## 0.1.16 - 2026-08-19

Changes:

- Kept ride authority active while signed Hall progress remains behind its
  startup position, allowing forward torque to decelerate a backward-rolling
  wheel through the unavoidable no-edge interval at zero speed. The existing
  two-second absolute startup deadline still bounds the maneuver, and the
  dynamic Hall-edge deadline resumes as soon as progress recovers to zero.
- Raised the projected DC-side ride-current limit from 390 to 400 counts,
  nominally 40 A in the phase-derived projection and approximately 41 A at the
  BMS using the previously observed three-percent projection error.

Reason: rider validation of 0.1.15 found the 390-count limit conservative and
reproduced a throttle cutoff when the bike was rolling backward. No telemetry
was captured for that ride; inspection of the deterministic ride policy showed
that the first valid backward edge entered startup tracking, then the ordinary
100--500 ms Hall deadline expired while torque naturally slowed the wheel
through zero. The Hall sensor already represents signed progress and safely
handles adjacent reversals, so retaining authority only while that progress is
negative restores the expected maneuver without weakening invalid-state,
transition-rate, current, fault, or absolute-startup protections. FOC,
observer, Hall geometry, crossover thresholds, and telemetry schema remain
unchanged.

## 0.1.15 - 2026-08-19

Changes:

- Restored OxiFOC's low-speed Hall behavior in the fixed-point sensor: below
  500 eRPM, without a complete interval, or after the edge-derived speed has
  decayed below the threshold, commutation uses the calibrated sector center.
  A genuinely moving high-speed estimate retains the existing eight-sector
  stale cutoff and ready-observer fallback.
- Made an adjacent first Hall edge opposite the direction remembered at ride
  entry establish a fresh center-to-boundary interval instead of invalidating
  the sensor. Invalid Hall states, skipped states, and sub-100-us transitions
  remain fail-closed.
- Reduced the projected DC-side ride-current limit from 480 to 390 counts,
  targeting approximately 40 A at the BMS while retaining the 838-count phase
  circle and 1,344-count measured-current trip.

Reason: the 0.1.14 ride validated the corrected observer model through full
blend at approximately 17,000 eRPM and 49 A without a BKIN break, but exposed
two independent limits. At high speed the battery output disconnected after
8.5 seconds continuously above 45 A; controller Vbus collapsed and the next
boot reported a power reset without a controller fault. The phase-derived DC
projection under-reported BMS current by about three percent, so 390 counts is
the corresponding conservative 40 A target. At creep speed, retained events
showed a backward Hall edge followed by a stale phase estimate and large
d-current. The tagged original OxiFOC snaps to the Hall-sector center below
500 eRPM and explicitly tolerates direction reversal, whereas the fixed-point
port merely disabled its rate limiter while continuing to use the crossed
boundary and rejected one ride-entry reversal case. Restoring those semantics
bounds low-speed angle error without weakening the existing raw-state,
transition, edge-rate, or ride-timeout safety checks. Hall geometry, observer
parameters, crossover thresholds, and telemetry schema remain unchanged.

## 0.1.14 - 2026-08-19

Changes:

- Unified phase-current conversion, DC-side projection, reporting, and the
  observer on one nominal 100 mA/ADC-count scale. Current regulation and all
  protection thresholds remain in their existing ADC-count domain.
- Replaced the observer model with the loaded terminal fit: 43 mOhm effective
  phase resistance, 75 uH effective phase inductance, and 13.4 mWb flux
  linkage. This preserves the measured 4.3 mV/current-count resistive term and
  7.5 uH*A/current-count inductive term.
- Extended page 21 with signed Hall electrical speed at 4 eRPM/count and the
  internal PLL acquisition-gate state. Existing pages 19 and 20 retain
  readiness, confidence, observer speed, full PLL error, and external-validity
  travel. Project telemetry advances to schema 10.

Reason: four loaded ride logs put the motor's steady terminal behavior at
approximately 4.3 mV/current-count of resistive drop, while the observer was
configured for 14.1 mV/current-count. In the Hall-to-observer band this made
the modeled resistive subtraction overwhelm back-EMF under load, collapse the
flux estimate, and repeatedly lose readiness with large Hall disagreement and
d-axis current before hardware BKIN breaks. Independent DC power balance gives
approximately 94--100 mA per phase-current count; 160 mA/count would make the
reported motor-terminal power exceed BMS input power by roughly 60 percent at
full throttle. The preceding high-speed pull was Hall-driven after the
observer lost readiness, so it did not validate observer-only operation under
load. Hall geometry, crossover thresholds, the 0.2-radian acquisition gate,
and current-count limits are deliberately unchanged for this isolated model
correction. The flag byte reports the PLL acquisition threshold in the same
quantized units already carried by page 20; it does not participate in observer
readiness decisions. The other acquisition conditions remain derivable from
pages 19 and 20 without duplicating their comparisons in the size-constrained
firmware.

## 0.1.13 - 2026-08-19

Changes:

- Made each successful local fault acknowledgement start a new maximum-|d|
  diagnostic episode. The next active 16 kHz sample is captured unconditionally,
  even when d is exactly zero, and subsequent samples resume maximum-|d|
  ownership. Event generation continues across acknowledgements, while the
  fault-frozen event cannot leak in from an earlier acknowledged break.
- Replaced the control-cycle counter's atomic read-modify-write with the
  single-ISR-writer load/store it requires, removing the Cortex-M3 exclusive
  retry sequence from every control pass.
- Deferred observer diagnostic conversion while output is inactive, a fault is
  latched, or the observer is being seeded on the first recovery pass. The
  observer and phase-selection algorithms are unchanged; only CAN diagnostic
  publication moves to the next eligible sample.

Reason: the 0.1.12 ride log contained four separate TIM1 break episodes, not
one latch that failed to clear: each acknowledgement successfully rearmed the
controller and each later event captured a new PWM failure. The first three
breaks nevertheless reported the same maximum-|d| context because that peak
still belonged to the whole boot. The final recovery also reached 4,503 cycles
against the 4,500-cycle period. Restarting only the diagnostic peak owner at
the acknowledgement boundary makes every repeat independently actionable,
while removing the exclusive counter update and non-control diagnostic work
recovers deterministic ISR margin without changing torque, observer, or
protection behavior. The CAN layout remains telemetry schema 9.

## 0.1.12 - 2026-08-19

Changes:

- Replaced recoverable-fault power-stage shutdown with a latched safe-off
  state. TIM1's six phase-output enables and their GPIO routes become inert,
  while PA2, the timer, passive CC4 current sampling, CAN telemetry, and
  watchdog supervision remain alive. IWDG accepts TIM1 progress alone only
  while a fault is latched and the motor channels are verified disabled.
- Added local fault acknowledgement. The ride state machine first revokes its
  output lease, then permits the fault latch and TIM1 break interrupt to rearm
  only from a valid throttle-rest sample while PB12 is inactive. No torque can
  be authorized in the acknowledgement pass.
- Made the retained PWM record first-failure-wins within each fault episode,
  added RCC clock-security status to its condition flags, and kept its latest
  complete value available without requiring a reboot. The maximum-|d| event
  is also frozen when a hardware fault is observed.
- Assigned separate scheduler slots to reset/crash pages 22--23 and PWM-fault
  pages 24--25, so one diagnostic class can no longer hide the other. Project
  telemetry advances to schema 9.

Reason: the loaded 0.1.11 captures stayed within the 4,500-cycle control
period and recorded no software phase-current trip, but several cutoffs were
followed by IWDG resets. The first useful retained state showed TIM1's hardware
break flag with valid compares; later records were secondary timing and
safe-off observations that had replaced the initiating state. A hardware
break is a recoverable OxiFOC output kill, not a reason to reboot the control
and diagnostic runtime. Keeping the inactive control loop alive preserves the
root event, prevents reset/restart cascades, and requires an explicit local
safe-input acknowledgement before another ride can start.

## 0.1.11 - 2026-08-19

Changes:

- Replaced the observer seed's signed 64-bit division with a rounded Q0.32
  reciprocal and multiply. Across the controller's observed -76,000 to 76,000
  electrical-RPM range, converting the resulting PLL step back to speed stays
  within two electrical RPM of the requested seed.
- Replaced the Hall-to-observer crossover's 64-bit fixed-point ratio with a
  bounded unsigned Q16.16 calculation using the Cortex-M3's native 32-bit
  divider. Every step in the configured 3,000-to-6,000-eRPM blend band remains
  bit-identical.
- Added the missing zero-blend endpoint fast path to `PhaseManager`, returning
  Hall directly below crossover after applying the existing half-turn
  ambiguity guard.

Reason: the loaded 0.1.10 capture recorded seven low-speed cutoffs. Every
retained record identified `ControlTiming` followed by an IWDG reset, with
whole-handler times of 4,545 to 4,638 cycles against the 4,500-cycle period.
All completed the normal control path, and the trips clustered around observer
seeding and the start of the Hall-to-observer crossover. The linked image had
exactly two calls to the software 64-bit division routine: one in each of those
paths. Using the M3's hardware 32-bit divider removes that nondeterministic
latency while preserving the configured crossover and observer behavior.

## 0.1.10 - 2026-08-19

Changes:

- Restored the STM32F103 ride controller's OxiFOC specialization to
  `NoDecoupling`. The generic fixed-point decoupling implementation and its
  reference-equation tests remain available for a future minor release with an
  explicit voltage-feasibility or field-weakening policy.
- Extended the controller result with the complete pre-limit voltage request
  and its motor-model feedforward component. The retained maximum-|d| event now
  reports requested d/q, feedforward d/q, and applied d/q voltage ticks on CAN
  pages 16 and 17; telemetry schema advances to 8.

Reason: the unloaded 0.1.9 log reached about -31,000 electrical RPM while the
voltage circle was limited to 1,273 ticks. At that operating point the
reference-current feedforward alone requested approximately -505 d-axis and
-1,704 q-axis ticks. It continued using the -579-count q reference after the
voltage limit had reduced measured q current to a small fraction of that value,
so the circular limiter also suppressed the d-axis PI correction and measured
d current reached -1,035 counts. OxiFOC's own decoupling tests deliberately stop
below this voltage-saturated regime because field weakening is not implemented.
This patch removes the invalid F103 activation without changing OxiFOC's
controller equations or saturation behavior. The retained pre-limit telemetry
will distinguish PI demand, model feedforward, and circular limiting directly
if decoupling is revisited.

## 0.1.9 - 2026-08-18

Changes:

- Extended the watchdog-retained safety record with the exact PWM failure
  cause, fault and pin predicates, TIM1 status/output registers, and all three
  attempted compares. Pages 24 and 25 expose the record after reboot, and the
  project telemetry schema advances to 7.
- Ported OxiFOC's phase-current-sign dead-time compensation at its original
  location: after inverse Park and actuation advance, immediately before
  SVPWM. The fixed controller expresses the board's 694 ns dead time as the
  equivalent 50/3 phase-voltage ticks; compile-time numeric parameters keep
  disabled and fixed-hardware variants on the same source path without adding
  runtime state.
- Ported OxiFOC's reference-current dq decoupling and permanent-magnet back-EMF
  feedforward before the circular voltage limit. The compile-time motor model
  uses the recovered 39 uH phase inductance, 12.2 mWb flux linkage, and
  0.16 A/current-count scale with the active signed electrical speed and live
  bus-voltage conversion.

Reason: the loaded 0.1.8 run reproduced a top-speed cutoff and proved that an
IWDG reset followed a `PwmOutput` safety loss, with neither an exception nor a
control-loop overrun. The earlier record could not distinguish a transient
BKIN assertion, disabled PA2, compare rejection, or failed TIM1 output-enable
readback. Capturing the failed predicate before shutdown makes another event
identify the initiating hardware condition instead of only its watchdog
consequence. The inverter also loses a current-direction-dependent fraction of
each PWM period to its configured dead time. Applying OxiFOC's compensation in
the modulation path removes that repeatable voltage error from the PI loops
without changing the voltage reported to the observer.
Reference-current decoupling removes the predictable speed-dependent d-axis
and q-axis voltage disturbances before they consume PI authority. Keeping the
feedforward inside the voltage circle, but outside the PI anti-windup charge,
preserves OxiFOC's saturation and recovery behavior while targeting the loaded
run's remaining d leakage and high-speed current-tracking error.

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
