# oxifoc-f103

`oxifoc-f103` is the isolated STM32F103 application for the recovered S73
controller. It is linked at `0x08003800` for the resident CAN bootloader and is
hard-limited to a 26,200-byte application image and 4 KiB of linked RAM.

The firmware has no async executor, allocator, floating-point operations, or
non-CAN communication stack. The 16 kHz loop uses ADC-count and PWM-tick
units, Q16.16 control values, Q0.32 electrical angle, and integer CORDIC
trigonometry.

## Architecture

The application uses the dependency-free `oxifoc-core` crate directly. The
split keeps reusable synchronous control logic out of the device crate:

- `oxifoc-core::foc` owns current-offset tracking, Clarke/Park,
  `PIController`, `FocController`, layered current limiting, voltage limiting,
  CORDIC, SVPWM, target slew, `HallSensor`, `BackEmfObserver`, and the phase
  provider/manager interfaces. These live in OxiFOC's canonical modules rather
  than a parallel F103 algorithm tree.
- `oxifoc-f103` owns the recovered pin map, direct register access, interrupt
  scheduling, board-specific gains and limits, ride safety, local inputs, and
  stock-bike CAN.

The package exposes that device layer through its additive `board` feature.
The sibling [`oxifoc-f103-calibration`](../oxifoc-f103-calibration/README.md)
enables `board`, retaining the identical ADC, TIM1, BKIN, watchdog, and
reset-forensics implementations while supplying its own control ISR and CAN
policy. The two applications remain separate packages because their current
loop and flash budgets require different optimization profiles; shared Cargo
features are additive capabilities, never mutually exclusive product modes.

The device crate follows OxiFOC's device/core boundary while remaining
synchronous:

```text
src/
├── app.rs                 foreground ride application and owned state
├── config.rs              board constants, limits, and Hall geometry
├── control/
│   ├── foc.rs             TIM1 current-control interrupt
│   └── ride.rs            local ride state machine
├── hardware/
│   └── peripherals.rs     clocks and raw STM32F103 peripheral access
├── protocol/mod.rs        pure stock/project CAN frame encoding
├── sensors/
│   ├── analog.rs          ADC/DMA input acquisition
│   ├── cadence.rs         pedal-pulse qualification and RPM
│   ├── cadence_input.rs   PB9 active-low cadence input
│   ├── environment.rs     voltage/temperature protection and derating
│   ├── inputs.rs          debounced active-low safety inputs
│   ├── throttle.rs        throttle qualification and demand mapping
│   ├── wheel.rs           wheel estimation and distance
│   └── wheel_capture.rs   TIM3 wheel capture
├── transport/
│   ├── can.rs             generic bxCAN register driver
│   └── ride.rs            stock-bike ISR policy and telemetry schedule
├── safety/
│   ├── mod.rs             public diagnostic types and pure supervision rule
│   ├── retained.rs        retained reset-record encoding and validation
│   └── runtime.rs         shutdown handlers and hardware watchdogs
├── lib.rs                 host-testable module boundary
└── main.rs                entry point and application-owned IRQ wrappers
```

Every image prepared for a real CAN flash receives one version bump and a brief
change/reason entry in [CHANGELOG.md](CHANGELOG.md). Feature additions bump the
minor version; bug fixes bump the patch version. Validation-only builds do not
bump the version. Page 18 of project telemetry reports the crate version
embedded in the running image so ride logs remain attributable.

Recovered firmware establishes the hardware map, peripheral timing, Hall
geometry, and reviewed electrical envelope. Ride telemetry supplies the
effective loaded motor model where the recovered constants and physical sensor
scale are ambiguous. Current regulation, limiting, estimation, and modulation
follow OxiFOC's control architecture.

The shared controller applies OxiFOC's current-sign dead-time compensation
after inverse Park and actuation-frame advance, immediately before SVPWM. The
hardware's 25 CKD-divided dead-time ticks are represented as 50/3 controller
phase-voltage ticks. Commanded stationary voltage remains uncompensated, as in
the original architecture, because the bridge correction makes applied
voltage track that command for the observer.

The shared controller retains OxiFOC's compile-time reference-current dq
decoupling and permanent-magnet back-EMF feedforward strategy. Its fixed model
uses one nominal 100 mA/current-count conversion throughout the F103 firmware
and the loaded terminal fit of 43 mOhm effective phase resistance, 75 uH
effective phase inductance, and 13.4 mWb flux linkage. The STM32F103 ride
specialization keeps `NoDecoupling` for the first field-weakening experiment.
`FocDriver` now follows OxiFOC's documented MESC-V2 path: motoring voltage
demand above 95 percent of the voltage circle ramps in negative d current, and
restored headroom ramps it back out. The first envelope begins above 6,000
eRPM and is capped at 200 current counts. Reference-current decoupling remains
a separate follow-up after the voltage-feasibility behavior is validated.

`PhaseManager` owns both the installed Hall sensor and fixed-point back-EMF
observer. Hall remains authoritative below 3,000 eRPM. Once OxiFOC's observer
confidence, PLL-lock, minimum-speed, and physical back-EMF validity gates pass
(or a trusted Hall seed grants initial validity), the manager blends to
observer angle through 6,000 eRPM. A
greater-than-90-degree disagreement while Hall still has authority reseeds the
observer instead of blending through a half-turn ambiguity. Hall, observer,
and Hall-to-observer crossover are the only runtime phase strategies retained
for this controller family.

## Runtime

- TIM1 generates center-aligned complementary three-phase PWM at 16 kHz. It
  raises 32,000 update interrupts per second; only the underflow half consumes
  the completed CC4-triggered injected ADC sample and runs FOC.
- ADC1 channel 11 and ADC2 channel 12 sample phase A/B current. Phase C is
  reconstructed as `-(A+B)`, and current offsets are calibrated with the
  motor PWM outputs disabled.
- TIM2 captures both polarities of the remapped XOR Hall input at 1 MHz. Ride
  control uses the four-run bidirectional calibration consensus
  `[5484, 15416, 26478, 38207, 49768, 60875]`. Below 500 eRPM,
  commutation snaps to the calibrated center of the live Hall sector; above
  that threshold, boundary corrections are rate-limited to 1.5 times the
  measured per-cycle angle travel rather than being applied as a single
  transform-frame jump.
- TIM3 captures the PB4 wheel input. Its qualified speed and volatile distance
  feed stock CAN telemetry; its independent quiet state is required for an
  updater reset.
- DWT measures every 16 kHz control pass. Page 6 of `0x2F7` reports the maximum
  and warning count; a pass over 4,500 cycles latches output off.
- WWDG requires TIM1 interrupt progress, while IWDG normally requires both
  control-cycle and injected-ADC progress. Once a fault is latched and all
  motor channels are verified disabled, TIM1 progress alone keeps IWDG alive
  so a blocked current trigger cannot erase fault telemetry with a reboot.

Motor PWM pins are analog/inert while stopped and all six TIM1 motor channels
remain disabled. After passive timer setup, PA2 is held in the
hardware-validated active-low state so the gate driver is settled before a
ride request; only fatal exception shutdown raises it. Normal stops and
recoverable faults restore MOE after an inactive break solely to keep the
internal CC4 ADC trigger alive; the six motor-channel enables remain clear.
Enabling motor channels requires a fresh 10 ms local authority lease, valid
Hall/current samples, no latched fault or pending TIM1 break, and an inactive
PB12 break input.

## Local ride inputs

Ride demand comes from the directly connected throttle and cadence sensor; CAN
never grants torque. ADC1's
regular DMA scan is:

1. `PC3/ADC_IN13` motor temperature
2. `PC5/ADC_IN15` throttle
3. `PB0/ADC_IN8` bus voltage
4. `PA5/ADC_IN5` unused torque input
5. ADC channel 16 controller temperature

The active-low brake input is PC4 with a pull-up and four matching 1 kHz
samples. Throttle is plausible at 250--3,750 ADC counts, is at rest through
850, and reaches full demand at 3,252. At high speed, brake enters the same
energized zero-q coast as throttle release so field weakening can unwind with
voltage headroom; at or below 2,500 eRPM it disarms. Boot-high, invalid
acquisition, current trip, Hall loss, or another safety loss disarms the ride
path until a fresh valid rest sample is observed. A latched hardware fault is
acknowledged only in a throttle-rest pass, after ride authority is revoked,
and only while PB12 is inactive; that pass cannot energize the motor.

The stock cadence sensor is active-low PB9 with a pull-up. Its 1 kHz tracker
requires four matching samples per edge and three complete pulses before it
can request torque. It reports RPM for the 24-pulse ring, drops qualification
after 500 ms without an edge at or below 20 km/h and after 300 ms above 20
km/h, and rejects a low phase longer than 400 ms. The average of the latest two
qualified low widths feeds the recovered stock `41074 / width_ms + 67` command
curve, whose 0--2,500 range is normalized to 0--100 percent and saturates at a
16 ms low width. Throttle travel is independently normalized to 0--100
percent; the larger of throttle and cadence becomes the ride demand applied to
the unchanged phase-current ceiling.

Cadence and throttle share one ride owner: either can sustain an active run,
throttle can take over without a stop, and brake or any local protection
removes both sources together. The display used on this bike reports zero in
the stock assist-percentage byte, so that compatibility field does not
suppress the directly wired cadence input.

The display is not a safety input. Losing CAN traffic does not revoke an
already valid local ride request; every authority decision still comes from
the directly wired inputs and local protection state.

Forward torque is negative q. Demand increases by four current counts per
millisecond and reductions apply immediately. The ride ceiling is 880 phase-
current counts (88.0 A at the loaded-run fit of 100 mA/count), including during
the two-second startup window. Startup requires its first Hall edge within 500
ms and twelve net-forward Hall transitions. While measured startup progress is
negative, the absolute two-second deadline allows the wheel to decelerate
through the no-edge interval at zero speed; the ordinary edge deadline resumes
as soon as progress recovers to the starting position. The software phase-
current guard trips above 1,344 counts (134.4 A) on A, B, or reconstructed C,
preserving at least a 1.5-times margin over commanded phase
current. The local 39 V undervoltage debounce and controller/motor thermal
envelopes can only reduce the battery-current envelope. Its continuous limit
is 431 projected counts, approximately 44 A at the BMS, while a full transient
budget permits 640 projected counts, approximately 65 A. Current above the
continuous limit consumes a budget equivalent to seven seconds at the full
transient limit. Operation below the continuous limit restores that budget at
one-quarter rate; after exhaustion, the continuous limit remains active until
recovery is complete. Budget accounting uses locally measured terminal power
`Vd * Id + Vq * Iq`; it does not require BMS communication. The current limiter
applies a circular `Id/Iq` command envelope, then derives the instantaneous
DC-side clamp from a 2 ms low-pass of applied dq voltage using
`Ibus ≈ 3 * (Vd * Id + Vq * Iq) / (2 * PWM_PERIOD)`. It has no vehicle-speed
input. Field weakening begins only when high-speed motoring voltage exceeds
1,209 ticks, ramps negative d toward 200 counts in 50 ms, and releases over
200 ms once headroom returns. The d-priority current circle leaves 856 counts
of q authority at that ceiling. The measured dq vector and each reconstructed
physical phase have independent overcurrent checks. The 1,273-tick voltage-
vector ceiling preserves the requested dq direction, and the 1,103-tick
centered PWM window reproduces the reviewed F103 modulation envelope. The PI
controller uses OxiFOC's trapezoidal integrator and coordinated circular-limit
anti-windup. The voltage vector alone is advanced by one measured electrical
control-period step to compensate the ADC-to-PWM pipeline; current Park
transformation remains at the sampled phase angle.

The interrupt-owned control state retains dq values in native fixed point. The
1 kHz ride path copies only the Hall, safety, measured-current, and applied-
voltage fields required for authority and local battery protection; CAN uses a
separate diagnostic snapshot. Unit conversion happens in the foreground after
interrupts are unmasked. Observer health is captured at 62.5 Hz on the first
scheduled control cycle without a Hall transition, preventing diagnostic and
Hall-edge work from forming one worst-case pass.

## CAN and updater

bxCAN runs at 250 kbit/s on PA11/PA12, with the active-low transceiver control
on PA13. The application provides:

- zero-length identity queries `0x210`--`0x212`;
- scheduled stock frames `0x200`--`0x204`, `0x265`, `0x266`, and `0x64A`,
  including local brake, wheel speed/distance, cadence RPM, temperatures, and
  fault pages;
- stock headlight control from `0x300.b2`, mirrored into `0x201.b4` bit 1 and
  `0x64A` bits 3 and 13 without granting or revoking ride authority;
- updater reset on `0x67F#AA552A002A...`;
- commissioning pages 6, 8--14, 18--20, and 22--25 on `0x2F7`.

Page 8 includes throttle and cadence demand percentages plus cadence-active,
debounced-level, and raw-level diagnostics alongside the other input
interlocks. Stock frame `0x203.b0..b1` reports cadence RPM multiplied by 40.

Pages 10 and 11 report live target/measured dq current, ride stage, output and
voltage-limit state, applied dq voltage, and the remaining equivalent
full-transient time in milliseconds. Page 12 carries the full internal fault
mask and safety-event count. Page 13 retains peak phase current from boot;
its direct-current maximum restarts after each successful fault
acknowledgement. Page 14 reports field-weakening active/at-limit/voltage-limit
flags, target d current, and requested dq voltage.
Page 12 also retains the specific cause of the latest safety loss. Pages 19
and 20 report observer configuration/readiness/activity, blend,
confidence, angle-coordinate eRPM, Hall disagreement, flux magnitude, q-axis
back-EMF, PLL error, and external-validity travel. Page 18 carries telemetry
schema 15 and the crate version. Page 19 supplies readiness, confidence, and
observer eRPM, while page 20 supplies the full PLL error and external-validity
progress.
Pages 22 and 23 report the RCC reset cause and, after a watchdog reset, the
retained fatal class, last control checkpoint, whole-handler timing, cycle
number, exception detail, and low address words of the stacked PC/LR. The
complete PC is recoverable because this application is confined to
`0x08003800..0x08009E57`. Pages 24 and 25 preserve the exact PWM failure
predicate, fault and condition state (including RCC clock-security status),
TIM1 status/output registers, and attempted compares. They report the first
failure of the latest fault episode during the same boot or after a watchdog
reset. Reset/crash and PWM-failure pages have independent slots, so all four
are always emitted. The project-page scheduler advances through all fifteen
slots without a wraparound phase discontinuity.
The 56-byte retained record is linked at `0x20004F00`: above the recovered
bootloader's `0x200000DC..0x20000CF7` zero-fill/stack range and outside the
application's conservative 4 KiB runtime RAM region.

On page 9, byte 1 remains a saturated one-byte current-limit value for older
tools. Byte 3 contains the overflow above 255; new readers add the two bytes to
recover effective DC limits through 510 counts. The 640-count transient state
therefore appears as 510 to this legacy layout; derating-reason bit `0x04`
distinguishes the exhausted continuous-recovery state, whose 431-count limit
remains exact. Page 11 bytes 1--2 report how many milliseconds of equivalent
full-transient excess remain. Firmware-version page 18 reports telemetry
schema 15.

An updater request is one-shot. It is accepted only with no local command,
motor channels disabled, at least 500 ms without a motor Hall edge, and the PB4
wheel input qualified quiet for roughly 0.85 seconds. An unsafe request is
discarded and must be retransmitted.

The resident bootloader branches to the application with `PRIMASK` set, so
startup explicitly enables global interrupts only after every peripheral
handler and its shared state are initialized. The captured USER option byte is
`0xFF` (`WDG_SW=1`); the application's software-started IWDG and WWDG return to
their inactive reset state before the bootloader begins an update.

Build and inspect the exact bootloader image from the repository root:

```sh
command just size
command just image-f103
command just flash-f103
```

`just flash-f103` is validation-only and transmits nothing. Supplying `--yes`
is the explicit destructive confirmation for a real CAN update. Do not use it
until passive bench and elevated-wheel commissioning are complete.

## Validation boundary

Host behavior tests, target Clippy, linking, section inspection, stack-size
metadata, and bootloader-image validation can establish software consistency
and the real flash/RAM footprint. They cannot prove GPIO polarity, current
scaling, Hall geometry, control-loop cycle time, or motor direction on this
specific board. Those remain hardware commissioning gates. Version 0.1.5 was
flashed over CAN on 2026-08-18 and answered the stock identity query after the
bootloader reset. A stationary post-flash capture reported telemetry schema 3,
no faults or safety losses, and a 1,091-cycle maximum against the 4,500-cycle
control budget. Its loaded capture subsequently exposed Hall-correlated timing
overruns and six resets. Version 0.1.6 replaced the wide Hall interval
divisions, but its loaded capture still reached 4,561 cycles, accumulated 122
warnings within 0.75 seconds, and restarted six times. Its schema-4 breakdown
reported 964 Hall-edge cycles and 2,445 driver cycles. Version 0.1.7 replaces
the remaining geometry scans with direct raw-state metadata and moves the
schema-5 timing boundary around the complete pre-driver phase path. Its loaded
capture held the observer through 56.3 km/h and felt good, but reached 4,334
cycles, accumulated 2,509 warning passes, and restarted once without a power
cycle. Version 0.1.8 retains reset/crash context across watchdog resets and
removes read-modify-write diagnostic atomics from the control interrupt. Its
loaded capture reduced timing warnings to 117 and held the whole-handler
maximum to 4,259 cycles, but reproduced one top-speed cutoff. The retained
record proved an IWDG reset followed a `PwmOutput` safety loss without an
exception or timing overrun. Version 0.1.9 extends that record with the exact
output predicate and TIM1 state so another event can identify the initiating
hardware condition. Its unloaded run also exposed reference-current
decoupling outside OxiFOC's tested voltage-feasible range: feedforward exceeded
the voltage circle and measured d current reached -1,035 counts despite a zero
d target. Version 0.1.10 restores the ride specialization to `NoDecoupling` and
adds retained pre-limit request/feedforward telemetry on schema 8. Its loaded
capture recorded seven low-speed `ControlTiming` losses followed by watchdog
resets, with retained handler times of 4,545 to 4,638 cycles against the 4,500-
cycle period. Version 0.1.11 replaces the two linked 64-bit divisions in the
observer-seed and crossover paths with native 32-bit arithmetic and returns
Hall directly at the zero-blend endpoint. Version 0.1.12 turns recoverable
faults into a persistent safe-off state, rearms only from local throttle rest
with inactive BKIN, and reports reset and first-failure context independently
on schema 9. Version 0.1.13 gives every acknowledged break fresh maximum-|d|
ownership and removes avoidable recovery-path timing work without changing the
control strategy or wire layout. Version 0.1.14 corrects the observer's loaded
terminal model and unifies the firmware on its ride-validated nominal
100 mA/current-count conversion. Schema 10 adds live Hall speed and the PLL
acquisition-gate state for the next loaded validation. Version 0.1.15 restores
the original low-speed Hall-sector-center and direction-reversal behavior and
sets the projected DC-side limit to 390 counts. Version 0.1.16 keeps ride
authority through a measured backward startup while retaining the absolute
startup deadline, and raises the projected DC-side limit to 400 counts. Version
0.1.17 also retains authority when rollback reverses inside the initially
seeded Hall sector, before any post-command Hall edge can occur, and permits a
40-count projected charge-current transient so plugging torque can reverse the
wheel instead of being clamped to zero. Version 0.1.18 treats the boundary
recross at an in-run direction reversal as an unknown-speed sector-center hold
until the next complete same-direction Hall interval. Version 0.1.19 permits
the existing stable live-Hall recovery to repair a stale estimate before
energizing even when a new ride lease is already active.
Version 0.2.0 raises the common phase-current ceiling to 880 counts, calibrates
the OxiFOC bus-power clamp to 392 counts, and restores OxiFOC's torqueless
self-recovery policy when the combined phase estimate is temporarily
unavailable. Version 0.2.1 makes energized observer reseeds preserve causal
current history while explicitly requested ride-start seeds clear current
history frozen during stopped output. Version 0.3.0 replaces the stock-derived
Hall boundaries with the circular consensus from four complete bidirectional
calibration runs; the motor direction, observer model, crossover, and current
envelope are unchanged. Version 0.4.0 restores the stock headlight status
mirror. Version 0.5.0 adds the 65 A transient/44 A continuous battery envelope
through OxiFOC's existing modulation-derived bus-current clamp. Version 0.5.1
makes its accumulated budget use measured dq terminal power instead of current
that the voltage-limited motor was unable to reach. Version 0.6.0 adds
saturation-feedback field weakening with a 200-count d-current ceiling and
extends the instantaneous bus clamp to complete dq terminal power. Version
0.6.1 makes its fixed-point current-circle and voltage-threshold arithmetic
meet the F103's 16 kHz deadline without changing that weakening policy. Version
0.7.0 moves diagnostic formatting out of the 16 kHz path, separates the compact
ride snapshot from CAN commissioning data, and speed-optimizes the shared core
without changing field weakening or the current envelopes. Version 0.7.1
slows recovery of the repeated-pull transient budget and reports its remaining
equivalent peak time without adding ISR work.
