# oxifoc-f103

`oxifoc-f103` is the isolated STM32F103 application for the recovered S73
controller. It is linked at `0x08003800` for the resident CAN bootloader and is
hard-limited to a 26,200-byte application image and 4 KiB of linked RAM.

The firmware has no async executor, allocator, floating-point operations,
formatting stack, USB/UART/RTT transport, persistent storage, HFI, or
motor-detection path. The 16 kHz loop uses ADC-count and PWM-tick units,
Q16.16 control values, Q0.32 electrical angle, and integer CORDIC
trigonometry.

## Architecture

The application uses `oxifoc-core` with default features disabled and only the
`fixed-point` feature enabled. The split keeps reusable control logic out of
the device crate without pulling the floating-point or async stack into the
image:

- `oxifoc-core::foc` owns current-offset tracking, Clarke/Park,
  `PIController`, `FocController`, layered current limiting, voltage limiting,
  CORDIC, SVPWM, target slew, `HallSensor`, `BackEmfObserver`, and the phase
  provider/manager interfaces. These live in OxiFOC's canonical modules rather
  than a parallel F103 algorithm tree.
- `oxifoc-f103` owns the recovered pin map, direct register access, interrupt
  scheduling, board-specific gains and limits, ride safety, local inputs, and
  stock-bike CAN.

The device crate follows OxiFOC's device/core boundary while remaining
synchronous:

```text
src/
├── config.rs              board constants, limits, and Hall geometry
├── control/
│   ├── foc.rs             TIM1 current-control interrupt
│   └── ride.rs            local ride state machine
├── hardware/
│   └── peripherals.rs     clocks and raw STM32F103 peripheral access
├── protocol/mod.rs        pure stock/project CAN frame encoding
├── sensors/
│   ├── analog.rs          ADC/DMA input acquisition
│   ├── environment.rs     voltage/temperature protection and derating
│   ├── inputs.rs          debounced active-low safety inputs
│   ├── throttle.rs        throttle qualification and demand mapping
│   ├── wheel.rs           wheel estimation and distance
│   └── wheel_capture.rs   TIM3 wheel capture
├── transport/can.rs       bxCAN driver and message schedule
├── safety.rs              fatal gate shutdown and watchdog supervision
├── lib.rs                 host-testable module boundary
└── main.rs                synchronous initialization and foreground loop
```

Every image prepared for a real CAN flash receives one patch-version bump and
a brief change/reason entry in [CHANGELOG.md](CHANGELOG.md). Validation-only
builds do not bump the version. Page 18 of project telemetry reports the crate
version embedded in the running image so ride logs remain attributable.

Recovered firmware is used only to establish hardware and motor constants:
pin assignments, peripheral timing, sensor conversions, Hall geometry, and
the reviewed electrical envelope. Current regulation, limiting, estimation,
and modulation follow OxiFOC's control architecture.

The shared controller applies OxiFOC's current-sign dead-time compensation
after inverse Park and actuation-frame advance, immediately before SVPWM. The
hardware's 25 CKD-divided dead-time ticks are represented as 50/3 controller
phase-voltage ticks. Commanded stationary voltage remains uncompensated, as in
the original architecture, because the bridge correction makes applied
voltage track that command for the observer.

`PhaseManager` owns both the installed Hall sensor and fixed-point back-EMF
observer. Hall remains authoritative below 3,000 eRPM. Once OxiFOC's observer
confidence, PLL-lock, minimum-speed, and physical back-EMF validity gates pass
(or a trusted Hall seed grants initial validity), the manager blends to
observer angle through 6,000 eRPM. A
greater-than-90-degree disagreement while Hall still has authority reseeds the
observer instead of blending through a half-turn ambiguity. Encoder, HFI,
manual, and open-loop source identities remain available for later experiments
but are rejected until their providers are installed.

## Runtime

- TIM1 generates center-aligned complementary three-phase PWM at 16 kHz. It
  raises 32,000 update interrupts per second; only the underflow half consumes
  the completed CC4-triggered injected ADC sample and runs FOC.
- ADC1 channel 11 and ADC2 channel 12 sample phase A/B current. Phase C is
  reconstructed as `-(A+B)`, and current offsets are calibrated with the
  motor PWM outputs disabled.
- TIM2 captures both polarities of the remapped XOR Hall input at 1 MHz. Ride
  control uses the hardware-validated stock boundary table
  `[5699, 16526, 26499, 37754, 49151, 59124]`. The later occupancy-derived
  candidate was rejected by hardware testing and is not used. Above 500 eRPM,
  boundary corrections are rate-limited to 1.5 times the measured per-cycle
  angle travel rather than being applied as a single transform-frame jump.
- TIM3 captures the PB4 wheel input. Its qualified speed and volatile distance
  feed stock CAN telemetry; its independent quiet state is required for an
  updater reset.
- DWT measures every 16 kHz control pass. Page 6 of `0x2F7` reports the maximum
  and warning count; a pass over 4,500 cycles latches output off. Page 21
  retains separate maxima for TIM1 entry through phase selection and the FOC
  driver step. Subtracting both from page 6's whole-handler maximum bounds the
  remaining output-publication cost.
- WWDG requires TIM1 interrupt progress, while IWDG is refreshed only when the
  foreground observes both control-cycle and injected-ADC progress.

Motor PWM pins are analog/inert while stopped and all six TIM1 motor channels
remain disabled. After passive timer setup, PA2 is held in the
hardware-validated active-low state so the gate driver is settled before a
ride request; fatal shutdown raises it. MOE is retained while normally stopped
only to keep the internal CC4 ADC trigger alive. Enabling motor channels
requires a fresh 10 ms local authority lease, valid Hall/current samples, no
latched fault, and an inactive PB12 break input.

## Local ride input

Ride demand comes directly from `PC5/ADC_IN15`; CAN never grants torque. ADC1's
regular DMA scan is:

1. `PC3/ADC_IN13` motor temperature
2. `PC5/ADC_IN15` throttle
3. `PB0/ADC_IN8` bus voltage
4. `PA5/ADC_IN5` unused torque input
5. ADC channel 16 controller temperature

The active-low brake input is PC4 with a pull-up and four matching 1 kHz
samples. Throttle is plausible at 250--3,750 ADC counts, is at rest through
850, and reaches full demand at 3,252. Boot-high, invalid acquisition, brake,
current trip, Hall loss, or another safety loss disarms the ride path until a
fresh valid rest sample is observed.

The display is not a safety input. Losing CAN traffic does not revoke an
already valid local ride request; every authority decision still comes from
the directly wired inputs and local protection state.

Forward torque is negative q. Demand increases by four current counts per
millisecond and reductions apply immediately. The ride ceiling is 838 phase-
current counts (83.8 A at the loaded-run fit of 100 mA/count), including during
the two-second startup window. Startup requires its first Hall edge within 500
ms and twelve net-forward Hall transitions. The software phase-current guard
trips above 1,344 counts (134.4 A) on A, B, or reconstructed C, preserving at
least the established 1.6-times margin over commanded phase current. The local
39 V undervoltage debounce and controller/motor thermal envelopes can only
reduce the 480-count DC-side limit. This is nominally 48 A at the loaded-run
current fit. The current limiter applies a circular `Id/Iq` command envelope,
then derives the DC-side clamp from a 2 ms low-pass of q-axis modulation using
`Ibus ≈ Iq × modq`; it has no vehicle-speed input. The measured dq vector
and each reconstructed physical phase have independent overcurrent checks. The
1,273-tick voltage-vector ceiling preserves the requested dq direction, and
the 1,103-tick centered PWM window reproduces the reviewed F103 modulation
envelope. The PI controller uses OxiFOC's trapezoidal integrator and coordinated
circular-limit anti-windup. The voltage vector alone is advanced by one
measured electrical control-period step to compensate the ADC-to-PWM pipeline;
current Park transformation remains at the sampled Hall angle.

## CAN and updater

bxCAN runs at 250 kbit/s on PA11/PA12, with the active-low transceiver control
on PA13. The application provides:

- zero-length identity queries `0x210`--`0x212`;
- scheduled stock frames `0x200`--`0x204`, `0x265`, `0x266`, and `0x64A`,
  including local brake, wheel speed/distance, temperatures, and fault pages;
- updater reset on `0x67F#AA552A002A...`;
- commissioning pages 6 and 8--25 on `0x2F7`.

Pages 10 and 11 report live target/measured dq current, ride stage, output and
voltage-limit state, dynamic phase-current limit, applied dq voltage, and PWM
span. Page 12 carries the full internal fault mask and safety-event count.
Page 13 retains peak phase current, direct current, quadrature tracking error,
and PWM span from boot so short transients remain visible at CAN telemetry
rates. Pages 14--17 retain signed dq current and target, Hall state/direction,
edge age and interval, measurement and unlimited Hall angles, phase A/B
current, applied dq voltage, and limiting flags from the exact cycle that set
the maximum |d|. A repeated event generation prevents readers from combining
different peaks. Page 12 also retains the specific cause of the latest safety
loss. Pages 19 and 20 report observer configuration/readiness/activity, blend,
confidence, angle-coordinate eRPM, Hall disagreement, flux magnitude, q-axis
back-EMF, PLL error, and external-validity travel. Page 18 carries telemetry
schema 7 and the crate version. Page 21 reports retained pre-driver phase-path
and driver-step timing maxima. Pages 22 and 23 report the RCC reset cause and,
after a watchdog reset, the retained fatal class, last control checkpoint,
whole-handler timing, cycle number, exception detail, and low address words of
the stacked PC/LR. The complete PC is recoverable because this application is
confined to `0x08003800..0x08009E57`. Pages 24 and 25 preserve the exact PWM
failure predicate, fault and pin state, TIM1 status/output registers, and
attempted compares across a watchdog reset. They occupy the reset-forensics
slots only when a PWM failure was retained; otherwise those slots remain pages
22 and 23. The project-page scheduler advances through all seventeen slots
without a wraparound phase discontinuity.
The 56-byte retained record is linked at `0x20004F00`: above the recovered
bootloader's `0x200000DC..0x20000CF7` zero-fill/stack range and outside the
application's conservative 4 KiB runtime RAM region.

On page 9, byte 1 remains a saturated one-byte current-limit value for older
tools. Byte 3 contains the overflow above 255; new readers add the two bytes to
recover the full effective DC limit (480 when it is not being derated).

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
hardware condition.
