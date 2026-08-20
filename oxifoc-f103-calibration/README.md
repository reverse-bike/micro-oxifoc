# oxifoc-f103-calibration

This is the separate, synchronous calibration application for the recovered
S73 STM32F103 controller. It links the same board drivers, hardware-break path,
watchdogs, reset forensics, current ADC setup, and CAN updater reset used by
`oxifoc-f103`, while replacing the ride state machine with explicitly armed
OxiFOC calibration sequences.

The image implements the useful OxiFOC detection sequence in the fixed-point
units used by this controller:

- a two-point differential phase-resistance sweep at 50 and 250 current counts;
- discharge-anchored, current-budgeted voltage pulses at four electrical lock
  positions and three shared stimulus amplitudes;
- the driven back-EMF-vector flux measurement
  `e = V - R*i - j*omega*L*i`, which is independent of open-loop load angle;
- six alternating forward/reverse electrical Hall sweeps at 80 current counts,
  retaining both directional centers and their circular midpoint by raw Hall
  state.

Resistance is reported primarily as effective microvolts per current-ADC
count, and inductance as `L * current_scale` in nanowebers per count. These are
the coefficients the fixed-point controller actually consumes and do not
pretend that the analog current scale is independently known. The secondary
physical resistance field assumes the nominal 100 mA/count conversion. Flux
linkage does not depend on that scale when the measured effective R/L
coefficients are used consistently.

Resistance must succeed before inductance, and both must succeed before flux.
Results live only in RAM for the current boot. The image reports them over CAN;
it never writes configuration or silently changes the ride firmware.

Calibration is disabled after every reset. It requires:

- valid current offsets and local analog acquisition;
- throttle at rest and brake inactive throughout the run;
- 39--60 V controller bus voltage;
- controller and motor temperatures below the ride derating thresholds;
- no latched hardware fault;
- at least 500 ms without a Hall edge;
- matching per-boot challenge in separate `ARMC` and routine-specific `RUNR`,
  `RUNL`, `RUNF`, or `RUNH` CAN frames.

`STOP` is accepted without an arm challenge. Hardware BKIN remains active and
the calibration software limit is 600 phase-current counts, well below the
observed comparator threshold. An exact updater reset on `0x67F` is an
unconditional recovery path: its CAN interrupt first forces the gate driver and
all TIM1 motor outputs off, then resets immediately from any calibration state.

Build and validate the exact resident-bootloader image from the repository
root:

```sh
command just check-f103-calibration
command just size
command just image-f103-calibration
```

Use `scripts/can_f103_calibration.py` to inspect status or run one routine. The
`full` action performs resistance, inductance, flux, and Hall in dependency
order during one boot:

```sh
command env UV_CACHE_DIR=scratch/uv-cache uv run scripts/can_f103_calibration.py status
command env UV_CACHE_DIR=scratch/uv-cache uv run scripts/can_f103_calibration.py full --yes
```

A real run requires the explicit `--yes` flag. Flux spins the motor to 6,000
electrical RPM, and Hall calibration rotates it through six slow electrical
turns. Elevate the wheel and keep the drivetrain clear. The host retries `ARMC`
and the selected `RUN*` command for up to six seconds while waiting for an
explicit acknowledgment, reports each arm predicate and sticky CAN-loss flag,
and submits `STOP` if any authorized run exits with an error. The calibration
image filters unrelated bike traffic in bxCAN hardware; updater and identity
requests remain accepted.

The schema-4 result reports a twelve-cell pulse grid: four lock positions at
90 electrical-degree intervals, each measured at one-half, one, and
three-halves of the calibrated base stimulus. Every cell includes the actual
pulse ticks, average current rise, and effective inductance. The legacy `Ld`
and `Lq` summary fields remain the base-amplitude results at the first two lock
positions so the driven flux routine can consume their mean; they are not
saliency measurements. Pulse-derived PI gains are intentionally not reported
because the pulse result has not matched the motor's loaded effective
inductance.

The result also includes Hall-derived electrical speed during the flux sample.
The host accepts flux only when Hall speed is within 15% of the 6,000 eRPM
command. After a successful Hall sweep it reports the exact forward and reverse
centers and their signed hysteresis, forms circular midpoints, validates the
raw-state cyclic order and 30--90 electrical-degree sector widths, and prints
an exact `HallGeometry::new(...)` candidate plus boundary deltas from the ride
firmware. The output is advisory: calibration never edits the ride
configuration.
