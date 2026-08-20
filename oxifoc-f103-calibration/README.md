# oxifoc-f103-calibration

This is the separate, synchronous calibration application for the recovered
S73 STM32F103 controller. It links the same board drivers, hardware-break path,
watchdogs, reset forensics, current ADC setup, and CAN updater reset used by
`oxifoc-f103`, while replacing the ride state machine with explicitly armed
OxiFOC calibration sequences.

The image implements the useful OxiFOC detection sequence in the fixed-point
units used by this controller:

- a two-point differential phase-resistance sweep at 50 and 250 current counts;
- discharge-anchored, current-budgeted voltage pulses for effective `Ld` and
  `Lq`, followed by PI gains at a documented 1,000 rad/s bandwidth;
- the driven back-EMF-vector flux measurement
  `e = V - R*i - j*omega*L*i`, which is independent of open-loop load angle;
- six alternating forward/reverse electrical Hall sweeps and a circular center
  average indexed by raw Hall state.

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
turns. Elevate the wheel and keep the drivetrain clear.
