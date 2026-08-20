# Changelog

## 0.2.0

- Restored OxiFOC's voltage-pulse fallback contract by calibrating the probe
  stimulus at the first rotor lock position and reusing it unchanged at the
  second. Both applied pulse amplitudes are reported independently so the
  invariant is visible in a real run.
- Added captured-Hall electrical speed during the flux-linkage sample window.
  The host rejects a result when measured speed is absent or differs from the
  commanded speed by more than 15%, preventing rotor slip from silently
  scaling the reported flux.
- Added a checked host-side conversion from raw-state Hall centers to the
  rotation-ordered boundaries consumed by `HallGeometry`. It validates the
  state order and sector widths, preserves the board direction, emits an exact
  Rust literal, and reports every boundary delta from the active ride config.
- Advanced the compact telemetry contract to schema 3. Dead-time is carried at
  1 mV resolution and flux at 0.01 mWb resolution to make room without adding
  CAN pages.

## 0.1.2

- Restricted the calibration image's bxCAN acceptance filters to calibration,
  updater, and identity traffic so unrelated bike frames cannot fill FIFO0.
- Added sticky CAN FIFO-overrun and calibration-command-queue-drop telemetry.
- Retried both arm and routine-start frames for six seconds at 10 Hz, covering
  adapter submission loss and making every energizing command acknowledged.

## 0.1.1

- Made the exact CAN updater command an unconditional recovery path that shuts
  down the bridge before resetting, even during an active routine.
- Exposed Hall-quiet and hardware-output-disabled arm predicates in telemetry.
- Retried arming across the required quiet window and made the host submit STOP
  after every authorized-run error.

## 0.1.0

- Added the separate synchronous STM32F103 calibration image.
- Reused the ride firmware's board support, CAN updater reset, BKIN shutdown,
  watchdogs, reset forensics, and fixed-point OxiFOC current controller.
- Added an explicitly armed two-point resistance sweep that reports the native
  effective voltage-per-current-count slope before applying the nominal sensor
  scale. This avoids embedding the unresolved physical current-scale assumption
  in the primary calibration result.
- Added OxiFOC's discharge-anchored voltage-pulse inductance method with an
  adaptive current excursion, residual dead-time diagnostics, and current-loop
  gains derived in the controller's fixed PWM-tick domain.
- Added OxiFOC's load-angle-independent driven back-EMF-vector flux method,
  including controlled capture/spin/ramp-down and sync-loss detection.
- Added OxiFOC's six-pass forward/reverse Hall-center sweep using Q0.32 circular
  averaging and raw Hall-state-indexed results.
- Added a passive-by-default CAN utility for inspecting or explicitly running
  individual routines or the dependency-ordered full sequence.
