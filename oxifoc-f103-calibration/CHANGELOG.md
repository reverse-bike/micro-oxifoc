# Changelog

## 0.1.0

- Added the separate synchronous STM32F103 calibration image.
- Reused the ride firmware's board support, CAN updater reset, BKIN shutdown,
  watchdogs, reset forensics, and fixed-point OxiFOC current controller.
- Made the exact CAN updater command an unconditional recovery path that shuts
  down the bridge before resetting, even during an active or stuck routine.
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
