# OxiFOC STM32F103

This fork is focused on the recovered S73 STM32F103 motor-controller family.
It keeps OxiFOC's proven control structure while replacing its async and
floating-point platform layers with a synchronous fixed-point implementation
that fits behind the controller's resident CAN bootloader.

## Repository layout

- [oxifoc-core](oxifoc-core/) contains the dependency-free, no_std control
  path: Clarke/Park transforms, PI current control, current and voltage circle
  limits, SVPWM, Hall estimation, and the back-EMF observer.
- [oxifoc-f103](oxifoc-f103/README.md) is the 16 kHz ride firmware. It owns
  direct STM32F103 peripheral access, throttle and wheel inputs, safety policy,
  stock-bike CAN, telemetry, and CAN DFU handoff.
- [oxifoc-f103-calibration](oxifoc-f103-calibration/README.md) is a separate
  explicitly armed image for resistance, pulse-inductance, flux-linkage, and
  Hall-geometry measurement.
- [scripts](scripts/) contains the CAN bootloader and calibration clients plus
  their protocol tests.

Only CAN 2.0B at 250 kbit/s is supported. There is no executor, allocator,
HFI, persistent storage, host GUI, virtual controller, or USB/UART/RTT
communication stack.

## Control architecture

The 16 kHz current interrupt runs entirely in ADC-count, PWM-tick, Q16.16, and
Q0.32-turn units:

    F103 ISR
      -> FocDriver<PhaseManager<HallSensor>>
         -> Hall / back-EMF phase selection
         -> current and DC-bus limiting
         -> FocController
            -> Clarke/Park
            -> d/q PI control
            -> voltage circle
            -> inverse Park
            -> dead-time compensation
            -> SVPWM

The ride and calibration crates share the same board, CAN, watchdog, break,
ADC, PWM, and fixed-point control implementations. Calibration sequencing
stays outside the core because it is application policy, not part of the
per-cycle FOC path.

## Build and validate

The pinned Rust nightly includes the thumbv7m-none-eabi target and LLVM tools.

    just check
    just size
    just image-f103
    just image-f103-calibration

Both application images are linked at 0x08003800 and padded to the exact
26,200-byte application region expected by the resident bootloader.

The flash commands are dry runs unless --yes is passed:

    just flash-f103
    just flash-f103 --yes
    just flash-f103-calibration --yes

Real flashing uses a gs_usb CAN adapter on channel 0 by default. Keep the
drivetrain clear and stationary; the application validates its updater reset
interlocks before handing control to the resident bootloader.

## License

Licensed under either Apache-2.0 or MIT, at your option. See
[LICENSE-APACHE](LICENSE-APACHE) and [LICENSE-MIT](LICENSE-MIT).
