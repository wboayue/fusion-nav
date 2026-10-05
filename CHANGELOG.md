# Changelog

Every published version of `fusion-nav`. What a version number promises is
[GUIDE.md, "Versioning"](https://github.com/wboayue/fusion-nav/blob/main/GUIDE.md#versioning): a
minor bump before 1.0 can break a build, a patch cannot, and a change that moves the numbers the
filter computes is named here in either.

## 0.1.0 (unreleased)

The first published version. MSRV Rust 1.89. Features: `magnetic-model` (default) and `defmt`.

What it is:

- A 15-state error-state Kalman filter, `no_std`, allocation-free, with no reachable panic on
  `thumbv7em-none-eabihf` or `thumbv6m-none-eabi`. It implements
  [EQUATIONS.md](https://github.com/wboayue/fusion-nav/blob/main/EQUATIONS.md) (1)–(52), except
  the three-axis magnetometer, (31)–(33), and in-motion leveling, (5′).
- Fused: GNSS position, height and velocity, dual-antenna heading, course over ground,
  barometric altitude, magnetic heading, the caller's stationary claim, and a position hold
  while unaided. A yaw estimator supplies heading to a vehicle without a heading sensor.
- Each measurement is fused at its own timestamp, gated per source, and a source locked out by
  its gate recovers by adoption, on by default.
- Health per source (`Diagnostics`), per output (`validity`, `predicted_validity`) and as a
  summary (`Status`).

How it was measured is
[VALIDATION.md](https://github.com/wboayue/fusion-nav/blob/main/VALIDATION.md): seeded simulated
flights scored against truth, the PX4 log corpus compared with EKF2, UrbanNav and INSANE against
their truth, and memory, stack and cycle counts on an STM32H743.

Known losses, each stated on the validation pages:

- A long GNSS outage flown through under the position hold leaves position overconfident
  ([#214](https://github.com/wboayue/fusion-nav/issues/214)).
- A sensor whose error persists longer than its configured correlation time leaves position
  overconfident ([#195](https://github.com/wboayue/fusion-nav/issues/195)).
- A start in motion, such as a hand launch, levels wrong without in-motion leveling
  ([#59](https://github.com/wboayue/fusion-nav/issues/59)).
- A receiver persistently wrong while claiming accuracy captures the filter
  ([#181](https://github.com/wboayue/fusion-nav/issues/181)).
- With no heading sensor, GNSS velocity is fused under a yaw nothing has measured until the yaw
  estimator supplies one ([#218](https://github.com/wboayue/fusion-nav/issues/218)).
