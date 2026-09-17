//! Embedded-first inertial navigation using a 15-state Error-State Kalman Filter (ESKF).
//!
//! **Status: design only.** No filter is implemented yet. See `README.md` for the
//! architecture, `EQUATIONS.md` for the mathematics, and `GOALS.md` for positioning.
#![no_std]
#![forbid(unsafe_code)]
#![deny(missing_docs)]
