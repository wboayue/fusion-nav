//! The board's half of #41: make every call in a trace on the ARK FPV's STM32H743, time it with
//! the cycle counter and paint the stack beneath it.
//!
//! ```text
//! onboard/build.sh primary            # build, then flash over DFU and tap NRST
//! uv run tools/onboard.py ...         # stream a trace, collect the figures
//! ```
//!
//! On a host this is an empty `main`, so `cargo clippy` reads it with `--features firmware`.

#![cfg_attr(target_os = "none", no_std, no_main)]

#[cfg(target_os = "none")]
mod firmware;

#[cfg(not(target_os = "none"))]
fn main() {}
