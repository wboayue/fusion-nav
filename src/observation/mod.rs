//! Observation models: for each source, the innovation `y`, the Jacobian `H` and the noise
//! `R_m` that the update reads. Equations (28)–(36), (35′) and (35″), and the constraints
//! (28″) and (29″) that an assumption makes rather than a sensor.
//!
//! One file per sensor, and `hold.rs` for the two assumptions, each doing no more than forming
//! an `Observation`; the update itself does not know which source it is correcting from.

pub(crate) mod baro;
pub(crate) mod gnss;
pub(crate) mod heading;
pub(crate) mod hold;
pub(crate) mod mag;
