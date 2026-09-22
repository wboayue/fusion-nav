//! Observation models: for each source, the innovation `y`, the Jacobian `H` and the noise
//! `R_m` that the update reads. Equations (28)–(36).
//!
//! One file per sensor, each doing no more than forming an `Observation`; the update itself
//! does not know which sensor it is correcting from.

pub(crate) mod gnss;
