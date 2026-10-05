//! What a call returned, in the two bytes the board reports, and the digest of what it left
//! behind: the two things host and board compare, call by call.

use fusion_nav::prelude::*;

use crate::machine::{Machine, Returned};

/// The kinds an [`Outcome`] names. An enum rather than strings looked up, so a kind that does
/// not exist does not compile.
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    Nothing,
    Ok,
    Refused,
    True,
    False,
    Static,
    Coarse,
    Seeded,
    Propagated,
    Coasted,
    StepTooLong,
    InvalidStep,
    InvalidInterval,
    NotFinite,
    StateNotFinite,
    NotInitialized,
    Accepted,
    Reset,
    Rejected,
    NoReference,
    Unobservable,
    InvalidNoise,
    StateInvalid,
    OutOfHorizon,
    Validity,
    InitRefused,
    ConfigRefused,
    State,
    Present,
    Absent,
}

/// Every kind, in discriminant order, which a test holds the enum to.
pub const KINDS: [Kind; 30] = [
    Kind::Nothing,
    Kind::Ok,
    Kind::Refused,
    Kind::True,
    Kind::False,
    Kind::Static,
    Kind::Coarse,
    Kind::Seeded,
    Kind::Propagated,
    Kind::Coasted,
    Kind::StepTooLong,
    Kind::InvalidStep,
    Kind::InvalidInterval,
    Kind::NotFinite,
    Kind::StateNotFinite,
    Kind::NotInitialized,
    Kind::Accepted,
    Kind::Reset,
    Kind::Rejected,
    Kind::NoReference,
    Kind::Unobservable,
    Kind::InvalidNoise,
    Kind::StateInvalid,
    Kind::OutOfHorizon,
    Kind::Validity,
    Kind::InitRefused,
    Kind::ConfigRefused,
    Kind::State,
    Kind::Present,
    Kind::Absent,
];

impl Kind {
    /// The spelling the per-call CSV and the pinned keys carry.
    pub const fn name(self) -> &'static str {
        match self {
            Kind::Nothing => "nothing",
            Kind::Ok => "ok",
            Kind::Refused => "refused",
            Kind::True => "true",
            Kind::False => "false",
            Kind::Static => "static",
            Kind::Coarse => "coarse",
            Kind::Seeded => "seeded",
            Kind::Propagated => "propagated",
            Kind::Coasted => "coasted",
            Kind::StepTooLong => "step_too_long",
            Kind::InvalidStep => "invalid_step",
            Kind::InvalidInterval => "invalid_interval",
            Kind::NotFinite => "not_finite",
            Kind::StateNotFinite => "state_not_finite",
            Kind::NotInitialized => "not_initialized",
            Kind::Accepted => "accepted",
            Kind::Reset => "reset",
            Kind::Rejected => "rejected",
            Kind::NoReference => "no_reference",
            Kind::Unobservable => "unobservable",
            Kind::InvalidNoise => "invalid_noise",
            Kind::StateInvalid => "state_invalid",
            Kind::OutOfHorizon => "out_of_horizon",
            Kind::Validity => "validity",
            Kind::InitRefused => "init_refused",
            Kind::ConfigRefused => "config_refused",
            Kind::State => "state",
            Kind::Present => "present",
            Kind::Absent => "absent",
        }
    }

    const fn of_propagation(p: Propagation) -> Self {
        match p {
            Propagation::Propagated => Kind::Propagated,
            Propagation::Coasted { .. } => Kind::Coasted,
            Propagation::StepTooLong { .. } => Kind::StepTooLong,
            Propagation::InvalidStep { .. } => Kind::InvalidStep,
            Propagation::InvalidInterval { .. } => Kind::InvalidInterval,
            Propagation::NotFinite => Kind::NotFinite,
            Propagation::StateNotFinite => Kind::StateNotFinite,
            Propagation::NotInitialized => Kind::NotInitialized,
        }
    }

    const fn of_fusion(f: Fusion) -> Self {
        match f {
            Fusion::Accepted { .. } => Kind::Accepted,
            Fusion::Reset => Kind::Reset,
            Fusion::Rejected { .. } => Kind::Rejected,
            Fusion::NotInitialized => Kind::NotInitialized,
            Fusion::NoReference => Kind::NoReference,
            Fusion::Unobservable => Kind::Unobservable,
            Fusion::NotFinite => Kind::NotFinite,
            Fusion::InvalidNoise => Kind::InvalidNoise,
            Fusion::StateInvalid => Kind::StateInvalid,
            Fusion::OutOfHorizon { .. } => Kind::OutOfHorizon,
        }
    }
}

/// A call's outcome in two bytes: its [`Kind`], and a detail (a GNSS fix's height verdict, the
/// validity flags, a state's status). The board returns it beside its timing, and the host
/// compares it with what it recorded.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Outcome(pub u16);

impl Outcome {
    pub const fn of(returned: &Returned) -> Self {
        let (kind, detail) = match *returned {
            Returned::Nothing => (Kind::Nothing, 0),
            Returned::New(Ok(())) => (Kind::Ok, 0),
            Returned::New(Err(_)) => (Kind::ConfigRefused, 0),
            Returned::Bool(true) => (Kind::True, 0),
            Returned::Bool(false) => (Kind::False, 0),
            Returned::Push(Ok(())) => (Kind::Ok, 0),
            Returned::Push(Err(_)) => (Kind::Refused, 0),
            Returned::Start(Ok(Alignment::Static)) => (Kind::Static, 0),
            Returned::Start(Ok(Alignment::Coarse(_))) => (Kind::Coarse, 0),
            Returned::Start(Ok(Alignment::Seeded)) => (Kind::Seeded, 0),
            Returned::Start(Err(_)) => (Kind::InitRefused, 0),
            Returned::Propagation(p) => (Kind::of_propagation(p), 0),
            Returned::Fusion(f) => (Kind::of_fusion(f), 0),
            Returned::Gnss(GnssFusion { horizontal, height }) => {
                (Kind::of_fusion(horizontal), Kind::of_fusion(height) as u8)
            }
            Returned::Validity(v) => (Kind::Validity, validity_bits(v)),
            Returned::State(state) => (Kind::State, status_code(state.status)),
            Returned::Geodetic(Some(_)) => (Kind::Present, 0),
            Returned::Geodetic(None) => (Kind::Absent, 0),
        };
        Self(((kind as u16) << 8) | detail as u16)
    }

    /// The kind's name, and the height verdict's where the detail is one.
    pub fn names(self) -> (&'static str, Option<&'static str>) {
        let [kind, detail] = self.0.to_be_bytes();
        let name = |k: u8| KINDS.get(usize::from(k)).map_or("unknown", |k| k.name());
        let first = name(kind);
        // A GNSS fix's detail is its height verdict, never "nothing" (0); every other outcome
        // leaves the detail zero or holds bits there that are not a kind.
        let carries_kind = !matches!(first, "validity" | "state") && detail != 0;
        (first, carries_kind.then(|| name(detail)))
    }
}

pub(crate) const fn status_code(status: Status) -> u8 {
    match status {
        Status::Healthy => 0,
        Status::Degraded => 1,
        Status::DeadReckoning => 2,
        Status::Aligning => 3,
    }
}

pub(crate) const fn validity_bits(validity: Validity) -> u8 {
    let Validity {
        tilt,
        heading,
        horizontal_position,
        vertical_position,
        horizontal_velocity,
        vertical_velocity,
    } = validity;
    (tilt as u8)
        | (heading as u8) << 1
        | (horizontal_position as u8) << 2
        | (vertical_position as u8) << 3
        | (horizontal_velocity as u8) << 4
        | (vertical_velocity as u8) << 5
}

impl Machine {
    /// FNV-1a over the bits of everything the filter reports after a call, and the outcome:
    /// two machines that agree here ran the same arithmetic, bit for bit. The clock, the
    /// barometric reference, the declination and every source's counters are in it as well as
    /// the state and covariance, so a divergence in what only a later call reads is caught at
    /// the call that made it.
    pub fn digest(&self, outcome: Outcome) -> u64 {
        let filter = &self.filter;
        let mut hash = Fnv::new();
        hash.write(&outcome.0.to_le_bytes());
        let state = filter.state();
        let q = state.attitude.body_to_ned();
        for value in [q.w, q.x, q.y, q.z] {
            hash.f32(value);
        }
        for vector in [
            state.position.to_array(),
            state.velocity.to_array(),
            state.accel_bias.to_array(),
            state.gyro_bias.to_array(),
        ] {
            for value in vector {
                hash.f32(value);
            }
        }
        hash.write(&[status_code(state.status), validity_bits(state.validity)]);
        for row in filter.covariance().to_rows() {
            for value in row {
                hash.f32(value);
            }
        }
        hash.write(
            &filter
                .time()
                .map_or(u64::MAX, Timestamp::as_micros)
                .to_le_bytes(),
        );
        hash.f32(
            filter
                .baro_reference()
                .map_or(f32::NAN, Altitude::as_meters),
        );
        hash.f32(filter.magnetic_declination().as_radians());
        let diagnostics = filter.diagnostics();
        for (_, source) in diagnostics.sources() {
            for count in [
                source.accepted,
                source.rejected,
                source.refused,
                source.adopted,
                source.recovered,
                u32::from(source.consecutive_rejections),
            ] {
                hash.write(&count.to_le_bytes());
            }
            hash.f32(
                source
                    .time_since_accepted
                    .map_or(f32::NAN, Seconds::as_secs),
            );
            hash.f32(source.test_ratio.unwrap_or(f32::NAN));
        }
        let propagation = &diagnostics.propagation;
        for count in [
            propagation.coasted,
            propagation.refused_too_long,
            propagation.refused_invalid,
            propagation.refused_not_finite,
            propagation.refused_state_not_finite,
            diagnostics.floored,
        ] {
            hash.write(&count.to_le_bytes());
        }
        hash.0
    }
}

struct Fnv(u64);

impl Fnv {
    const fn new() -> Self {
        Self(0xcbf2_9ce4_8422_2325)
    }

    fn write(&mut self, bytes: &[u8]) {
        for &byte in bytes {
            self.0 = (self.0 ^ u64::from(byte)).wrapping_mul(0x0000_0100_0000_01b3);
        }
    }

    fn f32(&mut self, value: f32) {
        self.write(&value.to_bits().to_le_bytes());
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_kind_sits_at_its_discriminant_and_names_itself_once() {
        for (i, kind) in KINDS.iter().enumerate() {
            assert_eq!(*kind as usize, i, "{kind:?}");
        }
        let mut names: Vec<_> = KINDS.iter().map(|k| k.name()).collect();
        names.sort_unstable();
        names.dedup();
        assert_eq!(names.len(), KINDS.len());
    }
}
