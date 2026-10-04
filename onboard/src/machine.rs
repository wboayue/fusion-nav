//! The one executor: the host runs it to record a trace, the board runs it to time one.

use fusion_nav::prelude::*;

use crate::record::{Record, status_code, validity_bits};

/// What a trace runs against: the filter, and the window a start folds before it commits.
pub struct Machine {
    pub filter: Eskf,
    pub window: StaticWindow,
}

impl Default for Machine {
    fn default() -> Self {
        Self {
            filter: Eskf::default(),
            window: StaticWindow::new(),
        }
    }
}

/// What a call returned, typed, so a host caller gets back what the filter gave it.
#[derive(Clone, Copy, Debug)]
pub enum Returned {
    Nothing,
    New(Result<(), ConfigError>),
    Bool(bool),
    Push(Result<(), SampleRefusal>),
    Start(Result<Alignment, InitError>),
    Propagation(Propagation),
    Fusion(Fusion),
    Gnss(GnssFusion),
    Validity(Validity),
}

impl Machine {
    /// Make the call `record` names. Everything between the two cycle-counter reads on the
    /// board is this function, so it does nothing but dispatch: the [`Record::Nop`] arm is the
    /// overhead the board subtracts.
    #[inline(never)]
    pub fn execute(&mut self, record: &Record) -> Returned {
        let filter = &mut self.filter;
        match *record {
            Record::Nop => Returned::Nothing,
            Record::New(config) => Returned::New(Eskf::new(config).map(|new| *filter = new)),
            Record::SetOrigin(origin) => Returned::Bool(filter.set_origin(origin)),
            Record::SetMagneticDeclination(declination) => {
                Returned::Bool(filter.set_magnetic_declination(declination))
            }
            Record::SetBaroReference(reference, noise) => {
                Returned::Bool(filter.set_baro_reference(reference, noise))
            }
            Record::ResetPositionTo(position, noise) => {
                Returned::Bool(filter.reset_position_to(position, noise))
            }
            Record::ResetVelocityTo(velocity, noise) => {
                Returned::Bool(filter.reset_velocity_to(velocity, noise))
            }
            Record::WindowNew => {
                self.window = StaticWindow::new();
                Returned::Nothing
            }
            Record::WindowPush(sample) => Returned::Push(self.window.push(sample)),
            Record::Initialize => Returned::Start(filter.initialize(&self.window)),
            Record::InitializeCoarse(imu) => Returned::Start(filter.initialize_coarse(imu)),
            Record::InitializeFrom(state, covariance, time) => {
                Returned::Start(filter.initialize_from(state, covariance, time))
            }
            Record::Predict(imu) => Returned::Propagation(filter.predict(imu)),
            Record::FuseGnssPosition(time, position, noise, antenna) => {
                Returned::Gnss(filter.fuse_gnss_position(time, position, noise, antenna))
            }
            Record::FuseGnssGeodetic(time, fix, noise, antenna) => {
                Returned::Gnss(filter.fuse_gnss_geodetic(time, fix, noise, antenna))
            }
            Record::FuseGnssVelocity(time, velocity, noise, antenna) => {
                Returned::Fusion(filter.fuse_gnss_velocity(time, velocity, noise, antenna))
            }
            Record::FuseBaroAltitude(time, altitude, noise) => {
                Returned::Fusion(filter.fuse_baro_altitude(time, altitude, noise))
            }
            Record::FuseMagHeading(time, field, noise) => {
                Returned::Fusion(filter.fuse_mag_heading(time, field, noise))
            }
            Record::FuseGnssHeading(time, heading, noise) => {
                Returned::Fusion(filter.fuse_gnss_heading(time, heading, noise))
            }
            Record::FuseCourse(time, sideslip) => {
                Returned::Fusion(filter.fuse_course(time, sideslip))
            }
            Record::FuseStationary(time, noise) => {
                Returned::Fusion(filter.fuse_stationary(time, noise))
            }
            Record::PredictedValidity => Returned::Validity(filter.predicted_validity()),
        }
    }

    /// FNV-1a over the bits of everything the filter reports after a call, and the outcome:
    /// two machines that agree here ran the same arithmetic, bit for bit.
    pub fn digest(&self, outcome: Outcome) -> u64 {
        let mut hash = Fnv::new();
        hash.write(&outcome.0.to_le_bytes());
        let state = self.filter.state();
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
        for row in self.filter.covariance().to_rows() {
            for value in row {
                hash.f32(value);
            }
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

/// A call's outcome in two bytes: its kind, and a detail (a GNSS fix's height verdict, the
/// validity flags). The board returns it beside its timing, and the host compares it with
/// what it recorded.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Outcome(pub u16);

/// The kinds an [`Outcome`] names, with the spelling the per-call CSV carries.
pub const KINDS: [&str; 27] = [
    "nothing",
    "ok",
    "refused",
    "true",
    "false",
    "static",
    "coarse",
    "seeded",
    "propagated",
    "coasted",
    "step_too_long",
    "invalid_step",
    "invalid_interval",
    "not_finite",
    "state_not_finite",
    "not_initialized",
    "accepted",
    "reset",
    "rejected",
    "no_reference",
    "unobservable",
    "invalid_noise",
    "state_invalid",
    "out_of_horizon",
    "validity",
    "init_refused",
    "config_refused",
];

const fn kind(name: &str) -> u8 {
    let mut i = 0;
    while i < KINDS.len() {
        if eq(KINDS[i].as_bytes(), name.as_bytes()) {
            return i as u8;
        }
        i += 1;
    }
    u8::MAX
}

const fn eq(a: &[u8], b: &[u8]) -> bool {
    if a.len() != b.len() {
        return false;
    }
    let mut i = 0;
    while i < a.len() {
        if a[i] != b[i] {
            return false;
        }
        i += 1;
    }
    true
}

const fn propagation(p: Propagation) -> u8 {
    match p {
        Propagation::Propagated => kind("propagated"),
        Propagation::Coasted { .. } => kind("coasted"),
        Propagation::StepTooLong { .. } => kind("step_too_long"),
        Propagation::InvalidStep { .. } => kind("invalid_step"),
        Propagation::InvalidInterval { .. } => kind("invalid_interval"),
        Propagation::NotFinite => kind("not_finite"),
        Propagation::StateNotFinite => kind("state_not_finite"),
        Propagation::NotInitialized => kind("not_initialized"),
    }
}

const fn fusion(f: Fusion) -> u8 {
    match f {
        Fusion::Accepted { .. } => kind("accepted"),
        Fusion::Reset => kind("reset"),
        Fusion::Rejected { .. } => kind("rejected"),
        Fusion::NotInitialized => kind("not_initialized"),
        Fusion::NoReference => kind("no_reference"),
        Fusion::Unobservable => kind("unobservable"),
        Fusion::NotFinite => kind("not_finite"),
        Fusion::InvalidNoise => kind("invalid_noise"),
        Fusion::StateInvalid => kind("state_invalid"),
        Fusion::OutOfHorizon { .. } => kind("out_of_horizon"),
    }
}

impl Outcome {
    pub const fn of(returned: &Returned) -> Self {
        let (kind_, detail) = match *returned {
            Returned::Nothing => (kind("nothing"), 0),
            Returned::New(Ok(())) => (kind("ok"), 0),
            Returned::New(Err(_)) => (kind("config_refused"), 0),
            Returned::Bool(true) => (kind("true"), 0),
            Returned::Bool(false) => (kind("false"), 0),
            Returned::Push(Ok(())) => (kind("ok"), 0),
            Returned::Push(Err(_)) => (kind("refused"), 0),
            Returned::Start(Ok(Alignment::Static)) => (kind("static"), 0),
            Returned::Start(Ok(Alignment::Coarse(_))) => (kind("coarse"), 0),
            Returned::Start(Ok(Alignment::Seeded)) => (kind("seeded"), 0),
            Returned::Start(Err(_)) => (kind("init_refused"), 0),
            Returned::Propagation(p) => (propagation(p), 0),
            Returned::Fusion(f) => (fusion(f), 0),
            Returned::Gnss(GnssFusion { horizontal, height }) => {
                (fusion(horizontal), fusion(height))
            }
            Returned::Validity(v) => (kind("validity"), validity_bits(v)),
        };
        Self(((kind_ as u16) << 8) | detail as u16)
    }

    /// The kind's name, and the detail's where it names a kind too.
    pub fn names(self) -> (&'static str, Option<&'static str>) {
        let [kind_, detail] = self.0.to_be_bytes();
        let name = |k: u8| KINDS.get(usize::from(k)).copied().unwrap_or("unknown");
        let first = name(kind_);
        // A GNSS fix's detail is its height verdict, which is never "nothing" (0): every
        // other outcome leaves the detail zero or, for validity, a bit set.
        let second = (first != "validity" && detail != 0).then(|| name(detail));
        (first, second)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_kind_is_found_by_its_name() {
        for (i, name) in KINDS.iter().enumerate() {
            assert_eq!(usize::from(kind(name)), i);
        }
        assert_eq!(kind("no such kind"), u8::MAX);
    }
}
