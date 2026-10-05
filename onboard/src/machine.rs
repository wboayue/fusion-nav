//! The one executor: the host runs it to record a trace, the board runs it to time one.

use fusion_nav::prelude::*;

use crate::record::Record;

/// What a trace runs against: the filter, and the window a start folds before it commits.
#[derive(Clone)]
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
    State(State),
    Geodetic(Option<Geodetic>),
}

impl Machine {
    /// Make the call `record` names. Everything between the two cycle-counter reads on the
    /// board is this function, so it does nothing but dispatch: the [`Record::Nop`] arm is the
    /// overhead the board subtracts.
    ///
    /// No call is in tail position. A tail call pops this frame before it jumps, so the callee
    /// runs where this frame was, and the board, which subtracts this frame from every call's
    /// painted stack, would publish that call 160 bytes low: `initialize` and
    /// `initialize_coarse` were, until the result was held past the call. `onboard/build.sh`
    /// refuses an ELF in which this function still jumps out.
    #[inline(never)]
    pub fn execute(&mut self, record: &Record) -> Returned {
        let returned = self.dispatch(record);
        core::hint::black_box(&returned);
        returned
    }

    #[inline(always)]
    fn dispatch(&mut self, record: &Record) -> Returned {
        let filter = &mut self.filter;
        match *record {
            Record::Nop => Returned::Nothing,
            Record::New(ref config) => self.renew(config),
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
            Record::WindowNew => self.new_window(),
            Record::WindowPush(sample) => Returned::Push(self.window.push(sample)),
            Record::Initialize => Returned::Start(filter.initialize(&self.window)),
            Record::InitializeCoarse(imu) => Returned::Start(filter.initialize_coarse(imu)),
            Record::InitializeFrom(..) => self.seed(record),
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
            Record::PredictedValidity | Record::State | Record::GeodeticPosition => {
                self.query(record)
            }
        }
    }

    // The three arms that hold a large value -- a filter, a window, a seed's covariance -- are
    // out of line, so that `execute`'s own frame, which the board measures every call's stack
    // beneath, is the few bytes a call's arguments take rather than a whole `Eskf`.

    #[inline(never)]
    fn renew(&mut self, config: &Config) -> Returned {
        Returned::New(Eskf::new(*config).map(|new| self.filter = new))
    }

    #[inline(never)]
    fn new_window(&mut self) -> Returned {
        self.window = StaticWindow::new();
        Returned::Nothing
    }

    #[inline(never)]
    fn seed(&mut self, record: &Record) -> Returned {
        match *record {
            Record::InitializeFrom(state, covariance, time) => {
                Returned::Start(self.filter.initialize_from(state, covariance, time))
            }
            _ => Returned::Nothing,
        }
    }

    /// The calls that only read, through `&self`: the host makes them where its caller holds a
    /// shared reference. Anything else is [`Returned::Nothing`].
    pub fn query(&self, record: &Record) -> Returned {
        match record {
            Record::PredictedValidity => Returned::Validity(self.filter.predicted_validity()),
            Record::State => Returned::State(self.filter.state()),
            Record::GeodeticPosition => Returned::Geodetic(self.filter.geodetic_position()),
            _ => Returned::Nothing,
        }
    }
}

/// How [`Machine::execute`] reaches a call, which says what lies between its frame and the
/// call's: the board paints beneath `execute`, and the published stack starts beneath this.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Arm {
    /// Called from `execute` itself: its frame alone is above the call.
    Direct,
    /// Called from an out-of-line arm, named as its symbol is: its frame is above the call too.
    Through(&'static str),
    /// Nothing called: the work is `execute`'s own, and touches part of its frame at most, so
    /// no stack is published for it.
    Inline,
}

impl Arm {
    /// The spelling the per-call CSV carries, which `tools/onboard.py` reads.
    pub const fn name(self) -> &'static str {
        match self {
            Arm::Direct => "execute",
            Arm::Through(arm) => arm,
            Arm::Inline => "inline",
        }
    }
}

impl Record {
    /// How [`Machine::execute`] reaches this record's call. Beside `execute` because it
    /// describes that function's shape: an arm added there is a line here, and
    /// `tools/onboard.py` reads every arm's frame off the ELF by this name.
    pub const fn arm(&self) -> Arm {
        match self {
            Record::Nop | Record::SetMagneticDeclination(_) => Arm::Inline,
            Record::New(_) => Arm::Through("renew"),
            Record::WindowNew => Arm::Through("new_window"),
            Record::InitializeFrom(..) => Arm::Through("seed"),
            Record::SetOrigin(_)
            | Record::SetBaroReference(..)
            | Record::ResetPositionTo(..)
            | Record::ResetVelocityTo(..)
            | Record::WindowPush(_)
            | Record::Initialize
            | Record::InitializeCoarse(_)
            | Record::Predict(_)
            | Record::FuseGnssPosition(..)
            | Record::FuseGnssGeodetic(..)
            | Record::FuseGnssVelocity(..)
            | Record::FuseBaroAltitude(..)
            | Record::FuseMagHeading(..)
            | Record::FuseGnssHeading(..)
            | Record::FuseCourse(..)
            | Record::FuseStationary(..)
            | Record::PredictedValidity
            | Record::State
            | Record::GeodeticPosition => Arm::Direct,
        }
    }
}
