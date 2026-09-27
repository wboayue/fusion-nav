//! Propagation and gate outcomes, per-source health, and the aggregate status.
//!
//! See [gate lockout](https://github.com/wboayue/fusion-nav/blob/main/EQUATIONS.md#gate-lockout)
//! for why a source's health is tracked, and what the filter does when one is locked out.

use nalgebra::{SMatrix, SVector};

use crate::display::{Decimals, Fixed};
use crate::units::Seconds;

/// How far the estimate can be trusted: whether attitude has converged, and how well
/// aided it is.
///
/// Payload-free, so [`State`](crate::State) stays `Copy` and cheap to read on the hot
/// path. The detail is in [`Diagnostics`].
///
/// Declared in order of increasing severity. When more than one applies the most severe
/// is reported, so [`Aligning`](Self::Aligning) hides [`Degraded`](Self::Degraded) — an
/// attitude that has not converged is the larger problem — and
/// [`DeadReckoning`](Self::DeadReckoning) hides everything, since nothing is arriving
/// that could align the filter anyway.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Status {
    /// Every source that has ever been fused is still being accepted, and attitude has
    /// converged. The course constraint is not counted: it reads the filter's own velocity
    /// rather than a sensor; see [`Eskf::fuse_course`](crate::Eskf::fuse_course).
    Healthy,
    /// At least one source has timed out, but the estimate is still aided.
    Degraded,
    /// The filter is running and aided, but its attitude has not converged: it was
    /// initialized without a static window, seeded coarsely, or started from a window
    /// with no magnetometer in it, and is still learning.
    ///
    /// Position and velocity are being estimated and are usable to the extent the
    /// covariance says. Attitude is not yet good enough to fly on.
    ///
    /// Two ways out, and neither is a timer or an acknowledgement. Tilt and a widened
    /// yaw leave on their own, as soon as the covariance falls within
    /// [`ALIGNED_TILT`](crate::ALIGNED_TILT) and [`ALIGNED_HEADING`](crate::ALIGNED_HEADING)
    /// — fixed bars, not [`Config::accuracy`](crate::Config::accuracy), which is the
    /// mission's and moves only [`Validity`](crate::Validity). A heading nothing has observed
    /// needs a measurement instead: stillness never supplies yaw, so a filter that
    /// started without a magnetometer stays here however small the covariance is, until a
    /// heading source — [`Eskf::fuse_mag_heading`](crate::Eskf::fuse_mag_heading),
    /// [`fuse_gnss_heading`](crate::Eskf::fuse_gnss_heading) or
    /// [`fuse_course`](crate::Eskf::fuse_course) — accepts one. A vehicle with none of the
    /// three therefore never leaves, and — since this hides
    /// [`Degraded`](Self::Degraded) — its source timeouts stop showing in `Status` and
    /// have to be read from [`Diagnostics`].
    ///
    /// Leaving is permanent, and that is a claim about this variant rather than about the
    /// estimate: it reports a start that has not been resolved yet, not attitude quality
    /// now. Quality now is [`Validity::tilt`](crate::Validity::tilt), which does fall back,
    /// and which an unaided filter loses within seconds of propagating. See
    /// [`Eskf::is_aligned`](crate::Eskf::is_aligned) for what re-entering this state cost on
    /// the corpus.
    Aligning,
    /// Nothing is aiding the filter. Position and velocity error grows without bound.
    #[default]
    DeadReckoning,
}

/// One word or two, as an operator log carries it.
impl core::fmt::Display for Status {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(match self {
            Self::Healthy => "healthy",
            Self::Degraded => "degraded",
            Self::Aligning => "aligning",
            Self::DeadReckoning => "dead reckoning",
        })
    }
}

/// The outcome of one measurement update.
///
/// Carries the dimensionless test ratio `r = ε / γ` of equation (38) in both the accepted
/// and rejected cases, so a rejection is diagnosable rather than a bare failure. `r > 1`
/// means rejected, whatever the dimension of the observation.
///
/// Not `#[must_use]`, unlike [`Propagation`] and the `reset_*` outcomes, because acceptance
/// and rejection reach the caller by a second route: [`Diagnostics`] keeps the latest test
/// ratio, the running counts and the timer for every source, so a loop that reads
/// [`Eskf::state`](crate::Eskf::state) each cycle loses nothing by discarding this value.
///
/// The refusals keep their own counters there rather than a lint here:
/// [`SourceHealth::refused`] and [`SourceHealth::last_refusal`] say how often a source was
/// turned away and why, and [`SourceHealth::adopted`] separates a [`Reset`](Self::Reset) from
/// the ordinary acceptance it otherwise looks like. A refusal still moves no timer — it is not
/// aiding, and pretending otherwise would let a stream of NaN hold off
/// [`Status::DeadReckoning`].
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Fusion {
    /// The measurement passed the gate and was fused.
    Accepted {
        /// Test ratio, in `[0, 1]`.
        test_ratio: f32,
    },
    /// The measurement was adopted outright rather than fused, because the filter had no
    /// estimate of that quantity to fuse it against, or none the gate would let it correct.
    ///
    /// The second is recovery from gate lockout: a source whose measurements have been
    /// rejected for longer than [`Recovery`](crate::Recovery) allows has the next one adopted,
    /// counted in [`SourceHealth::recovered`]. For the barometer the adoption is of its
    /// reference `α₀` rather than of any state, so nothing the estimate reports steps.
    ///
    /// The first happens once per quantity, and for three of them. Position and velocity are
    /// adopted on the first GNSS fix after a coarse start: a vehicle that initialized
    /// while moving knows neither where it is nor how fast it is going, and no gate can
    /// judge a measurement against nothing. A static start and a seed both have an
    /// estimate of those two, so their first fix is fused normally, or, given as latitude
    /// and longitude, places the origin under that estimate; see
    /// [`Eskf::fuse_gnss_geodetic`](crate::Eskf::fuse_gnss_geodetic).
    ///
    /// Heading is the third, and it is not a coarse start's alone: stillness observes
    /// tilt and never yaw, so a static window carrying no magnetometer leaves yaw a
    /// prior nothing measured, and the first heading from
    /// [`fuse_mag_heading`](crate::Eskf::fuse_mag_heading),
    /// [`fuse_gnss_heading`](crate::Eskf::fuse_gnss_heading) or
    /// [`fuse_course`](crate::Eskf::fuse_course) is adopted there too. Only a seed escapes,
    /// having vouched for every quantity.
    ///
    /// The state becomes the measurement and its covariance block becomes the
    /// measurement's, which is what fusing against an infinitely uncertain prior
    /// converges to — the limit, taken exactly rather than approached with an invented
    /// variance.
    ///
    /// Worth reacting to: position, velocity or **attitude** stepped, which a controller
    /// consuming the estimate may care about. Heading is the largest of the three — an
    /// adopted heading can turn the estimate by half a circle, where an adopted position
    /// moves a quantity no controller was flying on.
    Reset,
    /// The measurement failed the gate and was discarded. The state is unchanged.
    Rejected {
        /// Test ratio, greater than 1.
        test_ratio: f32,
    },
    /// No initialization has succeeded — [`Eskf::initialize`](crate::Eskf::initialize),
    /// `initialize_coarse` or `initialize_from` — so there is no state to fuse against. The measurement was discarded.
    NotInitialized,
    /// The measurement has nothing to be relative to. The measurement was discarded.
    ///
    /// Barometer: no reference altitude `α₀` is held, and position is not yet established to
    /// read one against — a start in motion before its first fix — or
    /// [`Config::baro_reference_from_estimate`](crate::Config::baro_reference_from_estimate)
    /// is off. Fusing it anyway would invent the origin from whichever sample happened to
    /// arrive first. [`Eskf::set_baro_reference`](crate::Eskf::set_baro_reference) names one.
    ///
    /// Geodetic GNSS: no navigation origin is held and this fix cannot place one, because
    /// its latitude is beyond ±90°. The next usable fix will. See
    /// [`Eskf::fuse_gnss_geodetic`](crate::Eskf::fuse_gnss_geodetic).
    ///
    /// Course: no GNSS velocity has been accepted within
    /// [`Timeouts::degraded_after`](crate::Timeouts::degraded_after), so the velocity the
    /// course would be taken along is dead-reckoned or was never established. Usually a call
    /// made before the first [`Eskf::fuse_gnss_velocity`](crate::Eskf::fuse_gnss_velocity), or
    /// after the receiver stopped; see [`Eskf::fuse_course`](crate::Eskf::fuse_course).
    NoReference,
    /// The geometry says nothing about the quantity measured. The measurement was discarded
    /// and no health timer moved.
    ///
    /// A heading from [`Eskf::fuse_gnss_heading`](crate::Eskf::fuse_gnss_heading) or
    /// [`Eskf::fuse_course`](crate::Eskf::fuse_course), in two ways. The vehicle's forward axis
    /// is within 30° of vertical, where the heading of that axis is undefined, and a tailsitter
    /// hovering is the case: PX4 refuses a GNSS yaw reset on the same bar
    /// (`EKF/aid_sources/gnss/gnss_yaw_control.cpp:221` at `c4e4ef98`). Or, for the course, the
    /// vehicle is not moving fast enough for the direction of its estimated velocity to mean
    /// anything: a cross-track uncertainty over 15° of course, which is a hover or a slow taxi.
    /// Fusing either would hand the gate an angle drawn from noise.
    ///
    /// Not the magnetometer's: (34) levels the field rather than reading the heading of body
    /// x, and (36′) prices the levelling at any tilt, so a magnetic heading is fused through
    /// 125° of tilt on `285ee2e7`.
    Unobservable,
    /// A number in the measurement or its noise is NaN or infinite. The measurement was
    /// discarded and no health timer moved.
    ///
    /// Refused before anything else, including the adoption of a quantity initialization
    /// left unestablished: one NaN in the state or covariance spreads to every quantity at
    /// the next update and never leaves.
    NotFinite,
    /// A variance in the measurement noise is zero or negative. The measurement was
    /// discarded and no health timer moved.
    ///
    /// `R` has to be a variance some sensor could have. Zero makes the innovation
    /// covariance `S = H P Hᵀ + R` of equation (24) singular as soon as the state it
    /// observes is itself certain, and a negative one is worse: it claims a measurement
    /// better than perfect, and where the measurement is adopted outright it writes that
    /// negative variance straight into `P`, which
    /// [`Validity`] then reads as an excellent estimate.
    ///
    /// Usually a floor or a unit mistake at the boundary — a receiver reporting `eph = 0`
    /// while it has no fix, or a variance arrived at by subtracting one σ² from another.
    /// Refused alongside [`NotFinite`](Self::NotFinite), before any adoption.
    ///
    /// One bad component refuses the measurement it belongs to. A GNSS fix is two, so a
    /// receiver in 2D-fix mode reporting a good `eph` with `epv = 0` keeps its horizontal
    /// aiding and has its height refused; see [`GnssFusion`]. The filter reports rather than
    /// repairs: PX4 and ArduPilot clamp such a value into range, and a caller that wants that
    /// behavior floors `noise` before the call, where the policy is visible. See
    /// [`Eskf::fuse_gnss_position`](crate::Eskf::fuse_gnss_position).
    InvalidNoise,
    /// The filter's own numbers could not support an update, so nothing was committed. The
    /// state and covariance are unchanged, no health timer moved, and the measurement is not
    /// at fault.
    ///
    /// Two ways to get here, one meaning. The innovation covariance `S = H P Hᵀ + R` of
    /// equation (24) was not positive-definite, which with a positive `R` means `P` itself had
    /// stopped being a covariance; or the correction of (26) and (39)–(41) overflowed f32 on
    /// finite inputs, as (11)–(14) can. Either way a caller learns the same thing: the
    /// estimate, not the sensor, is what needs attention.
    ///
    /// `State`-qualified by the rule [`Propagation::StateNotFinite`] follows, and not merged
    /// into it: [`NotFinite`](Self::NotFinite) is always the input handed in, and this is what
    /// the filter produced. Kept out of [`Rejected`](Self::Rejected) because the gate never gave
    /// a verdict — a rejection says the measurement disagreed with a sound estimate, and
    /// counting this as one would put a filter fault in the column that times out a sensor.
    StateInvalid,
    /// The measurement's time is one the filter cannot place it at: older than
    /// [`LATENCY_HORIZON`](crate::LATENCY_HORIZON), later than the state by more than
    /// [`Config::max_predict_dt`](crate::Config::max_predict_dt), or before a start that did not
    /// show the vehicle at rest, whose motion before it nothing describes. The measurement was
    /// discarded and no health timer moved.
    ///
    /// Inside those bounds the measurement is fused at its own time, equation (23′): against
    /// the state as it was, or, for one timed between the last IMU sample and the next, as it
    /// will be, carried forward on the estimated velocity and the last sample's rate.
    ///
    /// Fused anyway, an old measurement is a statement about where the vehicle was, applied to
    /// where it is: at 20 m/s a fix a second old is 20 m of error handed to the gate as truth.
    /// One later than the state is a clock the IMU and the sensor do not share, or a sample
    /// the caller has not yet handed to [`predict`](crate::Eskf::predict). Either way it is the
    /// timestamp that is wrong, which is a refusal rather than a verdict on the value.
    OutOfHorizon {
        /// How long before the state the measurement was taken, negative for one after it:
        /// what tells a latency longer than the horizon from a clock with another epoch.
        age: Seconds,
    },
}

impl Fusion {
    /// Whether the filter took the measurement, whether by fusing it or by adopting it
    /// outright. See [`is_reset`](Self::is_reset) to tell those apart.
    pub const fn is_accepted(self) -> bool {
        matches!(self, Self::Accepted { .. } | Self::Reset)
    }

    /// Whether the measurement replaced the estimate rather than correcting it, stepping
    /// the state.
    pub const fn is_reset(self) -> bool {
        matches!(self, Self::Reset)
    }

    /// Why the measurement was turned away, or `None` if it reached the gate.
    ///
    /// [`Reset`](Self::Reset) is not a refusal: the measurement was taken, just adopted rather
    /// than fused.
    pub const fn refusal(self) -> Option<Refusal> {
        match self {
            Self::NotInitialized => Some(Refusal::NotInitialized),
            Self::NoReference => Some(Refusal::NoReference),
            Self::Unobservable => Some(Refusal::Unobservable),
            Self::NotFinite => Some(Refusal::NotFinite),
            Self::InvalidNoise => Some(Refusal::InvalidNoise),
            Self::StateInvalid => Some(Refusal::StateInvalid),
            Self::OutOfHorizon { .. } => Some(Refusal::OutOfHorizon),
            Self::Accepted { .. } | Self::Rejected { .. } | Self::Reset => None,
        }
    }

    /// The test ratio, where the gate ran at all.
    pub const fn test_ratio(self) -> Option<f32> {
        match self {
            Self::Accepted { test_ratio } | Self::Rejected { test_ratio } => Some(test_ratio),
            // A reset ran no gate: there was nothing to be inconsistent with.
            Self::Reset
            | Self::NotInitialized
            | Self::NoReference
            | Self::Unobservable
            | Self::NotFinite
            | Self::InvalidNoise
            | Self::StateInvalid
            | Self::OutOfHorizon { .. } => None,
        }
    }
}

/// The verdict without the source, which the outcome does not know: `"gnss position {}"`
/// reads `gnss position rejected, ratio 2.70`. A refusal carries its [`Refusal`]'s words,
/// and [`Reset`](Self::Reset) reads `adopted`, which is what it did.
impl core::fmt::Display for Fusion {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match *self {
            Self::Accepted { test_ratio } => write!(f, "accepted, ratio {}", ratio(test_ratio)),
            Self::Reset => f.write_str("adopted"),
            Self::Rejected { test_ratio } => write!(f, "rejected, ratio {}", ratio(test_ratio)),
            Self::NotInitialized => write!(f, "refused, {}", Refusal::NotInitialized),
            Self::NoReference => write!(f, "refused, {}", Refusal::NoReference),
            Self::Unobservable => write!(f, "refused, {}", Refusal::Unobservable),
            Self::NotFinite => write!(f, "refused, {}", Refusal::NotFinite),
            Self::InvalidNoise => write!(f, "refused, {}", Refusal::InvalidNoise),
            Self::StateInvalid => write!(f, "refused, {}", Refusal::StateInvalid),
            Self::OutOfHorizon { age } => write!(
                f,
                "refused, {}, age {} s",
                Refusal::OutOfHorizon,
                seconds(age)
            ),
        }
    }
}

/// A test ratio to two decimals: the gate's bar is 1, so a third digit says nothing an
/// operator acts on.
fn ratio(test_ratio: f32) -> Fixed {
    Fixed::new(test_ratio, Decimals::Two)
}

/// What one GNSS position fix did: its horizontal and vertical halves, each gated and
/// reported on its own.
///
/// A fix carries two measurements that fail independently. A receiver's height wanders
/// further than its horizontal position, and it disagrees with a barometer about height
/// without either saying anything about north or east — so a single verdict over all three
/// axes turns a height disagreement into lost horizontal aiding. `2c42096b` is a stationary
/// vehicle whose barometer and receiver drift ~20 m apart in height over the first hour;
/// fusing both under one joint test rejects 3945 of its 4616 fixes, and every one of them
/// passes a test of the horizontal pair alone. Both production estimators split the same way: PX4 runs GNSS
/// position as a two-dimensional source and GNSS height as a one-dimensional one
/// (`_aid_src_gnss_pos` and `_aid_src_gnss_hgt`, `src/modules/ekf2/EKF/ekf.h:621-622` at
/// `c4e4ef98e9`), and ArduPilot tests the horizontal pair and the height separately
/// (`libraries/AP_NavEKF3/AP_NavEKF3_PosVelFusion.cpp:904-906` and `:1023` at
/// `368dc0c428`).
///
/// Each half is a [`Fusion`] with its own gate in [`Gates`](crate::Gates) and its own
/// [`SourceHealth`] in [`Diagnostics`]: [`gnss_position`](Diagnostics::gnss_position) for
/// the horizontal pair, [`gnss_height`](Diagnostics::gnss_height) for the vertical. The
/// halves can disagree: a number unusable in one half refuses that half alone, so a 2D fix
/// with `epv = 0` reads `horizontal: Accepted` beside `height: InvalidNoise`. What reads the
/// same in both is what concerns the fix as a whole — an adoption, an origin placed, a filter
/// not initialized, and any refusal on those paths, which write all three axes at once.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GnssFusion {
    /// North and east, gated at [`Gates::gnss_position`](crate::Gates::gnss_position).
    pub horizontal: Fusion,
    /// Down, gated at [`Gates::gnss_height`](crate::Gates::gnss_height).
    pub height: Fusion,
}

impl GnssFusion {
    /// The same outcome for both halves: a refusal, an adoption, an origin placed.
    pub(crate) const fn both(fusion: Fusion) -> Self {
        Self {
            horizontal: fusion,
            height: fusion,
        }
    }

    /// Whether the filter took both halves, by fusing or adopting them. See
    /// [`Fusion::is_accepted`].
    pub const fn is_accepted(self) -> bool {
        self.horizontal.is_accepted() && self.height.is_accepted()
    }

    /// Whether the fix replaced the position estimate rather than correcting it. Both halves
    /// or neither: a fix is adopted whole.
    pub const fn is_reset(self) -> bool {
        self.horizontal.is_reset()
    }
}

/// One verdict when the halves agree, as they do for every outcome that concerns the fix
/// whole, and both named when they do not.
impl core::fmt::Display for GnssFusion {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if self.horizontal == self.height {
            write!(f, "{}", self.horizontal)
        } else {
            write!(f, "horizontal {}; height {}", self.horizontal, self.height)
        }
    }
}

/// Why a measurement was turned away before the gate ran.
///
/// The cases [`Fusion`] reports that are not a verdict on the measurement's *value*: the
/// filter could not form an innovation to judge it against, or the numbers offered were not
/// ones any sensor could produce. Kept on [`SourceHealth::last_refusal`] so that a count of
/// refusals says which kind, which is the difference between a miswired sensor and a missing
/// reference.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Refusal {
    /// The filter had not been initialized. See [`Fusion::NotInitialized`].
    NotInitialized,
    /// Nothing to measure against: no barometric reference, no origin the fix could place, or
    /// no held velocity for a course.
    /// See [`Fusion::NoReference`].
    NoReference,
    /// The geometry observed nothing: a forward axis near vertical, or too little speed for a
    /// course. See [`Fusion::Unobservable`].
    Unobservable,
    /// A number in the measurement or its noise was NaN or infinite. See
    /// [`Fusion::NotFinite`].
    NotFinite,
    /// A variance in the measurement noise was zero or negative. See
    /// [`Fusion::InvalidNoise`].
    InvalidNoise,
    /// The filter's own covariance or correction could not support an update. See
    /// [`Fusion::StateInvalid`].
    StateInvalid,
    /// The measurement's time was outside what the filter can place it at. See
    /// [`Fusion::OutOfHorizon`].
    OutOfHorizon,
}

impl core::fmt::Display for Refusal {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(match self {
            Self::NotInitialized => "filter not initialized",
            Self::NoReference => "no reference to measure against",
            Self::Unobservable => "geometry observes nothing",
            Self::NotFinite => "measurement or noise not finite",
            Self::InvalidNoise => "noise variance not positive",
            Self::StateInvalid => "filter state cannot support an update",
            Self::OutOfHorizon => "measurement time outside the horizon",
        })
    }
}

/// The outcome of one propagation step.
///
/// Returned by [`Eskf::predict`](crate::Eskf::predict), which refuses a step it cannot
/// take — a sample that is not a number, or a `dt` longer than
/// [`Config::max_predict_dt`](crate::Config::max_predict_dt) with [`Config::coast`](crate::Config::coast)
/// off — rather than attempting it.
#[must_use = "a refused propagation leaves the state stale unless the outcome is inspected"]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Propagation {
    /// The state and covariance advanced across the sample.
    Propagated,
    /// `dt` exceeded [`Config::max_predict_dt`](crate::Config::max_predict_dt), so the sample
    /// was not integrated and the filter coasted across the gap instead. Equation (22′).
    ///
    /// The state and covariance advanced, but on an assumption rather than a measurement:
    /// position moved on the estimated velocity and nothing else did, while the covariance
    /// grew by what [`Coast`](crate::Coast) allows an unmeasured interval. So
    /// [`is_propagated`](Self::is_propagated) reads false, and a caller logging gaps sees this
    /// one. Counted in [`PropagationHealth::coasted`], not as a refusal.
    Coasted {
        /// The gap coasted across.
        dt: Seconds,
    },
    /// `dt` exceeded [`Config::max_predict_dt`](crate::Config::max_predict_dt) and
    /// [`Config::coast`](crate::Config::coast) is off. The state and covariance are unchanged.
    ///
    /// The per-source timers still advanced: the time really did pass, so the aiding
    /// really is that much staler, and [`Status`] must not claim otherwise.
    StepTooLong {
        /// The `dt` offered.
        dt: Seconds,
        /// The configured limit.
        limit: Seconds,
    },
    /// The sample's [`time`](crate::ImuSample::time) was not after the filter's: `dt` is
    /// zero or negative. Nothing was propagated, no timer advanced and the filter's clock
    /// did not move.
    ///
    /// Rejected before the bookkeeping rather than after: a negative `dt` would run the
    /// per-source timers backwards and make stale aiding look fresh. Zero is included
    /// because two IMU samples sharing a timestamp means duplicated data, which is worth
    /// knowing about even though propagating over it would be harmless.
    InvalidStep {
        /// The time from the filter's clock to the sample's.
        dt: Seconds,
    },
    /// An integration interval in the [`ImuSample`](crate::ImuSample) was under a microsecond,
    /// or longer than [`Config::max_predict_dt`](crate::Config::max_predict_dt), which one
    /// sample does not describe any more than a gap that long. The state and covariance are
    /// unchanged; the timers and the clock advanced, as under [`NotFinite`](Self::NotFinite) and
    /// for its reason.
    ///
    /// Refused because the interval scales what the increment is corrected by and what
    /// (21) adds: a negative one subtracts process noise and lands a variance below zero,
    /// which [`Validity`] reads as an estimate better than any the filter could have. An
    /// interval that is not a number is [`NotFinite`](Self::NotFinite).
    InvalidInterval {
        /// The interval offered.
        interval: Seconds,
    },
    /// A number in the [`ImuSample`](crate::ImuSample) is NaN or infinite. The state and
    /// covariance are unchanged.
    ///
    /// The timers advanced, as under [`StepTooLong`](Self::StepTooLong) and for the same
    /// reason: the `dt` was usable, so the time really did pass and the aiding really is
    /// that much staler. Only the sample was unusable. Holding them back instead would
    /// stop every source timing out, and an IMU emitting NaN would leave [`Status`]
    /// reporting [`Healthy`](Status::Healthy) on aiding that had long since stopped
    /// arriving.
    ///
    /// Refused rather than propagated because a non-finite sample cannot be noticed
    /// afterwards: it reaches `Exp(φ)` of equation (15), whose small-angle test is false
    /// for NaN, so the composition yields a `UnitQuaternion` that is not a unit
    /// quaternion, and it reaches velocity and position through (12)–(14) and `F` through
    /// (16)–(19). One NaN in `P` never leaves.
    ///
    /// ArduPilot refuses non-finite measurements at intake the same way
    /// (`libraries/AP_NavEKF3/AP_NavEKF3_Measurements.cpp:116-119`, at `368dc0c4`). PX4
    /// does not: `EstimatorInterface::setIMUData` constrains the integration period and
    /// never tests `delta_ang` or `delta_vel`
    /// (`src/modules/ekf2/EKF/estimator_interface.cpp:82-110`, at `c4e4ef98`).
    ///
    /// This is the **input**, which is what `NotFinite` means wherever it appears —
    /// [`Fusion::NotFinite`], [`InitError::NotFinite`](crate::InitError::NotFinite),
    /// [`Refusal::NotFinite`]. What propagation *produced* is
    /// [`StateNotFinite`](Self::StateNotFinite).
    NotFinite,
    /// The propagated state carried a NaN or an infinity, so it was not committed. The
    /// state and covariance are unchanged and the timers advanced.
    ///
    /// Distinct from [`NotFinite`](Self::NotFinite), which is the sample arriving: this is a
    /// finite sample overflowing on the way through (11)–(14) or (22). `a_n = R(q̂) a_b + g` and
    /// `F P Fᵀ` are both products of finite numbers, and f32 has a finite range, so a sensor
    /// reporting a plausible-looking `1e38` — or a covariance already wide enough that one more
    /// step leaves the range — is refused here rather than at intake. Either half failing
    /// discards both, since a state committed beside the covariance of a different step reports
    /// an uncertainty that describes something else.
    ///
    /// Refused rather than committed for the reason a NaN sample is: an infinity in the
    /// state reaches the quaternion through (15), the covariance through (16)–(22), and
    /// every estimate after it, with nothing downstream able to tell that it was ever a
    /// number. The previous state is stale, and a stale state the filter admits to is
    /// recoverable where a poisoned one it reports as an estimate is not.
    StateNotFinite,
    /// No initialization has succeeded; see [`Fusion::NotInitialized`]. Nothing was
    /// propagated and no timer advanced.
    NotInitialized,
}

impl Propagation {
    /// Whether the sample was integrated. A [`Coasted`](Self::Coasted) step advanced the state
    /// without it, and reads false.
    pub const fn is_propagated(self) -> bool {
        matches!(self, Self::Propagated)
    }
}

impl core::fmt::Display for Propagation {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match *self {
            Self::Propagated => f.write_str("propagated"),
            Self::Coasted { dt } => write!(f, "gap of {} s coasted", seconds(dt)),
            Self::StepTooLong { dt, limit } => write!(
                f,
                "step of {} s over the {} s limit, not propagated",
                seconds(dt),
                seconds(limit)
            ),
            Self::InvalidStep { dt } => {
                write!(f, "step of {} s not usable, not propagated", seconds(dt))
            }
            Self::InvalidInterval { interval } => write!(
                f,
                "imu interval of {} s not usable, not propagated",
                seconds(interval)
            ),
            Self::NotFinite => f.write_str("imu sample not finite, not propagated"),
            Self::StateNotFinite => f.write_str("propagated state not finite, not committed"),
            Self::NotInitialized => f.write_str("filter not initialized, not propagated"),
        }
    }
}

/// A duration to the millisecond, the resolution an IMU interval is read at.
fn seconds(value: Seconds) -> Fixed {
    Fixed::new(value.as_secs(), Decimals::Three)
}

/// Which parts of the estimate are good enough to use.
///
/// The single [`Status`] answers *how bad is the worst thing*; this answers *which
/// outputs can I use*, which is the question a controller actually has. They are
/// independent axes, and collapsing them loses real information: a filter that started
/// coarse and has adopted a GNSS fix has position as good as the receiver while its
/// attitude is still converging, and `Status::Aligning` alone cannot say so.
///
/// Every flag is derived from the covariance against
/// [`Config::accuracy`](crate::Config::accuracy), plus the requirement that the quantity
/// was ever established at all — a coarse start has no position until a fix arrives, a
/// window with no magnetometer has no heading until a heading source is fused, and a tight prior on a
/// number nobody set is not validity.
///
/// Horizontal and vertical are separate because sources are: a vehicle with a barometer
/// and no GNSS has a usable height and no horizontal position at all, which describes one
/// of the twelve logs in the replay corpus. Another carries neither source, so it has no
/// position of either kind.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Validity {
    /// Tilt: rotation about north and about east, each within
    /// [`Accuracy::tilt`](crate::Accuracy::tilt). Read on navigation axes rather than body
    /// ones, which are tilt only while the vehicle is level; see
    /// [`AttitudeVariance`](crate::AttitudeVariance).
    pub tilt: bool,
    /// Heading. False until something observes the rotation about gravity: a
    /// magnetometer in the initialization window, or an accepted
    /// [`Eskf::fuse_mag_heading`](crate::Eskf::fuse_mag_heading),
    /// [`fuse_gnss_heading`](crate::Eskf::fuse_gnss_heading) or
    /// [`fuse_course`](crate::Eskf::fuse_course). Stillness does not
    /// observe it, so a perfect static alignment on a vehicle with no magnetometer
    /// reports `tilt` and not this.
    pub heading: bool,
    /// North and east position.
    pub horizontal_position: bool,
    /// Down position.
    pub vertical_position: bool,
    /// North and east velocity.
    pub horizontal_velocity: bool,
    /// Down velocity.
    pub vertical_velocity: bool,
}

impl Validity {
    /// Nothing is valid. What an uninitialized filter reports.
    pub const NONE: Self = Self {
        tilt: false,
        heading: false,
        horizontal_position: false,
        vertical_position: false,
        horizontal_velocity: false,
        vertical_velocity: false,
    };

    /// Whether every part of the estimate is usable.
    pub const fn all(self) -> bool {
        self.tilt
            && self.heading
            && self.horizontal_position
            && self.vertical_position
            && self.horizontal_velocity
            && self.vertical_velocity
    }

    /// Whether attitude is usable: the part a stabilizing controller needs before
    /// anything else.
    pub const fn attitude(self) -> bool {
        self.tilt && self.heading
    }

    /// Whether the full navigation solution — position and velocity, both axes — is
    /// usable.
    pub const fn navigation(self) -> bool {
        self.horizontal_position
            && self.vertical_position
            && self.horizontal_velocity
            && self.vertical_velocity
    }
}

/// Six flags in the order the struct declares them, a letter where the quantity is usable and
/// `-` where it is not: `att:T- pos:HV vel:HV` is tilt without heading, position and velocity
/// on both axes. The shape PX4's `*_valid` flags and ArduPilot's `nav_filter_status` bits
/// are read in, and short enough for every line of a 1 Hz log.
impl core::fmt::Display for Validity {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let flag = |valid: bool, letter: &'static str| if valid { letter } else { "-" };
        write!(
            f,
            "att:{}{} pos:{}{} vel:{}{}",
            flag(self.tilt, "T"),
            flag(self.heading, "H"),
            flag(self.horizontal_position, "H"),
            flag(self.vertical_position, "V"),
            flag(self.horizontal_velocity, "H"),
            flag(self.vertical_velocity, "V"),
        )
    }
}

/// The innovation of one update and its variance: `ν` of equation (23) and the diagonal of
/// `S` of equation (24), one entry per component of the observation.
///
/// The diagonal rather than the whole of `S`, which is what PX4 publishes too
/// (`estimator_innovations` and `estimator_innovation_variances`): it is what a per-axis plot
/// of `ν` against `±3√S` reads. The joint `ε = νᵀ S⁻¹ ν` the gate tests is
/// [`test_ratio`](SourceHealth::test_ratio) times the source's threshold in
/// [`Gates`](crate::Gates), so it is not repeated here.
///
/// Room for three components, the largest observation this crate makes; a scalar source fills
/// one.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Innovation {
    nu: [f32; 3],
    variance: [f32; 3],
    dimension: usize,
}

/// The components the observation has, not the three the storage holds.
#[cfg(feature = "defmt")]
impl defmt::Format for Innovation {
    fn format(&self, f: defmt::Formatter<'_>) {
        defmt::write!(
            f,
            "Innovation {{ values: {}, variances: {} }}",
            self.values(),
            self.variances()
        )
    }
}

impl Innovation {
    /// Copy `y` and the diagonal of `S` out of one update.
    pub(crate) fn new<const M: usize>(y: &SVector<f32, M>, s: &SMatrix<f32, M, M>) -> Self {
        let mut innovation = Self {
            dimension: M.min(3),
            ..Self::default()
        };
        for (slot, value) in innovation.nu.iter_mut().zip(y.iter()) {
            *slot = *value;
        }
        for (slot, value) in innovation.variance.iter_mut().zip(s.diagonal().iter()) {
            *slot = *value;
        }
        innovation
    }

    /// `ν = z − h(x̂)`, one entry per component, in the observation's own frame and units.
    pub fn values(&self) -> &[f32] {
        self.nu.get(..self.dimension).unwrap_or_default()
    }

    /// The diagonal of `S = H P Hᵀ + R`, the variance of each entry of
    /// [`values`](Self::values).
    pub fn variances(&self) -> &[f32] {
        self.variance.get(..self.dimension).unwrap_or_default()
    }
}

/// Health of one observation source.
///
/// `#[non_exhaustive]`: read the fields, do not construct one. The set grows as the filter
/// learns to report more, and every addition would otherwise break every caller.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[non_exhaustive]
pub struct SourceHealth {
    /// Test ratio of the most recent measurement **that reached the gate**, or `None` if none
    /// has.
    ///
    /// A refusal leaves it alone: there was no innovation to normalize, so the alternative is
    /// inventing a ratio or discarding the last real one. It can therefore read healthy while
    /// [`refused`](Self::refused) climbs — a receiver that starts reporting `eph = 0` freezes
    /// this at its last good value — so a consumer plotting it reads
    /// [`last_refusal`](Self::last_refusal) beside it.
    pub test_ratio: Option<f32>,
    /// Innovation `ν` and its variance, from the most recent update the gate ran, whether it
    /// accepted or rejected; `None` if the last measurement that set [`test_ratio`] ran no
    /// update, and until one has.
    ///
    /// Published so that a consistency statistic — NIS against the gate, or a comparison with
    /// the innovations PX4 logs — reads the filter's own number rather than recomputing it from
    /// the measurement and the covariance, which would be a second implementation of (23) and
    /// (24) free to disagree with this one. `None` after an acceptance that ran no update: an
    /// adoption ([`Fusion::Reset`]) or a geodetic fix spent placing the origin.
    ///
    /// [`test_ratio`]: Self::test_ratio
    pub innovation: Option<Innovation>,
    /// Time since a measurement from this source was last accepted, or `None` if none
    /// ever has been. Advances with the `dt` passed to
    /// [`Eskf::predict`](crate::Eskf::predict).
    pub time_since_accepted: Option<Seconds>,
    /// Rejections since the last acceptance.
    pub consecutive_rejections: u16,
    /// Measurements accepted over the filter's life.
    pub accepted: u32,
    /// Measurements rejected over the filter's life.
    pub rejected: u32,
    /// Measurements turned away before the gate ran, over the filter's life.
    ///
    /// Distinct from [`rejected`](Self::rejected), which is the gate's verdict on a
    /// measurement it could judge. A refusal means it could not be judged at all, and a
    /// source with a rising count here is misconfigured or miswired rather than noisy: a
    /// source that only ever refuses reads as `never accepted`, exactly like one that was
    /// never connected.
    pub refused: u32,
    /// Why the most recent refusal happened, or `None` if none has.
    pub last_refusal: Option<Refusal>,
    /// Measurements adopted outright rather than fused, over the filter's life.
    ///
    /// For a quantity initialization left unobserved, once, and for every
    /// [`recovered`](Self::recovered) lockout; see [`Fusion::Reset`]. Also counted in
    /// [`accepted`](Self::accepted), because the measurement was taken and the timer restarted
    /// — this is what tells the two apart, since an adoption steps the state and an ordinary
    /// acceptance does not.
    pub adopted: u32,
    /// Adoptions that recovered from gate lockout, over the filter's life: a measurement the
    /// gate rejected after [`Recovery`](crate::Recovery)'s timeout, adopted instead. Also
    /// counted in [`adopted`](Self::adopted).
    ///
    /// Counted apart because it means something the first adoption does not: the covariance
    /// had shrunk around an error it could not see. A source that recovers regularly on good
    /// data is a finding against the covariance, not a working filter.
    pub recovered: u32,
    /// Time since a measurement from this source was last accepted or adopted: the `Δt` of
    /// equation (24′). (24′) discounts a measurement for the error it shares with those
    /// already fused, so only a fused one restarts it; a rejected or refused measurement
    /// changed nothing the next could repeat. Kept per source and restarted at each fused
    /// measurement, rather than read as a difference of `since_initialized`, because an `f32`
    /// clock counting hours loses the digits a 0.2 s interval needs.
    pub(crate) since_measured: Option<Seconds>,
}

impl SourceHealth {
    /// Whether this source has ever been fused.
    pub const fn has_been_used(&self) -> bool {
        self.accepted > 0
    }

    /// Whether a measurement from this source was accepted no more than `timeout` ago.
    /// Never, for a source that has not been accepted at all.
    pub fn accepted_within(&self, timeout: Seconds) -> bool {
        self.time_since_accepted
            .is_some_and(|elapsed| elapsed <= timeout)
    }

    /// Advance the fusion clock. Called from `predict`, since the filter has no clock.
    pub(crate) fn advance(&mut self, dt: Seconds) {
        for clock in [&mut self.time_since_accepted, &mut self.since_measured] {
            if let Some(elapsed) = *clock {
                *clock = Some(Seconds::from_secs(elapsed.as_secs() + dt.as_secs()));
            }
        }
    }

    /// Record a measurement that passed the gate: its test ratio and the innovation behind
    /// it, a restarted fusion clock, and the end of any run of rejections.
    pub(crate) fn record_accepted(&mut self, test_ratio: f32, innovation: Option<Innovation>) {
        self.test_ratio = Some(test_ratio);
        self.innovation = innovation;
        self.time_since_accepted = Some(Seconds::ZERO);
        self.since_measured = Some(Seconds::ZERO);
        self.consecutive_rejections = 0;
        self.accepted = self.accepted.saturating_add(1);
    }

    /// Record a measurement adopted outright. An acceptance for every other purpose, so the
    /// timer restarts with it; see [`Fusion::Reset`].
    pub(crate) fn record_adopted(&mut self) {
        self.record_accepted(0.0, None);
        self.adopted = self.adopted.saturating_add(1);
    }

    /// Record a rejected measurement adopted to recover from lockout.
    pub(crate) fn record_recovered(&mut self) {
        self.record_adopted();
        self.recovered = self.recovered.saturating_add(1);
    }

    /// Whether a measurement the gate has just rejected should be adopted instead: nothing
    /// accepted for at least `after`, counted from initialization for a source never accepted.
    /// `None` is recovery turned off.
    ///
    /// Counted from the last acceptance rather than the first rejection, as PX4 counts from
    /// `time_last_fuse`: a fix arriving after an outage longer than `after` and rejected is a
    /// lockout at once, since nothing constrained the quantity through the outage either.
    pub(crate) fn locked_out(&self, after: Option<Seconds>, since_initialized: Seconds) -> bool {
        after.is_some_and(|after| self.time_since_accepted.unwrap_or(since_initialized) >= after)
    }

    /// Record a measurement the gate refused. The fusion clock keeps running, which is
    /// what lets a source that is only ever rejected time out.
    pub(crate) fn record_rejected(&mut self, test_ratio: f32, innovation: Innovation) {
        self.test_ratio = Some(test_ratio);
        self.innovation = Some(innovation);
        self.consecutive_rejections = self.consecutive_rejections.saturating_add(1);
        self.rejected = self.rejected.saturating_add(1);
    }

    /// Record a measurement turned away before the gate.
    ///
    /// No timer moves and no test ratio is recorded: nothing was measured against the state,
    /// so the source is no fresher than it was. That is what keeps a stream of NaN from
    /// holding off [`Status::DeadReckoning`].
    pub(crate) fn record_refused(&mut self, refusal: Refusal) {
        self.refused = self.refused.saturating_add(1);
        self.last_refusal = Some(refusal);
    }
}

/// What propagation refused or coasted across, and the longest gap.
///
/// Not per source — [`Eskf::predict`](crate::Eskf::predict) is the one path that is not a
/// sensor — and the only record that a step was ever turned away. The state stops advancing
/// while `Status` goes on reporting on the aiding, so without this a filter running on refused
/// steps looks healthy.
///
/// `#[non_exhaustive]`; see [`SourceHealth`].
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[non_exhaustive]
pub struct PropagationHealth {
    /// Gaps longer than [`Config::max_predict_dt`](crate::Config::max_predict_dt) coasted
    /// across, [`Propagation::Coasted`].
    pub coasted: u32,
    /// Steps refused as longer than
    /// [`Config::max_predict_dt`](crate::Config::max_predict_dt), with
    /// [`Config::coast`](crate::Config::coast) off.
    pub refused_too_long: u32,
    /// Steps refused for their timing: a sample not after the last,
    /// [`Propagation::InvalidStep`], or an integration interval under a microsecond or past
    /// [`Config::max_predict_dt`](crate::Config::max_predict_dt),
    /// [`Propagation::InvalidInterval`].
    pub refused_invalid: u32,
    /// Steps refused because the [`ImuSample`](crate::ImuSample) carried a NaN or an
    /// infinity.
    ///
    /// Counted apart from [`refused_invalid`](Self::refused_invalid) because the two name
    /// different faults: that one is the caller's timing, this one is the sensor. A count
    /// climbing here is a miswired or failed IMU, and there is no other record of it —
    /// the state simply stops advancing while the aiding timers run on.
    pub refused_not_finite: u32,
    /// Steps whose propagated state or covariance carried a NaN or an infinity, discarded
    /// together.
    ///
    /// Counted apart from [`refused_not_finite`](Self::refused_not_finite) for the reason
    /// that one is counted apart from [`refused_invalid`](Self::refused_invalid): a
    /// different fault. That one is a sensor emitting a NaN; this one is arithmetic
    /// overflowing on values it accepted, which is a magnitude problem rather than a
    /// validity one and is read at a different place in a bring-up.
    pub refused_state_not_finite: u32,
    /// The longest `dt` past [`Config::max_predict_dt`](crate::Config::max_predict_dt),
    /// coasted or refused, or `None` if there has been none.
    ///
    /// How far past the limit the worst gap ran, which separates a scheduler that overran by a
    /// millisecond from a logger that dropped a second of data. The filter reads no clock, so
    /// it holds the size of the gap and not when it happened; a caller that needs the moment
    /// timestamps it from [`Propagation::Coasted`] or [`Propagation::StepTooLong`].
    pub longest_gap: Option<Seconds>,
}

impl PropagationHealth {
    /// Count what one step did, if it was coasted or refused.
    pub(crate) fn record(&mut self, outcome: Propagation) {
        match outcome {
            // The gap's length is noted by `Eskf::predict`, which sees it whatever the outcome.
            Propagation::Coasted { .. } => {
                self.coasted = self.coasted.saturating_add(1);
            }
            Propagation::StepTooLong { .. } => {
                self.refused_too_long = self.refused_too_long.saturating_add(1);
            }
            Propagation::InvalidStep { .. } | Propagation::InvalidInterval { .. } => {
                self.refused_invalid = self.refused_invalid.saturating_add(1);
            }
            Propagation::NotFinite => {
                self.refused_not_finite = self.refused_not_finite.saturating_add(1);
            }
            Propagation::StateNotFinite => {
                self.refused_state_not_finite = self.refused_state_not_finite.saturating_add(1);
            }
            Propagation::Propagated | Propagation::NotInitialized => {}
        }
    }

    /// Keep `dt` if it is the longest gap past the limit so far.
    pub(crate) fn note_gap(&mut self, dt: Seconds) {
        if self.longest_gap.is_none_or(|worst| dt > worst) {
            self.longest_gap = Some(dt);
        }
    }
}

/// Per-source health and what propagation refused, off the hot path.
///
/// Returned by [`Eskf::diagnostics`](crate::Eskf::diagnostics).
///
/// Every count here describes the filter's life since it last initialized, not since it was
/// constructed: committing a window resets these. Measurements offered before then are refused
/// with [`Fusion::NotInitialized`] and counted, and that count goes with the reset — which is
/// what makes a refusal count afterwards a statement about the flight rather than about the
/// caller's startup order.
///
/// `#[non_exhaustive]`; see [`SourceHealth`].
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[non_exhaustive]
pub struct Diagnostics {
    /// GNSS position updates, horizontal: north and east. See [`GnssFusion`].
    pub gnss_position: SourceHealth,
    /// GNSS position updates, vertical: the height half of the same fix.
    pub gnss_height: SourceHealth,
    /// GNSS velocity updates.
    pub gnss_velocity: SourceHealth,
    /// Barometric altitude updates.
    pub baro_altitude: SourceHealth,
    /// Magnetic heading updates.
    pub mag_heading: SourceHealth,
    /// Dual-antenna GNSS heading updates.
    pub gnss_heading: SourceHealth,
    /// Course constraint updates: heading along the estimated velocity.
    pub course: SourceHealth,
    /// What [`Eskf::predict`](crate::Eskf::predict) refused. Not a source, so not in
    /// [`sources`](Self::sources).
    pub propagation: PropagationHealth,
    /// Variances raised to the floor of equation (42′), counted per entry rather than per
    /// covariance.
    ///
    /// Expected to stay at zero, and it does across the whole corpus — the floor is set well
    /// below anything the filter reaches there, and `math.rs`'s `FLOOR` is where that margin
    /// is measured and recorded. So a count climbing here says a variance is collapsing for a
    /// reason of its own — an `R` far smaller than what the measurement actually observes, or
    /// a source fused faster than it carries independent information — and that the floor is
    /// masking it rather than protecting against it. It is the one number here whose
    /// interesting value is the one it does not have.
    ///
    /// Not counted per state, which would say *which* variance collapsed: a count that
    /// should be zero needs only to be non-zero to be worth reading, and the `sigma_*`
    /// columns of `examples/replay.rs` name the state as soon as anybody looks.
    pub floored: u32,
    /// Time since initialization, the clock a source never accepted is locked out on.
    pub(crate) since_initialized: Seconds,
}

impl Diagnostics {
    /// Every source, for iteration.
    pub const fn sources(&self) -> [(&'static str, SourceHealth); 7] {
        [
            ("gnss_position", self.gnss_position),
            ("gnss_height", self.gnss_height),
            ("gnss_velocity", self.gnss_velocity),
            ("baro_altitude", self.baro_altitude),
            ("mag_heading", self.mag_heading),
            ("gnss_heading", self.gnss_heading),
            ("course", self.course),
        ]
    }

    /// The sources [`Status`] counts: every one but the course constraint, which reads the
    /// filter's own velocity rather than a sensor. See
    /// [`Eskf::fuse_course`](crate::Eskf::fuse_course).
    ///
    /// Taken from [`sources`](Self::sources) rather than listed again, so a source added there
    /// counts toward `Status` unless it is excluded here by name.
    pub(crate) fn aiding(&self) -> impl Iterator<Item = SourceHealth> {
        self.sources()
            .into_iter()
            .filter(|&(name, _)| name != "course")
            .map(|(_, health)| health)
    }

    /// Advance every source's fusion clock.
    pub(crate) fn advance(&mut self, dt: Seconds) {
        self.since_initialized =
            Seconds::from_secs(self.since_initialized.as_secs() + dt.as_secs());
        self.gnss_position.advance(dt);
        self.gnss_height.advance(dt);
        self.gnss_velocity.advance(dt);
        self.baro_altitude.advance(dt);
        self.mag_heading.advance(dt);
        self.gnss_heading.advance(dt);
        self.course.advance(dt);
    }
}

#[cfg(test)]
mod tests {
    use std::format;

    use super::*;

    const fn secs(s: f32) -> Seconds {
        Seconds::from_secs(s)
    }

    #[test]
    fn an_outcome_reads_as_an_operator_log_line() {
        assert_eq!(format!("{}", Status::DeadReckoning), "dead reckoning");
        assert_eq!(
            format!("{}", Fusion::Rejected { test_ratio: 2.7 }),
            "rejected, ratio 2.70"
        );
        assert_eq!(
            format!("{}", Fusion::Accepted { test_ratio: 0.314 }),
            "accepted, ratio 0.31"
        );
        assert_eq!(format!("{}", Fusion::Reset), "adopted");
        assert_eq!(
            format!("{}", Fusion::NoReference),
            "refused, no reference to measure against"
        );
        assert_eq!(
            format!("{}", Fusion::Unobservable),
            "refused, geometry observes nothing"
        );
        assert_eq!(
            format!(
                "{}",
                Propagation::StepTooLong {
                    dt: secs(0.15),
                    limit: secs(0.1)
                }
            ),
            "step of 0.150 s over the 0.100 s limit, not propagated"
        );
        assert_eq!(
            format!("{}", Propagation::Coasted { dt: secs(1.2) }),
            "gap of 1.200 s coasted"
        );
    }

    #[test]
    fn status_counts_every_source_but_the_course() {
        // `aiding` excludes the course by name, so a misspelling would count it silently.
        let diagnostics = Diagnostics::default();
        assert_eq!(
            diagnostics.aiding().count(),
            diagnostics.sources().len() - 1
        );
        assert!(
            diagnostics
                .sources()
                .iter()
                .any(|&(name, _)| name == "course")
        );
    }

    #[test]
    fn a_fix_names_its_halves_only_when_they_disagree() {
        assert_eq!(format!("{}", GnssFusion::both(Fusion::Reset)), "adopted");
        let split = GnssFusion {
            horizontal: Fusion::Accepted { test_ratio: 0.5 },
            height: Fusion::InvalidNoise,
        };
        assert_eq!(
            format!("{split}"),
            "horizontal accepted, ratio 0.50; height refused, noise variance not positive"
        );
    }

    #[test]
    fn validity_is_a_letter_per_usable_quantity() {
        assert_eq!(format!("{}", Validity::NONE), "att:-- pos:-- vel:--");
        let tilt_and_navigation = Validity {
            tilt: true,
            heading: false,
            horizontal_position: true,
            vertical_position: true,
            horizontal_velocity: true,
            vertical_velocity: true,
        };
        assert_eq!(format!("{tilt_and_navigation}"), "att:T- pos:HV vel:HV");
    }

    #[test]
    fn lockout_counts_from_the_last_acceptance_or_from_initialization() {
        let mut source = SourceHealth::default();
        // Never accepted: the filter's own clock is what has run.
        assert!(!source.locked_out(Some(secs(7.0)), secs(6.9)));
        assert!(source.locked_out(Some(secs(7.0)), secs(7.0)));

        source.record_accepted(0.5, None);
        source.advance(secs(6.9));
        assert!(
            !source.locked_out(Some(secs(7.0)), secs(100.0)),
            "since the acceptance"
        );
        source.advance(secs(0.1));
        assert!(source.locked_out(Some(secs(7.0)), secs(100.0)));
        assert!(!source.locked_out(None, secs(100.0)), "off is off");
    }
}
