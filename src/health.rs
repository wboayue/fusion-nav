//! Propagation and gate outcomes, per-source health, and the aggregate status.
//!
//! See [gate lockout](https://github.com/wboayue/fusion-nav/blob/main/EQUATIONS.md#gate-lockout)
//! for why a source's health is tracked, and what the filter does when one is locked out.

use nalgebra::{SMatrix, SVector};

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
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Status {
    /// Every source that has ever been fused is still being accepted, and attitude has
    /// converged.
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
    /// started without a magnetometer stays here however small the covariance is, until
    /// [`Eskf::fuse_mag_heading`](crate::Eskf::fuse_mag_heading) accepts one. A vehicle
    /// carrying no magnetometer at all therefore never leaves, and — since this hides
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
    /// prior nothing measured, and the first
    /// [`fuse_mag_heading`](crate::Eskf::fuse_mag_heading) is adopted there too. Only a
    /// seed escapes, having vouched for every quantity.
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
    NoReference,
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
            Self::NotFinite => Some(Refusal::NotFinite),
            Self::InvalidNoise => Some(Refusal::InvalidNoise),
            Self::StateInvalid => Some(Refusal::StateInvalid),
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
            | Self::NotFinite
            | Self::InvalidNoise
            | Self::StateInvalid => None,
        }
    }
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

/// Why a measurement was turned away before the gate ran.
///
/// The four cases [`Fusion`] reports that are not a verdict on the measurement's *value*: the
/// filter could not form an innovation to judge it against, or the numbers offered were not
/// ones any sensor could produce. Kept on [`SourceHealth::last_refusal`] so that a count of
/// refusals says which kind, which is the difference between a miswired sensor and a missing
/// reference.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Refusal {
    /// The filter had not been initialized. See [`Fusion::NotInitialized`].
    NotInitialized,
    /// Nothing to measure against: no barometric reference, or no origin the fix could place.
    /// See [`Fusion::NoReference`].
    NoReference,
    /// A number in the measurement or its noise was NaN or infinite. See
    /// [`Fusion::NotFinite`].
    NotFinite,
    /// A variance in the measurement noise was zero or negative. See
    /// [`Fusion::InvalidNoise`].
    InvalidNoise,
    /// The filter's own covariance or correction could not support an update. See
    /// [`Fusion::StateInvalid`].
    StateInvalid,
}

/// The outcome of one propagation step.
///
/// Returned by [`Eskf::predict`](crate::Eskf::predict), which refuses a step it cannot
/// take — a `dt` longer than [`Config::max_predict_dt`](crate::Config::max_predict_dt),
/// or a sample that is not a number — rather than attempting it.
#[must_use = "a refused propagation leaves the state stale unless the outcome is inspected"]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Propagation {
    /// The state and covariance advanced over the full `dt`.
    Propagated,
    /// `dt` exceeded [`Config::max_predict_dt`](crate::Config::max_predict_dt). The state
    /// and covariance are unchanged.
    ///
    /// The per-source timers still advanced: the time really did pass, so the aiding
    /// really is that much staler, and [`Status`] must not claim otherwise.
    StepTooLong {
        /// The `dt` offered.
        dt: Seconds,
        /// The configured limit.
        limit: Seconds,
    },
    /// `dt` was zero, negative, or not a number. Nothing was propagated and no timer
    /// advanced.
    ///
    /// Rejected before the bookkeeping rather than after: a negative `dt` would run the
    /// per-source timers backwards and make stale aiding look fresh, and a NaN would
    /// poison them for the rest of the flight. Zero is included because two IMU samples
    /// sharing a timestamp means duplicated data, which is worth knowing about even
    /// though propagating over it would be harmless.
    InvalidStep {
        /// The `dt` offered.
        dt: Seconds,
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
    /// Whether the state advanced.
    pub const fn is_propagated(self) -> bool {
        matches!(self, Self::Propagated)
    }
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
/// window with no magnetometer has no heading until one is fused, and a tight prior on a
/// number nobody set is not validity.
///
/// Horizontal and vertical are separate because sources are: a vehicle with a barometer
/// and no GNSS has a usable height and no horizontal position at all, which describes one
/// of the twelve logs in the replay corpus. Another carries neither source, so it has no
/// position of either kind.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Validity {
    /// Tilt: rotation about north and about east, each within
    /// [`Accuracy::tilt`](crate::Accuracy::tilt). Read on navigation axes rather than body
    /// ones, which are tilt only while the vehicle is level; see
    /// [`AttitudeVariance`](crate::AttitudeVariance).
    pub tilt: bool,
    /// Heading. False until something observes the rotation about gravity: a
    /// magnetometer in the initialization window, or an accepted
    /// [`Eskf::fuse_mag_heading`](crate::Eskf::fuse_mag_heading). Stillness does not
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
    /// Time since a usable measurement from this source last arrived, whatever the gate
    /// made of it: the `Δt` of equation (28′). Kept per source and reset at each
    /// measurement, rather than read as a difference of `since_initialized`, because an
    /// `f32` clock counting hours loses the digits a 0.2 s interval needs.
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

    /// Record that a usable measurement arrived, restarting the interval (28′) reads.
    pub(crate) fn record_measured(&mut self) {
        self.since_measured = Some(Seconds::ZERO);
    }

    /// Record a measurement that passed the gate: its test ratio and the innovation behind
    /// it, a restarted fusion clock, and the end of any run of rejections.
    pub(crate) fn record_accepted(&mut self, test_ratio: f32, innovation: Option<Innovation>) {
        self.test_ratio = Some(test_ratio);
        self.innovation = innovation;
        self.time_since_accepted = Some(Seconds::ZERO);
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

/// What propagation refused, and the worst of it.
///
/// Not per source — [`Eskf::predict`](crate::Eskf::predict) is the one path that is not a
/// sensor — and the only record that a step was ever turned away. The state stops advancing
/// while `Status` goes on reporting on the aiding, so without this a filter running on refused
/// steps looks healthy.
///
/// `#[non_exhaustive]`; see [`SourceHealth`].
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[non_exhaustive]
pub struct PropagationHealth {
    /// Steps refused as longer than
    /// [`Config::max_predict_dt`](crate::Config::max_predict_dt).
    pub refused_too_long: u32,
    /// Steps refused as zero, negative, or not a number.
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
    /// The longest `dt` refused as too long, or `None` if none has been.
    ///
    /// How far past the limit the worst gap ran, which separates a scheduler that overran by a
    /// millisecond from a logger that dropped a second of data. The filter reads no clock, so
    /// it holds the size of the gap and not when it happened; a caller that needs the moment
    /// timestamps it from [`Propagation::StepTooLong`].
    pub longest_refused: Option<Seconds>,
}

impl PropagationHealth {
    /// Record what one step did, if it was refused.
    pub(crate) fn record(&mut self, outcome: Propagation) {
        match outcome {
            Propagation::StepTooLong { dt, .. } => {
                self.refused_too_long = self.refused_too_long.saturating_add(1);
                if self.longest_refused.is_none_or(|worst| dt > worst) {
                    self.longest_refused = Some(dt);
                }
            }
            Propagation::InvalidStep { .. } => {
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
    pub const fn sources(&self) -> [(&'static str, SourceHealth); 5] {
        [
            ("gnss_position", self.gnss_position),
            ("gnss_height", self.gnss_height),
            ("gnss_velocity", self.gnss_velocity),
            ("baro_altitude", self.baro_altitude),
            ("mag_heading", self.mag_heading),
        ]
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
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const fn secs(s: f32) -> Seconds {
        Seconds::from_secs(s)
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
