//! The filter itself: [`Eskf`], the fields every method shares, and the invariants they keep.
//!
//! Every covariance the filter commits passes through [`Eskf::commit_covariance`], the floor of
//! (42′), and every state through [`Estimate`]. The methods are split by topic into child
//! modules, each an `impl Eskf` block, in the order a flight calls them: `start`, `site`,
//! `predict`, `fuse`, `heading`, `adopt`, and `validity`, which with
//! [`derive_status`](Eskf::derive_status) here answers what the estimate can claim.

use crate::config::{Config, ConfigError};
use crate::frames::Body;
use crate::geodetic::LocalOrigin;
use crate::health::{Diagnostics, Status};
use crate::math::{floor_diagonal, floor_offset};
use crate::state::{Covariance, Offset, State};
use crate::units::{Altitude, AngularRate, Radians, Timestamp};

mod adopt;
mod estimate;
mod fuse;
mod heading;
mod predict;
mod site;
mod start;
mod validity;

use estimate::Estimate;

/// A 15-state error-state Kalman filter.
///
/// Every equation the filter needs is implemented: initialization, (5)–(8), propagation,
/// (9)–(22), and the update of (23)–(41) for every source. The state dead reckons from
/// where [`initialize`](Self::initialize) put it, the covariance grows around it, and
/// [`fuse_gnss_position`](Self::fuse_gnss_position),
/// [`fuse_gnss_geodetic`](Self::fuse_gnss_geodetic),
/// [`fuse_gnss_velocity`](Self::fuse_gnss_velocity),
/// [`fuse_baro_altitude`](Self::fuse_baro_altitude),
/// [`fuse_mag_heading`](Self::fuse_mag_heading),
/// [`fuse_gnss_heading`](Self::fuse_gnss_heading) and
/// [`fuse_course`](Self::fuse_course) each correct both, or are turned down by the gate of
/// (37)–(38).
///
/// # Example
///
/// A whole flight's worth of calls, in the order they happen, with what each one is allowed to
/// answer. The README's quick start is the same loop without the assertions.
///
/// ```
/// use fusion_nav::prelude::*;
///
/// let mut filter = Eskf::default();
///
/// // A vehicle sitting still: no rotation, gravity the only specific force, read by a
/// // 400 Hz rate IMU whose clock counts microseconds.
/// let still = |sample: u64| {
///     ImuSample::from_rates(
///         Timestamp::from_micros(2_500 * sample),
///         AngularRate::body(0.0, 0.0, 0.0),
///         Acceleration::body(0.0, 0.0, -GRAVITY),
///         Seconds::from_secs(0.0025),
///     )
/// };
///
/// // Initialization reports what it achieved rather than refusing what it dislikes. A
/// // window that is short or moving gives `Alignment::Coarse`, and the filter runs and
/// // says `Status::Aligning` until attitude converges. 800 samples at 400 Hz is the 2 s
/// // `Initialization::min_duration` wants, folded in one at a time rather than buffered.
/// let mut window = StaticWindow::new();
/// for sample in 1..=800 {
///     window.push(StaticSample {
///         imu: still(sample),
///         ..StaticSample::default()
///     })?;
/// }
/// assert_eq!(filter.initialize(&window)?, Alignment::Static);
///
/// // Nothing in that window carried a barometer, so it fixes no reference altitude, and
/// // the first `fuse_baro_altitude` reads one from the estimate. See `StaticSample::baro`.
///
/// assert!(filter.predict(still(801)).is_propagated());
///
/// // Every measurement carries the time it was taken, on the IMU's clock: a receiver's fix
/// // describes where the vehicle was when it was computed, not when it arrived. These were
/// // all taken at the last sample.
/// let now = Timestamp::from_micros(2_500 * 801);
///
/// // GNSS in latitude and longitude. The filter holds the navigation origin: the first
/// // fix places it, under the estimate, so every later fix converts about the same point.
/// // `clamped` bounds the receiver's own `eph` and `epv` the way both autopilots do.
/// let outcome = filter.fuse_gnss_geodetic(
///     now,
///     Geodetic::from_degrees(47.397_742, 8.545_594, 488.0),
///     PositionNoise::clamped(
///         1.5,
///         3.0,
///         SigmaBounds::new(0.5, 100.0),
///         SigmaBounds::new(0.75, 100.0),
///     ), Position::zero(),
/// );
/// assert!(outcome.is_accepted());
/// assert!(filter.origin().is_some());
///
/// // A solution whose vertical velocity the receiver did not measure: the down axis
/// // carries a σ large enough that its gain is negligible, rather than a claim.
/// let outcome = filter.fuse_gnss_velocity(
///     now,
///     Velocity::ned(0.0, 0.0, 0.0),
///     VelocityNoise::horizontal_vertical(0.3, 1000.0), Position::zero(),
/// );
/// assert!(outcome.is_accepted());
///
/// // Nothing in that window carried a magnetometer either, so nothing observed the
/// // rotation about gravity: heading is not valid, and `Aligning` says the attitude has
/// // not converged rather than the aiding having failed.
/// let state = filter.state();
/// assert_eq!(state.status, Status::Aligning);
/// assert!(state.validity.tilt && !state.validity.heading);
///
/// // The first accepted magnetic heading is what establishes yaw.
/// let outcome = filter.fuse_mag_heading(
///     now,
///     MagField::body(0.22, 0.0, 0.44),
///     HeadingNoise::from_sigma(0.1),
/// );
/// assert!(outcome.is_accepted());
/// assert_eq!(filter.state().status, Status::Healthy);
///
/// // A vehicle without a magnetometer could have taken it from its course instead, once
/// // moving: one sitting still goes nowhere, and the direction of no velocity is refused.
/// let outcome = filter.fuse_course(now, HeadingNoise::from_sigma(0.05));
/// assert_eq!(outcome, Fusion::Unobservable);
/// # Ok::<(), fusion_nav::InitError>(())
/// ```
#[derive(Clone, Debug)]
pub struct Eskf {
    config: Config,
    estimate: Estimate,
    covariance: Covariance,
    diagnostics: Diagnostics,
    baro_reference: Option<Altitude>,
    /// The covariance of the error in `baro_reference`, (30′). Zero while there is no
    /// reference; [`establish_reference`](Self::establish_reference) keeps the two together.
    offset: Offset,
    origin: Option<LocalOrigin>,
    /// `D_m` of equations (6) and (35); see
    /// [`set_magnetic_declination`](Self::set_magnetic_declination).
    declination: Radians,
    /// Whether the caller has set [`declination`](Self::declination), which the model the
    /// filter reads at each origin then never overrides.
    declination_set: bool,
    /// Whether the heading held is referred to true north through the declination alone:
    /// leveled from a window's magnetometer by (6) or adopted from `fuse_mag_heading`, with
    /// no true heading (a seed, a GNSS heading, a course) accepted since. While it is, a
    /// declination the filter learns turns the heading with it; see
    /// [`place_origin`](Self::place_origin).
    magnetic_north: bool,
    /// `ω` of equation (9) from the last step integrated from a sample; see
    /// [`angular_rate`](Self::angular_rate).
    angular_rate: Option<AngularRate<Body>>,
    /// The earliest time a measurement can be placed at: the start, or the epoch when the start
    /// was shown at rest and its state also describes the time before it. See
    /// [`admit`](Self::admit).
    earliest: Timestamp,
    unestablished: Unestablished,
    /// Whether the attitude has ever met [`ALIGNED_TILT`](crate::config::ALIGNED_TILT) and
    /// [`ALIGNED_HEADING`](crate::config::ALIGNED_HEADING) since initialization. Latched, and the
    /// test behind [`Status::Aligning`]; see [`is_aligned`](Self::is_aligned) for why it is not
    /// read live.
    aligned: bool,
    initialized: bool,
    /// The clock; see [`time`](Self::time). Meaningless until `initialized`.
    time: Timestamp,
}

/// Quantities the start never established, which wait for the first measurement that
/// observes them: position and velocity for the first GNSS fix after a coarse start (see
/// [`Fusion::Reset`](crate::Fusion::Reset)), heading for the first heading any heading source
/// offers.
///
/// The covariance cannot carry this on its own. Every entry on its diagonal is a prior,
/// and a prior tight enough to pass [`Config::accuracy`](crate::Config::accuracy) or
/// [`ALIGNED_HEADING`](crate::config::ALIGNED_HEADING) reads as an estimate whether or not anything
/// ever measured the quantity. This is the flag that tells the two apart.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct Unestablished {
    position: bool,
    velocity: bool,
    heading: bool,
}

impl Default for Eskf {
    /// A filter on [`Config::default`], which validates (a test holds it to that).
    fn default() -> Self {
        Self::with(Config::default())
    }
}

impl Eskf {
    /// Create an uninitialized filter, or refuse a [`Config`] with a value outside its bound
    /// ([`Config::validate`]). Measurements are refused with
    /// [`Fusion::NotInitialized`](crate::Fusion::NotInitialized) until
    /// [`initialize`](Self::initialize) succeeds.
    pub fn new(config: Config) -> Result<Self, ConfigError> {
        config.validate()?;
        Ok(Self::with(config))
    }

    /// The filter [`new`](Self::new) returns, for a `config` already known to be valid.
    fn with(config: Config) -> Self {
        Self {
            config,
            estimate: Estimate::default(),
            covariance: Covariance::zero(),
            diagnostics: Diagnostics::default(),
            baro_reference: None,
            offset: Offset::default(),
            origin: None,
            declination: Radians::ZERO,
            declination_set: false,
            magnetic_north: false,
            angular_rate: None,
            earliest: Timestamp::ZERO,
            unestablished: Unestablished::default(),
            aligned: false,
            initialized: false,
            time: Timestamp::ZERO,
        }
    }

    /// The configuration this filter was built with.
    pub const fn config(&self) -> &Config {
        &self.config
    }

    /// Whether [`initialize`](Self::initialize) has succeeded.
    pub const fn is_initialized(&self) -> bool {
        self.initialized
    }

    /// The filter's clock: the [`time`](crate::ImuSample::time) of the last sample
    /// [`predict`](Self::predict) was handed, or of the window or seed the filter started from.
    /// `None` until initialized.
    ///
    /// Every later sample is differenced against it and every measurement's age read from it.
    /// Only [`Propagation::InvalidStep`](crate::Propagation::InvalidStep), a sample not after it,
    /// leaves it where it was; every other outcome moves it, integrated or not, since the time did
    /// pass. So it is when the estimate is valid only while the samples are being integrated: after
    /// a refused one the state is as old as the last that was.
    pub const fn time(&self) -> Option<Timestamp> {
        if self.initialized {
            Some(self.time)
        } else {
            None
        }
    }

    /// Store the offset's covariance, floored by (42′) as
    /// [`commit_covariance`](Self::commit_covariance) floors the rest — while there is a
    /// reference. Without one the offset is zero by definition and nothing reads it, so it is
    /// stored as zero rather than as what (30′)'s walk would have grown it to.
    fn commit_offset(&mut self, mut offset: Offset) {
        if self.baro_reference.is_none() {
            self.offset = Offset::default();
            return;
        }
        if floor_offset(&mut offset) {
            self.diagnostics.floored = self.diagnostics.floored.saturating_add(1);
        }
        self.offset = offset;
    }

    /// Store a covariance, applying the diagonal floor of equation (42′) and counting what it
    /// raised.
    ///
    /// One place, so that the invariant is a property of the filter rather than of each
    /// equation that produces a `P`: **every covariance the filter commits has been floored**,
    /// and therefore so has every covariance it fuses against, tests [`Validity`](crate::Validity)
    /// against, or reads back into the `S` of (24). Committed, not held: an `Eskf` that has not
    /// initialized holds [`Covariance::zero`](crate::Covariance::zero) and
    /// [`covariance`](Self::covariance) will hand it over, which is the one place a caller can read
    /// a variance of zero off this filter. Nothing acts on it — every path that would is behind
    /// `initialized` — and initializing is itself a commit.
    ///
    /// A floor applied inside (22) and (27) instead would be two call sites protecting the
    /// two operations that shrink a variance, and would leave the ones that *write* one —
    /// the adoption of
    /// [`Fusion::Reset`](crate::Fusion::Reset), the `reset_*_to` methods, the initial
    /// covariance of (8) — unprotected, each carrying a variance that came from outside the
    /// filter.
    ///
    /// Symmetry stays where the algebra is, beside each product that can drift off it
    /// (EQUATIONS.md's table lists them), and at `initialize_from`, where a caller's matrix
    /// enters: (42)'s two halves answer different faults. `½(P + Pᵀ)` repairs drift a product
    /// introduces, so it belongs to the product; the floor bounds a value, so it belongs to
    /// the value.
    fn commit_covariance(&mut self, covariance: Covariance, offset: Offset) {
        let mut matrix = *covariance.as_matrix();
        let raised = floor_diagonal(&mut matrix);
        self.covariance = Covariance::from_matrix(matrix);
        self.commit_offset(offset);
        self.diagnostics.floored = self.diagnostics.floored.saturating_add(raised);
    }

    /// The current estimate, including its [`Status`].
    ///
    /// Status and validity are derived here rather than cached: they are pure functions
    /// of [`diagnostics`](Self::diagnostics), the covariance, and [`Config`], so computing
    /// them on read means there is no invariant for the mutating methods to maintain. The
    /// cost is two rotations of the attitude block, one each for tilt and heading, six other
    /// covariance entries and six source timers, compared.
    pub fn state(&self) -> State {
        // `self.estimate.state().status` and `.validity` are inert; the stored estimate never
        // carries meaningful ones, and every read overwrites them.
        let validity = self.validity();
        let mut state = *self.estimate.state();
        state.status = self.derive_status();
        state.validity = validity;
        state
    }

    /// The body angular rate with the estimated gyroscope bias removed, `ω` of equation (9),
    /// from the last step [`predict`](Self::predict) integrated from a sample.
    ///
    /// For moving the estimate to a point other than the IMU. The GNSS `fuse_*` take their
    /// antenna's offset and refer the fix to the IMU themselves, (28′) and (29′); the estimate
    /// they correct stays the IMU's, and a point at `r` in body axes (the center of mass a
    /// controller wants, a payload, a sensor this crate does not fuse) is at the IMU's position
    /// plus `R r` and moves at its velocity plus `R (ω × r)`. PX4 reports its estimate at the
    /// IMU the same way, and corrects each aiding source with this bias-corrected rate
    /// (`EKF/aid_sources/gnss/gps_control.cpp:313-318` at `c4e4ef98`).
    ///
    /// The algebra is the application's own; `nalgebra` here, reached through arrays so that
    /// its version is the application's too.
    ///
    /// ```
    /// use fusion_nav::prelude::*;
    /// use nalgebra::{Quaternion, UnitQuaternion, Vector3};
    /// # let mut filter = Eskf::default();
    /// # let dt = Seconds::from_secs(0.0025);
    /// # let (level, gravity) = (AngularRate::zero(), Acceleration::body(0.0, 0.0, -GRAVITY));
    /// # let at = |i: u64| Timestamp::from_micros(2_500 * i);
    /// # let mut window = StaticWindow::new();
    /// # for i in 1..=800 {
    /// #     window.push(StaticSample {
    /// #         imu: ImuSample::from_rates(at(i), level, gravity, dt),
    /// #         ..StaticSample::default()
    /// #     })?;
    /// # }
    /// # filter.initialize(&window)?;
    /// # let imu = ImuSample::from_rates(at(801), AngularRate::body(0.0, 0.0, 0.5), gravity, dt);
    /// # assert!(filter.predict(imu).is_propagated());
    /// // The center of mass, 0.2 m behind the IMU: measured on the airframe.
    /// let r = Vector3::from(Position::body(-0.2, 0.0, 0.0).to_array());
    ///
    /// // None before the first step and across a gap, where no sample says how fast the
    /// // vehicle turned.
    /// if let Some(omega) = filter.angular_rate() {
    ///     let state = filter.state();
    ///     let q = state.attitude.body_to_ned();
    ///     let rotation = UnitQuaternion::new_normalize(Quaternion::new(q.w, q.x, q.y, q.z));
    ///     let position = Vector3::from(state.position.to_array()) + rotation * r;
    ///     let omega = Vector3::from(omega.to_array());
    ///     let velocity = Vector3::from(state.velocity.to_array()) + rotation * omega.cross(&r);
    ///     // Yawing at 0.5 rad/s, a point 0.2 m aft swings sideways at 0.1 m/s.
    ///     assert!((velocity.norm() - 0.1).abs() < 1e-3);
    /// #   let _ = position;
    /// }
    /// # Ok::<(), InitError>(())
    /// ```
    ///
    /// Committed with the state, and only with it. `None` until a step is integrated, after
    /// any initialization, and after a gap coasted past
    /// [`Config::max_predict_dt`](crate::Config::max_predict_dt), until the next sample: a
    /// coast reads no sample, and the rate before the gap is not the rate after it. A refused
    /// step leaves the last rate in place, as it leaves the state.
    pub const fn angular_rate(&self) -> Option<AngularRate<Body>> {
        self.angular_rate
    }

    /// Per-source health. Off the hot path.
    pub const fn diagnostics(&self) -> &Diagnostics {
        &self.diagnostics
    }

    /// The 15 x 15 error covariance, in the error-state ordering.
    pub const fn covariance(&self) -> &Covariance {
        &self.covariance
    }

    /// Aggregate alignment and the per-source timers into one status, most severe first.
    ///
    /// [`DeadReckoning`](Status::DeadReckoning) reads horizontal aiding alone, GNSS position
    /// or velocity within [`Timeouts::dead_reckoning_after`](crate::Timeouts): with GNSS gone
    /// the barometer still holds height, and position drifts all the same. The rest reads
    /// every source, each against its own period.
    ///
    /// Only sources that have ever been accepted count toward `Degraded`: a vehicle with no
    /// magnetometer is not permanently `Degraded` for lacking one. The course constraint
    /// never counts ([`Diagnostics::aiding`]): it aids nothing the GNSS velocity it needs does
    /// not.
    ///
    /// Alignment enters as the latch of [`is_aligned`](Self::is_aligned) rather than as the
    /// live [`Validity`](crate::Validity), which is the one thing here that is not derived on read.
    fn derive_status(&self) -> Status {
        let d = &self.diagnostics;
        let horizontal = d
            .horizontal()
            .any(|source| source.accepted_within(self.config.timeouts.dead_reckoning_after));
        if !horizontal {
            // Position is unusable whatever the attitude is doing, so this outranks
            // `Aligning`.
            Status::DeadReckoning
        } else if !self.aligned {
            Status::Aligning
        } else if d
            .aiding()
            .filter(|source| source.has_been_used())
            .all(|source| self.accepted_recently(source))
        {
            Status::Healthy
        } else {
            Status::Degraded
        }
    }
}

impl Unestablished {
    /// What a start leaves unestablished, read off what its window showed rather than off
    /// which [`Alignment`](crate::Alignment) it earned.
    ///
    /// `settled` is [`init::at_rest`](crate::init::at_rest)'s verdict. A vehicle that held still
    /// through the window is where the origin says it is and is not moving, which is the whole of
    /// what a static start ever claimed about position and velocity — and a window too short to
    /// align an attitude from claims it just as honestly, which is
    /// [`Coarse::WindowTooShort`](crate::Coarse::WindowTooShort). A window taken in motion
    /// establishes neither: the vehicle passed through somewhere the filter cannot name. Those wait
    /// for the first fix.
    ///
    /// Heading needs a magnetometer in the window on top of stillness. Gravity pins tilt and
    /// nothing pins the rotation about it, so a window carrying no field leaves yaw a prior however
    /// long and however still it was —
    /// [`Initialization::sigma_yaw`](crate::Initialization::sigma_yaw) is 0.35 rad against an
    /// [`Accuracy::heading`](crate::Accuracy::heading) of 0.52, so the covariance alone would
    /// report a yaw nobody measured as good.
    const fn after(settled: bool, observed_field: bool) -> Self {
        Self {
            position: !settled,
            velocity: !settled,
            heading: !(settled && observed_field),
        }
    }
}

#[cfg(test)]
mod adversarial;

#[cfg(test)]
mod fixtures;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::{Config, Recovery};
    use crate::eskf::fixtures::*;

    use crate::geodetic::Geodetic;
    use crate::health::{Fusion, GnssFusion, Propagation, Refusal, Status};
    use crate::init::tests::{spaced, still};
    use crate::init::{Alignment, InitError, StaticSample, StaticWindow};

    use crate::propagate::ImuSample;
    use crate::state::{Covariance, Offset, State};
    use crate::units::{
        Acceleration, Altitude, AltitudeNoise, AngularRate, HeadingNoise, MagField, Position,
        PositionNoise, Radians, Seconds, Timestamp, Velocity, VelocityNoise,
    };

    #[test]
    fn a_config_outside_its_bounds_builds_no_filter() {
        // A NaN timeout never compares true: built, this filter would never recover GNSS height.
        let config = Config {
            recovery: Recovery {
                gnss_height: Some(Seconds::from_secs(f32::NAN)),
                ..Recovery::default()
            },
            ..Config::default()
        };
        let refused = Eskf::new(config).err();
        assert_eq!(
            refused.map(|e| (e.field, e.bound)),
            Some(("recovery.gnss_height", crate::ConfigBound::Positive))
        );
        assert!(refused.is_some_and(|e| e.value.is_nan()));
        assert_eq!(Eskf::default().config(), &Config::default());
    }

    // Found by the adversarial suite, `adversarial.rs`, and kept as literals so each
    // names the defect it guards.

    #[test]
    fn a_filter_with_no_barometric_reference_holds_no_offset() {
        let mut filter = initialized();
        assert_eq!(filter.baro_reference(), None);
        for _ in 0..10 {
            assert_eq!(filter.step(still().imu, DT), Propagation::Propagated);
        }
        assert_eq!(filter.offset, Offset::default());
    }

    #[test]
    fn a_start_that_overflows_is_refused_and_commits_nothing() {
        // Finite, so `push` takes it; (5) and (8) square it past `f32`.
        let mut filter = Eskf::default();
        let imu = ImuSample::from_rates(
            Timestamp::from_micros(1_000_000),
            AngularRate::body(0.0, 0.0, 0.0),
            Acceleration::body(0.0, 0.0, f32::MAX),
            DT,
        );
        assert_eq!(filter.initialize_coarse(imu), Err(InitError::NotFinite));
        assert!(!filter.is_initialized());
    }

    #[test]
    fn a_still_window_whose_barometric_reference_overflows_is_refused_whole() {
        // One reading `push` takes as finite; the mean and scatter of (30) square it past `f32`.
        let mut altitudes = [100.0, 100.2, 99.9, 100.1, 100.0, 99.8, 100.3, 100.0];
        altitudes[3] = f32::MAX;
        let window = StaticWindow::try_from(
            spaced(&window_with_baro(altitudes), Seconds::from_secs(0.25)).as_slice(),
        )
        .expect("every sample is finite");
        let mut filter = Eskf::default();
        assert_eq!(filter.alignment_of(&window), Err(InitError::NotFinite));
        assert_eq!(filter.initialize(&window), Err(InitError::NotFinite));
        assert!(!filter.is_initialized());
        assert_eq!(filter.baro_reference(), None);
    }

    #[cfg(feature = "magnetic-model")]
    #[test]
    fn a_geodetic_fix_with_no_origin_under_it_turns_no_heading() {
        // A magnetometer-set heading is turned by the declination the first origin reads, so
        // a fix whose origin cannot be placed must not have turned it on the way to refusal.
        let mut filter = Eskf::default();
        let _ = filter
            .initialize_over(&window_with_mag(), Seconds::from_secs(0.25))
            .expect("a still window");
        // Three meters out, so (44) must place an origin that puts a fix 3.4e38 m up three
        // meters away, which no `f64` resolves.
        let noise = PositionNoise::horizontal_vertical(1.0, 1.0);
        assert!(filter.reset_position_to(Position::ned(3.0, 0.0, 0.0), noise));
        let (attitude, declination) = (filter.state().attitude, filter.magnetic_declination());
        let outcome = filter.fuse_gnss_geodetic(
            filter.now(),
            Geodetic::from_degrees(56.0, 44.0, 3.4e38),
            PositionNoise::horizontal_vertical(1.0, 1.0),
            Position::zero(),
        );
        assert_eq!(outcome.horizontal, Fusion::NoReference);
        assert_eq!(filter.state().attitude, attitude);
        assert_eq!(filter.magnetic_declination(), declination);
        assert_eq!(filter.origin(), None);
    }

    #[test]
    fn an_adoption_that_would_overflow_is_refused_and_places_nothing() {
        // After a coarse start the first fix is adopted, carried to the IMU by (28′). A finite
        // arm of `f32::MAX` on every axis overflows once rotated, and a tilted start rotates it.
        let arm = Position::body(f32::MAX, f32::MAX, f32::MAX);
        let noise = PositionNoise::horizontal_vertical(1.0, 1.0);
        let mut filter = Eskf::default();
        let tilted = ImuSample::from_rates(
            Timestamp::from_micros(1_000_000),
            AngularRate::body(0.0, 0.0, 0.0),
            Acceleration::body(3.0, -2.0, -9.0),
            DT,
        );
        assert!(filter.initialize_coarse(tilted).is_ok());
        let (state, declination) = (*filter.estimate.state(), filter.magnetic_declination());

        let outcome = filter.fuse_gnss_position(filter.now(), Position::zero(), noise, arm);
        assert_eq!(outcome, GnssFusion::both(Fusion::NotFinite));
        let fix = Geodetic::from_degrees(56.0, 44.0, 100.0);
        let outcome = filter.fuse_gnss_geodetic(filter.now(), fix, noise, arm);
        assert_eq!(outcome, GnssFusion::both(Fusion::NotFinite));

        assert_eq!(*filter.estimate.state(), state);
        assert_eq!(filter.origin(), None);
        assert_eq!(filter.magnetic_declination(), declination);
        assert!(filter.unestablished.position);
    }

    #[test]
    fn a_seed_that_is_not_symmetric_is_committed_symmetric() {
        let mut p = crate::state::CovarianceMatrix::from_diagonal_element(0.1);
        p[(0, 1)] = 0.01;
        p[(1, 0)] = 0.03;
        let mut filter = Eskf::default();
        assert!(
            filter
                .seed(State::default(), Covariance::from_matrix(p))
                .is_ok()
        );
        let committed = filter.covariance().as_matrix();
        assert_eq!(*committed, committed.transpose());
        assert_eq!(committed[(0, 1)], 0.02);
    }

    #[test]
    fn a_heading_adoption_whose_variance_overflows_is_refused() {
        // A coarse start charges its tilt prior the peak rate it saw; (36′) squares that tilt
        // into the variance a first magnetic heading is adopted at.
        let mut filter = Eskf::default();
        let spinning = ImuSample::from_rates(
            Timestamp::from_micros(1_000_000),
            AngularRate::body(0.0, 0.0, 1.0e20),
            Acceleration::body(0.3, 0.0, -9.3),
            DT,
        );
        assert!(filter.initialize_coarse(spinning).is_ok());
        let (state, covariance) = (*filter.estimate.state(), *filter.covariance());
        let field = MagField::body(0.22, 0.0, 0.44);
        let outcome = filter.fuse_mag_heading(filter.now(), field, HeadingNoise::from_sigma(0.02));
        assert_eq!(outcome, Fusion::NotFinite);
        assert_eq!(
            (*filter.estimate.state(), *filter.covariance()),
            (state, covariance)
        );
        assert!(filter.unestablished.heading);
    }

    #[test]
    fn an_origin_the_position_cannot_be_written_about_is_refused() {
        let mut filter = initialized();
        assert!(filter.set_origin(Geodetic::from_degrees(47.4, 8.5, 488.0)));
        let held = (filter.origin(), filter.state().position);
        // A height `f64` holds and `f32` does not.
        assert!(!filter.set_origin(Geodetic::from_degrees(47.4, 8.5, -1.0e39)));
        assert_eq!((filter.origin(), filter.state().position), held);
    }

    /// The floor of (42′) exists to be unreachable by an honest source. CI asserts it here, on
    /// one run whose every fix is exact, and in `adversarial.rs`'s `ordinary`, on generated
    /// honest ones; `data/fetch.sh --check` pins `floored=0` on all thirteen corpus logs, and
    /// needs a network and PX4 tooling, so it runs locally. The margin between the floor and
    /// anything a filter that is propagating and fusing reaches is measured in `math.rs`'s
    /// `FLOOR` — a count here means the floor is masking a collapse rather than preventing
    /// one, and the `sigma_*` columns of `examples/replay/main.rs` say which state.
    #[test]
    fn an_ordinary_run_never_reaches_the_floor() {
        let mut filter = aided();
        for step in 0..400 {
            assert_eq!(filter.step(still().imu, DT), Propagation::Propagated);
            if step % 25 == 0 {
                assert!(
                    filter
                        .fuse_gnss_position(
                            filter.now(),
                            Position::ned(0.0, 0.0, 0.0),
                            PositionNoise::horizontal_vertical(1.5, 1.5),
                            Position::zero(),
                        )
                        .is_accepted()
                );
                assert!(
                    filter
                        .fuse_baro_altitude(
                            filter.now(),
                            Altitude::from_meters(100.0),
                            AltitudeNoise::from_sigma(2.0),
                        )
                        .is_accepted()
                );
            }
        }
        assert_eq!(filter.diagnostics().floored, 0);
    }

    /// Started still with a barometer at 0 m and a magnetometer, so that `Status` can reach
    /// `Healthy`: without a heading the start stays `Aligning`.
    fn aligned() -> Eskf {
        let window = window_at(0.0).map(|sample| StaticSample {
            mag: Some(MagField::body(0.22, 0.0, 0.44)),
            ..sample
        });
        let mut filter = Eskf::default();
        assert_eq!(
            filter.initialize_over(&window, Seconds::from_secs(0.25)),
            Ok(Alignment::Static)
        );
        assert!(filter.is_aligned());
        filter
    }

    #[test]
    fn a_fast_source_that_stops_degrades_the_status_on_its_own_schedule() {
        // A 10 Hz barometer beside a 1 Hz GNSS: the barometer times out after 0.25 s of
        // silence, where one threshold sized for the receiver would take 2.5 s to notice.
        let mut filter = aligned();
        let baro = |filter: &mut Eskf| {
            let outcome = filter.fuse_baro_altitude(
                filter.now(),
                Altitude::from_meters(0.0),
                AltitudeNoise::from_sigma(2.0),
            );
            assert!(outcome.is_accepted(), "{outcome:?}");
        };
        let gnss = |filter: &mut Eskf| {
            hold_velocity(filter, Velocity::ned(0.0, 0.0, 0.0));
        };
        hold(&mut filter, 10.0, 10, |filter| {
            baro(filter);
            if filter.diagnostics().baro_altitude.accepted % 10 == 0 {
                gnss(filter);
            }
        });
        assert_eq!(filter.state().status, Status::Healthy);

        hold(&mut filter, 0.4, 100, |_| {});
        assert_eq!(filter.state().status, Status::Degraded);
    }

    #[test]
    fn a_status_reads_dead_reckoning_once_gnss_stops_whatever_else_arrives() {
        let mut filter = aligned();
        hold(&mut filter, 2.0, 10, |filter| {
            hold_velocity(filter, Velocity::ned(0.0, 0.0, 0.0));
            let _ = filter.fuse_baro_altitude(
                filter.now(),
                Altitude::from_meters(0.0),
                AltitudeNoise::from_sigma(2.0),
            );
        });
        assert_eq!(filter.state().status, Status::Healthy);

        // The barometer keeps arriving, and is still accepted, past `dead_reckoning_after`.
        hold(&mut filter, 5.5, 10, |filter| {
            let _ = filter.fuse_baro_altitude(
                filter.now(),
                Altitude::from_meters(0.0),
                AltitudeNoise::from_sigma(2.0),
            );
        });
        assert!(
            filter
                .diagnostics()
                .baro_altitude
                .is_fresh(&crate::Timeouts::default())
        );
        assert_eq!(filter.state().status, Status::DeadReckoning);
    }

    #[test]
    fn a_course_refused_at_a_stop_does_not_degrade_the_status() {
        // Two filters fed the same velocities and GNSS headings, one also fusing the course: when
        // the vehicle stops and every course is refused, `Status` must read the same for both,
        // and with every counted source fresh that is `Healthy`.
        let east = Velocity::ned(0.0, 15.0, 0.0);
        let stopped = Velocity::ned(0.0, 0.0, 0.0);
        let heading = Radians::from_radians(core::f32::consts::FRAC_PI_2);
        let noise = HeadingNoise::from_sigma(0.05);
        let mut with = cruising(east);
        let mut without = cruising(east);
        for filter in [&mut with, &mut without] {
            assert!(
                filter
                    .fuse_gnss_heading(filter.now(), heading, noise)
                    .is_reset()
            );
        }
        assert!(with.fuse_course(with.now(), noise).is_accepted());
        for filter in [&mut with, &mut without] {
            assert!(filter.reset_velocity_to(stopped, VelocityNoise::from_speed_accuracy(0.3)));
            hold(filter, 4.0, 10, |filter| {
                hold_velocity(filter, stopped);
                let _ = filter.fuse_gnss_heading(filter.now(), heading, noise);
                let _ = filter.fuse_course(filter.now(), noise);
            });
        }
        assert_eq!(
            with.diagnostics().course.last_refusal,
            Some(Refusal::Unobservable)
        );
        assert_eq!(without.state().status, Status::Healthy);
        assert_eq!(with.state().status, without.state().status);
    }
}
