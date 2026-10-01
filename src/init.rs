//! Initialization: the attitude and biases the window yields, classifying the window, the
//! initial covariance, and the barometric reference. Equations (5)–(8), and `α₀` of (30).
//!
//! The entry points are methods on [`Eskf`](crate::Eskf) — `initialize`,
//! `initialize_coarse`, `initialize_from`, and `alignment_of` — which call into the pure
//! functions here and then commit the result to the filter.

use nalgebra::{ComplexField, Matrix3, RealField, Rotation3, UnitQuaternion, Vector3};

use crate::config::{Config, Initialization};
use crate::display::{Decimals, Fixed};
use crate::frames::{Body, Ned};
use crate::math::{skew, wrap_pi};
use crate::propagate::ImuSample;
use crate::state::{AttitudeVariance, Covariance, State};
use crate::units::{
    Acceleration, Altitude, AltitudeNoise, AngularRate, Attitude, MagField, MetersPerSecond2,
    Position, Radians, RadiansPerSecond, Seconds, Timestamp, Velocity,
};

/// One sample from the quasi-static initialization window.
///
/// The magnetometer is optional: without it nothing observes the rotation about gravity,
/// `ψ₀` of equation (6) stays zero, and the first accepted heading, magnetic, GNSS or
/// course, is what establishes it. A window with none anywhere in it says so:
/// [`Validity::heading`](crate::Validity) stays false until a heading source accepts one,
/// because
/// [`sigma_yaw`](crate::Initialization::sigma_yaw) is a prior and would otherwise read as
/// an estimate of a quantity nothing measured.
///
/// The barometer is optional in the same way, but less forgivingly: its reference is
/// established by a start and only refined after one (30′), so a window carrying none leaves
/// nothing for a later altitude to be relative to and barometric fusion is refused for the
/// whole flight.
///
/// GNSS velocity is the one field a window taken **in motion** can use, and the only
/// reason this type is not simply an IMU sample plus what stillness needs: see
/// [`velocity`](Self::velocity).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct StaticSample {
    /// IMU measurement.
    pub imu: ImuSample,
    /// Magnetometer measurement, if the vehicle has one.
    ///
    /// Averaged over the window to fix `ψ₀`, the initial heading (equation (6)), and
    /// averaged rather than taken from one sample for the reason the barometer and the
    /// gyroscope bias are: a single reading carries the sensor's full noise.
    pub mag: Option<MagField<Body>>,
    /// Barometric altitude, if the vehicle has a barometer.
    ///
    /// Averaged over the window to fix `α₀`, the barometer's reference at the navigation
    /// origin (equation (30)). Averaged rather than taken from one sample for the same
    /// reason the gyroscope bias is: a single reading carries the sensor's full noise.
    ///
    /// Without it the window fixes no reference, and
    /// [`Eskf::fuse_baro_altitude`](crate::Eskf::fuse_baro_altitude) reads one from the
    /// estimate at the first altitude instead. `α₀` is refined in flight by (30′), and the
    /// scatter of these readings is the variance it starts with, so a window needs two of
    /// them.
    pub baro: Option<Altitude>,
    /// GNSS velocity in the navigation frame, if the vehicle has a receiver.
    ///
    /// Differenced across the window for `ā_n`, the vehicle's own acceleration
    /// (equation (5′)). At rest that term is zero and equation (5) reads tilt straight
    /// off the accelerometer; in motion the accelerometer reads specific force, and
    /// `ā_n` is what separates the two. It is the whole of what a moving window has that
    /// a still one does not need.
    ///
    /// Attach it to the epoch it arrived on rather than holding it across the IMU epochs
    /// that follow, as `mag` and `baro` may be held: those are averaged, and an average
    /// survives a repeated value, while a difference is dated by the samples carrying it
    /// and a held reading dates from before the epoch it sits on. Holding costs up to one
    /// GNSS interval of span, in either direction.
    ///
    /// **Stub.** Measured and reported on [`Coarse::NotStationary`](Coarse::NotStationary); nothing
    /// levels with it. Not built: equation (5′), #59.
    pub velocity: Option<Velocity<Ned>>,
}

impl StaticSample {
    /// Whether every number in the sample is finite, the optional ones included.
    pub(crate) fn is_finite(&self) -> bool {
        self.imu.is_finite()
            && self.mag.is_none_or(|field| field.is_finite())
            && self.baro.is_none_or(|b| b.as_meters().is_finite())
            && self.velocity.is_none_or(|v| v.is_finite())
    }
}

/// The initialization window, accumulated one sample at a time.
///
/// [`Eskf::initialize`](crate::Eskf::initialize) needs only what the window reduces to: sums,
/// peaks, the span, the barometer's scatter and the first and last GNSS velocity. So the samples
/// are folded in as they arrive rather than buffered. A buffered window at the default
/// [`Initialization::min_duration`] is more RAM than a Cortex-M0 has; this one is the same size at
/// any rate and any length ([measured]).
///
/// ```
/// # use fusion_nav::prelude::*;
/// # let dt = Seconds::from_secs(0.0025);
/// # let gravity = Acceleration::body(0.0, 0.0, -GRAVITY);
/// # let sample = |i: u64| StaticSample {
/// #     imu: ImuSample::from_rates(Timestamp::from_micros(2_500 * i), AngularRate::zero(), gravity, dt),
/// #     ..StaticSample::default()
/// # };
/// let mut filter = Eskf::default();
/// let config = *filter.config();
/// let mut window = StaticWindow::new();
/// let mut i = 0;
/// while !window.is_long_enough(&config) {
///     i += 1;
///     // A refused sample leaves the window as it was: drop it and go on.
///     if window.push(sample(i)).is_err() {
///         continue;
///     }
///     // Waiting for stillness: a vehicle that moved starts the wait over.
///     if !window.is_at_rest(&config) {
///         window = StaticWindow::new();
///     }
/// }
/// assert_eq!(filter.initialize(&window)?, Alignment::Static);
/// # Ok::<(), InitError>(())
/// ```
///
/// Waiting for stillness restarts the window, as above; [`is_at_rest`](Self::is_at_rest) says
/// why it cannot slide. A buffered slice can, at the cost of the buffer, and
/// [`try_extend`](Self::try_extend) or [`TryFrom`] builds a window from one.
///
/// Each [`push`](Self::push) costs a few dozen floating-point operations, some in `f64`
/// ([counted]). On a core with no floating-point unit every one is a library call, inside the loop
/// that is already reading the IMU: the price of not buffering, paid only until the window commits.
/// `level_scatter` and `BaroReadings` say why their sums need `f64`. For the averages it is
/// precaution: summing a few thousand readings near `γ` in `f32` costs on the order of 10⁻⁵ rad of
/// tilt, against a 0.02 rad prior.
///
/// [measured]: https://github.com/wboayue/fusion-nav/blob/main/DESIGN.md#measured-cost-by-function
/// [counted]: https://github.com/wboayue/fusion-nav/blob/main/DESIGN.md#arithmetic
#[derive(Clone, Debug)]
pub struct StaticWindow {
    /// `Σ f fᵀ`, for the window's scatter about `f̄`; see [`level_scatter`].
    outer: Matrix3<f64>,
    /// `Σ ω`, the angular rate (7) takes as the gyroscope bias.
    rate: Vector3<f64>,
    peaks: Peaks,
    /// The time the samples integrated; see [`span`](Self::span).
    span: f64,
    /// The last sample's time: the start of the filter's clock, and what the next sample's
    /// step is differenced against.
    end: Option<Timestamp>,
    /// `Σ f` and `Σ m`, the sums (5) and (6) average, kept in blocks for the halves.
    sums: BlockSums,
    velocities: Velocities,
    baro: BaroReadings,
    /// The gyroscope's and the accelerometer's white noise; see [`Density`].
    gyro_noise: Density,
    accel_noise: Density,
}

impl Default for StaticWindow {
    fn default() -> Self {
        Self::new()
    }
}

impl StaticWindow {
    /// An empty window.
    #[must_use]
    pub fn new() -> Self {
        Self {
            outer: Matrix3::zeros(),
            rate: Vector3::zeros(),
            peaks: Peaks::NONE,
            span: 0.0,
            end: None,
            sums: BlockSums::new(),
            velocities: Velocities::default(),
            baro: BaroReadings::default(),
            gyro_noise: Density::EMPTY,
            accel_noise: Density::EMPTY,
        }
    }

    /// Add one sample to the window.
    ///
    /// Every check initialization makes on raw input is here, so that nothing downstream
    /// can be handed an unusable interval, a clock that does not run forward, or a value
    /// that is not a number. A refused sample leaves the window as it was, so the caller
    /// can drop it and go on, or start over. The clock is checked because it starts at the
    /// last sample's time: a window timed anywhere but forward would start it somewhere the
    /// samples do not describe, and the first `predict` would coast the whole flight so far.
    ///
    /// # Errors
    ///
    /// [`SampleRefusal`], naming which check the sample failed.
    pub fn push(&mut self, sample: StaticSample) -> Result<(), SampleRefusal> {
        if !sample.is_finite() {
            return Err(SampleRefusal::NotFinite);
        }
        if let Some(interval) = sample.imu.unusable_interval() {
            return Err(SampleRefusal::InvalidInterval { interval });
        }
        if let Some(end) = self.end {
            let dt = sample.imu.time.since(end);
            if !dt.is_usable_step() {
                return Err(SampleRefusal::InvalidStep { dt });
            }
        }

        // Rates, one division per sample: every statistic here is a statement about a
        // rate or a force.
        let (accel, gyro) = (
            sample.imu.specific_force().vector(),
            sample.imu.angular_rate().vector(),
        );
        self.peaks.push(accel, gyro);
        let interval = f64::from(sample.imu.angle_interval.as_secs());
        let velocity_interval = f64::from(sample.imu.velocity_interval.as_secs());
        let (accel, gyro) = (widen(accel), widen(gyro));
        self.outer += accel * accel.transpose();
        self.rate += gyro;
        self.gyro_noise
            .push(widen(sample.imu.delta_angle.vector()), interval);
        self.accel_noise
            .push(widen(sample.imu.delta_velocity.vector()), velocity_interval);
        self.span += interval;
        self.end = Some(sample.imu.time);
        self.sums.push(
            accel,
            sample.mag.map(|measurement| widen(measurement.vector())),
        );
        self.velocities.push(sample.velocity, interval);
        self.baro.push(sample.baro);
        Ok(())
    }

    /// Push every sample `samples` yields, in order, stopping at the first one refused.
    ///
    /// The samples before it stay in the window, and the refused one and those after it are
    /// not taken. A caller building one start from a buffer discards the window on an error,
    /// as [`TryFrom`] does: a sample the buffer cannot supply says it is not the window the
    /// caller thought it was.
    ///
    /// # Errors
    ///
    /// As [`push`](Self::push), for the first sample refused.
    pub fn try_extend(
        &mut self,
        samples: impl IntoIterator<Item = StaticSample>,
    ) -> Result<(), SampleRefusal> {
        samples.into_iter().try_for_each(|sample| self.push(sample))
    }

    /// Whether the window's peak motion so far is within the tolerances of a still one.
    ///
    /// The test [`Alignment::Static`] and the barometric reference rest on, asked of the
    /// peaks alone, so it is cheap after every sample where
    /// [`alignment_of`](crate::Eskf::alignment_of) measures the whole window. An empty window
    /// has not moved.
    ///
    /// A window only grows, and peaks do not fall, so once this is false it stays false: an
    /// application waiting for stillness starts a new window rather than sliding this one.
    #[must_use]
    pub fn is_at_rest(&self, config: &Config) -> bool {
        at_rest(self.peaks, &config.init, config.gravity)
    }

    /// Whether the window holds a sample and spans [`Initialization::min_duration`]: the
    /// condition a collection loop runs until.
    ///
    /// The sample is asked for as well as the span, so that a `min_duration` of zero still
    /// ends the loop on a window [`Eskf::initialize`](crate::Eskf::initialize) accepts rather
    /// than on an empty one it refuses.
    #[must_use]
    pub fn is_long_enough(&self, config: &Config) -> bool {
        self.end.is_some() && long_enough(self.span(), &config.init)
    }

    /// The time the window's samples integrated: the sum of their angle intervals, and what
    /// [`Initialization::min_duration`] is checked against.
    ///
    /// Integrated time rather than the distance between the first and last timestamps,
    /// because it is what the window observed: a window a logger dropped samples from covers
    /// the samples it kept, and a stillness test over time nobody measured would pass on
    /// nothing. The angle interval rather than the velocity one for no reason but one: the
    /// two cover the same sample and differ only by when a driver closed each integral.
    ///
    /// Summed in `f64`: in `f32`, 100 intervals of 20 ms come to 1.9999987 s, and a window of
    /// exactly `min_duration` would read as too short by the rounding. That is
    /// `data/flight.csv`'s window: summed in `f32`, it starts `short`.
    #[must_use]
    pub fn span(&self) -> Seconds {
        Seconds::from_secs(self.span as f32)
    }

    /// What the window measured, or [`InitError::NoSamples`] for a window with nothing in it.
    pub(crate) fn measured(&self) -> Result<Measured, InitError> {
        let end = self.end.ok_or(InitError::NoSamples)?;
        let (first, second) = self.sums.split();
        let whole = first.merged(second);
        Ok(Measured {
            force: Acceleration::from_vector(mean(whole.force, whole.samples)),
            rate: AngularRate::from_vector(mean(self.rate, whole.samples)),
            field: (whole.fields > 0)
                .then(|| MagField::from_vector(mean(whole.field, whole.fields))),
            inertial_accel: self.velocities.inertial_acceleration(),
            peaks: self.peaks,
            span: self.span(),
            end,
            halves: Halves::of(first, second),
            level_scatter: level_scatter(self.outer, whole.force, whole.samples),
        })
    }

    /// `α₀` of equation (30) and the variance of that mean, `P_bb` of (30′), as the window's
    /// barometer readings give them; see [`BaroReadings`].
    pub(crate) fn alpha0(&self) -> Option<(Altitude, f32)> {
        self.baro.reference()
    }

    /// The white noise this window measured on each sensor, a floor under the noise the
    /// filter should be told; [`WindowNoise`] says why a floor. Equation (8″).
    ///
    /// On the window rather than the filter, because the figure is for building a
    /// [`Config`] and so comes before any filter exists; it reads the same tolerances
    /// [`is_at_rest`](Self::is_at_rest) does. Measured and
    /// reported, never applied: a figure changes the filter only if the caller writes it into
    /// the `Config` (`GOALS.md` differentiator 7, where derived is not adaptive).
    ///
    /// `None` unless the window is at rest, the test the barometric reference rests on, and
    /// spans [`WindowNoise::MIN_READINGS`] of the [`WindowNoise::BLOCK`]s the IMU's densities
    /// are read over: a moving window's scatter is its motion. The barometer's figure takes
    /// the same count of its own readings.
    #[must_use]
    pub fn noise(&self, config: &Config) -> Option<WindowNoise> {
        if !self.is_at_rest(config) {
            return None;
        }
        Some(WindowNoise {
            gyro_white: self.gyro_noise.density()?.into(),
            accel_white: self.accel_noise.density()?.into(),
            baro: self.baro.noise(),
            blocks: self.gyro_noise.blocks.min(self.accel_noise.blocks),
            baro_readings: self.baro.count,
        })
    }
}

/// A window from samples already buffered, through [`StaticWindow::try_extend`]: the first
/// refused refuses the window. An empty slice is an empty window, which
/// [`Eskf::initialize`](crate::Eskf::initialize) refuses as [`InitError::NoSamples`].
impl TryFrom<&[StaticSample]> for StaticWindow {
    type Error = SampleRefusal;

    fn try_from(samples: &[StaticSample]) -> Result<Self, SampleRefusal> {
        let mut window = Self::new();
        window.try_extend(samples.iter().copied())?;
        Ok(window)
    }
}

/// The white noise a still window measured on each sensor, as [`StaticWindow::noise`] reports
/// it: a floor under the noise the filter should be told, never the noise itself.
///
/// A floor because the window is the quietest the sensors will be in the air. A vehicle sitting
/// still sees no propeller wash on its static port and no dynamic pressure, and vibration only
/// what its motors make at idle; several corpus windows carry some, from props spinning on the
/// ground or aliased near the sample rate. In flight each of these grows. [`ImuNoise::default`]'s
/// white noise is ten times PX4's density for that reason and others its doc comment measures,
/// so a figure here copied into [`Config::imu`](crate::Config::imu) is the datasheet-grade `Q`
/// that default exists to avoid. What the figure is for: a default below it is wrong, and a
/// default far above it is a statement about the airframe rather than the sensor.
///
/// A density `N` is what [`ImuNoise`] takes: an increment over `Δt` carries `N √Δt` of noise,
/// and (16)–(21) add `N² Δt` to `Q`, so a sample's rates scatter by `N / √Δt`, twenty times `N`
/// at 400 Hz. Reading that scatter as the density is the mistake (8″) avoids.
///
/// Per axis, in body axes; [`ImuNoise`] takes the worst, which
/// [`worst_gyro_white`](Self::worst_gyro_white) and
/// [`worst_accel_white`](Self::worst_accel_white) give. The bias random walks and each source's
/// correlation time are not here: both need hours of data rather than seconds, an Allan
/// variance and a replay log's autocorrelation, and belong to the offline tool (#51). How well
/// a figure is known is [`MIN_READINGS`](Self::MIN_READINGS)'s and [`BLOCK`](Self::BLOCK)'s to
/// say, and the block length is the larger share.
///
/// [`ImuNoise`]: crate::ImuNoise
/// [`ImuNoise::default`]: crate::ImuNoise#impl-Default-for-ImuNoise
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct WindowNoise {
    /// Gyroscope white noise per body axis, rad s⁻¹ / √Hz: a floor under
    /// [`ImuNoise::gyro_white`](crate::ImuNoise::gyro_white).
    pub gyro_white: [f32; 3],
    /// Accelerometer white noise per body axis, m s⁻² / √Hz: a floor under
    /// [`ImuNoise::accel_white`](crate::ImuNoise::accel_white).
    pub accel_white: [f32; 3],
    /// The barometer readings' variance: a floor under the `R_m` a barometric altitude is
    /// fused with, the white part (24′) multiplies by its correlation factor. `None` under
    /// [`MIN_READINGS`](Self::MIN_READINGS) distinct readings, where the IMU's figures can
    /// still be taken and the window can still have set `α₀`, which takes two.
    ///
    /// A held reading counts once, as it does for `α₀`, and two genuine readings that agree
    /// are merged by the same test, so a barometer quantized coarsely enough to repeat itself
    /// reads a variance larger than its own.
    ///
    /// Fused as `R_m` it is too small, which is what calling it a floor claims: on five of the six
    /// real logs that report it, `nis_baro` reads past 1 ([evidence]).
    ///
    /// [evidence]: https://github.com/wboayue/fusion-nav/blob/main/DESIGN.md#windownoise
    pub baro: Option<AltitudeNoise>,
    /// The [`BLOCK`](Self::BLOCK)s the densities were taken over, the fewer of the two
    /// sensors'.
    pub blocks: u64,
    /// The distinct barometer readings `baro` was taken over.
    pub baro_readings: u64,
}

#[cfg(feature = "defmt")]
impl defmt::Format for WindowNoise {
    fn format(&self, f: defmt::Formatter<'_>) {
        let (g, a) = (self.gyro_white, self.accel_white);
        defmt::write!(
            f,
            "WindowNoise {{ gyro_white: ({=f32}, {=f32}, {=f32}), accel_white: ({=f32}, {=f32}, {=f32}), baro: {}, blocks: {=u64}, baro_readings: {=u64} }}",
            g[0],
            g[1],
            g[2],
            a[0],
            a[1],
            a[2],
            self.baro.map(AltitudeNoise::variance),
            self.blocks,
            self.baro_readings
        )
    }
}

impl WindowNoise {
    /// The fewest readings a sensor's figure is taken from, a barometer's readings or an IMU's
    /// [`BLOCK`](Self::BLOCK)s: under it, [`StaticWindow::noise`] reports none for that sensor.
    ///
    /// A standard deviation taken from `n` readings is known to about `1/√(2(n − 1))` of itself,
    /// so nine is ±25 % and a 2 s window's 40 blocks ±11 %, and a variance, the barometer's, to
    /// twice that. Fewer is the two-sample variance a window too short to measure anything
    /// would otherwise report as a measurement.
    ///
    /// The barometric reference `α₀` does not wait for it, and takes a variance from two readings:
    /// that variance starts an estimate (30′) goes on refining, where a figure reported here is
    /// final. Holding `α₀` to nine would leave the corpus no real vehicle whose short still start
    /// sets its own reference ([evidence]).
    ///
    /// [evidence]: https://github.com/wboayue/fusion-nav/blob/main/DESIGN.md#windownoise
    pub const MIN_READINGS: u64 = 9;

    /// The span an IMU's increments are summed over before their scatter is taken: the time
    /// scale its white-noise density is read at, `T` of (8″).
    ///
    /// Long enough to average out what a still airframe adds near the sample rate, and short enough
    /// that a 2 s window holds 40 blocks. Shorter still reads the aliasing; longer does not settle
    /// the figure either, which moves by up to 2.3× between 50 and 125 ms on the corpus: that is
    /// its real uncertainty, larger than the ±11 % of its 40 blocks ([evidence]). A time rather
    /// than a count, so the figure means the same at every IMU rate.
    ///
    /// [evidence]: https://github.com/wboayue/fusion-nav/blob/main/DESIGN.md#windownoise
    pub const BLOCK: Seconds = Seconds::from_secs(0.05);

    /// The gyroscope's worst axis, the one density [`ImuNoise::gyro_white`] takes.
    ///
    /// [`ImuNoise::gyro_white`]: crate::ImuNoise::gyro_white
    #[must_use]
    pub fn worst_gyro_white(&self) -> f32 {
        Vector3::from(self.gyro_white).max()
    }

    /// The accelerometer's worst axis, the one density [`ImuNoise::accel_white`] takes.
    ///
    /// [`ImuNoise::accel_white`]: crate::ImuNoise::accel_white
    #[must_use]
    pub fn worst_accel_white(&self) -> f32 {
        Vector3::from(self.accel_white).max()
    }
}

/// What the initialization window measures: the averages equations (5)–(7) level from, the
/// peaks [`at_rest`] judges, and the span [`classify`] measures.
///
/// One value rather than a function per quantity, because the state and its covariance
/// have to describe the *same* average. [`nominal_state`] levels from `force` and
/// [`attitude_sigmas`] bounds how well it levelled; two readings of the window would be two
/// definitions of `f̄`, free to drift apart while each still looked right on its own.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Measured {
    /// `f̄`, the averaged specific force (5) levels from. Every sample carries one.
    pub force: Acceleration<Body>,
    /// `ω̄`, the averaged angular rate (7) takes as the gyroscope bias. Every sample
    /// carries one.
    pub rate: AngularRate<Body>,
    /// `m̄`, the averaged magnetic field (6) takes a heading from, over the samples that
    /// carry one. `None` if none do, which is a vehicle with no magnetometer.
    pub field: Option<MagField<Body>>,
    /// `ā_n` of equation (5′); see [`Velocities`].
    pub inertial_accel: Option<Acceleration<Ned>>,
    /// The window's largest motion, which [`at_rest`] judges.
    pub peaks: Peaks,
    /// The time the window's samples integrated, [`StaticWindow::span`].
    pub span: Seconds,
    /// The last sample's time, where the filter's clock starts.
    pub end: Timestamp,
    /// The same averages over each half of the window, for [`window_drift`]. `None` for
    /// a window of one, which has no halves to disagree.
    pub halves: Option<Halves>,
    /// How well `f̄` itself is known across gravity: the variance, m² s⁻⁴ per horizontal
    /// axis, of the mean of the samples' specific force. Over `γ²` it is the part of (5)'s
    /// level error that is not the accelerometer bias, measured by the window's own scatter;
    /// see [`initial_covariance`]. `None` for a window of one, or one whose average is zero,
    /// which measure no scatter. `f64`, because the division by `γ²` waits for the `γ` the
    /// filter is configured with.
    pub level_scatter: Option<f64>,
}

/// The averages of equations (5)–(6) taken over each half of the window separately, so
/// that the attitude the first half yields can be compared with the second half's.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Halves {
    /// `f̄` over the first and the second half.
    force: [Acceleration<Body>; 2],
    /// `m̄` over each half, and `None` unless *both* halves carried a field: a heading
    /// the two can be compared on is the whole point, and one half alone yields nothing
    /// to compare.
    field: Option<[MagField<Body>; 2]>,
}

impl Halves {
    /// The averages of two parts of the window, and `None` unless both hold a sample.
    fn of(first: Block, second: Block) -> Option<Self> {
        (first.samples > 0 && second.samples > 0).then(|| Self {
            force: [
                Acceleration::from_vector(mean(first.force, first.samples)),
                Acceleration::from_vector(mean(second.force, second.samples)),
            ],
            field: (first.fields > 0 && second.fields > 0).then(|| {
                [
                    MagField::from_vector(mean(first.field, first.fields)),
                    MagField::from_vector(mean(second.field, second.fields)),
                ]
            }),
        })
    }
}

/// The window's largest motion: what [`at_rest`] judges and
/// [`Coarse::NotStationary`] reports.
///
/// The specific force is kept as its smallest and largest magnitude rather than as a
/// departure from gravity, because the window is folded before it meets the `γ` a
/// [`Config`] names; [`Peaks::deviation`] takes the departure then, and the largest
/// `|‖f‖ − γ|` over the samples is exactly the larger of the two ends' departures.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Peaks {
    /// Largest angular rate magnitude.
    pub gyro: RadiansPerSecond,
    /// Smallest and largest specific-force magnitude, m s⁻².
    least: f32,
    most: f32,
}

impl Peaks {
    /// No motion at all: what an empty window has seen.
    const NONE: Self = Self {
        gyro: RadiansPerSecond::from_rad_per_s(0.0),
        least: f32::INFINITY,
        most: f32::NEG_INFINITY,
    };

    fn push(&mut self, accel: Vector3<f32>, gyro: Vector3<f32>) {
        let norm = accel.norm();
        self.gyro = RadiansPerSecond::from_rad_per_s(self.gyro.as_rad_per_s().max(gyro.norm()));
        self.least = self.least.min(norm);
        self.most = self.most.max(norm);
    }

    /// Largest departure of the specific-force magnitude from `gravity`; zero for an empty
    /// window.
    pub(crate) fn deviation(&self, gravity: f32) -> MetersPerSecond2 {
        let deviation = (self.most - gravity).max(gravity - self.least).max(0.0);
        MetersPerSecond2::from_m_per_s2(deviation)
    }
}

impl Measured {
    /// The direction equation (5) called down, `−f̄ / ‖f̄‖`: the axis the tilt bound and
    /// the dip are both measured about.
    ///
    /// `None` for an average of zero, which is a stopped or disconnected accelerometer:
    /// that window has no vertical to measure anything about. [`level_from_accel`] levels
    /// it to zero rather than to NaN and [`coarse_sigmas`] keeps the same promise, falling
    /// back to bounds that do not need a direction rather than dividing by the norm.
    fn down(&self) -> Option<Vector3<f32>> {
        self.force
            .vector()
            .try_normalize(f32::MIN_POSITIVE)
            .map(|f| -f)
    }
}

/// How many blocks [`BlockSums`] keeps.
///
/// The split lands within half a block of the middle, and at least `BLOCKS / 2` blocks are
/// full, so the halves are within `n / BLOCKS` samples of equal: 37.5 % to 62.5 % of the
/// window at worst. A steady turn is still charged exactly: the halves' centres sit half the
/// window apart wherever the split falls.
///
/// Eight is measured against an exact split: the halves' disagreement moves by about 4° on the
/// coarse starts, and the outputs barely do, because it reaches only [`coarse_sigmas`] and only
/// where it is the largest bound ([evidence]).
///
/// [evidence]: https://github.com/wboayue/fusion-nav/blob/main/DESIGN.md#blocks
const BLOCKS: usize = 8;

/// The window's `Σ f` and `Σ m`, kept so that [`Halves`] can be taken from them in a window
/// whose length is not known until it ends.
///
/// The halves are the only statistic here a single pass cannot take exactly, because the
/// middle of a window moves as it grows. So the window is kept as up to [`BLOCKS`] blocks of
/// equal size, each summing its own samples; when the last fills, adjacent pairs merge and
/// the size doubles. [`split`](Self::split) cuts at the block boundary nearest the middle,
/// within the bound [`BLOCKS`] states. The whole window's sums are the two halves' total
/// rather than a running total kept beside them, so the two cannot disagree.
#[derive(Clone, Copy, Debug)]
struct BlockSums {
    blocks: [Block; BLOCKS],
    /// Samples a block holds before the next one starts.
    size: u64,
    /// The block now filling.
    current: usize,
}

/// One block of [`BlockSums`].
#[derive(Clone, Copy, Debug)]
struct Block {
    force: Vector3<f64>,
    samples: u64,
    field: Vector3<f64>,
    fields: u64,
}

impl Block {
    const EMPTY: Self = Self {
        force: Vector3::new(0.0, 0.0, 0.0),
        samples: 0,
        field: Vector3::new(0.0, 0.0, 0.0),
        fields: 0,
    };

    fn total(blocks: &[Self]) -> Self {
        blocks.iter().fold(Self::EMPTY, |sum, b| sum.merged(*b))
    }

    fn merged(self, other: Self) -> Self {
        Self {
            force: self.force + other.force,
            samples: self.samples + other.samples,
            field: self.field + other.field,
            fields: self.fields + other.fields,
        }
    }
}

impl BlockSums {
    const fn new() -> Self {
        Self {
            blocks: [Block::EMPTY; BLOCKS],
            size: 1,
            current: 0,
        }
    }

    fn push(&mut self, force: Vector3<f64>, field: Option<Vector3<f64>>) {
        if self
            .blocks
            .get(self.current)
            .is_some_and(|b| b.samples >= self.size)
        {
            if self.current + 1 < BLOCKS {
                self.current += 1;
            } else {
                self.halve();
            }
        }
        if let Some(block) = self.blocks.get_mut(self.current) {
            block.force += force;
            block.samples += 1;
            if let Some(field) = field {
                block.field += field;
                block.fields += 1;
            }
        }
    }

    /// Merge adjacent pairs, which halves the blocks in use and doubles their size, and start
    /// filling the first block the merge left empty.
    fn halve(&mut self) {
        // In place, since pair `i` is read from `2i` and `2i + 1`, never from below `i`: a copy of
        // the blocks would be a stack temporary.
        for pair in 0..BLOCKS / 2 {
            if let (Some(&first), Some(&second)) =
                (self.blocks.get(2 * pair), self.blocks.get(2 * pair + 1))
                && let Some(merged) = self.blocks.get_mut(pair)
            {
                *merged = first.merged(second);
            }
        }
        for empty in self.blocks.iter_mut().skip(BLOCKS / 2) {
            *empty = Block::EMPTY;
        }
        self.size *= 2;
        self.current = BLOCKS / 2;
    }

    /// The window's two halves, split at the block boundary nearest the middle, the earlier
    /// of two equally near: for a window of `n` blocks of one sample, that is `n / 2`.
    fn split(&self) -> (Block, Block) {
        let blocks = self.blocks.get(..=self.current).unwrap_or_default();
        let total: u64 = blocks.iter().map(|b| b.samples).sum();
        // Twice the distance from the middle, in samples, so an odd window stays integral.
        let (mut split, mut nearest, mut before) = (0, total, 0u64);
        for (index, block) in blocks.iter().enumerate() {
            before += block.samples;
            let distance = (2 * before).abs_diff(total);
            if distance < nearest {
                (split, nearest) = (index + 1, distance);
            }
        }
        let (first, second) = blocks.split_at(split.min(blocks.len()));
        (Block::total(first), Block::total(second))
    }
}

/// The first and last GNSS velocity in the window, for `ā_n` of equation (5′), the term
/// in-motion levelling subtracts from the averaged specific force.
///
/// Endpoints only; the velocities in between are not differenced at all. The mean of a
/// derivative *is* its endpoint difference over the span, and the mean is what is wanted,
/// because (5) levels the averaged specific force and the term to subtract is therefore
/// the averaged acceleration. Differencing consecutive samples and averaging those gives
/// the same number with the intermediate noise added back — which matters here, since
/// this is the noisiest part of in-motion levelling: a receiver's velocity error divided
/// by a span, and a 1 Hz receiver over a 2 s window divides it by very little.
///
/// The span is the time the samples after the first velocity's, up to the last's,
/// integrated, so it is only as honest as the dating of the window; see
/// [`StaticSample::velocity`].
#[derive(Clone, Copy, Debug, Default)]
struct Velocities {
    first: Option<Velocity<Ned>>,
    last: Option<Velocity<Ned>>,
    /// Integrated since the first velocity's sample.
    since_first: f64,
    /// `since_first` as of the last velocity's sample.
    span: f64,
}

impl Velocities {
    fn push(&mut self, velocity: Option<Velocity<Ned>>, interval: f64) {
        if self.first.is_some() {
            self.since_first += interval;
        }
        if let Some(velocity) = velocity {
            if self.first.is_none() {
                self.first = Some(velocity);
            } else {
                self.last = Some(velocity);
                self.span = self.since_first;
            }
        }
    }

    /// `ā_n`. `None` unless two samples separated in time carry a velocity.
    fn inertial_acceleration(&self) -> Option<Acceleration<Ned>> {
        let (first, last) = (self.first?, self.last?);
        let span = self.span as f32;
        // `push` refuses an interval under a microsecond, so two velocities are at least that
        // far apart and this never refuses one. It stays because this is the only division
        // here, and a difference over zero seconds is an infinity, not an acceleration.
        (span > 0.0).then(|| Acceleration::from_vector((last.vector() - first.vector()) / span))
    }
}

/// One sensor's white-noise density per axis, equation (8″): the increments it integrated,
/// summed into blocks of [`WindowNoise::BLOCK`], and their scatter weighted by each block's
/// length.
///
/// Blocks rather than samples because a still airframe's samples are not white, and one sample's
/// scatter then measures the noise at the sample rate rather than the density that integrates into
/// attitude and velocity error. On the corpus's windows the lag-one autocorrelation runs from −0.98
/// to +0.96 across the axes, and one sample's scatter misreads the blocks' figure by up to 6× high
/// and 2.9× low; correcting it by the lag-one `(1 + ρ)/(1 − ρ)` of (24′) still misses by up to 3.3×
/// ([evidence]).
///
/// About zero rather than about the first increment, unlike [`BaroReadings`]: the loss is at
/// most `J ε γ² T / N²` of the scatter, under 10⁻⁷ in `f64` even at a datasheet
/// accelerometer's 7 × 10⁻⁴ m s⁻²/√Hz, where the barometer's metres above sea level cost
/// 5 × 10⁻⁴. A constant in the rate, the bias, gravity, the Earth's rotation, cancels in the
/// scatter.
///
/// [evidence]: https://github.com/wboayue/fusion-nav/blob/main/DESIGN.md#windownoise
#[derive(Clone, Copy, Debug)]
struct Density {
    /// The increment and interval of the block filling.
    open: Vector3<f64>,
    open_interval: f64,
    /// `J`, the blocks closed.
    blocks: u64,
    /// `Σ T` over the closed blocks.
    interval: f64,
    /// `Σ B`.
    sum: Vector3<f64>,
    /// `Σ B² / T`.
    squares: Vector3<f64>,
}

impl Density {
    const EMPTY: Self = Self {
        open: Vector3::new(0.0, 0.0, 0.0),
        open_interval: 0.0,
        blocks: 0,
        interval: 0.0,
        sum: Vector3::new(0.0, 0.0, 0.0),
        squares: Vector3::new(0.0, 0.0, 0.0),
    };

    /// Add a sample's increment over its interval, closing the block at the sample boundary
    /// nearest [`WindowNoise::BLOCK`], the earlier of two equally near: a boundary exactly half
    /// an interval short of it closes. Whether a tie is exact depends on how the intervals
    /// round: exact 20 ms steps would close at 40 ms, but `0.02` in `f32` sits just under, so
    /// 20 ms samples close 60 ms blocks, while 25 ms ones close 50 ms blocks. The weighting
    /// takes either as it comes. The last block, still filling, is left out of the figure.
    fn push(&mut self, increment: Vector3<f64>, interval: f64) {
        self.open += increment;
        self.open_interval += interval;
        if self.open_interval + interval / 2.0 >= f64::from(WindowNoise::BLOCK.as_secs()) {
            let (block, length) = (self.open, self.open_interval);
            self.blocks += 1;
            self.interval += length;
            self.sum += block;
            // One division rather than three: each is a library call on a core with no FPU.
            self.squares += block.component_mul(&block) * (1.0 / length);
            (self.open, self.open_interval) = (Vector3::zeros(), 0.0);
        }
    }

    /// `N` per axis, and `None` under [`WindowNoise::MIN_READINGS`] blocks.
    fn density(&self) -> Option<Vector3<f32>> {
        if self.blocks < WindowNoise::MIN_READINGS {
            return None;
        }
        let j = self.blocks as f64;
        let axis = |squares: f64, sum: f64| {
            let variance = scatter(squares, sum, self.interval) / (j - 1.0);
            ComplexField::sqrt(variance.max(0.0)) as f32
        };
        Some(Vector3::new(
            axis(self.squares.x, self.sum.x),
            axis(self.squares.y, self.sum.y),
            axis(self.squares.z, self.sum.z),
        ))
    }
}

/// The barometer readings in the window, for `α₀` of equation (30) and the variance of that
/// mean, `P_bb` of (30′).
///
/// A reading is a value that differs from the previous sample's, because
/// [`StaticSample::baro`] may be held across IMU epochs: a 20 Hz barometer held on a 200 Hz
/// IMU would otherwise count each reading ten times and report a variance ten times too
/// small, and one reading held across the whole window would pass for many with no scatter.
/// Two genuine readings that happen to agree are merged by the same test, which can only
/// make the variance larger.
///
/// The variance is the window's own: the sample variance of the readings over their count,
/// the standard error of a mean. It is measured rather than asked for, because the window
/// is the one moment the barometer's scatter can be read with the vehicle known to be still
/// (GOALS.md differentiator 7), and a reading's `R` on the corpus is a constant the
/// converter substitutes. It assumes the readings independent; a correlated barometer
/// averages down more slowly than this says.
///
/// Summed about the first reading, in `f64`. The textbook one-pass form, `Σa² − (Σa)²/n`,
/// differences two squares of metres above mean sea level down to a scatter of centimetres,
/// and at 10 km over 400 readings 3 cm apart it is 5 × 10⁻⁴ out
/// (`the_barometer_scatter_is_the_two_pass_one_at_any_altitude`); about the first reading
/// the squares are of the scatter itself. Welford's running mean avoids the same loss at a
/// division per reading.
#[derive(Clone, Copy, Debug, Default)]
struct BaroReadings {
    /// The previous sample's reading, fresh or held.
    previous: Option<Altitude>,
    count: u64,
    /// The first reading, which the sums below are taken about.
    first: f64,
    /// `Σ (a − a₁)`.
    sum: f64,
    /// `Σ (a − a₁)²`.
    squares: f64,
}

impl BaroReadings {
    fn push(&mut self, baro: Option<Altitude>) {
        let fresh = baro.filter(|altitude| Some(*altitude) != self.previous);
        self.previous = baro;
        if let Some(altitude) = fresh {
            let a = f64::from(altitude.as_meters());
            if self.count == 0 {
                self.first = a;
            }
            let d = a - self.first;
            self.count += 1;
            self.sum += d;
            self.squares += d * d;
        }
    }

    /// The mean and its variance, and `None` for fewer than two readings. One reading is
    /// refused rather than given a variance: it has no scatter to measure, and a reference
    /// whose error is invented is what (30′) exists to stop.
    fn reference(&self) -> Option<(Altitude, f32)> {
        if self.count < 2 {
            return None;
        }
        let n = self.count as f64;
        let mean = self.first + self.sum / n;
        Some((
            Altitude::from_meters(mean as f32),
            (self.variance() / n) as f32,
        ))
    }

    /// The readings' sample variance, the floor [`WindowNoise::baro`] reports, and `None`
    /// under [`WindowNoise::MIN_READINGS`].
    fn noise(&self) -> Option<AltitudeNoise> {
        (self.count >= WindowNoise::MIN_READINGS)
            .then(|| AltitudeNoise::from_variance(self.variance() as f32))
    }

    /// The readings' sample variance, for two or more.
    fn variance(&self) -> f64 {
        let n = self.count as f64;
        scatter(self.squares, self.sum, n) / (n - 1.0)
    }
}

/// `Σ w x² − (Σ w x)² / Σ w`, the weighted sum of squared deviations from the mean, for
/// [`Density`] and [`BaroReadings`], each saying why its sums hold the precision.
fn scatter(squares: f64, sum: f64, weight: f64) -> f64 {
    squares - sum * sum / weight
}

/// [`Measured::level_scatter`], from the window's `Σ f` and `Σ f fᵀ`.
///
/// The scatter across `f̄` over the sample count is how far the average could sit from the
/// window's true mean force: vibration, a vehicle rocking on its gear, sensor noise. Divided
/// by `γ²` it is a tilt, and it is what `P₀` keeps independent of the bias. It assumes the
/// samples independent, so vibration slower than the sample rate is undercounted, and it is
/// a floor rather than the whole independent share for that reason.
fn level_scatter(outer: Matrix3<f64>, sum: Vector3<f64>, count: u64) -> Option<f64> {
    if count < 2 {
        return None;
    }
    let n = count as f64;
    let mean = sum / n;
    let down = mean.try_normalize(f64::MIN_POSITIVE)?;
    // Sums near `γ²` per sample, differenced down to a scatter many orders smaller: the
    // subtraction that needs f64.
    let scatter = (outer - mean * mean.transpose() * n) / (n - 1.0);
    let across = scatter.trace() - down.dot(&(scatter * down));
    Some((across / 2.0).max(0.0) / n)
}

/// Widen a measurement for accumulation; see [`StaticWindow`].
fn widen(measurement: Vector3<f32>) -> Vector3<f64> {
    Vector3::new(
        f64::from(measurement.x),
        f64::from(measurement.y),
        f64::from(measurement.z),
    )
}

/// An accumulated sum as its mean, and zero for nothing accumulated — which
/// [`StaticWindow::measured`] refuses before any caller sees it.
fn mean(sum: Vector3<f64>, count: u64) -> Vector3<f32> {
    if count == 0 {
        return Vector3::zeros();
    }
    let n = count as f64;
    Vector3::new((sum.x / n) as f32, (sum.y / n) as f32, (sum.z / n) as f32)
}

/// What [`Eskf::initialize`](crate::Eskf::initialize) achieved.
///
/// A window that is short or moving is not a failure — it is a coarser start, and the
/// filter says which it got rather than refusing to run. See
/// [`Status::Aligning`](crate::Status::Aligning).
#[must_use = "whether the filter aligned or only started coarsely changes what the estimate is worth"]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Alignment {
    /// The window was long enough and genuinely still: tilt from averaged gravity, gyro
    /// bias from the averaged rate, and the covariance
    /// [`Initialization`](crate::Initialization) describes. Equations (5)–(8).
    Static,
    /// The window was usable but not a static interval, so the covariance says what that
    /// window supports rather than what a still one would. The filter runs and reports
    /// [`Status::Aligning`](crate::Status::Aligning) until tilt and heading uncertainty
    /// first come within [`ALIGNED_TILT`](crate::ALIGNED_TILT) and
    /// [`ALIGNED_HEADING`](crate::ALIGNED_HEADING).
    Coarse(Coarse),
    /// The state came from [`Eskf::initialize_from`](crate::Eskf::initialize_from) rather
    /// than from a window. Whether it counts as aligned is a question for the covariance
    /// the caller supplied, not for this value.
    Seeded,
}

impl Alignment {
    /// Whether this was a full static alignment.
    pub const fn is_static(self) -> bool {
        matches!(self, Self::Static)
    }
}

impl core::fmt::Display for Alignment {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Static => f.write_str("static"),
            Self::Coarse(coarse) => write!(f, "coarse, {coarse}"),
            Self::Seeded => f.write_str("seeded"),
        }
    }
}

/// Why alignment was coarse rather than static.
///
/// Carries what was measured, so "not stationary" is diagnosable rather than a bare
/// verdict: an integrator tuning
/// [`Initialization`](crate::Initialization) needs to know by how much.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Coarse {
    /// The vehicle held still, but the window spans less than
    /// [`Initialization::min_duration`](crate::Initialization::min_duration).
    ///
    /// Still is measured before length, so a short window that moved is
    /// [`NotStationary`](Self::NotStationary) instead: this variant is a parked vehicle
    /// that has not been parked for long, and the filter trusts its position, velocity,
    /// barometric reference and heading as it would a static window's.
    WindowTooShort {
        /// Duration the configuration requires.
        required: Seconds,
        /// Duration the window covers: the time its samples integrated.
        provided: Seconds,
    },
    /// The vehicle was moving: angular rate or specific force left the tolerance
    /// [`Initialization`](crate::Initialization) allows.
    NotStationary {
        /// Largest angular rate magnitude in the window.
        peak_gyro: RadiansPerSecond,
        /// Largest departure of the specific-force magnitude from gravity.
        peak_accel_deviation: MetersPerSecond2,
        /// How long the window spanned. With `peak_gyro`, this bounds how far the
        /// vehicle turned while its gravity vector was being averaged, which is the
        /// other way a moving window spoils tilt.
        span: Seconds,
        /// Mean navigation-frame acceleration over the window, `ā_n` of equation (5′),
        /// differenced from [`StaticSample::velocity`]. `None` when no two samples
        /// carried one.
        ///
        /// What separates a vehicle that is accelerating from an accelerometer that is
        /// lying: `peak_accel_deviation` measures specific force that is not gravity,
        /// and this is the part of it GNSS can account for. A launch off a moving deck
        /// reads both, and only one of them spoils tilt.
        ///
        /// **Stub.** Reported, not subtracted (equation (5′), #59), so it narrows nothing:
        /// [`attitude_sigmas`] bounds tilt by how far the window's *averaged* specific force is
        /// from gravity, and a real `ā_n` is part of what puts it there.
        inertial_accel: Option<Acceleration<Ned>>,
    },
}

/// What was measured, in the units [`Initialization`](crate::Initialization) is tuned in.
/// `inertial_accel` is left out while (5′) only reports it.
impl Coarse {
    /// The coarse start a window that moved gives, reporting what it measured.
    pub(crate) fn not_stationary(measured: &Measured, gravity: f32) -> Self {
        Self::NotStationary {
            peak_gyro: measured.peaks.gyro,
            peak_accel_deviation: measured.peaks.deviation(gravity),
            span: measured.span,
            inertial_accel: measured.inertial_accel,
        }
    }
}

impl core::fmt::Display for Coarse {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let three = |value| Fixed::new(value, Decimals::Three);
        match *self {
            Self::WindowTooShort { required, provided } => write!(
                f,
                "window of {} s, {} s required",
                three(provided.as_secs()),
                three(required.as_secs())
            ),
            Self::NotStationary {
                peak_gyro,
                peak_accel_deviation,
                span,
                ..
            } => write!(
                f,
                "moving: peak rate {} rad/s, specific force {} m/s² off gravity, over {} s",
                three(peak_gyro.as_rad_per_s()),
                three(peak_accel_deviation.as_m_per_s2()),
                three(span.as_secs())
            ),
        }
    }
}

/// Why initialization could not run at all.
///
/// Distinct from [`Coarse`]: these are inputs the filter can make nothing of, not starts
/// of lower quality. Every one of them is the caller handing over something broken, which
/// is why they are checked rather than trusted — a seed in particular crosses a boundary
/// the filter does not control, arriving from another estimator or from storage that may
/// be stale or corrupt.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum InitError {
    /// The window held no samples, so there is nothing to align from.
    NoSamples,
    /// A sample's integration interval was under a microsecond, so it covers no span of time
    /// to measure the window over or divide its increments by. One that is not a number is
    /// [`NotFinite`](Self::NotFinite).
    InvalidInterval {
        /// The interval offered.
        interval: Seconds,
    },
    /// A sample was timed no later than the one before it, so the window's timestamps say
    /// nothing about when it ended, which is where the filter's clock starts. As
    /// [`Propagation::InvalidStep`](crate::Propagation::InvalidStep) for a step.
    InvalidStep {
        /// The time from the previous sample to this one.
        dt: Seconds,
    },
    /// A measurement, state, or covariance carried a value that is not finite.
    NotFinite,
    /// A seed covariance had a variance on its diagonal that no prior has: below the floor
    /// of equation (42′), which includes zero and negative.
    ///
    /// Zero is the one that arrives in practice, from a warm start deserialized out of
    /// storage that never populated the diagonal. It reads as a tight prior and is not
    /// one: it claims perfect knowledge, so the gain `K = P Hᵀ S⁻¹` of equation (25) is
    /// zero for that quantity and no measurement ever corrects it, while
    /// [`Validity`](crate::Validity) compares the zero variance against
    /// [`Config::accuracy`](crate::Config::accuracy) and reports the quantity good from
    /// the first read. A negative variance claims better than perfect.
    ///
    /// The bar is the floor rather than zero because zero is only the tidiest member of
    /// that class: the same accident with an exponent left in it arrives at 1e-30 and is
    /// indistinguishable in f32 — the gain is still zero and the quantity still reads good
    /// from the first epoch. A seed is the one path that writes a covariance in whole, so
    /// it is refused here where a `reset_*_to` writing one block is floored instead. See
    /// [`Fusion::InvalidNoise`](crate::Fusion::InvalidNoise) for the bar every `fuse_*`
    /// puts on `R`, which is still strict positivity.
    ///
    /// Symmetry is imposed by (42) rather than checked, and positive-definiteness is not
    /// checked: that is a factorization on the caller's data, not a guard.
    InvalidVariance,
}

impl core::fmt::Display for InitError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::NoSamples => write!(f, "initialization window held no samples"),
            Self::InvalidInterval { interval } => {
                let interval = Fixed::new(interval.as_secs(), Decimals::Three);
                write!(f, "initialization interval of {interval} s is not usable")
            }
            Self::InvalidStep { dt } => {
                let dt = Fixed::new(dt.as_secs(), Decimals::Three);
                write!(f, "initialization window stepped {dt} s, not forward")
            }
            Self::NotFinite => write!(f, "initialization input was not finite"),
            Self::InvalidVariance => {
                write!(f, "seed covariance had a variance below the floor of (42′)")
            }
        }
    }
}

impl core::error::Error for InitError {}

/// Why [`StaticWindow::push`] refused a sample.
///
/// The three of [`InitError`]'s variants a sample can cause, so a caller handling a refused
/// sample matches on what a sample can be wrong about and not on an empty window or a bad
/// seed. It converts into [`InitError`], so `?` carries it out of a function that initializes
/// the filter.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SampleRefusal {
    /// The sample carried a value that is not finite: [`InitError::NotFinite`].
    NotFinite,
    /// Its integration interval was under a microsecond: [`InitError::InvalidInterval`].
    InvalidInterval {
        /// The interval offered.
        interval: Seconds,
    },
    /// It was timed no later than the sample before it: [`InitError::InvalidStep`].
    InvalidStep {
        /// The time from the previous sample to this one.
        dt: Seconds,
    },
}

impl From<SampleRefusal> for InitError {
    fn from(refusal: SampleRefusal) -> Self {
        match refusal {
            SampleRefusal::NotFinite => Self::NotFinite,
            SampleRefusal::InvalidInterval { interval } => Self::InvalidInterval { interval },
            SampleRefusal::InvalidStep { dt } => Self::InvalidStep { dt },
        }
    }
}

impl core::fmt::Display for SampleRefusal {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        InitError::from(*self).fmt(f)
    }
}

impl core::error::Error for SampleRefusal {}

/// Classify a measured window as a static or a coarse start.
///
/// The test behind [`Eskf::alignment_of`](crate::Eskf::alignment_of): still enough, then
/// long enough, against the tolerances in `init`. Total, because [`StaticWindow::push`] has
/// refused every sample that can be refused and [`StaticWindow::measured`] an empty window.
///
/// Motion is tested first so that each coarse variant answers one question. A window that
/// moved is [`Coarse::NotStationary`] whatever its length, and its `span` says how long it
/// was; a short one is [`Coarse::WindowTooShort`] only if it was still. Tested the other
/// way round, a short window said nothing about motion, and a caller holding a still
/// window that had not yet reached `min_duration` could not tell from the outcome that it
/// was still — which is what deciding to initialize at the onset of motion asks.
pub(crate) fn classify(measured: &Measured, init: &Initialization, gravity: f32) -> Alignment {
    if !at_rest(measured.peaks, init, gravity) {
        return Alignment::Coarse(Coarse::not_stationary(measured, gravity));
    }
    if !long_enough(measured.span, init) {
        return Alignment::Coarse(Coarse::WindowTooShort {
            required: init.min_duration,
            provided: measured.span,
        });
    }
    Alignment::Static
}

/// Whether a window spanning `span` is long enough to align from: the half of [`classify`]
/// that [`StaticWindow::is_long_enough`] asks too, so a collection loop stops on the window
/// `classify` calls long enough and no other.
fn long_enough(span: Seconds, init: &Initialization) -> bool {
    span >= init.min_duration
}

/// The nominal state a window yields. Equation (7), from the attitude of (5)–(6).
///
/// `at_rest` is [`at_rest`]'s verdict on the window, and it gates the gyroscope bias
/// alone. (7) takes `β̂_g,0 = ω̄` because stillness is what makes the bias observable; a
/// window that was moving offers the vehicle's own rotation under the same name, and
/// seeding that would subtract a turn rate from every later measurement as though it
/// were a sensor error. Neither production estimator averages at all — ArduPilot zeroes
/// the bias at bootstrap (`libraries/AP_NavEKF3/AP_NavEKF3_core.cpp:546`, `368dc0c4`),
/// and PX4 refuses to initialize outside 0.8–1.2 g and 15°/s
/// (`src/modules/ekf2/EKF/ekf.cpp:213-227`, `c4e4ef98`) — so averaging is this crate's,
/// and it is worth taking only where their precondition holds.
///
/// Every value read here is finite and the window is non-empty: [`StaticWindow`] refuses
/// both before any of this is reached.
pub(crate) fn nominal_state(measured: &Measured, declination: Radians, at_rest: bool) -> State {
    let (roll, pitch) = level_from_accel(measured.force);
    // Stillness observes tilt and never the rotation about it, so a window with no
    // magnetometer anywhere in it keeps ψ₀ = 0 — a stated direction rather than a
    // measured one, which is what `Unestablished::heading` records.
    let yaw = measured.field.map_or(Radians::ZERO, |field| {
        heading_from_mag(field, roll, pitch, declination)
    });

    State {
        // q_ZYX(ψ₀, θ₀, φ₀) of (7): `from_euler_angles` composes Rz(ψ) Ry(θ) Rx(φ), the
        // sequence (6) levels with and `Attitude::euler_angles` reads back.
        attitude: Attitude::from_quaternion(UnitQuaternion::from_euler_angles(
            roll.as_radians(),
            pitch.as_radians(),
            yaw.as_radians(),
        )),
        position: Position::zero(),
        velocity: Velocity::zero(),
        // β̂_a,0 = 0. At rest an accelerometer bias is indistinguishable from a tilt —
        // it leans the measured gravity vector and (5) has already read that lean as
        // attitude — so there is nothing left for this to hold.
        accel_bias: Acceleration::zero(),
        gyro_bias: if at_rest {
            measured.rate
        } else {
            AngularRate::zero()
        },
        // `status` and `validity` are inert in the stored state; `Eskf::state`
        // overwrites both on every read.
        ..State::default()
    }
}

/// Roll and pitch from the averaged specific force. Equation (5).
///
/// The signs are the down-positive convention's: a level, stationary accelerometer reads
/// `f = [0, 0, -γ]ᵀ`, so `-f_z` is `+γ`, `-f_y` is zero, and both angles come out zero.
///
/// Written with `atan2` rather than the `asin` on a normalized vector ArduPilot takes
/// (`libraries/AP_NavEKF3/AP_NavEKF3_core.cpp:527-536`, `368dc0c4`). The two agree; this
/// form never divides by `‖f‖`, so it needs neither the normalization nor the
/// `length() > 0.001` guard standing in front of it there, and a stopped or
/// disconnected accelerometer reading zero levels to zero rather than to NaN. PX4 builds
/// the same tilt differently again, as the shortest rotation carrying `f` onto `-e₃`
/// (`src/modules/ekf2/EKF/ekf.cpp:224`, `c4e4ef98`), which is this attitude with the yaw
/// left at zero; (6) needs the angles themselves.
pub(crate) fn level_from_accel(specific_force: Acceleration<Body>) -> (Radians, Radians) {
    let f = specific_force.vector();
    let roll = RealField::atan2(-f.y, -f.z);
    let pitch = RealField::atan2(f.x, ComplexField::sqrt(f.y * f.y + f.z * f.z));
    (Radians::from_radians(roll), Radians::from_radians(pitch))
}

/// True heading from the averaged magnetic field, levelled by (5)'s roll and pitch.
/// Equation (6).
///
/// The levelling is what makes this a heading rather than a projection. `R₀` is (7)'s
/// attitude with the yaw left out, so `m̃` sits in a frame differing from NED by the yaw
/// alone. Reading `atan2(m_y, m_x)` off the body field instead is correct only for a
/// vehicle already level and wrong by about `tan(dip)` times the tilt everywhere else:
/// at the 1.107 rad of dip the corpus carries, that factor is 1.96, so a 10° roll is
/// worth 19° of heading. A test at zero tilt cannot tell the two apart.
///
/// `declination` is east-positive, and adding it is what turns magnetic heading into
/// true.
pub(crate) fn heading_from_mag(
    field: MagField<Body>,
    roll: Radians,
    pitch: Radians,
    declination: Radians,
) -> Radians {
    // R₀ = R_y(θ₀) R_x(φ₀), which is the ZYX composition of (7) with its yaw set to zero.
    let levelled =
        Rotation3::from_euler_angles(roll.as_radians(), pitch.as_radians(), 0.0) * field.vector();
    // Wrapped because `D_m − atan2(·)` reaches π + |D_m|, and (7) is read back as Euler
    // angles that a test compares against a heading in range.
    Radians::from_radians(wrap_pi(
        declination.as_radians() - RealField::atan2(levelled.y, levelled.x),
    ))
}

/// What the window's two halves disagree about: the tilt between the attitudes they
/// yield, and the heading between them. The drift terms of equation (8′).
///
/// (5)–(6) commit one attitude for the whole window, and the filter propagates from the
/// window's **end**. A window whose halves disagree cannot vouch for either, and the
/// disagreement is how far the attitude moved while it was being averaged — measured on
/// the very vectors being averaged, by the very equations that average them.
///
/// For a turn at a steady rate it is the error exactly: the whole-window mean lands
/// mid-window, the state starts half a turn later at the end, and the two half-means sit
/// that same half-turn apart. For a vehicle that swings out and comes back it is the
/// excursion, which is what the net rotation `ω̄ · T` cannot see — that cancels to nothing
/// and charges nothing, while the attitude the window commits is the middle of an arc the
/// vehicle has already left.
///
/// Vibration averages out of each half, which is what keeps this off a vehicle sitting
/// with its props spinning: `2c42096b`'s 0.80 s window disagrees by 0.18° of tilt and 0.09°
/// of heading, both under the configured priors, where integrating `‖ω_⊥‖` over the same
/// window accumulates 0.66° — the peak's mistake in another coat, since a path length
/// does not average either.
///
/// The heading is `None` unless both halves carried a field. Declination cancels in the
/// difference, so none is applied.
fn window_drift(halves: &Halves) -> (f32, Option<f32>) {
    let [first, second] = halves.force;
    let tilt = angle_between(first.vector(), second.vector());
    let heading = halves.field.map(|[first_field, second_field]| {
        let of = |force: Acceleration<Body>, field| {
            let (roll, pitch) = level_from_accel(force);
            heading_from_mag(field, roll, pitch, Radians::ZERO)
        };
        wrap_pi(of(second, second_field).as_radians() - of(first, first_field).as_radians()).abs()
    });
    (tilt, heading)
}

/// Angle between two vectors, and zero where either has no direction — the same answer
/// [`Measured::down`] gives for an average of zero, and for the same reason.
fn angle_between(first: Vector3<f32>, second: Vector3<f32>) -> f32 {
    let norms = first.norm() * second.norm();
    if norms <= 0.0 {
        return 0.0;
    }
    // Clamped because a dot product of parallel vectors overshoots 1 in f32, and `acos`
    // of 1.000001 is NaN.
    ComplexField::acos((first.dot(&second) / norms).clamp(-1.0, 1.0))
}

/// How much a tilt error leaks into the heading of (6): `tan δ` of equation (8′), measured
/// off the window's own field rather than configured.
///
/// (6) levels `m̄` and takes the `atan2` of what is left horizontal, so an error in the
/// tilt it levelled by tips the field and turns that horizontal part. The leak is the
/// ratio of the field's vertical component to its horizontal one, which is `tan(dip)`:
/// 1.96 at the 1.107 rad of dip [`heading_from_mag`] cites, and 1.22 on the window
/// `2c42096b` starts from. Both components are taken about `down`, the direction (5)
/// levelled to, so no Euler angles enter and the ratio does not depend on the frame they
/// would be read in.
///
/// `None` where the horizontal part is zero: a field pointing straight down observes no
/// heading at any tilt, so there is no error to scale rather than an infinite one.
///
/// Shared with [`observation::mag`](crate::observation::mag), which needs the same `tan δ`
/// for the levelling variance of (36′), and the direction the field lies in as well: only
/// tilt about that horizontal direction leaks. The ratio is between the field and a
/// direction, so it is the same number in any frame the two are expressed in together:
/// the window passes the body-frame gravity direction, and the update passes the
/// body-frame navigation down axis.
pub(crate) fn heading_sensitivity(
    field: MagField<Body>,
    down: Vector3<f32>,
) -> Option<HeadingSensitivity> {
    let field = field.vector();
    let vertical = field.dot(&down);
    let horizontal = field - down * vertical;
    let norm = horizontal.norm();
    (norm > 0.0).then(|| HeadingSensitivity {
        tan_dip: vertical.abs() / norm,
        horizontal: horizontal / norm,
    })
}

/// What [`heading_sensitivity`] measures of a field about a down axis.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct HeadingSensitivity {
    /// `tan δ`, the field's vertical component over its horizontal one.
    pub(crate) tan_dip: f32,
    /// `f̂`, the unit horizontal direction of the field, in the frame `down` was given in.
    pub(crate) horizontal: Vector3<f32>,
}

/// Initial tilt and yaw standard deviations for an alignment: the attitude block of
/// equation (8).
///
/// A static start gets the configured figures. A coarse one gets what its own window
/// supports, equation (8′); see [`coarse_sigmas`].
///
/// `gyro_bias` is the one [`nominal_state`] committed from the same window, and the
/// coarse bound reads the window's rotation net of it. Passed rather than re-derived so
/// that the two cannot disagree about what the average held.
pub(crate) fn attitude_sigmas(
    init: &Initialization,
    alignment: Alignment,
    measured: &Measured,
    gyro_bias: AngularRate<Body>,
    gravity: f32,
) -> (Radians, Radians) {
    match alignment {
        Alignment::Static | Alignment::Seeded => (init.sigma_tilt, init.sigma_yaw),
        Alignment::Coarse(_) => coarse_sigmas(init, measured, gyro_bias, gravity),
    }
}

/// What a window that is not a static interval supports. Equation (8′): the widest of the
/// configured tilt, how far the window's average is from gravity, how far the vehicle
/// turned across gravity, and how far its two halves disagree — with a heading the dip
/// scales that tilt into, plus the turn about gravity and the halves' own disagreement.
///
/// Equations (5)–(6) level the *averaged* specific force, so what bounds the attitude
/// they yield is how far that average is from what a still vehicle reads — not how far
/// the worst sample in the window was. The two differed by two orders of magnitude on the
/// moving window `2c42096b` offered at `PATIENCE` (#77): its peak `|f| − γ` was 5.46 m/s²
/// (31.9°) on a vehicle vibrating in place, while the mean vector sat 0.06 m/s² (0.33°)
/// off gravity and levelled to roll 0.42° and pitch −0.89°, under a degree off plumb.
/// Vibration averages out; a peak does not know that.
///
/// A prior that wide is not free. [`Status::Aligning`](crate::Status::Aligning) outranks
/// `Degraded`, so a start that cannot resolve masks every aiding transition behind it: on
/// that log the peak's 0.557 rad of tilt and full-circle yaw are worth 888 transitions
/// masked over 7127 s, where the averages resolve the start 0.20 s in.
///
/// Two measures of how far the attitude moved during the window, and both are needed
/// because each is blind to what the other catches. The net rotation `ω̄ · T` cancels for
/// a vehicle that swings out and comes back, and charges nothing where the attitude (5)
/// commits is the middle of an arc already left. The disagreement between the window's
/// halves never cancels — but it cannot see a *coordinated* turn, where the specific
/// force stays put in body axes while the vehicle banks, so every average in the window
/// agrees and the tilt is wrong by the bank angle. `moving_start` is that case, and it
/// is measured: on the drift alone the filter reads `nees_att=2.83` and 626 falsely valid
/// quantity-epochs — overconfident, the failure a bound exists to prevent — against 0.50
/// and 108 with both (`data/scenarios.txt`). A bound is not tighter for dropping the term
/// that was holding it up.
///
/// The rotation is `ω̄ − β̂_g`, not `ω̄`, because (7) has already taken part of that average
/// out of the measurement: a window at rest commits the whole of it as
/// [`State::gyro_bias`], so charging it again as motion bounds the attitude by an error
/// the state has just removed. A parked vehicle whose gyroscope reads 0.05 rad/s —
/// ordinary for a MEMS part, and well inside the 0.262 rad/s `max_gyro_rate` — over a
/// 1.6 s window that is coarse only for being short otherwise starts at 0.080 rad of
/// tilt against an [`Accuracy::tilt`](crate::Accuracy::tilt) of 0.052. That start never
/// resolves: the alignment latch only promotes, so
/// [`Status::Aligning`](crate::Status::Aligning) masks every aiding transition for the
/// flight — the same cost the averages above exist to avoid, from a vehicle that never
/// moved.
///
/// [`Coarse`]'s own payload is not read here. It reports why [`classify`] refused the
/// window as static, which is a question about peaks; this is a question about averages,
/// and the same window answers the two differently.
fn coarse_sigmas(
    init: &Initialization,
    measured: &Measured,
    gyro_bias: AngularRate<Body>,
    gravity: f32,
) -> (Radians, Radians) {
    // Small-angle, as (5) reads it: an average that is not gravity leans the levelled
    // vertical by the fraction of `γ` it is out by.
    let from_force = (measured.force.vector().norm() - gravity).abs() / gravity;

    // What the gyroscope says the vehicle did, split about the vertical (5) levelled to:
    // rotation across gravity moves that vector and spoils the tilt, rotation about it
    // leaves the vector alone and spoils the heading of (6) instead.
    let net = (measured.rate.vector() - gyro_bias.vector()) * measured.span.as_secs();
    let down = measured.down();
    let across = down.map_or(net.norm(), |down| (net - down * net.dot(&down)).norm());
    let about = down.map_or(0.0, |down| net.dot(&down).abs());

    // And what the window's own halves say, which is a different question with the same
    // units; see `window_drift`.
    let (tilt_drift, heading_drift) = measured.halves.as_ref().map_or((0.0, None), window_drift);

    let tilt = init
        .sigma_tilt
        .as_radians()
        .max(from_force)
        .max(across)
        .max(tilt_drift);

    // (6) levels the field by that tilt, so the tilt error leaks into the heading scaled
    // by the dip — and the heading has its own drift besides. Both saturate at the circle,
    // since `π/√3` is the widest standard deviation a heading can have and a bound past it
    // claims a spread the quantity does not have. A window no magnetometer observed gets
    // the circle outright, as does one whose field observes no heading to be wrong about.
    let circle = UNKNOWN_HEADING_SIGMA.as_radians();
    let yaw = down
        .zip(measured.field)
        .and_then(|(down, field)| heading_sensitivity(field, down))
        .map_or(circle, |sensitivity| {
            init.sigma_yaw
                .as_radians()
                .max(sensitivity.tan_dip * tilt)
                .max(about)
                .max(heading_drift.unwrap_or(0.0))
                .min(circle)
        });

    (Radians::from_radians(tilt), Radians::from_radians(yaw))
}

/// The initial covariance `P₀`. Equation (8).
///
/// Position, velocity and bias sigmas come from `init`; the attitude sigmas from
/// [`attitude_sigmas`], because they depend on how good the alignment was.
///
/// Tilt and yaw are uncertainties about navigation axes, and the block they land in is the
/// covariance of the body-frame `δθ`, so they are rotated in through `attitude`, the
/// alignment's own. Diagonal only at level: on a vehicle standing on its tail, body x points
/// up and the yaw prior belongs on `δθ_x`.
///
/// The tilt and the accelerometer bias are **one error**, not two. (5) levels the window's
/// average specific force, which reads a horizontal accelerometer bias as gravity leaning, so
/// the committed attitude is wrong by `δθ = −[d̂]× δβa / γ` with `d̂` navigation down in body
/// axes (EQUATIONS.md, (8)). A block-diagonal `P₀` says the two are independent: when
/// velocity fusion then learns the bias, nothing tells the tilt it caused to follow, and the
/// attitude is overconfident by exactly the share of it the bias explains. Measured on
/// `harsh_imu`, 50 seeds with every source fused white (`Correlation::WHITE`, where the
/// inflation of (24′) cannot hide it): 2281 epochs over the family-wise bound with the
/// block diagonal, 214 at the wider [`Initialization::sigma_accel_bias`] alone, none with
/// the correlation as well.
///
/// So the tilt prior is the bias's share, `σ_βa² / γ²`, carried as correlation, plus an
/// independent share of `max(σ_tilt² − σ_βa² / γ², level)`: `sigma_tilt` is the whole tilt
/// uncertainty where it covers the bias, and a bias prior wider than it raises it rather than
/// being claimed away. Adding all of `σ_tilt²` instead counts the bias's share twice, and it
/// costs: `f16771dd` lost its unaided tilt at 3.41 s against 3.84, and `tilt` rose on ten of
/// the eleven scenarios (`harsh_imu` 1.022° against 0.828).
///
/// `level` is the window's own [`Measured::level_scatter`] over `γ²`, and it is what keeps the
/// independent share from reaching zero. At the defaults the bias's share, 0.0204 rad, is
/// over `sigma_tilt`'s 0.02, so without it `P₀` would claim every tilt error is the bias and
/// be singular across the two tilt directions: once velocity fusion knew the bias, nothing
/// would be left for the vibration or noise the average levelled through. A window of one
/// measures no scatter, and keeps the whole of `σ_tilt²` independent instead.
///
/// Writing the blocks after construction costs a copy of `P`, on a chain that stays well under
/// `update`'s, so the crate's peak does not move ([measured]).
///
/// [measured]: https://github.com/wboayue/fusion-nav/blob/main/DESIGN.md#measured-cost-by-function
pub(crate) fn initial_covariance(
    init: &Initialization,
    attitude: &Attitude,
    sigma_tilt: Radians,
    sigma_yaw: Radians,
    level_scatter: Option<f64>,
    gravity: f32,
) -> Covariance {
    let position = init.sigma_position.as_meters();
    let velocity = init.sigma_velocity.as_m_per_s();
    let (tilt, yaw) = (sigma_tilt.as_radians(), sigma_yaw.as_radians());
    let accel_bias = init.sigma_accel_bias.as_m_per_s2();
    let gyro_bias = init.sigma_gyro_bias.as_rad_per_s();
    // In the `ErrorState` ordering: `[δp δv δθ δβa δβg]`.
    #[rustfmt::skip]
    let sigmas = [
        position,   position,   position,
        velocity,   velocity,   velocity,
        0.0,        0.0,        0.0,
        accel_bias, accel_bias, accel_bias,
        gyro_bias,  gyro_bias,  gyro_bias,
    ];
    let explained = accel_bias / gravity;
    let gamma = f64::from(gravity);
    let level_variance = level_scatter.map(|scatter| (scatter / (gamma * gamma)) as f32);
    let independent = level_variance.map_or(tilt * tilt, |level| {
        (tilt * tilt - explained * explained).max(level)
    });
    let tilt_variance = explained * explained + independent;
    let mut covariance = Covariance::from_sigmas(sigmas);
    covariance.set_attitude_block(
        AttitudeVariance {
            tilt_north: tilt_variance,
            tilt_east: tilt_variance,
            heading: yaw * yaw,
        }
        .in_body(attitude),
    );
    // `P_θβa = E[δθ δβaᵀ] = −[d̂]× σ_βa² / γ`, from `δθ = −[d̂]× δβa / γ` and `δβa` of
    // variance `σ_βa² I`. Horizontal only: `[d̂]×` has no component along `d̂`, and a bias
    // along gravity moves `‖f̄‖`, not the direction (5) levels to.
    let down = attitude
        .quaternion()
        .inverse_transform_vector(&Vector3::z());
    covariance.set_attitude_accel_bias_block(skew(down) * (-accel_bias * explained));
    covariance
}

/// Standard deviation of a heading known only to lie somewhere on the circle: `π / √3`,
/// the standard deviation of a uniform distribution over `[-π, π]`.
///
/// Honest, and at the same time a number the error state cannot really carry — the
/// three-component attitude error of equation (2) is a small-angle quantity, and a yaw
/// error of a radian is not small. It stands in until a heading source arrives, and the
/// right response to that source is a yaw **reset** rather than a gradual correction.
/// This is why PX4 and ArduPilot align yaw with a bank of hypotheses rather than one wide
/// prior; see `GOALS.md`.
const UNKNOWN_HEADING_SIGMA: Radians = Radians::from_radians(1.813_799_4);

/// Whether the window's peak motion is within the tolerances that make it a still one.
///
/// Half of what [`classify`] asks, and all of what the barometric reference and what the
/// start establishes ask, which is why it is separate: a window can be too short to align
/// an attitude from and still be a window of a vehicle sitting on the ground, which is
/// what [`Coarse::WindowTooShort`] reports.
///
/// GNSS velocity does not enter, however well it explains the specific force. The things
/// that hang on this answer — [`Alignment::Static`], the barometric reference, and
/// whether position and velocity were established at all — all mean *the vehicle was on
/// the ground*, and a deck accelerating under it is not that however precisely the
/// acceleration is known.
///
/// It takes the peaks rather than a [`Measured`] so that [`StaticWindow::is_at_rest`] can ask
/// it after every sample without measuring the window.
pub(crate) fn at_rest(peaks: Peaks, init: &Initialization, gravity: f32) -> bool {
    peaks.gyro <= init.max_gyro_rate && peaks.deviation(gravity) <= init.max_accel_deviation
}

#[cfg(test)]
pub(crate) mod tests {
    use std::format;

    use super::*;
    use crate::config::GRAVITY;
    use crate::state::ErrorState;

    /// What a buffered window measures: every sample pushed in order, as
    /// [`StaticWindow::try_from`] does.
    fn measure(window: &[StaticSample]) -> Result<Measured, InitError> {
        StaticWindow::try_from(window)?.measured()
    }

    /// `α₀` and its variance over `window`, each sample `DT` after the last.
    fn alpha0(window: &[StaticSample]) -> Option<(Altitude, f32)> {
        StaticWindow::try_from(spaced(window, DT).as_slice())
            .expect("a usable window")
            .alpha0()
    }

    /// `ā_n` over a window already spaced.
    fn inertial_acceleration(window: &[StaticSample]) -> Option<Acceleration<Ned>> {
        measure(window).expect("a usable window").inertial_accel
    }

    #[test]
    fn a_coarse_start_reads_as_what_was_measured() {
        let short = Alignment::Coarse(Coarse::WindowTooShort {
            required: Seconds::from_secs(2.0),
            provided: Seconds::from_secs(0.8),
        });
        assert_eq!(
            format!("{short}"),
            "coarse, window of 0.800 s, 2.000 s required"
        );
        assert_eq!(
            format!(
                "{}",
                InitError::InvalidInterval {
                    interval: Seconds::from_secs(-0.0025)
                }
            ),
            "initialization interval of -0.003 s is not usable"
        );
    }

    /// A sample from a vehicle genuinely sitting still: no rotation, gravity the only
    /// specific force. `StaticSample::default()` is not this — its zero acceleration is
    /// a full `g` away from anything the world does — so the stationarity check reads it
    /// as motion, correctly.
    pub(crate) fn still() -> StaticSample {
        StaticSample {
            imu: ImuSample::reading(
                AngularRate::body(0.0, 0.0, 0.0),
                Acceleration::body(0.0, 0.0, -GRAVITY),
            ),
            ..StaticSample::default()
        }
    }

    /// 8 samples at 4 Hz: exactly the default 2 s `min_duration`.
    const DT: Seconds = Seconds::from_secs(0.25);

    /// `window` with each sample integrated over `dt` and timed `dt` after the last, the
    /// first ending at `dt`: the fixtures are written as rates, and this is where they get a
    /// clock.
    pub(crate) fn spaced(window: &[StaticSample], dt: Seconds) -> std::vec::Vec<StaticSample> {
        let mut time = Timestamp::ZERO;
        window
            .iter()
            .map(|sample| {
                time = time.after(dt);
                StaticSample {
                    imu: sample.imu.timed(time, dt),
                    ..*sample
                }
            })
            .collect()
    }

    fn classify_default(window: &[StaticSample], dt: Seconds) -> Result<Alignment, InitError> {
        Ok(classify(
            &measure(&spaced(window, dt))?,
            &Initialization::default(),
            GRAVITY,
        ))
    }

    /// The nominal state a window yields at the default 4 Hz.
    fn nominal(window: &[StaticSample], declination: Radians, at_rest: bool) -> State {
        nominal_state(
            &measure(&spaced(window, DT)).expect("a usable window"),
            declination,
            at_rest,
        )
    }

    /// The tilt and yaw sigmas a window would start with, in radians.
    ///
    /// The gyroscope bias comes from (7) on the same window, the way `apply_alignment`
    /// supplies it, so a test sees the pair as the filter commits them.
    fn sigmas(window: &[StaticSample], dt: Seconds) -> (f32, f32) {
        let init = Initialization::default();
        let measured = measure(&spaced(window, dt)).expect("a usable window");
        let state = nominal_state(
            &measured,
            Radians::ZERO,
            at_rest(measured.peaks, &init, GRAVITY),
        );
        let (tilt, yaw) = attitude_sigmas(
            &init,
            classify(&measured, &init, GRAVITY),
            &measured,
            state.gyro_bias,
            GRAVITY,
        );
        (tilt.as_radians(), yaw.as_radians())
    }

    /// The tilt sigma alone, at the default 4 Hz, which most cases here are about.
    fn coarse_tilt(window: &[StaticSample]) -> f32 {
        sigmas(window, DT).0
    }

    /// `tan` of the dip [`INCLINATION`] writes, which is what a tilt error costs a
    /// heading. No `f32::tan` here: the crate is `no_std`.
    fn dip_gain() -> f32 {
        let (sin, cos) = ComplexField::sin_cos(INCLINATION);
        sin / cos
    }

    /// Magnetic inclination, rad, down-positive: the mid-latitude dip the corpus carries
    /// and `examples/simulate.rs` writes its field with.
    const INCLINATION: f32 = 1.107;

    /// Both signs on each axis, and the two together. A down-positive convention inverts
    /// one sign at a time, so a case list that leans one way passes with the sign flipped.
    const TILTS: [(f32, f32); 6] = [
        (0.0, 0.0),
        (0.3, 0.0),
        (-0.3, 0.0),
        (0.0, 0.2),
        (0.0, -0.2),
        (0.4, -0.25),
    ];

    /// `R = Rz(ψ) Ry(θ) Rx(φ)`, the attitude of equation (7).
    fn attitude_of(roll: f32, pitch: f32, yaw: f32) -> Rotation3<f32> {
        Rotation3::from_euler_angles(roll, pitch, yaw)
    }

    /// What a vehicle held at this attitude and sitting still reads on its
    /// accelerometer: equation (11) with `a_n = 0`, so `f = Rᵀ(−g)`.
    pub(crate) fn gravity_at(roll: f32, pitch: f32, yaw: f32) -> Acceleration<Body> {
        let f = attitude_of(roll, pitch, yaw).inverse() * Vector3::new(0.0, 0.0, -GRAVITY);
        Acceleration::from_vector(f)
    }

    /// What it reads on its magnetometer, for a site whose field dips by [`INCLINATION`]
    /// and points `declination` east of true north.
    fn field_at(roll: f32, pitch: f32, yaw: f32, declination: f32) -> MagField<Body> {
        let (sin_dip, cos_dip) = ComplexField::sin_cos(INCLINATION);
        let north = attitude_of(0.0, 0.0, declination) * Vector3::new(cos_dip, 0.0, sin_dip);
        MagField::from_vector(attitude_of(roll, pitch, yaw).inverse() * north)
    }

    /// Eight samples of a vehicle turning at a steady rate about one body axis, each
    /// reading the gravity and the field that attitude produces.
    ///
    /// Consistent by construction, which the drift terms of (8′) need: [`window_drift`]
    /// reads how far the attitude moved off the specific force and the field themselves,
    /// so a gyroscope sample set without moving the accelerometer under it describes a
    /// vehicle that is not turning — which is what it is.
    ///
    /// Over eight samples at [`DT`] the half-means sit at 0.375 and 1.375 sample periods,
    /// so a rate of `ω` drifts by exactly `ω · 1 s` between them, whatever the axis.
    pub(crate) fn turning(roll_rate: f32, pitch_rate: f32, yaw_rate: f32) -> [StaticSample; 8] {
        let mut window = [still(); 8];
        for (index, sample) in window.iter_mut().enumerate() {
            let t = index as f32 * DT.as_secs();
            let (roll, pitch, yaw) = (roll_rate * t, pitch_rate * t, yaw_rate * t);
            sample.imu = sample.imu.with_accel(gravity_at(roll, pitch, yaw));
            // Exact while one axis turns at a time, which is all this builds.
            sample.imu = sample
                .imu
                .with_gyro(AngularRate::body(roll_rate, pitch_rate, yaw_rate));
            sample.mag = Some(field_at(roll, pitch, yaw, 0.0));
        }
        window
    }

    /// A still window of `[still(); 8]` reading this specific force and this field.
    fn window_at(roll: f32, pitch: f32, yaw: f32, declination: f32) -> [StaticSample; 8] {
        [StaticSample {
            imu: still().imu.with_accel(gravity_at(roll, pitch, yaw)),
            mag: Some(field_at(roll, pitch, yaw, declination)),
            ..still()
        }; 8]
    }

    #[test]
    fn the_tilt_prior_predicts_the_level_error_a_biased_accelerometer_causes() {
        // What (8)'s cross-covariance claims, checked against what (5) actually commits:
        // told the bias, `P₀` must predict the tilt error, `E[δθ | δβa] = P_θβa P_βaβa⁻¹ δβa`.
        // Flipping the sign of `P_θβa` fails every case here; so does leaving it out.
        let init = Initialization::default();
        let variance = init.sigma_accel_bias.as_m_per_s2().powi(2);
        let bias = Vector3::new(0.15, -0.1, 0.05);
        for (roll, pitch) in TILTS {
            let truth = attitude_of(roll, pitch, 0.7);
            let window = [StaticSample {
                imu: still().imu.with_accel(Acceleration::from_vector(
                    gravity_at(roll, pitch, 0.7).vector() + bias,
                )),
                mag: Some(field_at(roll, pitch, 0.7, 0.0)),
                ..still()
            }; 8];
            let state = nominal(&window, Radians::ZERO, true);
            let committed = state.attitude.quaternion();
            // `q = q̂ ⊗ δq`, the local error of equation (2).
            let error =
                (committed.inverse() * UnitQuaternion::from_rotation_matrix(&truth)).scaled_axis();

            let measured = measure(&spaced(&window, DT)).expect("a usable window");
            let p = initial_covariance(
                &init,
                &state.attitude,
                init.sigma_tilt,
                init.sigma_yaw,
                measured.level_scatter,
                GRAVITY,
            );
            let theta = ErrorState::AttitudeX.index();
            let beta = ErrorState::AccelBiasX.index();
            let predicted = p.as_matrix().fixed_view::<3, 3>(theta, beta) * bias / variance;

            // (5) observes no heading, so only the error across gravity is predictable.
            let down = committed.inverse_transform_vector(&Vector3::z());
            let across = |v: Vector3<f32>| v - down * v.dot(&down);
            assert!(
                (across(error) - across(predicted)).norm() < 2e-4,
                "roll {roll} pitch {pitch}: levelled {:?}, predicted {:?}",
                across(error),
                across(predicted)
            );
        }
    }

    #[test]
    fn the_level_variance_is_the_scatter_across_gravity_over_the_count() {
        // ±0.5 m/s² on body x, alternating, so the mean is gravity and the sample variance
        // across it is 0.25 · 8/7. Half of it per horizontal axis, over 8 samples, over γ².
        // Scatter along gravity is not a tilt: the same shake on z adds nothing.
        let mut window = [still(); 8];
        for (index, sample) in window.iter_mut().enumerate() {
            let shake = if index % 2 == 0 { 0.5 } else { -0.5 };
            sample.imu = sample
                .imu
                .with_accel(Acceleration::body(shake, 0.0, -GRAVITY + shake));
        }
        let level = measure(&spaced(&window, DT))
            .expect("a usable window")
            .level_scatter
            .expect("eight samples scatter");
        let expected = 0.25 * 8.0 / 7.0 / 2.0 / 8.0;
        assert!(
            (level - expected).abs() < 1e-3 * expected,
            "expected {expected}, got {level}"
        );
        assert_eq!(
            measure(&spaced(&[still()], DT))
                .expect("one sample")
                .level_scatter,
            None
        );
    }

    #[test]
    fn a_scattered_window_keeps_tilt_uncertain_once_the_bias_is_known() {
        // At the defaults the bias explains all of `sigma_tilt`, so the tilt variance left
        // once the bias is known is the window's own scatter and nothing else. Zero there is
        // the singular `P₀` that claims every level error is the bias.
        let init = Initialization::default();
        let level = 4e-6;
        let p = initial_covariance(
            &init,
            &Attitude::default(),
            init.sigma_tilt,
            init.sigma_yaw,
            // The scatter whose tilt is `level`.
            Some(f64::from(level) * f64::from(GRAVITY) * f64::from(GRAVITY)),
            GRAVITY,
        );
        let m = p.as_matrix();
        let (theta, beta) = (
            ErrorState::AttitudeX.index(),
            ErrorState::AccelBiasX.index(),
        );
        let p_tb = m.fixed_view::<3, 3>(theta, beta);
        let p_bb = m.fixed_view::<3, 3>(beta, beta);
        let given_bias = m.fixed_view::<3, 3>(theta, theta)
            - p_tb * p_bb.try_inverse().expect("a bias prior") * p_tb.transpose();
        for axis in 0..2 {
            let left = given_bias[(axis, axis)];
            assert!((left - level).abs() < 1e-7, "axis {axis}: {left}");
        }
    }

    /// The roll, pitch and yaw equation (7) committed, in radians.
    fn committed_angles(state: &State) -> (f32, f32, f32) {
        state.attitude.euler_angles()
    }

    #[test]
    fn levelling_recovers_the_tilt_gravity_was_rotated_by() {
        for (roll, pitch) in TILTS {
            // A yaw the answer must not depend on: gravity says nothing about rotation
            // about itself, so `Rz(ψ)ᵀ` leaves the specific force where it was.
            let (measured_roll, measured_pitch) = level_from_accel(gravity_at(roll, pitch, 0.9));
            assert!(
                (measured_roll.as_radians() - roll).abs() < 1e-5
                    && (measured_pitch.as_radians() - pitch).abs() < 1e-5,
                "({roll}, {pitch}) read back as ({:?}, {:?})",
                measured_roll,
                measured_pitch
            );
        }
    }

    #[test]
    fn the_committed_attitude_carries_the_specific_force_back_onto_gravity() {
        // (5) and (7) round-tripped: the attitude the filter commits must rotate the
        // vector it was derived from back onto `−g`, or the two disagree about which
        // sequence they mean.
        for (roll, pitch) in TILTS {
            let window = window_at(roll, pitch, 0.0, 0.0);
            let state = nominal(&window, Radians::ZERO, true);
            let navigation = state.attitude.quaternion() * window[0].imu.specific_force().vector();
            assert!(
                (navigation - Vector3::new(0.0, 0.0, -GRAVITY)).norm() < 1e-4,
                "({roll}, {pitch}) rotated back to {navigation:?}"
            );
        }
    }

    #[test]
    fn heading_survives_the_tilt_it_is_levelled_by() {
        // The test that fails if the levelling is dropped: a field synthesised for a
        // known yaw at a tilt that is not zero must still give that yaw back.
        const DECLINATION: f32 = -0.06;
        for (roll, pitch) in TILTS {
            for yaw in [0.0, 0.9, -2.5, 3.0] {
                let state = nominal(
                    &window_at(roll, pitch, yaw, DECLINATION),
                    Radians::from_radians(DECLINATION),
                    true,
                );
                let (_, _, committed) = committed_angles(&state);
                assert!(
                    wrap_pi(committed - yaw).abs() < 1e-4,
                    "yaw {yaw} at ({roll}, {pitch}) read back as {committed}"
                );
            }
        }
    }

    #[test]
    fn an_unlevelled_heading_is_wrong_by_the_dip() {
        // What the levelling is worth, and why a zero-tilt test proves nothing: at 10°
        // of roll the raw body field gives a heading 19° from the true one, because the
        // dip leans into the horizontal axes. `tan(1.107)` is 1.96.
        let roll = 10.0f32.to_radians();
        let field = field_at(roll, 0.0, 0.0, 0.0).vector();
        let unlevelled = RealField::atan2(field.y, field.x);
        assert!(
            (unlevelled.to_degrees().abs() - 19.1).abs() < 0.1,
            "expected about 19 deg of error, got {}",
            unlevelled.to_degrees()
        );

        let levelled = heading_from_mag(
            MagField::from_vector(field),
            Radians::from_radians(roll),
            Radians::ZERO,
            Radians::ZERO,
        );
        assert!(levelled.as_radians().abs() < 1e-6, "{levelled:?}");
    }

    #[test]
    fn declination_moves_the_heading_by_exactly_itself() {
        // The same field, read at two sites: true heading differs from magnetic by the
        // declination and by nothing else.
        let (roll, pitch) = (0.2, -0.1);
        let field = field_at(roll, pitch, 0.7, 0.0);
        let heading = |declination: f32| {
            heading_from_mag(
                field,
                Radians::from_radians(roll),
                Radians::from_radians(pitch),
                Radians::from_radians(declination),
            )
            .as_radians()
        };
        assert!((heading(0.35) - heading(0.0) - 0.35).abs() < 1e-6);
        assert!((heading(-0.06) - heading(0.0) + 0.06).abs() < 1e-6);
    }

    #[test]
    fn a_window_with_no_magnetometer_keeps_a_heading_of_zero() {
        // Stillness observes tilt and never the rotation about it. Zero is a stated
        // direction rather than a measured one, which `Unestablished::heading` records.
        let state = nominal(&[still(); 8], Radians::from_radians(0.35), true);
        assert_eq!(committed_angles(&state).2, 0.0);
    }

    #[test]
    fn a_still_window_takes_its_gyroscope_bias_from_the_average() {
        // A gyroscope reading a constant offset while the vehicle does not turn is
        // reading its own bias, and at rest that is the one place it is observable.
        let offset = AngularRate::body(0.01, -0.02, 0.003);
        let window = [StaticSample {
            imu: still().imu.with_gyro(offset),
            ..still()
        }; 8];
        let state = nominal(&window, Radians::ZERO, true);
        assert!(
            (state.gyro_bias.vector() - offset.vector()).norm() < 1e-7,
            "{:?}",
            state.gyro_bias
        );
    }

    #[test]
    fn a_window_taken_in_motion_takes_no_gyroscope_bias_at_all() {
        // The average is the vehicle turning, not the sensor lying, and seeding it would
        // subtract a turn rate from every later measurement as a sensor error.
        let window = [StaticSample {
            imu: still().imu.with_gyro(AngularRate::body(0.0, 0.4, 0.0)),
            ..still()
        }; 8];
        let state = nominal(&window, Radians::ZERO, false);
        assert_eq!(state.gyro_bias, AngularRate::zero());
    }

    #[test]
    fn a_still_window_of_min_duration_is_static() {
        assert_eq!(classify_default(&[still(); 8], DT), Ok(Alignment::Static));
    }

    #[test]
    fn the_static_window_is_measured_in_seconds_not_samples() {
        // The same 8 samples, now spanning 0.8 s instead of 2 s.
        let alignment = classify_default(&[still(); 8], Seconds::from_secs(0.1));
        assert!(matches!(
            alignment,
            Ok(Alignment::Coarse(Coarse::WindowTooShort { .. }))
        ));
    }

    #[test]
    fn a_short_window_that_moved_is_not_stationary_rather_than_too_short() {
        // Motion is measured before length, so `WindowTooShort` only ever means still.
        let mut window = [still(); 8];
        window[3].imu = window[3].imu.with_gyro(AngularRate::body(0.0, 0.4, 0.0));
        let alignment = classify_default(&window, Seconds::from_secs(0.1));
        let Ok(Alignment::Coarse(Coarse::NotStationary { span, .. })) = alignment else {
            panic!("0.8 s and moving: {alignment:?}");
        };
        assert!((span.as_secs() - 0.8).abs() < 1e-6, "got {span:?}");
    }

    #[test]
    fn a_moving_window_is_coarse_and_reports_what_it_measured() {
        let mut window = [still(); 8];
        window[3].imu = window[3].imu.with_gyro(AngularRate::body(0.0, 0.4, 0.0));
        let alignment = classify_default(&window, DT).expect("moving, not unusable");
        let Alignment::Coarse(Coarse::NotStationary { peak_gyro, .. }) = alignment else {
            panic!("0.4 rad/s is over the 0.262 default: {alignment:?}");
        };
        let peak_gyro = peak_gyro.as_rad_per_s();
        assert!((peak_gyro - 0.4).abs() < 1e-6, "got {peak_gyro}");
    }

    #[test]
    fn a_sample_short_of_gravity_moves_the_window_as_one_over_it_does() {
        // 3 m/s² light on one sample and 1 m/s² heavy on another: the light one is the peak,
        // and against a different `γ` the same window departs by a different amount.
        let mut window = [still(); 8];
        window[2].imu = window[2]
            .imu
            .with_accel(Acceleration::body(0.0, 0.0, -GRAVITY + 3.0));
        window[5].imu = window[5]
            .imu
            .with_accel(Acceleration::body(0.0, 0.0, -GRAVITY - 1.0));
        let measured = measure(&spaced(&window, DT)).expect("a usable window");
        let deviation = measured.peaks.deviation(GRAVITY).as_m_per_s2();
        assert!((deviation - 3.0).abs() < 1e-5, "{deviation}");
        let lighter = measured.peaks.deviation(GRAVITY - 0.5).as_m_per_s2();
        assert!((lighter - 2.5).abs() < 1e-5, "{lighter}");
        assert_eq!(Peaks::NONE.deviation(GRAVITY).as_m_per_s2(), 0.0);
    }

    #[test]
    fn a_coarse_start_widens_tilt_in_proportion_to_the_motion_it_saw() {
        let mut window = [still(); 8];
        // 2.94 m/s^2 of unexplained specific force on one sample of eight: over the
        // stationarity tolerance, so the window is coarse, and a mean 0.368 m/s^2 out.
        window[0].imu =
            window[0]
                .imu
                .with_accel(Acceleration::body(0.0, 0.0, -GRAVITY - 2.941_995));
        let tilt = coarse_tilt(&window);
        assert!(
            (tilt - 0.0375).abs() < 1e-4,
            "an eighth of the spike is 0.0375 rad; charging the peak reads 0.3, got {tilt}"
        );
    }

    #[test]
    fn vibration_averages_out_of_the_tilt_where_a_peak_charges_all_of_it() {
        // The shape `2c42096b` has: a vehicle sitting with its props spinning, whose
        // specific force is never gravity and whose average is. Alternating, so every
        // sample is over the 1.961 m/s^2 tolerance and the mean cancels exactly.
        let mut window = [still(); 8];
        for (index, sample) in window.iter_mut().enumerate() {
            let shake = if index % 2 == 0 { 3.0 } else { -3.0 };
            sample.imu = sample
                .imu
                .with_accel(Acceleration::body(0.0, 0.0, -GRAVITY + shake));
        }
        let tilt = coarse_tilt(&window);
        assert!(
            (tilt - Initialization::default().sigma_tilt.as_radians()).abs() < 1e-6,
            "nothing above the floor; the peak of 3 m/s^2 would charge 0.306, got {tilt}"
        );
    }

    #[test]
    fn a_turn_about_gravity_charges_the_heading_and_leaves_the_tilt_alone() {
        // Yawing: |a| stays exactly g and the gravity vector does not move in body axes
        // at all, so the tilt its average yields is as good as a still window's. The
        // heading is what moves, and it is charged the 0.8 rad the vehicle turned.
        let (tilt, yaw) = sigmas(&turning(0.0, 0.0, 0.4), DT);
        assert!(
            (tilt - Initialization::default().sigma_tilt.as_radians()).abs() < 1e-6,
            "a turn about gravity is worth no tilt; without the split about `down` this \
             reads 0.8, got {tilt}"
        );
        assert!(
            (yaw - 0.8).abs() < 1e-4,
            "0.4 rad/s over a 2 s window, all of it heading, got {yaw}"
        );
    }

    #[test]
    fn a_coordinated_turn_is_charged_although_its_averages_all_agree() {
        // The case the halves cannot see, and what `moving_start` is: the vehicle banks
        // into a turn, the specific force it measures stays put in body axes, and every
        // average in the window agrees while the tilt is wrong by the bank angle. Only
        // the gyroscope witnesses it.
        let mut window = [still(); 8];
        for sample in &mut window {
            sample.imu = sample.imu.with_gyro(AngularRate::body(0.0, 0.4, 0.0));
        }
        let tilt = coarse_tilt(&window);
        assert!(
            (tilt - 0.8).abs() < 1e-4,
            "0.4 rad/s over a 2 s window is 0.8 rad of turn across gravity, and the \
             window's own averages say nothing at all, got {tilt}"
        );
    }

    #[test]
    fn a_window_that_swings_out_and_back_is_charged_the_excursion() {
        // The case the gyroscope cannot see: the vehicle ends where it started, `ω̄ · T`
        // cancels to nothing, and the attitude (5) commits is the middle of an arc it has
        // already left. The halves disagree by 0.4 rad and say so.
        const PITCH: [f32; 8] = [0.0, 0.2, 0.4, 0.2, 0.0, -0.2, -0.4, -0.2];
        // dθ/dt between samples, so the gyroscope agrees with the attitude above and its
        // mean really is zero. The last repeats the previous, having no successor.
        const RATE: [f32; 8] = [0.8, 0.8, -0.8, -0.8, -0.8, -0.8, 0.8, 0.8];
        let mut window = [still(); 8];
        for ((sample, pitch), rate) in window.iter_mut().zip(PITCH).zip(RATE) {
            sample.imu = sample.imu.with_accel(gravity_at(0.0, pitch, 0.0));
            sample.imu = sample.imu.with_gyro(AngularRate::body(0.0, rate, 0.0));
        }
        let tilt = coarse_tilt(&window);
        assert!(
            (tilt - 0.4).abs() < 1e-4,
            "the halves mean +0.2 and -0.2 rad; on the net rotation alone this reads the \
             0.0297 rad the shortened average is worth, got {tilt}"
        );
    }

    #[test]
    fn a_still_short_window_keeps_the_heading_its_magnetometer_observed() {
        // The window of #85: still, carrying a field, and coarse only because it is
        // short. `classify` calls it short before it measures motion, so nothing about
        // that verdict says the heading is worth less than a 2 s window's.
        let window = window_at(0.0, 0.0, 0.9, 0.0);
        let (tilt, yaw) = sigmas(&window, Seconds::from_secs(0.1));
        let init = Initialization::default();
        assert!(
            (tilt - init.sigma_tilt.as_radians()).abs() < 1e-6,
            "got {tilt}"
        );
        assert!(
            (yaw - init.sigma_yaw.as_radians()).abs() < 1e-6,
            "the configured prior, not the 1.81 rad of a heading nothing observed, got {yaw}"
        );
    }

    #[test]
    fn a_parked_vehicles_gyroscope_bias_is_not_also_charged_as_motion() {
        // (7) commits ω̄ as the gyroscope bias of a window at rest, so the same rotation
        // cannot also bound the attitude — it is not motion the window failed to see, it
        // is a sensor offset the state now carries. 0.05 rad/s is ordinary for a MEMS
        // part and well inside the 0.262 rad/s tolerance; eight samples at 0.2 s is a
        // 1.6 s window, still, and coarse only for being short.
        let mut window = window_at(0.0, 0.0, 0.0, 0.0);
        for sample in &mut window {
            sample.imu = sample.imu.with_gyro(AngularRate::body(0.05, 0.0, 0.0));
        }
        let dt = Seconds::from_secs(0.2);
        let init = Initialization::default();
        let measured = measure(&spaced(&window, dt)).expect("a usable window");
        assert!(
            at_rest(measured.peaks, &init, GRAVITY),
            "0.05 rad/s is inside 0.262"
        );
        let bias = nominal_state(&measured, Radians::ZERO, true)
            .gyro_bias
            .vector();
        assert!(
            (bias.x - 0.05).abs() < 1e-6,
            "(7) commits the average, got {bias:?}"
        );

        let (tilt, yaw) = sigmas(&window, dt);
        assert!(
            (tilt - init.sigma_tilt.as_radians()).abs() < 1e-6,
            "charging ω̄ rather than ω̄ − β̂_g reads 0.08 rad here, over the 0.0524 (3°) of \
             `ALIGNED_TILT`, and latches `Aligning` for the flight; got {tilt}"
        );
        assert!(
            (yaw - init.sigma_yaw.as_radians()).abs() < 1e-6,
            "the configured prior; a bias about gravity would charge the heading the \
             same way, got {yaw}"
        );
    }

    #[test]
    fn a_window_no_magnetometer_observed_gets_the_circle_however_still_it_was() {
        // Stillness observes tilt and never the rotation about it.
        let (tilt, yaw) = sigmas(&[still(); 8], Seconds::from_secs(0.1));
        assert!(
            (tilt - Initialization::default().sigma_tilt.as_radians()).abs() < 1e-6,
            "got {tilt}"
        );
        assert!(
            (yaw - UNKNOWN_HEADING_SIGMA.as_radians()).abs() < 1e-6,
            "got {yaw}"
        );
    }

    #[test]
    fn a_window_charges_its_heading_the_tilt_it_levelled_by() {
        // The dip couples them: (6) levels the field by the tilt of (5), so a tilt this
        // window cannot vouch for is a heading error scaled by `tan(dip)`. Steady
        // unexplained specific force rather than a turn, so that the halves agree and the
        // dip is the only thing charging the heading.
        let mut window = window_at(0.0, 0.0, 0.9, 0.0);
        for sample in &mut window {
            let leaning = sample.imu.specific_force().vector() - Vector3::new(0.0, 0.0, 2.941_995);
            sample.imu = sample.imu.with_accel(Acceleration::from_vector(leaning));
        }
        let (tilt, yaw) = sigmas(&window, DT);
        assert!((tilt - 0.3).abs() < 1e-4, "2.942 / g, got {tilt}");
        assert!(
            (yaw - dip_gain() * tilt).abs() < 1e-4,
            "tan(dip) is {}, so the yaw is {}, got {yaw}",
            dip_gain(),
            dip_gain() * tilt
        );
    }

    #[test]
    fn a_heading_bound_saturates_at_the_circle_it_cannot_be_wider_than() {
        // 2 rad/s of yaw drifts the halves 2 rad apart, which is more spread than a
        // heading on a circle can have.
        let (_, yaw) = sigmas(&turning(0.0, 0.0, 2.0), DT);
        assert!(
            (yaw - UNKNOWN_HEADING_SIGMA.as_radians()).abs() < 1e-6,
            "pi/sqrt(3) is the widest a heading gets, got {yaw}"
        );
    }

    #[test]
    fn the_heading_sensitivity_is_the_tangent_of_the_dip_at_any_tilt() {
        for (roll, pitch) in TILTS {
            let down = -gravity_at(roll, pitch, 0.4).vector().normalize();
            let measured = heading_sensitivity(field_at(roll, pitch, 0.4, 0.0), down)
                .expect("a field with a horizontal part")
                .tan_dip;
            assert!(
                (measured - dip_gain()).abs() < 1e-4,
                "at ({roll}, {pitch}) got {measured}, want {}",
                dip_gain()
            );
        }
    }

    #[test]
    fn a_field_straight_down_observes_no_heading_at_any_tilt() {
        // `tan(dip)` is unbounded there, and a bound that divided by the horizontal part
        // would put that infinity into `P` through `initial_covariance`.
        let down = Vector3::new(0.0, 0.0, 1.0);
        assert_eq!(
            heading_sensitivity(MagField::body(0.0, 0.0, 0.5), down),
            None
        );
        assert_eq!(
            heading_sensitivity(MagField::body(0.0, 0.0, 0.0), down),
            None
        );
    }

    #[test]
    fn an_accelerometer_averaging_to_zero_is_bounded_and_not_made_a_nan() {
        // Stopped or disconnected: `Measured::down` reports no vertical, so the turn
        // cannot be split about one and the whole of it is charged to tilt.
        //
        // The rate is 0.6 rad/s for a reason. At 1.2 rad it is the widest of the bounds,
        // so that fallback has to carry it: with `normalize()` in place of
        // `try_normalize` this reads the 1.0 rad of `|0 − g| / g` instead, because the
        // NaN that makes is dropped by `f32::max` rather than surfacing. A slower turn
        // passes either way. The magnetometer is there so that the circle the heading
        // falls back to is the missing vertical's doing and not its own.
        let mut window = [still(); 8];
        for sample in &mut window {
            sample.imu = sample.imu.with_accel(Acceleration::body(0.0, 0.0, 0.0));
            sample.imu = sample.imu.with_gyro(AngularRate::body(0.0, 0.6, 0.0));
            sample.mag = Some(MagField::body(0.22, 0.0, 0.44));
        }
        let (tilt, yaw) = sigmas(&window, DT);
        assert!(
            (tilt - 1.2).abs() < 1e-6,
            "0.6 rad/s over 2 s, above the 1.0 rad that |0 - g| / g is worth, got {tilt}"
        );
        assert!(
            (yaw - UNKNOWN_HEADING_SIGMA.as_radians()).abs() < 1e-6,
            "no down means no dip to scale by, got {yaw}"
        );
    }

    /// [`StaticWindow::span`]'s `f64`: the sum in `f32` falls short of the product it replaced.
    #[test]
    fn a_window_of_exactly_the_minimum_duration_is_long_enough_at_any_rate() {
        for (dt, samples) in [(0.02, 100), (0.01, 200), (0.005, 400), (0.0025, 800)] {
            let window = std::vec![still(); samples];
            assert_eq!(
                classify_default(&window, Seconds::from_secs(dt)),
                Ok(Alignment::Static),
                "{samples} samples of {dt} s"
            );
        }
    }

    #[test]
    fn an_empty_window_or_an_unusable_interval_is_still_an_error() {
        assert_eq!(classify_default(&[], DT), Err(InitError::NoSamples));
        let zero = Seconds::from_secs(0.0);
        assert_eq!(
            classify_default(&[still(); 8], zero),
            Err(InitError::InvalidInterval { interval: zero })
        );
        // The interval named is the one that is unusable, whichever of the two it is.
        let mut window = spaced(&[still(); 8], DT);
        let backwards = Seconds::from_secs(-0.1);
        window[5].imu.velocity_interval = backwards;
        assert_eq!(
            measure(&window).map(|_| ()),
            Err(InitError::InvalidInterval {
                interval: backwards
            })
        );
        let mut poisoned = [still(); 8];
        poisoned[2].imu = poisoned[2]
            .imu
            .with_accel(Acceleration::body(f32::NAN, 0.0, 0.0));
        assert_eq!(classify_default(&poisoned, DT), Err(InitError::NotFinite));
    }

    /// The clock starts at the last sample's time, so a window whose times do not run forward
    /// is refused rather than aligned: stamped all at zero, it would start the clock at zero
    /// and the first `predict` would coast the whole flight so far.
    #[test]
    fn a_window_timed_other_than_forward_is_refused() {
        let unstamped: std::vec::Vec<_> = spaced(&[still(); 8], DT)
            .into_iter()
            .map(|s| StaticSample {
                imu: s.imu.timed(Timestamp::ZERO, DT),
                ..s
            })
            .collect();
        assert_eq!(
            measure(&unstamped).map(|_| ()),
            Err(InitError::InvalidStep {
                dt: Seconds::from_secs(0.0)
            })
        );
        let mut reversed = spaced(&[still(); 8], DT);
        reversed.reverse();
        assert_eq!(
            measure(&reversed).map(|_| ()),
            Err(InitError::InvalidStep {
                dt: Seconds::from_secs(-0.25)
            })
        );
    }

    #[test]
    fn the_baro_reference_is_the_mean_over_the_window() {
        let window =
            [99.0, 101.0, 100.0, 99.5, 100.5, 100.0, 99.8, 100.2].map(|altitude| StaticSample {
                baro: Some(Altitude::from_meters(altitude)),
                ..still()
            });
        let (reference, variance) = alpha0(&window).expect("the window carried barometer samples");
        // Squared deviations sum to 2.58 m², over 7 degrees of freedom and then 8 readings.
        assert!((variance - 2.58 / 7.0 / 8.0).abs() < 1e-6, "{variance}");
        assert!(
            (reference.as_meters() - 100.0).abs() < 1e-4,
            "mean of the window is 100 m, got {}",
            reference.as_meters()
        );
    }

    #[test]
    fn samples_without_a_barometer_stay_out_of_the_mean() {
        // A barometer runs slower than the IMU, so most samples in a real window carry
        // nothing. Counting those as zero would drag the reference to the ground.
        let mut window = [still(); 8];
        window[0].baro = Some(Altitude::from_meters(10.0));
        window[7].baro = Some(Altitude::from_meters(20.0));
        assert_eq!(alpha0(&window), Some((Altitude::from_meters(15.0), 25.0)));
    }

    #[test]
    fn a_held_reading_counts_once() {
        // Two readings at 20 Hz on an IMU four times faster, each held until the next: the
        // mean and variance of two readings, not of eight samples.
        let window =
            [10.0, 10.0, 10.0, 10.0, 20.0, 20.0, 20.0, 20.0].map(|altitude| StaticSample {
                baro: Some(Altitude::from_meters(altitude)),
                ..still()
            });
        assert_eq!(alpha0(&window), Some((Altitude::from_meters(15.0), 25.0)));

        // One reading held across the window is one reading, however many samples carry it.
        let held = [StaticSample {
            baro: Some(Altitude::from_meters(10.0)),
            ..still()
        }; 8];
        assert_eq!(alpha0(&held), None);
    }

    #[test]
    fn one_reading_is_no_reference_because_it_has_no_scatter_to_measure() {
        let mut window = [still(); 8];
        window[3].baro = Some(Altitude::from_meters(10.0));
        assert_eq!(alpha0(&window), None);
    }

    #[test]
    fn the_barometer_scatter_is_the_two_pass_one_at_any_altitude() {
        // Centimetres of scatter at 10 km, on 101 levels, each reading different from the one
        // before.
        let window: std::vec::Vec<_> = (0..400)
            .map(|i| StaticSample {
                baro: Some(Altitude::from_meters(
                    10_000.0 + 0.001 * ((i * 37 % 101) as f32 - 50.0),
                )),
                ..still()
            })
            .collect();
        let readings: std::vec::Vec<f64> = window
            .iter()
            .filter_map(|s| s.baro)
            .map(|a| f64::from(a.as_meters()))
            .collect();
        let n = readings.len() as f64;
        let mean = readings.iter().sum::<f64>() / n;
        let scatter = readings
            .iter()
            .map(|a| (a - mean) * (a - mean))
            .sum::<f64>()
            / (n - 1.0);
        // The textbook one-pass form misses here by more than the tolerance below, so this
        // test can tell a single pass that cancels from one that does not.
        let squares = readings.iter().map(|a| a * a).sum::<f64>();
        let naive = (squares - n * mean * mean) / (n - 1.0);
        let lost = ((naive - scatter) / scatter).abs();
        assert!(lost > 1e-5, "{lost}");
        let (reference, variance) = alpha0(&window).expect("400 readings");
        assert_eq!(reference, Altitude::from_meters(mean as f32));
        let expected = (scatter / n) as f32;
        assert!(
            ((variance - expected) / expected).abs() < 1e-6,
            "{variance} against {expected}"
        );
    }

    /// What the window reports under the default tolerances.
    fn noise(window: &[StaticSample]) -> Option<WindowNoise> {
        StaticWindow::try_from(window)
            .expect("a usable window")
            .noise(&Config::default())
    }

    /// `n` samples of a still vehicle, the `i`th integrating `gyro(i)` over `angle` and
    /// `accel(i)`, gravity added, over `velocity`.
    fn built(
        n: usize,
        angle: f32,
        velocity: f32,
        gyro: impl Fn(usize) -> Vector3<f32>,
        accel: impl Fn(usize) -> Vector3<f32>,
    ) -> std::vec::Vec<StaticSample> {
        let mut time = Timestamp::ZERO;
        (0..n)
            .map(|i| {
                time = time.after(Seconds::from_secs(angle));
                let force = accel(i) + Vector3::new(0.0, 0.0, -GRAVITY);
                StaticSample {
                    imu: ImuSample {
                        time,
                        delta_angle: crate::units::DeltaAngle::from_vector(gyro(i) * angle),
                        angle_interval: Seconds::from_secs(angle),
                        delta_velocity: crate::units::DeltaVelocity::from_vector(force * velocity),
                        velocity_interval: Seconds::from_secs(velocity),
                    },
                    ..still()
                }
            })
            .collect()
    }

    /// `+1` or `−1`, flipping every `every` samples.
    fn flip(i: usize, every: usize) -> f32 {
        if (i / every).is_multiple_of(2) {
            1.0
        } else {
            -1.0
        }
    }

    fn assert_close(got: [f32; 3], want: Vector3<f32>, tolerance: f32) {
        for (g, w) in got.iter().zip(want.iter()) {
            assert!(((g - w) / w).abs() < tolerance, "{got:?} against {want:?}");
        }
    }

    #[test]
    fn the_density_is_the_block_scatter_times_the_root_of_the_block() {
        // Ten 50 ms blocks of five 10 ms samples, the rate ±a flipping block by block: each
        // block's increment is ±a T, so N² = J a² T / (J − 1) and N = a √(T J / (J − 1)).
        // Dropping the `1/T` reads 4.5× low, and J for J − 1 reads 5 % low.
        let (gyro, accel) = (Vector3::new(0.01, 0.02, 0.03), Vector3::new(0.1, 0.2, 0.3));
        let window = built(
            50,
            0.01,
            0.01,
            |i| gyro * flip(i, 5),
            |i| accel * flip(i, 5),
        );
        let reported = noise(&window).expect("ten still blocks");
        let factor = (0.05f32 * 10.0 / 9.0).sqrt();
        assert_close(reported.gyro_white, gyro * factor, 1e-5);
        assert_close(reported.accel_white, accel * factor, 1e-5);
        assert_eq!(reported.blocks, 10);
    }

    #[test]
    fn a_rate_alternating_sample_by_sample_cancels_within_a_block() {
        // Vibration aliased to the sample rate, which integrates to nothing: every 50 ms block
        // of ten 5 ms samples sums to zero. One sample's scatter would read it as a √Δt, 0.07 a.
        let a = Vector3::new(0.01, 0.02, 0.03);
        let window = built(
            200,
            0.005,
            0.005,
            |i| a * flip(i, 1),
            |i| a * 10.0 * flip(i, 1),
        );
        let reported = noise(&window).expect("twenty still blocks");
        assert!(
            Vector3::from(reported.gyro_white).max() < 1e-6,
            "{:?}",
            reported.gyro_white
        );
        assert!(
            Vector3::from(reported.accel_white).max() < 1e-5,
            "{:?}",
            reported.accel_white
        );
    }

    #[test]
    fn each_sensor_is_blocked_by_its_own_interval() {
        // A gyroscope at 10 ms and an accelerometer at 30 ms, each flipping sign every block
        // of its own: five gyroscope samples to a 50 ms block and two accelerometer ones to a
        // 60 ms block, no boundary landing on a tie. Blocking the accelerometer by the angle
        // interval puts five of its samples in a block that flips every two, and closing a
        // block a whole interval early gives it 30 ms ones; both read it wrong.
        let c = Vector3::new(1.0e-3, 2.0e-3, 3.0e-3);
        let window = built(100, 0.01, 0.03, |i| c * flip(i, 5), |i| c * flip(i, 2));
        let reported = noise(&window).expect("still");
        assert_close(
            reported.gyro_white,
            c * (0.05f32 * 20.0 / 19.0).sqrt(),
            1e-5,
        );
        // The increments are `f32`, and gravity's 0.3 m s⁻¹ rounds `c`'s 10⁻⁴ by that much.
        assert_close(
            reported.accel_white,
            c * (0.06f32 * 50.0 / 49.0).sqrt(),
            1e-3,
        );
        assert_eq!(reported.blocks, 20);
    }

    #[test]
    fn the_worst_axis_is_the_largest_whichever_it_is() {
        // What `ImuNoise` takes, one density per sensor for all three axes: the middle axis
        // here, so neither the first nor the last is mistaken for it.
        let (gyro, accel) = (Vector3::new(0.02, 0.03, 0.01), Vector3::new(0.3, 0.1, 0.2));
        let window = built(
            50,
            0.01,
            0.01,
            |i| gyro * flip(i, 5),
            |i| accel * flip(i, 5),
        );
        let reported = noise(&window).expect("ten still blocks");
        assert_eq!(reported.worst_gyro_white(), reported.gyro_white[1]);
        assert_eq!(reported.worst_accel_white(), reported.accel_white[0]);
    }

    /// `seconds` of white noise of density `n_gyro` and `n_accel` on every axis, as a sensor
    /// integrating over `dt` would hand it over: each increment carries `N √Δt` of noise.
    fn white(seconds: f32, dt: f32, n_gyro: f32, n_accel: f32) -> std::vec::Vec<StaticSample> {
        // A 64-bit LCG and Box–Muller: deterministic, with no dependency.
        let mut state: u64 = 0x2545_f491_4f6c_dd1d;
        let mut uniform = move || {
            state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            ((state >> 11) as f64 + 0.5) / (1u64 << 53) as f64
        };
        let mut gaussian = move || {
            let (u, v) = (uniform(), uniform());
            ((-2.0 * u.ln()).sqrt() * (2.0 * core::f64::consts::PI * v).cos()) as f32
        };
        let noise: std::vec::Vec<_> = (0..(seconds / dt) as usize)
            .map(|_| {
                let mut draw = || Vector3::new(gaussian(), gaussian(), gaussian());
                (draw() * n_gyro, draw() * n_accel)
            })
            .collect();
        let scale = 1.0 / dt.sqrt();
        built(
            noise.len(),
            dt,
            dt,
            |i| noise[i].0 * scale,
            |i| noise[i].1 * scale,
        )
    }

    #[test]
    fn a_sensors_white_noise_density_is_recovered_at_any_rate() {
        // Ten seconds at the simulator's IMU densities at 50, 100 and 400 Hz, where the
        // per-sample noise differs by a factor of three and the density does not. At 50 Hz a block holds three
        // samples, 60 ms, which the weighting takes as it is. 200 blocks read N to ±5 %.
        let (n_gyro, n_accel) = (2.6e-4, 2.0e-3);
        for rate in [50.0f32, 100.0, 400.0] {
            let reported = noise(&white(10.0, 1.0 / rate, n_gyro, n_accel)).expect("still");
            assert_close(reported.gyro_white, Vector3::repeat(n_gyro), 0.15);
            assert_close(reported.accel_white, Vector3::repeat(n_accel), 0.15);
        }
    }

    #[test]
    fn the_barometer_floor_is_the_variance_of_its_distinct_readings() {
        // Nine readings held three samples each: the variance of nine readings, 7.5 m² about
        // their mean of 5 m, not of 27 samples.
        let window: std::vec::Vec<_> = (0..27)
            .map(|i| StaticSample {
                baro: Some(Altitude::from_meters((i / 3) as f32 + 1.0)),
                ..still()
            })
            .collect();
        let reported = noise(&spaced(&window, WindowNoise::BLOCK)).expect("still");
        assert_eq!(reported.baro, Some(AltitudeNoise::from_variance(7.5)));
        assert_eq!(reported.baro_readings, 9);
    }

    #[test]
    fn too_few_readings_report_nothing_for_that_sensor() {
        // Seven barometer readings: the IMU's 27 blocks still report, the barometer does not.
        let window: std::vec::Vec<_> = (0..27)
            .map(|i| StaticSample {
                baro: Some(Altitude::from_meters((i / 4) as f32)),
                ..still()
            })
            .collect();
        let reported = noise(&spaced(&window, WindowNoise::BLOCK)).expect("still");
        assert_eq!((reported.baro, reported.baro_readings), (None, 7));

        let few = [still(); (WindowNoise::MIN_READINGS - 1) as usize];
        assert_eq!(noise(&spaced(&few, WindowNoise::BLOCK)), None);
        let enough = [still(); WindowNoise::MIN_READINGS as usize];
        assert!(noise(&spaced(&enough, WindowNoise::BLOCK)).is_some());
    }

    #[test]
    fn a_window_that_moved_reports_no_noise() {
        // Its scatter would be the motion.
        let mut window = [still(); 20];
        window[10].imu = ImuSample::reading(
            AngularRate::body(0.0, 0.0, 1.0),
            Acceleration::body(0.0, 0.0, -GRAVITY),
        );
        assert_eq!(noise(&spaced(&window, WindowNoise::BLOCK)), None);
    }

    /// The window's halves over samples whose specific force is `forces` along body x.
    fn halves_of(forces: &[f64]) -> Option<Halves> {
        let mut sums = BlockSums::new();
        for &force in forces {
            sums.push(Vector3::new(force, 0.0, 0.0), None);
        }
        let (first, second) = sums.split();
        Halves::of(first, second)
    }

    #[test]
    fn a_window_within_the_blocks_splits_at_the_middle_rounded_down() {
        // Seven samples in seven blocks of one: 3 and 4, as `n / 2` splits them.
        let halves = halves_of(&[1.0, 1.0, 1.0, 5.0, 5.0, 5.0, 5.0]).expect("two halves");
        assert_eq!(
            halves.force,
            [
                Acceleration::body(1.0, 0.0, 0.0),
                Acceleration::body(5.0, 0.0, 0.0)
            ]
        );
    }

    #[test]
    fn a_window_past_the_blocks_splits_at_the_boundary_nearest_the_middle() {
        // Twenty samples merge twice, into five blocks of four. The middle, 10, is two from
        // the boundaries at 8 and 12, and the earlier is taken; the later would average
        // four 3s into the first half.
        let forces: std::vec::Vec<f64> = (0..20).map(|i| if i < 8 { 1.0 } else { 3.0 }).collect();
        let halves = halves_of(&forces).expect("two halves");
        assert_eq!(
            halves.force,
            [
                Acceleration::body(1.0, 0.0, 0.0),
                Acceleration::body(3.0, 0.0, 0.0)
            ]
        );
    }

    #[test]
    fn the_halves_are_within_an_eighth_of_the_window_of_equal_at_every_length() {
        // Sample `i` reads `i`, so a first half of `k` samples averages `(k − 1) / 2`: the
        // count comes back out of the mean. Blocks left unequal by a merge would let the
        // first half grow past the bound `BLOCKS` sets.
        for n in 2..=300u32 {
            let forces: std::vec::Vec<f64> = (0..n).map(f64::from).collect();
            let halves = halves_of(&forces).expect("two halves");
            let first = 2.0 * halves.force[0].vector().x + 1.0;
            let n = n as f32;
            assert!(
                first >= 3.0 * n / 8.0 - 0.5 && first <= 5.0 * n / 8.0 + 0.5,
                "a first half of {first} samples in {n}"
            );
        }
    }

    #[test]
    fn a_window_of_one_has_no_halves() {
        assert_eq!(halves_of(&[1.0]), None);
    }

    #[test]
    fn a_window_is_at_rest_until_it_moves_and_never_again_after() {
        let config = Config::default();
        let mut samples = spaced(&[still(); 6], DT);
        // Over the 0.262 rad/s default.
        samples[2].imu = samples[2].imu.with_gyro(AngularRate::body(0.0, 0.4, 0.0));
        let mut window = StaticWindow::new();
        assert!(window.is_at_rest(&config), "an empty window has not moved");
        for (index, sample) in samples.into_iter().enumerate() {
            assert_eq!(window.push(sample), Ok(()));
            assert_eq!(
                window.is_at_rest(&config),
                index < 2,
                "after sample {index}"
            );
            // The same verdict `alignment_of` reaches by measuring the whole window.
            let moved = matches!(
                classify(
                    &window.measured().expect("a sample"),
                    &config.init,
                    config.gravity
                ),
                Alignment::Coarse(Coarse::NotStationary { .. })
            );
            assert_eq!(moved, !window.is_at_rest(&config), "after sample {index}");
        }
    }

    /// The figures `DESIGN.md`, "Measured cost, by function", quotes. Measured on `thumbv6m` with
    /// `-Zprint-type-sizes`, where `u64` and `f64` align to 8 as they do on the 64-bit hosts CI
    /// runs, so the host's `size_of` pins the same figure.
    #[test]
    fn the_window_is_the_size_its_documentation_quotes() {
        assert_eq!(core::mem::size_of::<StaticWindow>(), 944);
        assert_eq!(core::mem::size_of::<StaticSample>(), 80);
    }

    #[test]
    fn a_window_is_long_enough_only_once_it_holds_a_sample() {
        // A `min_duration` of zero is met by any span, the empty window's included, and an
        // empty window is the one `initialize` refuses.
        let mut config = Config::default();
        config.init.min_duration = Seconds::from_secs(0.0);
        let mut window = StaticWindow::new();
        assert!(!window.is_long_enough(&config));
        assert_eq!(window.push(spaced(&[still()], DT)[0]), Ok(()));
        assert!(window.is_long_enough(&config));

        let two_seconds = Config::default();
        let mut window = StaticWindow::new();
        let samples = spaced(&[still(); 8], Seconds::from_secs(0.25));
        for (index, sample) in samples.into_iter().enumerate() {
            assert!(
                !window.is_long_enough(&two_seconds),
                "after {index} samples"
            );
            assert_eq!(window.push(sample), Ok(()));
        }
        assert!(window.is_long_enough(&two_seconds), "8 samples of 0.25 s");
    }

    #[test]
    fn extending_a_window_stops_at_the_first_sample_refused() {
        let mut samples = spaced(&[still(); 5], DT);
        samples[2].imu = samples[2]
            .imu
            .with_accel(Acceleration::body(f32::NAN, 0.0, 0.0));
        let mut window = StaticWindow::new();
        assert_eq!(
            window.try_extend(samples.clone()),
            Err(SampleRefusal::NotFinite)
        );
        // The two before it are in, and the two after it, usable as they are, are not.
        assert_eq!(window.measured(), measure(&samples[..2]));
    }

    #[test]
    fn a_refused_sample_leaves_the_window_as_it_was() {
        // With a barometer, whose readings a refused sample must not reach either: `measured`
        // does not read them.
        let mut good = spaced(&[still(); 4], DT);
        for (index, sample) in good.iter_mut().enumerate() {
            sample.baro = Some(Altitude::from_meters(100.0 + index as f32));
        }
        let mut window = StaticWindow::new();
        assert_eq!(window.push(good[0]), Ok(()));
        assert_eq!(window.push(good[1]), Ok(()));
        // Refused for being earlier, and the clock stays where it was: a sample between the
        // refused time and the last accepted is still behind the window.
        assert!(matches!(
            window.push(good[0]),
            Err(SampleRefusal::InvalidStep { .. })
        ));
        assert!(matches!(
            window.push(good[1]),
            Err(SampleRefusal::InvalidStep { .. })
        ));
        let mut poisoned = good[1];
        poisoned.imu = poisoned
            .imu
            .with_accel(Acceleration::body(f32::NAN, 0.0, 0.0));
        poisoned.baro = Some(Altitude::from_meters(500.0));
        assert_eq!(window.push(poisoned), Err(SampleRefusal::NotFinite));
        assert_eq!(window.push(good[2]), Ok(()));
        assert_eq!(window.push(good[3]), Ok(()));
        let clean = StaticWindow::try_from(good.as_slice()).expect("usable");
        assert_eq!(window.measured(), clean.measured());
        assert_eq!(window.alpha0(), clean.alpha0());
    }

    /// A window whose GNSS reports the vehicle gaining 4 m/s of north velocity over the
    /// four samples — one second at `DT` — that separate the two fixes.
    fn accelerating_window() -> [StaticSample; 8] {
        let mut window = [still(); 8];
        window[1].velocity = Some(Velocity::ned(1.0, 0.0, 0.0));
        window[5].velocity = Some(Velocity::ned(5.0, 0.0, 0.0));
        window
    }

    #[test]
    fn the_window_acceleration_is_the_endpoint_difference_over_the_span() {
        let measured = inertial_acceleration(&spaced(&accelerating_window(), DT))
            .expect("two samples a second apart carry a velocity");
        assert_eq!(measured, Acceleration::ned(4.0, 0.0, 0.0));
    }

    #[test]
    fn velocity_between_the_endpoints_does_not_reach_the_mean() {
        // The mean of a derivative is its endpoint difference, so a noisy fix in the
        // middle of the window is not averaged in — it is not read at all.
        let mut window = accelerating_window();
        window[3].velocity = Some(Velocity::ned(-40.0, 12.0, 7.0));
        assert_eq!(
            inertial_acceleration(&spaced(&window, DT)),
            Some(Acceleration::ned(4.0, 0.0, 0.0))
        );
    }

    #[test]
    fn a_window_with_no_two_dated_velocities_reports_no_acceleration() {
        // Nothing to difference, and a difference over zero seconds is an infinity
        // rather than an acceleration.
        assert_eq!(inertial_acceleration(&spaced(&[still(); 8], DT)), None);
        let mut one = [still(); 8];
        one[4].velocity = Some(Velocity::ned(9.0, 0.0, 0.0));
        assert_eq!(inertial_acceleration(&spaced(&one, DT)), None);
    }

    #[test]
    fn a_moving_window_reports_the_acceleration_gnss_accounts_for() {
        let mut window = accelerating_window();
        // Over the 0.262 rad/s default, so the window classifies as moving.
        window[3].imu = window[3].imu.with_gyro(AngularRate::body(0.0, 0.4, 0.0));
        let alignment = classify_default(&window, DT).expect("moving, not unusable");
        let Alignment::Coarse(Coarse::NotStationary { inertial_accel, .. }) = alignment else {
            panic!("0.4 rad/s is over the default: {alignment:?}");
        };
        assert_eq!(inertial_accel, Some(Acceleration::ned(4.0, 0.0, 0.0)));
    }

    #[test]
    fn a_vehicle_gnss_explains_entirely_is_still_not_at_rest() {
        // A launch off a deck accelerating north at a steady 8 m/s^2, level and not
        // turning: |f| is sqrt(8^2 + g^2), 2.85 m/s^2 off gravity, and GNSS accounts for
        // every bit of it. The window is still not one taken on the ground, which is
        // what `Alignment::Static` and the barometric reference both mean.
        let mut window = [still(); 8];
        for sample in &mut window {
            sample.imu = sample
                .imu
                .with_accel(Acceleration::body(8.0, 0.0, -GRAVITY));
        }
        window[1].velocity = Some(Velocity::ned(1.0, 0.0, 0.0));
        window[5].velocity = Some(Velocity::ned(9.0, 0.0, 0.0));

        let alignment = classify_default(&window, DT).expect("moving, not unusable");
        let Alignment::Coarse(Coarse::NotStationary { inertial_accel, .. }) = alignment else {
            panic!("2.85 m/s^2 is over the 1.961 default: {alignment:?}");
        };
        assert_eq!(inertial_accel, Some(Acceleration::ned(8.0, 0.0, 0.0)));
    }

    #[test]
    fn a_velocity_that_is_not_a_number_is_refused_with_the_rest() {
        let mut window = accelerating_window();
        window[5].velocity = Some(Velocity::ned(f32::NAN, 0.0, 0.0));
        assert_eq!(classify_default(&window, DT), Err(InitError::NotFinite));
    }
}
