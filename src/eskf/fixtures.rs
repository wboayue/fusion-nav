//! Fixtures the tests of `eskf`'s modules share: a clock for rate-and-`dt` samples, and the
//! filters and windows more than one module's tests start from.

use crate::config::Config;
use crate::eskf::Eskf;
use crate::frames::{Body, Ned};
use crate::geodetic::Geodetic;
use crate::health::Propagation;
use crate::init::tests::{gravity_at, spaced, still, turning};
use crate::init::{Alignment, InitError, StaticSample, StaticWindow};
use crate::propagate::ImuSample;
use crate::state::{Covariance, ErrorState, STATES, State};
use crate::units::{
    Altitude, AltitudeNoise, AngularRate, Attitude, MagField, Position, PositionNoise, Seconds,
    Timestamp, Velocity, VelocityNoise,
};

pub(super) const DT: Seconds = Seconds::from_secs(0.01);

/// The fixtures are written as rates and a `dt`; these give them a clock. Only the method
/// names differ from the filter's own, so a test reads as the call it makes.
pub(super) trait Clocked {
    /// [`Eskf::predict`] on `imu`'s rates over `dt`, timed `dt` after the filter's clock.
    fn step(&mut self, imu: ImuSample, dt: Seconds) -> Propagation;
    /// [`Eskf::initialize`] on `window`, each sample over `dt`.
    fn initialize_over(
        &mut self,
        window: &[StaticSample],
        dt: Seconds,
    ) -> Result<Alignment, InitError>;
    /// [`Eskf::initialize_from`] at the epoch.
    fn seed(&mut self, state: State, covariance: Covariance) -> Result<Alignment, InitError>;
    /// The filter's time, for a measurement taken now: the epoch before initialization.
    fn now(&self) -> Timestamp;
}

impl Clocked for Eskf {
    fn step(&mut self, imu: ImuSample, dt: Seconds) -> Propagation {
        let time = self.time().unwrap_or_default().after(dt);
        self.predict(imu.timed(time, dt))
    }

    fn initialize_over(
        &mut self,
        window: &[StaticSample],
        dt: Seconds,
    ) -> Result<Alignment, InitError> {
        self.initialize(&StaticWindow::try_from(spaced(window, dt).as_slice())?)
    }

    fn seed(&mut self, state: State, covariance: Covariance) -> Result<Alignment, InitError> {
        self.initialize_from(state, covariance, Timestamp::ZERO)
    }

    fn now(&self) -> Timestamp {
        self.time().unwrap_or_default()
    }
}

/// A window of exactly `Initialization::min_duration`: 8 samples at 4 Hz is 2 s.
/// No barometer, so no reference is established.
pub(super) fn initialized() -> Eskf {
    let mut filter = Eskf::default();
    let alignment = filter
        .initialize_over(&[still(); 8], Seconds::from_secs(0.25))
        .expect("a 2 s window of stillness");
    assert_eq!(alignment, Alignment::Static);
    filter
}

/// [`initialized`] with the position hold off: for a test of what an unaided filter does
/// without it, which the default hold, engaged from the first step, would otherwise hide.
pub(super) fn unheld() -> Eskf {
    let mut filter = Eskf::new(Config {
        hold: None,
        ..Config::default()
    })
    .unwrap();
    let alignment = filter
        .initialize_over(&[still(); 8], Seconds::from_secs(0.25))
        .expect("a 2 s window of stillness");
    assert_eq!(alignment, Alignment::Static);
    filter
}

/// The same window with a barometer reading on every sample, scattered about `altitude`
/// as a real one is. Eight identical readings are one reading held, which sets no
/// reference. The offsets are exact in binary, so the mean is exactly `altitude`.
pub(super) fn window_at(altitude: f32) -> [StaticSample; 8] {
    window_with_baro([-0.5, 0.5, 0.0, -0.25, 0.25, 0.0, -0.125, 0.125].map(|d| altitude + d))
}

/// The same window with a barometer reading on every sample.
pub(super) fn window_with_baro(altitudes: [f32; 8]) -> [StaticSample; 8] {
    altitudes.map(|altitude| StaticSample {
        baro: Some(Altitude::from_meters(altitude)),
        ..still()
    })
}

/// A still window carrying a magnetometer on every sample, which is what makes
/// heading an estimate rather than a prior. `initialized()`'s window has none.
pub(super) fn window_with_mag() -> [StaticSample; 8] {
    [StaticSample {
        mag: Some(MagField::body(0.22, 0.0, 0.44)),
        ..still()
    }; 8]
}

/// Timers only run for a source that has been accepted, so fuse one first. Baro
/// fusion needs a reference, so the window carries one.
pub(super) fn aided() -> Eskf {
    aided_with(Config::default())
}

/// [`aided`] under another configuration.
pub(super) fn aided_with(config: Config) -> Eskf {
    let mut filter = Eskf::new(config).unwrap();
    let _ = filter
        .initialize_over(&window_at(100.0), Seconds::from_secs(0.25))
        .expect("a 2 s window of stillness");
    assert!(
        filter
            .fuse_baro_altitude(
                filter.now(),
                Altitude::from_meters(100.0),
                AltitudeNoise::from_sigma(2.0)
            )
            .is_accepted()
    );
    filter
}

/// An aided filter flying north at 20 m/s, as `logging_dropout` does into its gap.
pub(super) fn flying(config: Config) -> Eskf {
    let mut filter = aided_with(config);
    assert!(filter.reset_position_to(
        Position::ned(0.0, 0.0, -100.0),
        PositionNoise::horizontal_vertical(0.5, 0.5)
    ));
    assert!(filter.reset_velocity_to(
        Velocity::ned(20.0, 0.0, 0.0),
        VelocityNoise::from_speed_accuracy(0.1)
    ));
    filter
}

/// `Config::gravity` is the `γ` every equation reads, and each test built on [`at_site`], in
/// `predict.rs` and `start.rs`, fails if one of them reads `GRAVITY` instead: 9.79 is a site's
/// value the default is 0.017 m s⁻² off.
pub(super) const SITE_GRAVITY: f32 = 9.79;

/// The default configuration at a site whose gravity is [`SITE_GRAVITY`].
pub(super) fn at_site() -> Config {
    Config {
        gravity: SITE_GRAVITY,
        ..Config::default()
    }
}

/// A window taken while the vehicle was moving, reading 250 m: a restart in flight.
pub(super) fn moving_window_at(altitude: f32) -> [StaticSample; 8] {
    let mut window = window_at(altitude);
    window[3].imu = window[3].imu.with_gyro(AngularRate::body(0.0, 0.4, 0.0));
    window
}

/// An attitude and biases such as a companion AHRS would hand over, with the
/// uncertainty that source reports rather than the static-window figures.
pub(super) fn seed() -> (State, Covariance) {
    let state = State {
        velocity: Velocity::ned(18.0, 0.0, 0.0),
        gyro_bias: AngularRate::body(0.001, -0.002, 0.0005),
        ..State::default()
    };
    let mut sigmas = [0.5f32; STATES];
    sigmas[ErrorState::AttitudeZ.index()] = 1.0; // a moving start knows yaw poorly
    (state, Covariance::from_sigmas(sigmas))
}

/// A still window of a vehicle parked at this attitude, reading nothing but gravity.
pub(super) fn window_tilted(roll: f32, pitch: f32) -> [StaticSample; 8] {
    [StaticSample {
        imu: still().imu.with_accel(gravity_at(roll, pitch, 0.0)),
        ..still()
    }; 8]
}

/// [`seed`] at `attitude`, with gyroscope bias `bias`.
pub(super) fn seeded(attitude: Attitude, bias: AngularRate<Body>) -> Eskf {
    let mut filter = Eskf::default();
    let (state, covariance) = seed();
    let state = State {
        attitude,
        gyro_bias: bias,
        ..state
    };
    let _ = filter.seed(state, covariance).expect("a sane seed");
    filter
}

/// A filter that started while moving: it knows neither where it is nor how fast.
pub(super) fn coarse() -> Eskf {
    let mut filter = Eskf::default();
    let mut window = turning(0.0, 0.4, 0.0);
    // No magnetometer, so heading stays a prior whatever the covariance says.
    for sample in &mut window {
        sample.mag = None;
    }
    let alignment = filter
        .initialize_over(&window, Seconds::from_secs(0.25))
        .expect("moving, not unusable");
    assert!(matches!(alignment, Alignment::Coarse(..)));
    filter
}

pub(super) fn zurich() -> Geodetic {
    Geodetic::from_degrees(47.3977, 8.5456, 488.0)
}

pub(super) fn near(a: Position<Ned>, b: Position<Ned>) -> bool {
    (a.vector() - b.vector()).norm() < 1e-2
}

pub(super) fn mast() -> Position<Body> {
    Position::body(1.0, 0.0, -0.5)
}

/// A static window of a tailsitter standing on its tail, pitched 90° nose-up: body x
/// points up, so the body diagonal of the attitude block is heading, one tilt and the
/// other tilt, in that order.
pub(super) fn on_its_tail() -> Eskf {
    let mut filter = Eskf::default();
    let alignment = filter
        .initialize_over(
            &window_tilted(0.0, core::f32::consts::FRAC_PI_2),
            Seconds::from_secs(0.25),
        )
        .expect("a parked vehicle reads exactly g, however it is standing");
    assert_eq!(alignment, Alignment::Static);
    filter
}

/// Predict at rest for `seconds`, offering `offer` every `every` steps.
pub(super) fn hold(
    filter: &mut Eskf,
    seconds: f32,
    every: usize,
    mut offer: impl FnMut(&mut Eskf),
) {
    let steps = (seconds / DT.as_secs()).round() as usize;
    for step in 1..=steps {
        assert_eq!(filter.step(still().imu, DT), Propagation::Propagated);
        if step % every == 0 {
            offer(filter);
        }
    }
}

pub(super) fn one_metre() -> PositionNoise<Ned> {
    PositionNoise::from_sigma(1.0, 1.0, 1.0)
}

/// A static start, then flying `velocity` with its heading still unobserved, a GNSS
/// velocity just accepted to hold it.
pub(super) fn cruising(velocity: Velocity<Ned>) -> Eskf {
    let mut filter = initialized();
    assert!(filter.reset_velocity_to(velocity, VelocityNoise::from_speed_accuracy(0.3)));
    hold_velocity(&mut filter, velocity);
    filter
}

pub(super) fn hold_velocity(filter: &mut Eskf, velocity: Velocity<Ned>) {
    let noise = VelocityNoise::from_speed_accuracy(0.3);
    let outcome = filter.fuse_gnss_velocity(filter.now(), velocity, noise, Position::zero());
    assert!(outcome.is_accepted(), "{outcome:?}");
}
