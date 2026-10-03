//! Adversarial inputs at every public entry point: hostile values, times and orderings,
//! with the filter's invariants asserted after every call.
//!
//! A child of `eskf` rather than an integration test, so the invariants read the whole
//! covariance of (30′), `[P, P_xb; P_xbᵀ, P_bb]`, and not only the `P` the API publishes.
//! Seeded and fixed in count, so a failure reproduces and CI time is bounded; proptest shrinks
//! a failing sequence to the fewest calls that still fail.
//!
//! Two properties share one generator. [`hostile`] draws the values that break estimators
//! (NaN, ±∞, subnormals, `f32::MAX`, zero and negative variances, times backwards and past
//! every horizon) and asserts only what must hold whatever arrives. [`ordinary`] draws a
//! vehicle sitting still under honest sensors, and asserts as well that the floor of (42′)
//! is never reached, since a floored variance there is a covariance collapsing rather than a
//! refusal ([`Diagnostics::floored`]).

use std::{format, vec};

use proptest::prelude::*;
use proptest::sample::select;
use proptest::test_runner::{Config as Runner, RngSeed};

use crate::config::GRAVITY;
use crate::prelude::*;
use crate::state::{Offset, STATES};
use crate::units::Quaternion;

use super::Eskf;

/// 400 Hz, the rate the embedded example collects at.
const STEP_US: i64 = 2_500;

/// Where [`ordinary`]'s geodetic fixes sit: `data/flight.csv`'s origin.
const SITE: (f64, f64, f64) = (47.397_742, 8.545_594, 488.0);

/// Cases per property, sized so the suite adds seconds rather than minutes to
/// `cargo test --all-targets`.
const CASES: u32 = 256;

fn runner() -> Runner {
    Runner {
        cases: CASES,
        rng_seed: RngSeed::Fixed(44),
        failure_persistence: None,
        ..Runner::default()
    }
}

/// A scalar in `lo..hi`, or, when `hostile`, sometimes one of the values that break
/// estimators.
fn scalar(hostile: bool, lo: f32, hi: f32) -> BoxedStrategy<f32> {
    if !hostile {
        return (lo..hi).boxed();
    }
    prop_oneof![
        3 => lo..hi,
        1 => select(vec![
            f32::NAN,
            f32::INFINITY,
            f32::NEG_INFINITY,
            0.0,
            -0.0,
            f32::MIN_POSITIVE,
            1.0e-40,
            -1.0e-40,
            f32::MAX,
            f32::MIN,
            1.0e20,
            -1.0,
        ]),
    ]
    .boxed()
}

/// Three of [`scalar`], or, when `hostile`, sometimes one value on every axis: an overflow
/// that needs two large components together, a lever arm rotated, is otherwise a draw in
/// thousands.
fn vector(hostile: bool, lo: f32, hi: f32) -> BoxedStrategy<[f32; 3]> {
    let apart = [
        scalar(hostile, lo, hi),
        scalar(hostile, lo, hi),
        scalar(hostile, lo, hi),
    ];
    if !hostile {
        return apart.boxed();
    }
    prop_oneof![4 => apart, 1 => scalar(true, lo, hi).prop_map(|v| [v; 3])].boxed()
}

/// A measurement's time relative to the filter's clock, µs: in the recent past, or, when
/// `hostile`, either side of the history's reach and ahead of the state.
fn age(hostile: bool) -> BoxedStrategy<i64> {
    if !hostile {
        return (-200_000i64..=0).boxed();
    }
    prop_oneof![
        3 => -400_000i64..=120_000,
        1 => select(vec![
            0,
            i64::MIN / 2,
            i64::MAX / 2,
            -300_001,
            -299_999,
            99_999,
            100_001,
        ]),
    ]
    .boxed()
}

/// An IMU step, µs: ordinary, or at and either side of `Config::max_predict_dt`, zero,
/// backwards, a coast's length and past the end of `u64` time.
fn step(hostile: bool) -> BoxedStrategy<i64> {
    if !hostile {
        return (2_000i64..=3_000).boxed();
    }
    prop_oneof![
        6 => 2_000i64..=3_000,
        1 => select(vec![
            0,
            -STEP_US,
            1,
            99_999,
            100_000,
            100_001,
            3_000_000,
            6_500_000,
            i64::MAX / 2,
        ]),
    ]
    .boxed()
}

/// Delta intervals that disagree with the step, when `hostile`.
fn intervals(hostile: bool) -> BoxedStrategy<Option<(f32, f32)>> {
    if !hostile {
        return Just(None).boxed();
    }
    let sigma = || scalar(true, 0.0001, 0.2);
    prop::option::weighted(0.2, (sigma(), sigma())).boxed()
}

/// One sample of a still window replaced by whatever arrives, when `hostile`: its IMU, its
/// barometer or both, since a hostile IMU reading breaks rest and a window not at rest sets
/// no barometric reference for a hostile reading to reach.
fn odd(hostile: bool) -> BoxedStrategy<Option<OddSample>> {
    if !hostile {
        return Just(None).boxed();
    }
    let h = true;
    prop::option::weighted(
        0.3,
        (
            0usize..900,
            select(vec![Odd::Imu, Odd::Baro, Odd::Both]),
            vector(h, -1.0, 1.0),
            vector(h, -10.0, 10.0),
            scalar(h, -5.0, 5.0),
        ),
    )
    .boxed()
}

/// The sample [`odd`] replaces, which of its readings, and the gyroscope, accelerometer and
/// barometer values it gets.
type OddSample = (usize, Odd, [f32; 3], [f32; 3], f32);

/// Which of a window sample's readings [`odd`] replaces.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Odd {
    Imu,
    Baro,
    Both,
}

#[derive(Clone, Debug)]
enum Op {
    Predict {
        step: i64,
        gyro: [f32; 3],
        accel: [f32; 3],
        /// Delta intervals that disagree with the step; `None` integrates over the step.
        intervals: Option<(f32, f32)>,
        /// Hand the step over as two halves summed by [`ImuSample::accumulate`].
        split: bool,
    },
    GnssPosition {
        age: i64,
        ned: [f32; 3],
        sigma: (f32, f32),
        antenna: [f32; 3],
    },
    GnssGeodetic {
        age: i64,
        offset: [f32; 3],
        sigma: (f32, f32),
        antenna: [f32; 3],
    },
    GnssVelocity {
        age: i64,
        ned: [f32; 3],
        sigma: f32,
        antenna: [f32; 3],
    },
    Baro {
        age: i64,
        altitude: f32,
        sigma: f32,
    },
    Mag {
        age: i64,
        field: [f32; 3],
        sigma: f32,
    },
    GnssHeading {
        age: i64,
        heading: f32,
        sigma: f32,
    },
    Course {
        age: i64,
        sigma: f32,
    },
    ResetPosition {
        ned: [f32; 3],
        sigma: (f32, f32),
    },
    ResetVelocity {
        ned: [f32; 3],
        sigma: f32,
    },
    SetOrigin {
        offset: [f32; 3],
    },
    SetBaroReference {
        altitude: f32,
        sigma: f32,
    },
    SetDeclination(f32),
}

fn op(hostile: bool) -> BoxedStrategy<Op> {
    let h = hostile;
    let sigma = || scalar(h, 0.05, 5.0);
    let accel = [
        scalar(h, -0.5, 0.5),
        scalar(h, -0.5, 0.5),
        scalar(h, -GRAVITY - 0.5, -GRAVITY + 0.5),
    ];
    let predict = (
        step(h),
        vector(h, -0.05, 0.05),
        accel,
        intervals(h),
        any::<bool>(),
    )
        .prop_map(|(step, gyro, accel, intervals, split)| Op::Predict {
            step,
            gyro,
            accel,
            intervals,
            split,
        });
    prop_oneof![
        12 => predict,
        2 => (age(h), vector(h, -3.0, 3.0), (sigma(), sigma()), vector(h, -0.2, 0.2))
            .prop_map(|(age, ned, sigma, antenna)| Op::GnssPosition { age, ned, sigma, antenna }),
        1 => (age(h), vector(h, -3.0, 3.0), (sigma(), sigma()), vector(h, -0.2, 0.2))
            .prop_map(|(age, offset, sigma, antenna)| Op::GnssGeodetic { age, offset, sigma, antenna }),
        2 => (age(h), vector(h, -0.3, 0.3), sigma(), vector(h, -0.2, 0.2))
            .prop_map(|(age, ned, sigma, antenna)| Op::GnssVelocity { age, ned, sigma, antenna }),
        2 => (age(h), scalar(h, -2.0, 2.0), sigma())
            .prop_map(|(age, altitude, sigma)| Op::Baro { age, altitude, sigma }),
        1 => (age(h), vector(h, -0.6, 0.6), scalar(h, 0.02, 0.5))
            .prop_map(|(age, field, sigma)| Op::Mag { age, field, sigma }),
        1 => (age(h), scalar(h, -3.2, 3.2), scalar(h, 0.02, 0.5))
            .prop_map(|(age, heading, sigma)| Op::GnssHeading { age, heading, sigma }),
        1 => (age(h), scalar(h, 0.02, 0.5)).prop_map(|(age, sigma)| Op::Course { age, sigma }),
        1 => (vector(h, -3.0, 3.0), (sigma(), sigma()))
            .prop_map(|(ned, sigma)| Op::ResetPosition { ned, sigma }),
        1 => (vector(h, -0.3, 0.3), sigma()).prop_map(|(ned, sigma)| Op::ResetVelocity { ned, sigma }),
        1 => vector(h, -3.0, 3.0).prop_map(|offset| Op::SetOrigin { offset }),
        1 => (scalar(h, -2.0, 2.0), sigma())
            .prop_map(|(altitude, sigma)| Op::SetBaroReference { altitude, sigma }),
        1 => scalar(h, -0.3, 0.3).prop_map(Op::SetDeclination),
    ]
    .boxed()
}

/// How the filter starts.
#[derive(Clone, Debug)]
enum Start {
    /// Never initialized: every measurement is refused.
    Uninitialized,
    /// A still window of `samples` at 400 Hz, one of them replaced by `odd` when present.
    Window {
        samples: usize,
        baro: bool,
        mag: bool,
        odd: Option<OddSample>,
    },
    /// One sample, whatever it reads.
    Coarse { gyro: [f32; 3], accel: [f32; 3] },
    /// A seed: level at the origin, every variance `variance`, and the first two position
    /// errors correlated by `asymmetry` one way and half as much again the other, as a
    /// caller's own arithmetic can leave them.
    Seeded { variance: f32, asymmetry: f32 },
}

fn start(hostile: bool) -> BoxedStrategy<Start> {
    let h = hostile;
    let window = (1usize..=900, any::<bool>(), any::<bool>(), odd(h)).prop_map(
        |(samples, baro, mag, odd)| Start::Window {
            samples,
            baro,
            mag,
            odd,
        },
    );
    let coarse = (
        vector(h, -0.05, 0.05),
        [
            scalar(h, -0.5, 0.5),
            scalar(h, -0.5, 0.5),
            scalar(h, -GRAVITY - 0.5, -GRAVITY + 0.5),
        ],
    )
        .prop_map(|(gyro, accel)| Start::Coarse { gyro, accel });
    let seeded =
        (scalar(h, 1.0e-4, 1.0), scalar(h, -1.0e-5, 1.0e-5)).prop_map(|(variance, asymmetry)| {
            Start::Seeded {
                variance,
                asymmetry,
            }
        });
    if hostile {
        prop_oneof![1 => Just(Start::Uninitialized), 6 => window, 2 => coarse, 2 => seeded].boxed()
    } else {
        prop_oneof![6 => window, 2 => coarse, 2 => seeded].boxed()
    }
}

/// The configurations every switch the filter has can be in.
fn config() -> impl Strategy<Value = Config> {
    let quick = Some(Seconds::from_secs(0.3));
    select(vec![
        Config::default(),
        Config {
            recovery: Recovery::OFF,
            ..Config::default()
        },
        Config {
            recovery: Recovery {
                gnss_position: quick,
                gnss_height: quick,
                gnss_velocity: quick,
                baro_altitude: quick,
                mag_heading: quick,
                gnss_heading: quick,
                course: quick,
            },
            ..Config::default()
        },
        Config {
            coast: None,
            correlation: Correlation::WHITE,
            ..Config::default()
        },
        Config {
            baro_reference_from_estimate: false,
            accuracy: Accuracy {
                horizon: Seconds::ZERO,
                ..Accuracy::default()
            },
            ..Config::default()
        },
    ])
}

fn at(time: Timestamp, offset_us: i64) -> Timestamp {
    Timestamp::from_micros(time.as_micros().saturating_add_signed(offset_us))
}

fn sample(time: Timestamp, gyro: [f32; 3], accel: [f32; 3], interval: Seconds) -> ImuSample {
    ImuSample::from_rates(
        time,
        AngularRate::body(gyro[0], gyro[1], gyro[2]),
        Acceleration::body(accel[0], accel[1], accel[2]),
        interval,
    )
}

/// The `i`th sample of a still window: a deterministic scatter, so the barometer's readings
/// differ and set a reference, and the IMU is not a single value held.
fn still(i: usize, baro: bool, mag: bool) -> StaticSample {
    let wobble = ((i * 7919) % 13) as f32 / 13.0 - 0.5;
    let time = Timestamp::from_micros(1_000_000 + i as u64 * STEP_US as u64);
    StaticSample {
        imu: sample(
            time,
            [0.001 * wobble, -0.001 * wobble, 0.0005],
            [0.01 * wobble, -0.01 * wobble, -GRAVITY + 0.02 * wobble],
            Seconds::from_secs(STEP_US as f32 * 1e-6),
        ),
        mag: mag.then(|| MagField::body(0.22, 0.01 * wobble, 0.44)),
        baro: baro.then(|| Altitude::from_meters(100.0 + 0.3 * wobble)),
        velocity: None,
    }
}

fn geodetic(offset: [f32; 3]) -> Geodetic {
    // A meter is about 9e-6° of latitude here; the offset is in meters, or hostile.
    let degree = 9.0e-6;
    Geodetic::from_degrees(
        SITE.0 + f64::from(offset[0]) * degree,
        SITE.1 + f64::from(offset[1]) * degree,
        SITE.2 + f64::from(offset[2]),
    )
}

/// Start the filter, checking the window's own contract on the way: a refused sample leaves
/// the window as it was.
fn begin(filter: &mut Eskf, start: &Start) -> Result<(), TestCaseError> {
    match *start {
        Start::Uninitialized => {}
        Start::Window {
            samples,
            baro,
            mag,
            odd,
        } => {
            let mut window = StaticWindow::new();
            for i in 0..samples {
                let mut s = still(i, baro, mag);
                if let Some((j, which, gyro, accel, altitude)) = odd
                    && i == j
                {
                    if which != Odd::Baro {
                        s.imu = sample(s.imu.time, gyro, accel, s.imu.angle_interval);
                    }
                    if which != Odd::Imu {
                        s.baro = Some(Altitude::from_meters(altitude));
                    }
                }
                let before = format!("{window:?}");
                if window.push(s).is_err() {
                    prop_assert_eq!(
                        &before,
                        &format!("{window:?}"),
                        "a refused sample moved the window"
                    );
                }
            }
            let _ = window.is_at_rest(filter.config());
            let _ = window.is_long_enough(filter.config());
            let _ = window.noise(filter.config());
            let classified = filter.alignment_of(&window);
            let initialized = filter.initialize(&window);
            prop_assert_eq!(
                classified,
                initialized,
                "alignment_of disagrees with initialize"
            );
        }
        Start::Coarse { gyro, accel } => {
            let imu = sample(
                Timestamp::from_micros(1_000_000),
                gyro,
                accel,
                Seconds::from_secs(STEP_US as f32 * 1e-6),
            );
            let _ = filter.initialize_coarse(imu);
        }
        Start::Seeded {
            variance,
            asymmetry,
        } => {
            let state = State {
                attitude: Attitude::level(),
                ..State::default()
            };
            let mut p = *Covariance::from_sigmas([variance.sqrt(); STATES]).as_matrix();
            p[(0, 1)] = asymmetry;
            p[(1, 0)] = asymmetry * 1.5;
            let covariance = Covariance::from_matrix(p);
            let _ = filter.initialize_from(state, covariance, Timestamp::from_micros(1_000_000));
        }
    }
    check(filter, "start", true)
}

/// Everything a refusal must leave as it was.
#[derive(PartialEq, Debug)]
struct Snapshot {
    state: State,
    covariance: Covariance,
    offset: Offset,
    time: Option<Timestamp>,
    origin: Option<LocalOrigin>,
    declination: (Radians, bool, bool),
    baro_reference: Option<Altitude>,
    unestablished: super::Unestablished,
    aligned: bool,
}

fn snapshot(filter: &Eskf) -> Snapshot {
    Snapshot {
        // The stored estimate: `state()` derives a status from the clock, which a refused
        // step still moves.
        state: *filter.estimate.state(),
        covariance: *filter.covariance(),
        offset: filter.offset,
        time: filter.time(),
        origin: filter.origin,
        declination: (
            filter.declination,
            filter.declination_set,
            filter.magnetic_north,
        ),
        baro_reference: filter.baro_reference,
        unestablished: filter.unestablished,
        aligned: filter.aligned,
    }
}

/// What must hold after any call, whatever it was handed. `project` also runs
/// [`Eskf::predicted_validity`], which projects `P` up to 64 steps and is most of the suite's
/// time, so [`run`] asks for it on a sample of calls rather than every one.
fn check(filter: &Eskf, after: &str, project: bool) -> Result<(), TestCaseError> {
    let state = filter.estimate.state();
    prop_assert!(state.is_finite(), "{after}: state not finite: {state:?}");
    let norm = state.attitude.quaternion().norm();
    prop_assert!((norm - 1.0).abs() < 1e-3, "{after}: quaternion norm {norm}");

    let p = filter.covariance.as_matrix();
    for i in 0..STATES {
        prop_assert!(
            p[(i, i)] > 0.0 && p[(i, i)].is_finite() || !filter.initialized,
            "{after}: P[{i},{i}] = {}",
            p[(i, i)]
        );
        for j in 0..STATES {
            prop_assert!(p[(i, j)].is_finite(), "{after}: P[{i},{j}] = {}", p[(i, j)]);
            prop_assert_eq!(
                p[(i, j)],
                p[(j, i)],
                "{}: P not symmetric at {},{}",
                after,
                i,
                j
            );
        }
    }

    let offset = &filter.offset;
    prop_assert!(
        offset.variance.is_finite() && offset.variance >= 0.0,
        "{after}: P_bb = {}",
        offset.variance
    );
    prop_assert!(
        offset.cross.iter().all(|c| c.is_finite()),
        "{after}: P_xb not finite"
    );
    if filter.baro_reference.is_none() {
        prop_assert_eq!(
            *offset,
            Offset::default(),
            "{}: an offset with no reference",
            after
        );
    }

    // Every read is derivable, and none claims what nothing established.
    let validity = filter.validity();
    if project {
        let _ = filter.predicted_validity();
    }
    let _ = filter.attitude_variance();
    let _ = filter.geodetic_position();
    let _ = filter.angular_rate();
    let _ = filter.diagnostics().sources();
    let _ = filter.state();
    if !filter.initialized {
        prop_assert_eq!(
            validity,
            Validity::NONE,
            "{}: validity before a start",
            after
        );
    }
    let never = filter.unestablished;
    prop_assert!(
        !(never.position && (validity.horizontal_position || validity.vertical_position)),
        "{after}: a position nothing established reads valid"
    );
    prop_assert!(
        !(never.velocity && (validity.horizontal_velocity || validity.vertical_velocity)),
        "{after}: a velocity nothing established reads valid"
    );
    prop_assert!(
        !(never.heading && validity.heading),
        "{after}: a heading nothing established reads valid"
    );
    Ok(())
}

/// A `fuse_*` touches its own sources' health and no other's, and a refusal counts once on
/// the source it names.
fn health_of(
    filter: &Eskf,
    before: &Diagnostics,
    own: &[&str],
    refused: &[(&str, bool)],
) -> Result<(), TestCaseError> {
    let after = filter.diagnostics().sources();
    for ((name, was), (_, now)) in before.sources().iter().zip(after.iter()) {
        if !own.contains(name) {
            prop_assert_eq!(was, now, "{} moved by another source's measurement", name);
        }
        if let Some((_, true)) = refused.iter().find(|(source, _)| source == name) {
            prop_assert_eq!(
                now.refused,
                was.refused + 1,
                "{} refused and not counted",
                name
            );
        }
    }
    Ok(())
}

/// Run one call and check it: a refusal or rejection leaves the state, the covariance, the
/// offset and the clock as they were, and a `predict` moves the clock as its contract says.
fn apply(filter: &mut Eskf, op: &Op, project: bool) -> Result<(), TestCaseError> {
    let before = snapshot(filter);
    let health = *filter.diagnostics();
    let now = filter.time().unwrap_or(Timestamp::from_micros(1_000_000));
    let is_refused = |outcome: Fusion| outcome.refusal().is_some();
    let untouched = |filter: &Eskf, what: &str| -> Result<(), TestCaseError> {
        prop_assert_eq!(&snapshot(filter), &before, "{} changed the filter", what);
        Ok(())
    };
    let fused = |outcome: Fusion| outcome.is_accepted() || outcome.is_reset();
    let horizontal = |sigma: (f32, f32)| PositionNoise::horizontal_vertical(sigma.0, sigma.1);

    match *op {
        Op::Predict {
            step,
            gyro,
            accel,
            intervals,
            split,
        } => {
            let time = at(now, step);
            let dt = Seconds::from_secs(step as f32 * 1e-6);
            let mut imu = sample(time, gyro, accel, dt);
            if let Some((angle, velocity)) = intervals {
                imu.angle_interval = Seconds::from_secs(angle);
                imu.velocity_interval = Seconds::from_secs(velocity);
            }
            if split {
                let half = Seconds::from_secs(dt.as_secs() / 2.0);
                let first = sample(at(now, step / 2), gyro, accel, half);
                imu = first.accumulate(sample(time, gyro, accel, half));
            }
            let outcome = filter.predict(imu);
            match outcome {
                Propagation::Propagated | Propagation::Coasted { .. } => {
                    prop_assert_eq!(
                        filter.time(),
                        Some(imu.time),
                        "{:?} left the clock",
                        outcome
                    );
                }
                Propagation::InvalidStep { .. } => untouched(filter, "InvalidStep")?,
                refused => {
                    prop_assert_eq!(
                        (
                            *filter.estimate.state(),
                            *filter.covariance(),
                            filter.offset
                        ),
                        (before.state, before.covariance, before.offset),
                        "{:?} changed the estimate",
                        refused
                    );
                    if filter.initialized {
                        prop_assert_eq!(
                            filter.time(),
                            Some(imu.time),
                            "{:?} held the clock",
                            refused
                        );
                    }
                }
            }
        }
        Op::GnssPosition {
            age,
            ned,
            sigma,
            antenna,
        } => {
            let outcome = filter.fuse_gnss_position(
                at(now, age),
                Position::ned(ned[0], ned[1], ned[2]),
                horizontal(sigma),
                Position::body(antenna[0], antenna[1], antenna[2]),
            );
            if !fused(outcome.horizontal) && !fused(outcome.height) {
                untouched(filter, "an unfused GNSS position")?;
            }
            let refused = [
                ("gnss_position", is_refused(outcome.horizontal)),
                ("gnss_height", is_refused(outcome.height)),
            ];
            health_of(filter, &health, &["gnss_position", "gnss_height"], &refused)?;
        }
        Op::GnssGeodetic {
            age,
            offset,
            sigma,
            antenna,
        } => {
            let outcome = filter.fuse_gnss_geodetic(
                at(now, age),
                geodetic(offset),
                horizontal(sigma),
                Position::body(antenna[0], antenna[1], antenna[2]),
            );
            if !fused(outcome.horizontal) && !fused(outcome.height) {
                untouched(filter, "an unfused geodetic fix")?;
            }
            let refused = [
                ("gnss_position", is_refused(outcome.horizontal)),
                ("gnss_height", is_refused(outcome.height)),
            ];
            health_of(filter, &health, &["gnss_position", "gnss_height"], &refused)?;
        }
        Op::GnssVelocity {
            age,
            ned,
            sigma,
            antenna,
        } => {
            let outcome = filter.fuse_gnss_velocity(
                at(now, age),
                Velocity::ned(ned[0], ned[1], ned[2]),
                VelocityNoise::from_speed_accuracy(sigma),
                Position::body(antenna[0], antenna[1], antenna[2]),
            );
            if !fused(outcome) {
                untouched(filter, "an unfused GNSS velocity")?;
            }
            health_of(
                filter,
                &health,
                &["gnss_velocity"],
                &[("gnss_velocity", is_refused(outcome))],
            )?;
        }
        Op::Baro {
            age,
            altitude,
            sigma,
        } => {
            let outcome = filter.fuse_baro_altitude(
                at(now, age),
                Altitude::from_meters(100.0 + altitude),
                AltitudeNoise::from_sigma(sigma),
            );
            if !fused(outcome) {
                // A first altitude seeds the reference from the estimate, which is
                // the one unfused outcome that moves the offset.
                prop_assert_eq!(
                    (*filter.estimate.state(), *filter.covariance()),
                    (before.state, before.covariance),
                    "an unfused altitude changed the estimate"
                );
            }
            let refused = [("baro_altitude", is_refused(outcome))];
            health_of(filter, &health, &["baro_altitude"], &refused)?;
        }
        Op::Mag { age, field, sigma } => {
            let outcome = filter.fuse_mag_heading(
                at(now, age),
                MagField::body(0.22 + field[0], field[1], 0.44 + field[2]),
                HeadingNoise::from_sigma(sigma),
            );
            if !fused(outcome) {
                untouched(filter, "an unfused magnetic heading")?;
            }
            health_of(
                filter,
                &health,
                &["mag_heading"],
                &[("mag_heading", is_refused(outcome))],
            )?;
        }
        Op::GnssHeading {
            age,
            heading,
            sigma,
        } => {
            let outcome = filter.fuse_gnss_heading(
                at(now, age),
                Radians::from_radians(heading),
                HeadingNoise::from_sigma(sigma),
            );
            if !fused(outcome) {
                untouched(filter, "an unfused GNSS heading")?;
            }
            health_of(
                filter,
                &health,
                &["gnss_heading"],
                &[("gnss_heading", is_refused(outcome))],
            )?;
        }
        Op::Course { age, sigma } => {
            let outcome = filter.fuse_course(at(now, age), HeadingNoise::from_sigma(sigma));
            if !fused(outcome) {
                untouched(filter, "an unfused course")?;
            }
            health_of(
                filter,
                &health,
                &["course"],
                &[("course", is_refused(outcome))],
            )?;
        }
        Op::ResetPosition { ned, sigma } => {
            if !filter.reset_position_to(Position::ned(ned[0], ned[1], ned[2]), horizontal(sigma)) {
                untouched(filter, "a refused position reset")?;
            }
        }
        Op::ResetVelocity { ned, sigma } => {
            let noise = VelocityNoise::from_speed_accuracy(sigma);
            if !filter.reset_velocity_to(Velocity::ned(ned[0], ned[1], ned[2]), noise) {
                untouched(filter, "a refused velocity reset")?;
            }
        }
        Op::SetOrigin { offset } => {
            if !filter.set_origin(geodetic(offset)) {
                untouched(filter, "a refused origin")?;
            }
        }
        Op::SetBaroReference { altitude, sigma } => {
            let accepted = filter.set_baro_reference(
                Altitude::from_meters(100.0 + altitude),
                AltitudeNoise::from_sigma(sigma),
            );
            if !accepted {
                untouched(filter, "a refused barometric reference")?;
            }
        }
        Op::SetDeclination(declination) => {
            if !filter.set_magnetic_declination(Radians::from_radians(declination)) {
                untouched(filter, "a refused declination")?;
            }
        }
    }
    check(filter, &format!("{op:?}"), project)
}

fn run(config: Config, start: &Start, ops: &[Op]) -> Result<Eskf, TestCaseError> {
    let mut filter = Eskf::new(config).map_err(|e| TestCaseError::fail(format!("{e}")))?;
    begin(&mut filter, start)?;
    for (i, op) in ops.iter().enumerate() {
        apply(&mut filter, op, i % 8 == 7 || i + 1 == ops.len())?;
    }
    Ok(filter)
}

proptest! {
    #![proptest_config(runner())]

    #[test]
    fn hostile(
        config in config(),
        start in start(true),
        ops in prop::collection::vec(op(true), 1..80),
    ) {
        run(config, &start, &ops)?;
    }

    #[test]
    fn ordinary(
        config in config(),
        start in start(false),
        ops in prop::collection::vec(op(false), 1..120),
    ) {
        let filter = run(config, &start, &ops)?;
        prop_assert_eq!(filter.diagnostics().floored, 0, "an honest run reached the floor of (42′)");
    }

    /// The constructors a caller converts through first: a place the filter is handed a
    /// geodetic fix or a frame it did not choose. Latitudes at and past the poles, longitudes
    /// either side of the antimeridian, and non-finite heights. Finite heights stop at 1000 km:
    /// at 1e30 m an ECEF `f64` resolves nothing finer than 1e14 m, and the round trip below
    /// would measure that rather than (43).
    #[test]
    fn geodetic_and_frame_conversions(
        lat in prop_oneof![-90.0f64..=90.0, select(vec![90.0, -90.0, 90.000_1, f64::NAN, f64::INFINITY])],
        lon in prop_oneof![-180.0f64..=180.0, select(vec![180.0, -180.0, 179.999_999, 540.0, f64::NAN])],
        height in prop_oneof![-500.0f64..9_000.0, select(vec![f64::NAN, f64::INFINITY, 1.0e6])],
        ned in vector(true, -10_000.0, 10_000.0),
        q in [scalar(true, -1.0, 1.0), scalar(true, -1.0, 1.0), scalar(true, -1.0, 1.0), scalar(true, -1.0, 1.0)],
    ) {
        let point = Geodetic::from_degrees(lat, lon, height);
        #[cfg(feature = "magnetic-model")]
        let _ = point.magnetic_declination();
        if let Some(origin) = LocalOrigin::new(point) {
            let there = Position::ned(ned[0], ned[1], ned[2]);
            let back = origin.to_ned(origin.to_geodetic(there));
            if there.vector().iter().all(|v| v.abs() < 1.0e4) {
                prop_assert!(
                    (back.vector() - there.vector()).norm() < 0.05,
                    "round trip through the origin moved {:?} to {:?}", there, back
                );
            }
        }
        let given = Quaternion { w: q[0], x: q[1], y: q[2], z: q[3] };
        let rotation = q.iter().any(|c| *c != 0.0) && q.iter().all(|c| c.is_finite());
        // Normalized in f64, where no f32 component overflows or underflows the norm, so the
        // reference does not share the constructors' arithmetic.
        let norm = q.iter().map(|c| f64::from(*c).powi(2)).sum::<f64>().sqrt();
        let [w, x, y, z] = q.map(|c| (f64::from(c) / norm) as f32);
        // Each constructor's getter hands back what it was given, normalized, up to the
        // quaternion's sign: `|dot| = 1` against the unit reference, which a zero or a
        // non-unit result fails. A zero or non-finite one comes out non-finite, not a panic.
        if rotation {
            let trips = [
                ("body_to_ned", Attitude::from_body_to_ned(given).body_to_ned()),
                ("ned_to_body", Attitude::from_ned_to_body(given).ned_to_body()),
                ("flu_to_enu", Attitude::from_flu_to_enu(given).flu_to_enu()),
                ("flu_to_nwu", Attitude::from_flu_to_nwu(given).flu_to_nwu()),
            ];
            for (name, back) in trips {
                let dot = back.w * w + back.x * x + back.y * y + back.z * z;
                prop_assert!((1.0 - dot.abs()).abs() < 1e-3, "{name}: {q:?} came back {back:?}");
            }
        }
        if !rotation {
            let refused = State { attitude: Attitude::from_body_to_ned(given), ..State::default() };
            prop_assert!(!refused.is_finite(), "{q:?} is no rotation and came out {refused:?}");
        }
        for attitude in [
            Attitude::from_body_to_ned(given),
            Attitude::from_ned_to_body(given),
            Attitude::from_flu_to_enu(given),
            Attitude::from_flu_to_nwu(given),
        ] {
            let _ = (attitude.ned_to_body(), attitude.flu_to_enu(), attitude.flu_to_nwu(), attitude.euler_angles());
        }
        let _ = Position::enu(ned[0], ned[1], ned[2]).to_ned().to_enu();
        let _ = AngularRate::flu(ned[0], ned[1], ned[2]).to_flu();
    }
}
