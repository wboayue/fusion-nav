//! A trace built to reach the paths the corpus does not, or not at their worst: #41's
//! worst cases, timed on purpose rather than hoped for.
//!
//! ```text
//! cargo run --release -p onboard --example paths -- target/onboard/paths.trace
//! ```
//!
//! The corpus never fuses a geodetic fix, a course or a stationary claim, never seeds or starts
//! coarse from one sample, and coasts at most 3.1 s (`4b473e91`). Each section here starts a
//! fresh filter, labels the calls it reaches a path with, and asserts on the host that they
//! reach it, so a change that moves a path out of reach fails here rather than timing
//! something else. `DESIGN.md`, "Execution time bounded by constants", lists the bounds these
//! are the worst cases of.

use std::path::PathBuf;

use bench::{
    IMU_HZ, SITE, altitude, imu_sample, magnetic_field, position_noise, sample_time,
    stationary_sample, velocity, velocity_noise,
};
use fusion_nav::prelude::*;
use onboard::{Recorder, TraceFile};

type Filter = Recorder<TraceFile>;

/// A long gap and a long horizon. (22′) takes one step at any, which the 10 s gap beside it shows.
const LONGEST: f32 = 6.4;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let path = std::env::args().nth(1).map_or_else(
        || PathBuf::from("target/onboard/paths.trace"),
        PathBuf::from,
    );
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent)?;
    }
    let config = Config {
        accuracy: Accuracy {
            horizon: Seconds::from_secs(LONGEST),
            ..Config::default().accuracy
        },
        ..Config::default()
    };
    let mut filter = Recorder::new(config, Some(TraceFile::create(&path)?))?;

    starts(&mut filter, config)?;
    let n = aided(&mut filter, config)?;
    recoveries(&mut filter, n);
    unaided(&mut filter, config)?;
    stationary(&mut filter, config)?;

    let calls = filter.take_sink().ok_or("the sink")?.finish()?;
    println!("{}: {calls} calls", path.display());
    Ok(())
}

/// A fresh filter under `config`: `Eskf::new`, recorded.
fn renew(filter: &mut Filter, config: Config) -> Result<(), Box<dyn std::error::Error>> {
    let mut fresh = Recorder::new(config, filter.take_sink())?;
    std::mem::swap(filter, &mut fresh);
    Ok(())
}

/// The three starts, and a window long enough that `StaticWindow::halve` merges its blocks.
fn starts(filter: &mut Filter, config: Config) -> Result<(), Box<dyn std::error::Error>> {
    filter.label("start: static, 10 s window");
    let window = (0..10 * IMU_HZ).map(stationary_sample);
    assert_eq!(filter.initialize_on(window), Ok(Alignment::Static));

    renew(filter, config)?;
    filter.label("start: coarse, one sample");
    let coarse = filter.initialize_coarse(imu_sample(sample_time(1)));
    assert!(matches!(coarse, Ok(Alignment::Coarse(_))), "{coarse:?}");
    Ok(())
}

/// The fix the vehicle reports at `t` seconds: north at the bench's speed from the site.
fn fix_at(origin: &LocalOrigin, t: f32) -> Geodetic {
    origin.to_geodetic(bench::position(t))
}

/// Seeded and flown straight north with every source fused, then each source once at the
/// oldest age the history holds, the arming query over a long horizon, and a heading near
/// `f32::MAX`. The IMU sample index it ends on.
fn aided(filter: &mut Filter, config: Config) -> Result<u32, Box<dyn std::error::Error>> {
    renew(filter, config)?;
    assert!(filter.set_magnetic_declination(Radians::from_radians(0.0)));
    let state = State {
        velocity: velocity(),
        ..State::default()
    };
    let sigmas = [
        1.0, 1.0, 1.0, 0.2, 0.2, 0.2, 0.02, 0.02, 0.05, 0.1, 0.1, 0.1, 0.002, 0.002, 0.002,
    ];
    filter.label("start: seeded");
    let seeded = filter.initialize_from(state, Covariance::from_sigmas(sigmas), sample_time(0));
    assert_eq!(seeded, Ok(Alignment::Seeded));

    let (lat, lon, height) = SITE;
    let site = Geodetic::from_degrees(lat, lon, height);
    filter.label("fuse_gnss_geodetic: the first fix places the origin");
    let first = filter.fuse_gnss_geodetic(sample_time(0), site, position_noise(), Position::zero());
    assert!(first.is_accepted(), "{first:?}");
    let origin = LocalOrigin::new(site).ok_or("the site")?;

    filter.label("aided");
    let heading = HeadingNoise::from_sigma(0.05);
    let end = 4 * IMU_HZ;
    for n in 1..=end {
        let time = sample_time(n);
        assert!(filter.predict(imu_sample(time)).is_propagated());
        // Nothing in the last 0.4 s, so the fixes below arrive after their sources' last.
        if n > end - 2 * IMU_HZ / 5 {
            continue;
        }
        let t = n as f32 / IMU_HZ as f32;
        if n % (IMU_HZ / 5) == 0 {
            let fix = filter.fuse_gnss_geodetic(
                time,
                fix_at(&origin, t),
                position_noise(),
                Position::zero(),
            );
            assert!(fix.is_accepted(), "{fix:?} at {t} s");
            let v = filter.fuse_gnss_velocity(time, velocity(), velocity_noise(), Position::zero());
            assert!(v.is_accepted(), "{v:?} at {t} s");
            let h = filter.fuse_gnss_heading(time, Radians::from_radians(0.0), heading);
            assert!(h.is_accepted(), "{h:?} at {t} s");
            let c = filter.fuse_course(time, heading);
            assert!(c.is_accepted(), "{c:?} at {t} s");
        }
        if n % (IMU_HZ / 20) == 0 {
            let b = filter.fuse_baro_altitude(time, altitude(), AltitudeNoise::from_sigma(0.5));
            assert!(b.is_accepted(), "{b:?} at {t} s");
            let m = filter.fuse_mag_heading(time, magnetic_field(), heading);
            assert!(m.is_accepted(), "{m:?} at {t} s");
        }
    }

    // The history reaches back `LATENCY_HORIZON`; 10 ms inside it, every source reads the
    // oldest entries and carries `H` through the longest `A`.
    filter.label("oldest age the history holds");
    let age = LATENCY_HORIZON.as_secs() - 0.01;
    let old = sample_time(end).before(Seconds::from_secs(age));
    let t = end as f32 / IMU_HZ as f32 - age;
    let fix =
        filter.fuse_gnss_geodetic(old, fix_at(&origin, t), position_noise(), Position::zero());
    assert!(fix.is_accepted(), "{fix:?}");
    let checks = [
        filter.fuse_gnss_velocity(old, velocity(), velocity_noise(), Position::zero()),
        filter.fuse_gnss_heading(old, Radians::from_radians(0.0), heading),
        filter.fuse_course(old, heading),
        filter.fuse_baro_altitude(old, altitude(), AltitudeNoise::from_sigma(0.5)),
        filter.fuse_mag_heading(old, magnetic_field(), heading),
    ];
    assert!(checks.iter().all(|f| f.is_accepted()), "{checks:?}");

    filter.label("predicted_validity: a 6.4 s horizon");
    filter.predicted_validity();

    // `wrap_pi`'s `fmodf` loops on the exponent gap between the angle and 2π.
    filter.label("fuse_gnss_heading: 3e38 rad, fmodf's longest reduction");
    let far = filter.fuse_gnss_heading(sample_time(end), Radians::from_radians(3.0e38), heading);
    assert!(
        matches!(far, Fusion::Accepted { .. } | Fusion::Rejected { .. }),
        "{far:?}"
    );
    Ok(end)
}

/// Each source rejected past its `Config::recovery` timeout, so its next measurement is
/// adopted: a recovery is the update that lands on an adoption.
fn recoveries(filter: &mut Filter, from: u32) {
    let (lat, lon, height) = SITE;
    let Some(origin) = LocalOrigin::new(Geodetic::from_degrees(lat, lon, height)) else {
        return;
    };
    let heading = HeadingNoise::from_sigma(0.05);
    let mut n = from;
    let mut fly =
        |filter: &mut Filter, seconds: u32, each: &mut dyn FnMut(&mut Filter, u32, Timestamp)| {
            for _ in 0..seconds * IMU_HZ {
                n += 1;
                let time = sample_time(n);
                assert!(filter.predict(imu_sample(time)).is_propagated());
                each(filter, n, time);
            }
        };

    filter.label("recovery: GNSS position 100 m out");
    let before = filter.diagnostics().gnss_position.recovered;
    fly(filter, 8, &mut |filter, n, time| {
        if n % (IMU_HZ / 5) == 0 {
            let t = n as f32 / IMU_HZ as f32;
            let out = Position::ned(100.0, 0.0, 0.0);
            let fix = origin.to_geodetic(Position::from_array(core::array::from_fn(|i| {
                bench::position(t).to_array()[i] + out.to_array()[i]
            })));
            filter.fuse_gnss_geodetic(time, fix, position_noise(), Position::zero());
            filter.fuse_gnss_velocity(time, velocity(), velocity_noise(), Position::zero());
        }
    });
    assert!(filter.diagnostics().gnss_position.recovered > before);

    // Positions at the estimate hold the velocity's covariance tight (the recovery above moved
    // the estimate 100 m, so not the flight's), and a velocity 30 m/s east of it is rejected
    // until its source's recovery adopts it: at 3 m/s the gate reopens as the covariance grows.
    filter.label("recovery: GNSS velocity 30 m/s out");
    let (velocity_before, yaw_before) = (
        filter.diagnostics().gnss_velocity.recovered,
        filter.diagnostics().yaw_estimator.recovered,
    );
    fly(filter, 8, &mut |filter, n, time| {
        if n % (IMU_HZ / 5) == 0 {
            let fix = origin.to_geodetic(filter.state().position);
            filter.fuse_gnss_geodetic(time, fix, position_noise(), Position::zero());
            let out = Velocity::ned(5.0, 30.0, 0.0);
            filter.fuse_gnss_velocity(time, out, velocity_noise(), Position::zero());
        }
    });
    assert!(filter.diagnostics().gnss_velocity.recovered > velocity_before);
    // Not the yaw estimator's replacement, which needs a bank settled by acceleration this
    // straight flight does not have; the corpus reaches it (`yaw_recovered=` on `7ce66f0d`).
    assert_eq!(filter.diagnostics().yaw_estimator.recovered, yaw_before);

    // Heading recovers only while no horizontal GNSS arrives (PX4's guard), so none does.
    filter.label("recovery: magnetic heading half a circle out");
    let before = filter.diagnostics().mag_heading.recovered;
    let south = MagField::body(-0.22, 0.0, 0.44);
    fly(filter, 9, &mut |filter, n, time| {
        if n % (IMU_HZ / 20) == 0 {
            filter.fuse_mag_heading(time, south, heading);
        }
    });
    assert!(filter.diagnostics().mag_heading.recovered > before);
}

/// Seeded with tilt σ past the hold's 3° and never aided: every 0.2 s a step fuses the hold,
/// then two long coasts, each with a hold on the same step.
fn unaided(filter: &mut Filter, config: Config) -> Result<(), Box<dyn std::error::Error>> {
    renew(filter, config)?;
    let sigmas = [
        1.0, 1.0, 1.0, 0.2, 0.2, 0.2, 0.1, 0.1, 0.05, 0.1, 0.1, 0.1, 0.002, 0.002, 0.002,
    ];
    filter.label("start: seeded, tilt σ 5.7°");
    let seeded = filter.initialize_from(
        State::default(),
        Covariance::from_sigmas(sigmas),
        sample_time(0),
    );
    assert_eq!(seeded, Ok(Alignment::Seeded));

    // Which steps hold, found on a copy that records nothing, so each is labeled before it is
    // made: a step and the hold's update beside it are timed as one call.
    let mut dry = filter.clone();
    let holding: Vec<bool> = (1..=2 * IMU_HZ)
        .map(|n| {
            let before = dry.diagnostics().position_hold.accepted;
            assert!(dry.predict(imu_sample(sample_time(n))).is_propagated());
            dry.diagnostics().position_hold.accepted > before
        })
        .collect();
    for (n, holds) in (1..).zip(&holding) {
        filter.label(if *holds {
            "predict: unaided, a step and a hold"
        } else {
            "predict: unaided, a step"
        });
        assert!(filter.predict(imu_sample(sample_time(n))).is_propagated());
    }
    let holds = filter.diagnostics().position_hold.accepted;
    assert_eq!(holds as usize, holding.iter().filter(|h| **h).count());
    assert!(holds >= 9, "{holds} holds in 2 s");

    // Both coasts land on a hold, the last having been 0.2 s or more before.
    let mut time = sample_time(2 * IMU_HZ);
    for (label, gap) in [
        ("predict: coast of 6.4 s, and a hold", LONGEST),
        ("predict: coast of 10 s, and a hold", 10.0),
    ] {
        filter.label(label);
        time = time.after(Seconds::from_secs(gap));
        let before = filter.diagnostics().position_hold.accepted;
        let step = filter.predict(imu_sample(time));
        assert!(matches!(step, Propagation::Coasted { .. }), "{step:?}");
        assert_eq!(filter.diagnostics().position_hold.accepted, before + 1);
    }
    Ok(())
}

/// A still start, and the caller's claim that the vehicle is not moving.
fn stationary(filter: &mut Filter, config: Config) -> Result<(), Box<dyn std::error::Error>> {
    renew(filter, config)?;
    filter.label("start: static");
    let window = (0..2 * IMU_HZ).map(stationary_sample);
    assert_eq!(filter.initialize_on(window), Ok(Alignment::Static));
    filter.label("fuse_stationary");
    for n in 2 * IMU_HZ + 1..=4 * IMU_HZ {
        let time = sample_time(n);
        assert!(filter.predict(imu_sample(time)).is_propagated());
        if n % (IMU_HZ / 5) == 0 {
            let claim = filter.fuse_stationary(time, VelocityNoise::from_sigma(0.05, 0.05, 0.05));
            assert!(claim.is_accepted(), "{claim:?}");
        }
    }
    Ok(())
}
