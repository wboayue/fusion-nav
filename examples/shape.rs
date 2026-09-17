//! Walks the intended API end to end, at the call sites of README.md.
//!
//! Nothing here estimates anything: `predict` propagates nothing and every `fuse_*`
//! accepts unconditionally. What it does exercise is the shape — which types a caller has
//! to build, how verbose the units are at the boundary, and how the status behaves when a
//! source stops arriving.
//!
//! Run with `cargo run --example shape`.

use fusion_nav::{
    Acceleration, Altitude, AltitudeVariance, AngularRate, Config, ErrorState, Eskf, Fusion,
    GRAVITY, HeadingVariance, ImuSample, MagField, Ned, Position, PositionVariance, Radians,
    Seconds, StaticSample, Status, Timeouts, Velocity, VelocityVariance,
};

const IMU_HZ: u32 = 400;
const GNSS_HZ: u32 = 5;
const BARO_HZ: u32 = 20;
const MAG_HZ: u32 = 50;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("fusion-nav API sketch — no filtering is performed\n");

    let config = Config {
        magnetic_declination: Radians::from_radians(-0.06),
        timeouts: Timeouts {
            degraded_after: Seconds::from_secs(1.0),
            ..Timeouts::default()
        },
        ..Config::default()
    };

    let mut filter = Eskf::new(config);

    // A measurement before initialization is refused rather than silently dropped.
    let early =
        filter.fuse_baro_altitude(Altitude::from_meters(0.0), AltitudeVariance::from_m2(4.0));
    println!("before initialize: {early:?}");

    // Quasi-static initialization. A real window is captured from the IMU while the
    // vehicle sits still; the filter validates the stationarity assumption.
    let window = [stationary_sample(); 200];
    filter.initialize(&window)?;
    println!("initialized:       {:?}\n", filter.state().status);

    // --- steady state: every source arriving -------------------------------------------
    run(&mut filter, 2 * IMU_HZ, Sources::all());
    report("all sources", &filter);

    // --- GNSS drops out ----------------------------------------------------------------
    run(&mut filter, 3 * IMU_HZ, Sources::without_gnss());
    report("no GNSS for 3 s", &filter);

    // --- everything drops out ----------------------------------------------------------
    run(&mut filter, 6 * IMU_HZ, Sources::none());
    report("nothing for 6 s", &filter);

    // Recovery is the application's call, not the filter's.
    if filter.state().status == Status::DeadReckoning {
        filter.reset_position_to(
            Position::<Ned>::from_meters(120.0, -43.0, -60.0),
            PositionVariance::isotropic(2.5),
        );
        filter.reset_velocity_to(Velocity::<Ned>::zero(), VelocityVariance::isotropic(0.25));
        println!("applied an external position reset\n");
    }

    let state = filter.state();
    let (roll, pitch, yaw) = state.attitude.euler_angles();
    println!("state");
    println!("  status    {:?}", state.status);
    println!("  position  {:?}", state.position);
    println!("  velocity  {:?}", state.velocity);
    println!("  attitude  roll {roll:.3} pitch {pitch:.3} yaw {yaw:.3} rad");
    println!("  gyro bias {:?}", state.gyro_bias);

    let p = filter.covariance();
    println!(
        "\ncovariance: north position variance {:.3} m^2, yaw variance {:.4} rad^2",
        p.variance(ErrorState::PositionNorth),
        p.variance(ErrorState::AttitudeZ),
    );

    // Frames are checked at compile time. Uncommenting this fails to build:
    //
    //     filter.fuse_gnss_position(
    //         Position::<fusion_nav::Enu>::from_meters(0.0, 0.0, 0.0),
    //         PositionVariance::isotropic(1.0),
    //     );

    Ok(())
}

/// Which sources are still delivering measurements.
#[derive(Clone, Copy)]
struct Sources {
    gnss: bool,
    baro: bool,
    mag: bool,
}

impl Sources {
    const fn all() -> Self {
        Self {
            gnss: true,
            baro: true,
            mag: true,
        }
    }

    const fn without_gnss() -> Self {
        Self {
            gnss: false,
            baro: true,
            mag: true,
        }
    }

    const fn none() -> Self {
        Self {
            gnss: false,
            baro: false,
            mag: false,
        }
    }
}

/// The integration loop a flight controller would write.
fn run(filter: &mut Eskf, ticks: u32, sources: Sources) {
    let dt = Seconds::from_secs(1.0 / IMU_HZ as f32);

    for tick in 0..ticks {
        filter.predict(imu_sample(), dt);

        if sources.gnss && tick % (IMU_HZ / GNSS_HZ) == 0 {
            let fix = Position::<Ned>::from_meters(120.0, -43.0, -60.0);
            check(
                "gnss position",
                filter.fuse_gnss_position(fix, PositionVariance::isotropic(1.5)),
            );
            check(
                "gnss velocity",
                filter.fuse_gnss_velocity(
                    Velocity::<Ned>::from_m_per_s(14.0, 0.5, -0.2),
                    VelocityVariance::isotropic(0.09),
                ),
            );
        }

        if sources.baro && tick % (IMU_HZ / BARO_HZ) == 0 {
            check(
                "baro",
                filter.fuse_baro_altitude(
                    Altitude::from_meters(60.0),
                    AltitudeVariance::from_m2(4.0),
                ),
            );
        }

        if sources.mag && tick % (IMU_HZ / MAG_HZ) == 0 {
            check(
                "mag",
                filter.fuse_mag_heading(
                    MagField::from_components(0.21, 0.03, 0.44),
                    HeadingVariance::from_rad2(0.05),
                ),
            );
        }
    }
}

/// The outcome is `#[must_use]`: a rejection cannot be dropped without a warning.
fn check(source: &str, outcome: Fusion) {
    if let Fusion::Rejected { test_ratio } = outcome {
        println!("  {source} rejected, test ratio {test_ratio:.2}");
    }
}

fn report(label: &str, filter: &Eskf) {
    println!("after {label}: {:?}", filter.state().status);
    for (name, health) in filter.diagnostics().sources() {
        match health.time_since_accepted {
            Some(elapsed) => println!(
                "  {name:<14} last accepted {:>5.1} s ago, {} accepted, {} rejected",
                elapsed.as_secs(),
                health.accepted,
                health.rejected
            ),
            None => println!("  {name:<14} never used"),
        }
    }
    println!();
}

fn imu_sample() -> ImuSample {
    ImuSample {
        gyro: AngularRate::from_rad_per_s(0.01, -0.002, 0.03),
        accel: Acceleration::from_m_per_s2(0.2, 0.1, -GRAVITY),
    }
}

fn stationary_sample() -> StaticSample {
    StaticSample {
        imu: ImuSample {
            gyro: AngularRate::from_rad_per_s(0.0, 0.0, 0.0),
            accel: Acceleration::from_m_per_s2(0.0, 0.0, -GRAVITY),
        },
        mag: Some(MagField::from_components(0.22, 0.0, 0.44)),
    }
}
