//! What happens when sources stop arriving: status transitions, per-source diagnostics,
//! and an application-driven recovery.
//!
//! `predict` propagates the nominal state and its covariance, (9)–(22), but every `fuse_*`
//! still accepts unconditionally, so no measurement is ever actually gated out and the estimate
//! here is IMU-only dead reckoning with a monotonically growing uncertainty. What is real is
//! the attitude the window yields, that dead reckoning, and the health bookkeeping: the timers,
//! the aggregate [`Status`], and the fact that recovery is the application's decision rather
//! than the filter's.
//!
//! Run with `cargo run --example degradation`. For the loop itself, see `basic.rs`.

use fusion_nav::prelude::*;

const IMU_HZ: u32 = 400;
const GNSS_HZ: u32 = 5;
const BARO_HZ: u32 = 20;
const MAG_HZ: u32 = 50;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("fusion-nav degradation example — no filtering is performed\n");

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
        filter.fuse_baro_altitude(Altitude::from_meters(0.0), AltitudeNoise::from_sigma(2.0));
    println!("before initialize: {early:?}");

    // Quasi-static initialization. A real window is captured from the IMU while the
    // vehicle sits still; the filter validates the stationarity assumption.
    let window = [stationary_sample(); (2 * IMU_HZ) as usize];
    let alignment = filter.initialize(&window, Seconds::from_secs(1.0 / IMU_HZ as f32))?;
    println!("alignment:         {alignment:?}");
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
        // A reset refuses a fix or a noise the covariance could not hold, so it answers.
        assert!(filter.reset_position_to(
            Position::ned(120.0, -43.0, -60.0),
            PositionNoise::horizontal_vertical(1.6, 1.6),
        ));
        assert!(filter.reset_velocity_to(
            Velocity::<Ned>::zero(),
            VelocityNoise::from_speed_accuracy(0.5),
        ));
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
    //         Position::enu(0.0, 0.0, 0.0),
    //         PositionNoise::horizontal_vertical(1.0, 1.0),
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
        assert!(filter.predict(imu_sample(), dt).is_propagated());

        if sources.gnss && tick % (IMU_HZ / GNSS_HZ) == 0 {
            let fix = Position::ned(120.0, -43.0, -60.0);
            check(
                "gnss position",
                filter.fuse_gnss_position(fix, PositionNoise::horizontal_vertical(1.5, 3.0)),
            );
            check(
                "gnss velocity",
                filter.fuse_gnss_velocity(
                    Velocity::ned(14.0, 0.5, -0.2),
                    VelocityNoise::from_speed_accuracy(0.3),
                ),
            );
        }

        if sources.baro && tick % (IMU_HZ / BARO_HZ) == 0 {
            check(
                "baro",
                filter.fuse_baro_altitude(
                    Altitude::from_meters(60.0),
                    AltitudeNoise::from_sigma(2.0),
                ),
            );
        }

        if sources.mag && tick % (IMU_HZ / MAG_HZ) == 0 {
            check(
                "mag",
                filter.fuse_mag_heading(
                    MagField::body(0.21, 0.03, 0.44),
                    HeadingNoise::from_sigma(0.22),
                ),
            );
        }
    }
}

/// Report anything that is not an ordinary acceptance.
///
/// A rejection is the gate doing its job and shows up in `diagnostics()` either way. The
/// rest do not: a measurement refused for a NaN or a bad variance moves no timer, and one
/// refused for a missing reference never reaches the gate at all, so a loop that ignores
/// them sees a source that looks simply absent.
fn check(source: &str, outcome: Fusion) {
    match outcome {
        Fusion::Accepted { .. } => {}
        Fusion::Rejected { test_ratio } => {
            println!("  {source} rejected, test ratio {test_ratio:.2}");
        }
        Fusion::Reset => println!("  {source} adopted outright — nothing to fuse it against"),
        other => println!("  {source} refused: {other:?}"),
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
        gyro: AngularRate::body(0.01, -0.002, 0.03),
        accel: Acceleration::body(0.2, 0.1, -GRAVITY),
    }
}

fn stationary_sample() -> StaticSample {
    StaticSample {
        imu: ImuSample {
            gyro: AngularRate::body(0.0, 0.0, 0.0),
            accel: Acceleration::body(0.0, 0.0, -GRAVITY),
        },
        mag: Some(MagField::body(0.22, 0.0, 0.44)),
        // Ground level at the launch point. This is what fixes the barometer's
        // reference, so the 60 m fused later reads as 8 m above the origin rather than
        // as an absolute altitude. Without it `fuse_baro_altitude` refuses.
        baro: Some(Altitude::from_meters(52.0)),
        // A vehicle on the ground has nothing to difference; `velocity` is what a
        // window taken in motion carries. See `StaticSample::velocity`.
        ..StaticSample::default()
    }
}
