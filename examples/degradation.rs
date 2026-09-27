//! What happens when sources stop arriving: status transitions, per-source diagnostics,
//! and an application-driven recovery.
//!
//! Every source is fused and gated, equations (23)–(41), so between fixes the estimate is dead
//! reckoning because nothing is arriving rather than because nothing corrects. What this shows
//! is the gate turning down a glitch, the health bookkeeping — the timers, the aggregate
//! [`Status`] — and an application that owns recovery: it turns the filter's own off with
//! [`Recovery::OFF`] and resets the state itself.
//!
//! Two outages, one per rule. A source that stops is timed out against its own period, so a
//! 20 Hz barometer is missed within 125 ms rather than on a 1 Hz receiver's schedule. And the
//! status reads `DeadReckoning` once GNSS has been gone for
//! [`Timeouts::dead_reckoning_after`], with the barometer and magnetometer still arriving:
//! height and heading are held, and horizontal position drifts regardless.
//!
//! Run with `cargo run --example degradation`. For the loop itself, see `basic.rs`.

use fusion_nav::prelude::*;

const IMU_HZ: u32 = 400;
const GNSS_HZ: u32 = 5;
const BARO_HZ: u32 = 20;
const MAG_HZ: u32 = 50;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("fusion-nav degradation example\n");

    let config = Config {
        // This application resets the state itself, below, so the filter's recovery is off.
        recovery: Recovery::OFF,
        ..Config::default()
    };

    let mut filter = Eskf::new(config);
    assert!(filter.set_magnetic_declination(Radians::from_radians(-0.06)));

    // A measurement before initialization is refused rather than silently dropped.
    let early = filter.fuse_baro_altitude(
        Timestamp::ZERO,
        Altitude::from_meters(0.0),
        AltitudeNoise::from_sigma(2.0),
    );
    println!("before initialize: {early:?}");

    // Quasi-static initialization. A real window is captured from the IMU while the
    // vehicle sits still; the filter validates the stationarity assumption.
    let window: [StaticSample; (2 * IMU_HZ) as usize] = core::array::from_fn(stationary_sample);
    // The IMU driver's sample count, which dates every sample after the window as it did
    // the window's own.
    let mut samples = window.len() as u64;
    let alignment = filter.initialize(&window)?;
    println!("alignment:         {alignment:?}");
    println!("initialized:       {:?}\n", filter.state().status);

    // --- steady state: every source arriving -------------------------------------------
    run(&mut filter, &mut samples, 2 * IMU_HZ, Sources::all());
    report("all sources", &filter);

    // A glitch: one fix 50 m north of where every fix before it put the vehicle. The gate
    // turns the horizontal half down with a test ratio far above 1 and leaves the estimate
    // untouched there; the height half agrees, and is fused.
    let now = filter.time().unwrap_or_default();
    check_gnss(
        "gnss position glitch",
        filter.fuse_gnss_position(
            now,
            Position::ned(50.0, 0.0, 0.0),
            PositionNoise::horizontal_vertical(1.5, 3.0),
        ),
    );
    println!();

    // --- the barometer drops out ------------------------------------------------------
    run(
        &mut filter,
        &mut samples,
        IMU_HZ / 2,
        Sources::without_baro(),
    );
    report("no barometer for 0.5 s", &filter);
    run(&mut filter, &mut samples, IMU_HZ, Sources::all());
    report("the barometer back for 1 s", &filter);

    // --- GNSS drops out ----------------------------------------------------------------
    run(
        &mut filter,
        &mut samples,
        6 * IMU_HZ,
        Sources::without_gnss(),
    );
    report("no GNSS for 6 s", &filter);

    // Recovery is this application's call, having turned the filter's off.
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
    //     filter.fuse_gnss_position(now,
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

    const fn without_baro() -> Self {
        Self {
            gnss: true,
            baro: false,
            mag: true,
        }
    }
}

/// The integration loop a flight controller would write.
fn run(filter: &mut Eskf, samples: &mut u64, ticks: u32, sources: Sources) {
    for tick in 0..ticks {
        *samples += 1;
        let time = sample_time(*samples);
        assert!(filter.predict(imu_sample(time)).is_propagated());

        if sources.gnss && tick % (IMU_HZ / GNSS_HZ) == 0 {
            // The vehicle is still near where it started, which is where the fix puts it.
            let fix = Position::ned(0.0, 0.0, 0.0);
            check_gnss(
                "gnss position",
                filter.fuse_gnss_position(time, fix, PositionNoise::horizontal_vertical(1.5, 3.0)),
            );
            check(
                "gnss velocity",
                filter.fuse_gnss_velocity(
                    time,
                    Velocity::ned(0.0, 0.0, 0.0),
                    VelocityNoise::from_speed_accuracy(0.3),
                ),
            );
        }

        if sources.baro && tick % (IMU_HZ / BARO_HZ) == 0 {
            check(
                "baro",
                filter.fuse_baro_altitude(
                    time,
                    // Still on the ground, which is where the window put the reference.
                    Altitude::from_meters(52.0),
                    AltitudeNoise::from_sigma(2.0),
                ),
            );
        }

        if sources.mag && tick % (IMU_HZ / MAG_HZ) == 0 {
            check(
                "mag",
                filter.fuse_mag_heading(
                    time,
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

/// A GNSS fix is two measurements, each with its own verdict.
fn check_gnss(source: &str, outcome: GnssFusion) {
    check(&format!("{source} (horizontal)"), outcome.horizontal);
    check(&format!("{source} (height)"), outcome.height);
}

fn report(label: &str, filter: &Eskf) {
    println!("after {label}: {:?}", filter.state().status);
    for (name, health) in filter.diagnostics().sources() {
        // A source's own timeout, off its measured period, is what `Status` reads. `main`
        // leaves `Config::timeouts` at its default, which caps it.
        let timeout = health.timeout(&Timeouts::default()).as_secs();
        match health.time_since_accepted {
            Some(elapsed) => println!(
                "  {name:<14} last accepted {:>5.2} s ago, times out after {timeout:.3} s, \
                 {} accepted, {} rejected",
                elapsed.as_secs(),
                health.accepted,
                health.rejected
            ),
            None => println!("  {name:<14} never used"),
        }
    }
    println!();
}

/// The time between IMU samples.
fn interval() -> Seconds {
    Seconds::from_secs(1.0 / IMU_HZ as f32)
}

/// A rate IMU's reading, converted to the increments the filter takes.
fn imu_sample(time: Timestamp) -> ImuSample {
    ImuSample::from_rates(
        time,
        AngularRate::body(0.01, -0.002, 0.03),
        Acceleration::body(0.2, 0.1, -GRAVITY),
        interval(),
    )
}

/// When the `n`th IMU sample since power-on ends: the driver's clock, never the filter's.
fn sample_time(n: u64) -> Timestamp {
    Timestamp::from_micros(n * 1_000_000 / u64::from(IMU_HZ))
}

/// The `i`th sample of a window on the ground.
fn stationary_sample(i: usize) -> StaticSample {
    let time = sample_time(i as u64 + 1);
    StaticSample {
        imu: ImuSample::from_rates(
            time,
            AngularRate::body(0.0, 0.0, 0.0),
            Acceleration::body(0.0, 0.0, -GRAVITY),
            interval(),
        ),
        mag: Some(MagField::body(0.22, 0.0, 0.44)),
        // Ground level at the launch point, 52 m, with the scatter a real barometer
        // has. This is what fixes the barometer's reference, so the 52 m fused later
        // reads as the origin's height rather than as an absolute altitude, and the
        // scatter is how well it is fixed: one reading held across the window has none
        // and fixes nothing. Without it the first altitude is spent reading a reference
        // from the estimate instead.
        baro: Some(Altitude::from_meters(if i.is_multiple_of(2) {
            52.25
        } else {
            51.75
        })),
        // A vehicle on the ground has nothing to difference; `velocity` is what a
        // window taken in motion carries. See `StaticSample::velocity`.
        ..StaticSample::default()
    }
}
