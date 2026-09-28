//! The smallest useful integration: initialize, propagate at IMU rate, fuse what arrives.
//!
//! The filter levels and takes a heading from the window, then `predict` propagates the
//! nominal state and its covariance, (9)–(22), and every source it is offered corrects both
//! through the gate of (37)–(38) — GNSS position and velocity, barometric altitude and magnetic
//! heading, (23)–(41). What it shows is the shape of the loop a flight controller would write.
//!
//! Run with `cargo run --example basic`. For dropouts, gate outcomes, and diagnostics,
//! see `degradation.rs`.

use fusion_nav::prelude::*;

const IMU_HZ: u32 = 400;
const GNSS_HZ: u32 = 5;
const BARO_HZ: u32 = 20;
const MAG_HZ: u32 = 50;

fn main() -> Result<(), InitError> {
    let mut filter = Eskf::new(Config::default());
    // The site's declination, before initializing: the window's heading reads it.
    assert!(filter.set_magnetic_declination(Radians::from_radians(-0.06)));

    // Quasi-static initialization: the vehicle sits still, the filter validates it. The
    // window folds each sample in as it arrives rather than buffering it, and
    // `Initialization::min_duration` is a span of time, so it is collected until it spans
    // that: 800 samples at 400 Hz here.
    //
    // A window that is short or moving is not refused — it gives `Alignment::Coarse` and
    // the filter runs, reporting `Status::Aligning` until attitude converges. Checking
    // which you got is the point of the return value.
    println!("fusion-nav basic example\n");

    let mut window = StaticWindow::new();
    let mut samples = 0;
    while !window.is_long_enough(&filter.config().init) {
        window.push(stationary_sample(samples))?;
        samples += 1;
    }
    let alignment = filter.initialize(&window)?;
    println!("alignment {alignment:?}\n");

    for tick in 0..(5 * IMU_HZ) {
        // Hot path: one propagation per IMU sample, timed by the caller's clock. The
        // outcome is `#[must_use]`: a step the filter cannot integrate is coasted or
        // refused, and a caller that ignores it would never know there was a gap.
        let time = sample_time(samples as u32 + tick + 1);
        assert!(filter.predict(imu_sample(time)).is_propagated());

        if tick % (IMU_HZ / GNSS_HZ) == 0 {
            // Latitude and longitude straight from the receiver: the filter places its
            // origin on the first fix and converts every later one about it. Reading the
            // outcome is optional — `diagnostics()` keeps the test ratio and the counts —
            // but a refusal the health path cannot show up in is worth catching here.
            let outcome = filter.fuse_gnss_geodetic(
                time,
                Geodetic::from_degrees(47.397_742, 8.545_594, 488.0),
                PositionNoise::horizontal_vertical(1.5, 3.0),
            );
            // Two verdicts: the horizontal half and the height are gated apart.
            for half in [outcome.horizontal, outcome.height] {
                if let Some(test_ratio) = half.test_ratio() {
                    assert!(test_ratio <= 1.0, "gnss position rejected");
                }
            }

            filter.fuse_gnss_velocity(
                time,
                Velocity::ned(14.0, 0.5, -0.2),
                VelocityNoise::from_speed_accuracy(0.3),
            );
        }

        if tick % (IMU_HZ / BARO_HZ) == 0 {
            filter.fuse_baro_altitude(
                time,
                Altitude::from_meters(60.0),
                AltitudeNoise::from_sigma(2.0),
            );
        }

        if tick % (IMU_HZ / MAG_HZ) == 0 {
            filter.fuse_mag_heading(
                time,
                MagField::body(0.21, 0.03, 0.44),
                HeadingNoise::from_sigma(0.3),
            );
        }
    }

    // The estimate carries its own status, so the trust level is in hand alongside the
    // numbers it qualifies.
    let state = filter.state();
    let (roll, pitch, yaw) = state.attitude.euler_angles();

    println!("status    {:?}", state.status);
    println!("position  {:?}", state.position);
    if let (Some(origin), Some(here)) = (filter.origin(), filter.geodetic_position()) {
        let origin = origin.geodetic();
        println!(
            "origin    {:.6}, {:.6}; here {:.6}, {:.6}",
            origin.latitude_deg(),
            origin.longitude_deg(),
            here.latitude_deg(),
            here.longitude_deg()
        );
    }
    println!("velocity  {:?}", state.velocity);
    println!("attitude  roll {roll:.3} pitch {pitch:.3} yaw {yaw:.3} rad");
    println!("biases    {:?}", state.gyro_bias);

    match state.status {
        Status::Healthy => println!("\nevery source that has been fused is still accepted"),
        Status::Aligning => println!("\nattitude has not converged yet; do not fly on it"),
        Status::Degraded => {
            println!("\na source has timed out; horizontal position is still aided")
        }
        Status::DeadReckoning => {
            println!("\nno GNSS is holding horizontal position; drift is unbounded")
        }
    }

    Ok(())
}

/// When the `n`th IMU sample ends, on a clock counting microseconds from power-on.
fn sample_time(n: u32) -> Timestamp {
    Timestamp::from_micros(u64::from(n) * 1_000_000 / u64::from(IMU_HZ))
}

/// A rate IMU's reading, converted to the increments the filter takes.
fn imu_sample(time: Timestamp) -> ImuSample {
    ImuSample::from_rates(
        time,
        AngularRate::body(0.01, -0.002, 0.03),
        Acceleration::body(0.2, 0.1, -GRAVITY),
        Seconds::from_secs(1.0 / IMU_HZ as f32),
    )
}

/// The `i`th sample of a window on the ground.
fn stationary_sample(i: usize) -> StaticSample {
    StaticSample {
        imu: ImuSample::from_rates(
            sample_time(i as u32 + 1),
            AngularRate::body(0.0, 0.0, 0.0),
            Acceleration::body(0.0, 0.0, -GRAVITY),
            Seconds::from_secs(1.0 / IMU_HZ as f32),
        ),
        mag: Some(MagField::body(0.22, 0.0, 0.44)),
        // Ground level at the launch point, 52 m, with the scatter a real barometer
        // has. This is what fixes the barometer's reference, so the 60 m fused later
        // reads as 8 m above the origin rather than as an absolute altitude, and the
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
