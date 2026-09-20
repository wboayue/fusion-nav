//! The smallest useful integration: initialize, propagate at IMU rate, fuse what arrives.
//!
//! Nothing here estimates anything — `predict` propagates nothing and every `fuse_*`
//! accepts unconditionally. What it shows is the shape of the loop a flight controller
//! would write.
//!
//! Run with `cargo run --example basic`. For dropouts, gate outcomes, and diagnostics,
//! see `degradation.rs`.

use fusion_nav::prelude::*;

const IMU_HZ: u32 = 400;
const GNSS_HZ: u32 = 5;
const BARO_HZ: u32 = 20;
const MAG_HZ: u32 = 50;

fn main() -> Result<(), InitError> {
    let mut filter = Eskf::new(Config {
        magnetic_declination: Radians::from_radians(-0.06),
        ..Config::default()
    });

    let dt = Seconds::from_secs(1.0 / IMU_HZ as f32);

    // Quasi-static initialization: the vehicle sits still, the filter validates it.
    // `Initialization::min_duration` is a span of time, so the sample count depends on the
    // IMU rate — 2 s at 400 Hz here. An array length has to be const, so this 2 tracks the
    // default by hand; raise that default and this window stops covering it.
    //
    // A window that is short or moving is not refused — it gives `Alignment::Coarse` and
    // the filter runs, reporting `Status::Aligning` until attitude converges. Checking
    // which you got is the point of the return value.
    println!("fusion-nav basic example — no filtering is performed\n");

    let window = [stationary_sample(); (2 * IMU_HZ) as usize];
    let alignment = filter.initialize(&window, dt)?;
    println!("alignment {alignment:?}\n");

    for tick in 0..(5 * IMU_HZ) {
        // Hot path: one propagation per IMU sample, `dt` supplied by the caller. The
        // outcome is `#[must_use]`: a step the filter refuses as too long leaves the
        // state stale, and a caller that ignores it would never know.
        assert!(filter.predict(imu_sample(), dt).is_propagated());

        if tick % (IMU_HZ / GNSS_HZ) == 0 {
            // Latitude and longitude straight from the receiver: the filter places its
            // origin on the first fix and converts every later one about it. Reading the
            // outcome is optional — `diagnostics()` keeps the test ratio and the counts —
            // but a refusal the health path cannot show up in is worth catching here.
            let outcome = filter.fuse_gnss_geodetic(
                Geodetic::from_degrees(47.397_742, 8.545_594, 488.0),
                PositionNoise::horizontal_vertical(1.5, 3.0),
            );
            if let Some(test_ratio) = outcome.test_ratio() {
                assert!(test_ratio <= 1.0, "gnss position rejected");
            }

            filter.fuse_gnss_velocity(
                Velocity::ned(14.0, 0.5, -0.2),
                VelocityNoise::from_speed_accuracy(0.3),
            );
        }

        if tick % (IMU_HZ / BARO_HZ) == 0 {
            filter.fuse_baro_altitude(Altitude::from_meters(60.0), AltitudeNoise::from_sigma(2.0));
        }

        if tick % (IMU_HZ / MAG_HZ) == 0 {
            filter.fuse_mag_heading(
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
        Status::Degraded => println!("\na source has timed out; the estimate is still aided"),
        Status::DeadReckoning => println!("\nnothing is aiding the filter; drift is unbounded"),
    }

    Ok(())
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
