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

    // Quasi-static initialization: the vehicle sits still, the filter validates it.
    let window = [stationary_sample(); 200];
    filter.initialize(&window)?;

    let dt = Seconds::from_secs(1.0 / IMU_HZ as f32);

    for tick in 0..(5 * IMU_HZ) {
        // Hot path: one propagation per IMU sample, `dt` supplied by the caller.
        filter.predict(imu_sample(), dt);

        if tick % (IMU_HZ / GNSS_HZ) == 0 {
            // The outcome is `#[must_use]`, so a rejection cannot be dropped silently.
            let outcome = filter.fuse_gnss_position(
                Position::<Ned>::from_meters(120.0, -43.0, -60.0),
                PositionVariance::isotropic(1.5),
            );
            if let Some(test_ratio) = outcome.test_ratio() {
                assert!(test_ratio <= 1.0, "gnss position rejected");
            }

            let _ = filter.fuse_gnss_velocity(
                Velocity::<Ned>::from_m_per_s(14.0, 0.5, -0.2),
                VelocityVariance::isotropic(0.09),
            );
        }

        if tick % (IMU_HZ / BARO_HZ) == 0 {
            let _ = filter
                .fuse_baro_altitude(Altitude::from_meters(60.0), AltitudeVariance::from_m2(4.0));
        }

        if tick % (IMU_HZ / MAG_HZ) == 0 {
            let _ = filter.fuse_mag_heading(
                MagField::from_components(0.21, 0.03, 0.44),
                HeadingVariance::from_rad2(0.05),
            );
        }
    }

    // The estimate carries its own status, so the trust level is in hand alongside the
    // numbers it qualifies.
    let state = filter.state();
    let (roll, pitch, yaw) = state.attitude.euler_angles();

    println!("fusion-nav basic example — no filtering is performed\n");
    println!("status    {:?}", state.status);
    println!("position  {:?}", state.position);
    println!("velocity  {:?}", state.velocity);
    println!("attitude  roll {roll:.3} pitch {pitch:.3} yaw {yaw:.3} rad");
    println!("biases    {:?}", state.gyro_bias);

    match state.status {
        Status::Healthy => println!("\nevery source that has been fused is still accepted"),
        Status::Degraded => println!("\na source has timed out; the estimate is still aided"),
        Status::DeadReckoning => println!("\nnothing is aiding the filter; drift is unbounded"),
    }

    Ok(())
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
