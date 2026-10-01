//! The flight every benchmark in `benches/filter.rs` starts from (#42).
//!
//! A benchmark that times a refusal reads as a fast update, so each starts from a filter in
//! which every source is fused rather than refused, adopted or rejected: seeded level and
//! heading north at 5 m/s, then flown two seconds at 400 Hz with every source agreeing with
//! the seed. `benches/filter.rs` asserts each call it times is fused, and moves a variance.

use fusion_nav::prelude::*;

/// IMU rate, and so the step `predict` integrates.
pub const IMU_HZ: u32 = 400;
/// Speed north, constant.
pub const SPEED: f32 = 5.0;
/// Where the seed is placed, and the origin the first fix puts under it.
pub const SITE: (f64, f64, f64) = (47.397_742, 8.545_594, 488.0);

/// The `n`th IMU sample's time.
pub fn sample_time(n: u32) -> Timestamp {
    Timestamp::from_micros(u64::from(n) * 1_000_000 / u64::from(IMU_HZ))
}

/// The fixture's last sample, where every timed measurement is taken.
pub fn now() -> Timestamp {
    sample_time(2 * IMU_HZ)
}

/// Level, unrotating, unaccelerated: the reading of a vehicle flying straight.
pub fn imu_sample(time: Timestamp) -> ImuSample {
    ImuSample::from_rates(
        time,
        AngularRate::body(0.0, 0.0, 0.0),
        Acceleration::body(0.0, 0.0, -GRAVITY),
        Seconds::from_secs(1.0 / IMU_HZ as f32),
    )
}

/// The `i`th sample of a still window on the ground, with a barometer that scatters.
pub fn stationary_sample(i: u32) -> StaticSample {
    StaticSample {
        imu: imu_sample(sample_time(i + 1)),
        mag: Some(magnetic_field()),
        baro: Some(Altitude::from_meters(if i.is_multiple_of(2) {
            52.25
        } else {
            51.75
        })),
        ..StaticSample::default()
    }
}

/// A still window just long enough for the default `Config`.
pub fn window() -> StaticWindow {
    let config = Config::default();
    let mut window = StaticWindow::new();
    let mut i = 0;
    while !window.is_long_enough(&config) {
        assert!(window.push(stationary_sample(i)).is_ok());
        i += 1;
    }
    window
}

/// North and down, so a level vehicle heading north reads a heading of zero.
pub fn magnetic_field() -> MagField<Body> {
    MagField::body(0.22, 0.0, 0.44)
}

/// Where the vehicle is at `t` seconds, by construction.
pub fn position(t: f32) -> Position<Ned> {
    Position::ned(SPEED * t, 0.0, 0.0)
}

/// The velocity it holds throughout.
pub fn velocity() -> Velocity<Ned> {
    Velocity::ned(SPEED, 0.0, 0.0)
}

/// The barometer's reading: the site's height, which the flight never leaves.
pub fn altitude() -> Altitude {
    Altitude::from_meters(SITE.2 as f32)
}

/// The noise every GNSS fix is fused with.
pub fn position_noise() -> PositionNoise<Ned> {
    PositionNoise::horizontal_vertical(0.5, 1.0)
}

/// The noise every GNSS velocity is fused with.
pub fn velocity_noise() -> VelocityNoise<Ned> {
    VelocityNoise::from_speed_accuracy(0.2)
}

/// The filter two seconds into the flight, at [`now`]. Every source has been fused up to its
/// last period before it, so a GNSS velocity is fresh for `fuse_course`.
pub fn aided() -> Eskf {
    let mut filter = Eskf::default();
    // Declination fixed, so the magnetic model turns nothing at the origin and the seed's
    // heading stays the one every heading source agrees with.
    assert!(filter.set_magnetic_declination(Radians::from_radians(0.0)));
    let state = State {
        velocity: velocity(),
        ..State::default()
    };
    let sigmas = [
        1.0, 1.0, 1.0, 0.2, 0.2, 0.2, 0.02, 0.02, 0.05, 0.1, 0.1, 0.1, 0.002, 0.002, 0.002,
    ];
    assert!(
        filter
            .initialize_from(state, Covariance::from_sigmas(sigmas), sample_time(0))
            .is_ok()
    );
    let (lat, lon, height) = SITE;
    let fix = filter.fuse_gnss_geodetic(
        sample_time(0),
        Geodetic::from_degrees(lat, lon, height),
        position_noise(),
        Position::zero(),
    );
    assert!(
        fix.is_accepted(),
        "the first fix places the origin: {fix:?}"
    );

    for n in 1..=2 * IMU_HZ {
        let time = sample_time(n);
        assert!(filter.predict(imu_sample(time)).is_propagated());
        // Nothing at the last sample: a timed measurement there then arrives a period after
        // its source's last, as one does in flight, rather than at the same instant.
        if n == 2 * IMU_HZ {
            continue;
        }
        if n % (IMU_HZ / 5) == 0 {
            let t = n as f32 / IMU_HZ as f32;
            filter.fuse_gnss_position(time, position(t), position_noise(), Position::zero());
            filter.fuse_gnss_velocity(time, velocity(), velocity_noise(), Position::zero());
            let heading = HeadingNoise::from_sigma(0.05);
            filter.fuse_gnss_heading(time, Radians::from_radians(0.0), heading);
        }
        if n % (IMU_HZ / 20) == 0 {
            filter.fuse_baro_altitude(time, altitude(), AltitudeNoise::from_sigma(0.5));
        }
        if n % (IMU_HZ / 50) == 0 {
            filter.fuse_mag_heading(time, magnetic_field(), HeadingNoise::from_sigma(0.1));
        }
    }
    filter
}
