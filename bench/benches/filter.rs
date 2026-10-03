//! Host timings of the filter's hot path: `predict`, every `fuse_*`, and a start.
//!
//! A host figure is not a target figure, and nothing gates on one: CI runs each benchmark
//! once (`cargo test -p bench --benches`) so that they keep building and every outcome
//! asserted here keeps holding. What they are for is a before-and-after on one machine,
//! which catches an update that turned cubic before #41's hardware does.
//!
//! Each call is timed on a clone of one aided filter, so every iteration does the same work;
//! the clone is setup, untimed.

use bench::{
    IMU_HZ, aided, altitude, imu_sample, magnetic_field, now, position, position_noise,
    sample_time, stationary_sample, velocity, velocity_noise, window,
};
use criterion::{BatchSize, Criterion, criterion_group, criterion_main};
use fusion_nav::prelude::*;

/// Time `call` on a fresh clone of `filter`, having checked once that it is fused there and
/// carries information. A refusal or a rejection is cheap and would time as a fast update; so,
/// in a sense, would a measurement taken when its source's last one was, which (24′) prices at
/// its ceiling, `R` a million times over, and fuses as nothing.
fn bench_fusion<T>(
    c: &mut Criterion,
    name: &str,
    filter: &Eskf,
    call: fn(&mut Eskf) -> T,
    fused: fn(T) -> bool,
) {
    let mut probe = filter.clone();
    assert!(
        fused(call(&mut probe)),
        "{name} is not fused from the fixture"
    );
    let (before, after) = (filter.covariance().to_rows(), probe.covariance().to_rows());
    let shrink = (0..before.len())
        .map(|i| 1.0 - after[i][i] / before[i][i])
        .fold(0.0, f32::max);
    // The fixture's weakest real update, the magnetometer's, moves yaw by 4.4e-4; the same
    // measurements at their sources' last instants moved none by more than 1.7e-6.
    assert!(
        shrink > 1e-4,
        "{name} moved no variance by a ten-thousandth: {shrink}"
    );
    c.bench_function(name, |b| {
        b.iter_batched_ref(|| filter.clone(), call, BatchSize::SmallInput)
    });
}

fn accepted(outcome: Fusion) -> bool {
    matches!(outcome, Fusion::Accepted { .. })
}

fn both_accepted(outcome: GnssFusion) -> bool {
    accepted(outcome.horizontal) && accepted(outcome.height)
}

fn hot_path(c: &mut Criterion) {
    let filter = aided();

    let next = imu_sample(sample_time(2 * IMU_HZ + 1));
    assert!(matches!(
        filter.clone().predict(next),
        Propagation::Propagated
    ));
    c.bench_function("predict", |b| {
        b.iter_batched_ref(
            || filter.clone(),
            |f| f.predict(next),
            BatchSize::SmallInput,
        )
    });

    bench_fusion(
        c,
        "fuse_gnss_position",
        &filter,
        |f| f.fuse_gnss_position(now(), position(2.0), position_noise(), Position::zero()),
        both_accepted,
    );
    bench_fusion(
        c,
        "fuse_gnss_geodetic",
        &filter,
        |f| {
            let fix = f
                .geodetic_position()
                .unwrap_or(Geodetic::from_degrees(0.0, 0.0, 0.0));
            f.fuse_gnss_geodetic(now(), fix, position_noise(), Position::zero())
        },
        both_accepted,
    );
    bench_fusion(
        c,
        "fuse_gnss_velocity",
        &filter,
        |f| f.fuse_gnss_velocity(now(), velocity(), velocity_noise(), Position::zero()),
        accepted,
    );
    bench_fusion(
        c,
        "fuse_baro_altitude",
        &filter,
        |f| f.fuse_baro_altitude(now(), altitude(), AltitudeNoise::from_sigma(0.5)),
        accepted,
    );
    bench_fusion(
        c,
        "fuse_mag_heading",
        &filter,
        |f| f.fuse_mag_heading(now(), magnetic_field(), HeadingNoise::from_sigma(0.1)),
        accepted,
    );
    bench_fusion(
        c,
        "fuse_gnss_heading",
        &filter,
        |f| {
            let noise = HeadingNoise::from_sigma(0.05);
            f.fuse_gnss_heading(now(), Radians::from_radians(0.0), noise)
        },
        accepted,
    );
    bench_fusion(
        c,
        "fuse_course",
        &filter,
        |f| f.fuse_course(now(), HeadingNoise::from_sigma(0.05)),
        accepted,
    );
    // On a still start rather than the flight, which a standstill would rightly be rejected
    // against.
    let mut still = Eskf::default();
    assert!(still.initialize(&window()).is_ok());
    bench_fusion(
        c,
        "fuse_stationary",
        &still,
        |f| {
            let time = f.time().unwrap_or_default();
            f.fuse_stationary(time, VelocityNoise::from_speed_accuracy(0.1))
        },
        accepted,
    );
}

fn start(c: &mut Criterion) {
    let full = window();
    let next = stationary_sample(10_000);
    assert!(full.clone().push(next).is_ok());
    c.bench_function("StaticWindow::push", |b| {
        b.iter_batched_ref(|| full.clone(), |w| w.push(next), BatchSize::SmallInput)
    });
    assert!(Eskf::default().initialize(&full).is_ok());
    c.bench_function("initialize", |b| {
        b.iter_batched_ref(
            Eskf::default,
            |f| f.initialize(&full),
            BatchSize::SmallInput,
        )
    });
}

criterion_group!(benches, hot_path, start);
criterion_main!(benches);
