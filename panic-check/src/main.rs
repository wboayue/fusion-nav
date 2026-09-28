//! A bare-metal program that calls the whole public filter API, linked so that
//! `panic-check/run.sh` can read the panic paths out of the result.
//!
//! The library cannot be linked on its own: an `rlib` is not linked at all, so its dead
//! panic paths survive as undefined references and the scan reports functions no caller
//! reaches. This binary supplies the missing half — an entry point, a panic handler, and
//! a call to every entry point the filter publishes — so the linker's own dead-code
//! elimination decides what is reachable.
//!
//! Every value comes through [`black_box`] and every result goes back into it. Without
//! that the optimizer constant-folds the calls away and the scan passes by proving
//! nothing; with it, the compiler must emit each call against operands it cannot see.
//!
//! `run.sh` checks that this file names every `pub fn` in `src/`, and formats every type
//! with a `Display` impl through `show`, so a new entry point cannot join the API without
//! joining the gate.

#![no_std]
#![no_main]

use core::hint::black_box;

use fusion_nav::STATES;
use fusion_nav::prelude::*;

/// A handler is required to link, and its body is irrelevant: the gate fails on the
/// *reachability* of `core::panicking`, not on what happens after it is reached.
#[panic_handler]
fn panicked(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

/// The linker's only root. Nothing calls it — `run.sh` never runs the program, it reads
/// the symbols — so its signature only has to be one lld accepts as an ELF entry point.
#[unsafe(no_mangle)]
pub extern "C" fn _start() -> ! {
    drive();
    loop {}
}

fn drive() {
    let mut gates = Gates::at(black_box(Percentile::P99));
    gates.mag_heading = Gate::<1>::at(black_box(Percentile::P999));
    if let Some(gate) = Gate::new(black_box(gates.baro_altitude.threshold())) {
        gates.baro_altitude = gate;
    }
    let mut filter = Eskf::new(black_box(Config {
        gates,
        ..Config::default()
    }));
    let dt = Seconds::from_secs(black_box(0.0025));
    // Every `Timestamp` conversion, since each one saturates or rounds a float into integers.
    let start = Timestamp::from_secs_f64(black_box(12.5));
    let time = Timestamp::from_micros(black_box(start.as_micros()))
        .after(dt)
        .before(Seconds::from_secs(black_box(0.001)));
    let _ = black_box(time.since(start));
    let rates = ImuSample::from_rates(
        time,
        AngularRate::body(black_box(0.01), black_box(-0.02), black_box(0.03)),
        Acceleration::body(black_box(0.1), black_box(-0.2), black_box(-GRAVITY)),
        dt,
    );
    let _ = black_box(black_box(rates).accumulate(black_box(rates)));
    let sample = StaticSample {
        // Built field by field as well, so an interval the sample carries is as opaque as
        // the increments.
        imu: ImuSample {
            time: rates.time,
            delta_angle: DeltaAngle::body(black_box(2.5e-5), black_box(-5.0e-5), black_box(0.0)),
            angle_interval: Seconds::from_secs(black_box(0.0025)),
            delta_velocity: DeltaVelocity::flu(
                black_box(2.5e-4),
                black_box(0.0),
                black_box(0.0245),
            ),
            velocity_interval: Seconds::from_secs(black_box(0.0025)),
        },
        baro: Some(Altitude::from_meters(black_box(112.0))),
        mag: Some(MagField::body(
            black_box(0.22),
            black_box(0.0),
            black_box(0.44),
        )),
        velocity: None,
    };
    let mut window = [sample; 8];
    // Two dated GNSS velocities, so the window's mean acceleration is differenced and
    // divided by a span — the only division initialization does, and the one place a
    // zero span would reach `core::panicking` if it were an index or an unwrap.
    window[1].velocity = Some(Velocity::ned(
        black_box(1.0),
        black_box(0.0),
        black_box(0.0),
    ));
    window[5].velocity = Some(Velocity::ned(
        black_box(9.0),
        black_box(0.0),
        black_box(0.0),
    ));
    let fix = Geodetic::from_degrees(
        black_box(47.397_742),
        black_box(8.545_594),
        black_box(488.0),
    );
    let position = Position::ned(black_box(1.0), black_box(2.0), black_box(3.0));
    let velocity = Velocity::ned(black_box(4.0), black_box(-5.0), black_box(6.0));
    // `clamped` on both types, with the bounds opaque too: it is the one noise
    // constructor that compares and selects, and `f32::clamp` would have panicked here.
    let position_noise = PositionNoise::clamped(
        black_box(1.5),
        black_box(3.0),
        SigmaBounds::new(black_box(0.5), black_box(100.0)),
        SigmaBounds::new(black_box(0.75), black_box(100.0)),
    );
    let velocity_noise = VelocityNoise::clamped(
        black_box(0.3),
        black_box(0.45),
        SigmaBounds::at_least(black_box(0.5)),
        SigmaBounds::new(black_box(0.75), black_box(50.0)),
    );
    let _ = black_box(PositionNoise::horizontal_vertical(
        black_box(1.5),
        black_box(3.0),
    ));
    let _ = black_box(VelocityNoise::horizontal_vertical(
        black_box(0.3),
        black_box(1000.0),
    ));
    let _ = black_box(VelocityNoise::from_speed_accuracy(black_box(0.3)));

    // Folded in one at a time, as a firmware does, and from a buffered slice, as a desktop
    // caller does: two routes into the same accumulator.
    let mut streamed = StaticWindow::new();
    for sample in black_box(window) {
        let _ = black_box(streamed.push(black_box(sample)));
    }
    let _ = black_box(streamed.span());
    let _ = black_box(streamed.is_long_enough(black_box(&Initialization::default())));
    let _ = black_box(streamed.is_at_rest(black_box(&Initialization::default())));
    let _ = black_box(streamed.try_extend(black_box(window)));
    let _ = black_box(StaticWindow::try_from(black_box(&window[..])));
    let _ = black_box(filter.alignment_of(black_box(&streamed)));
    let _ = black_box(filter.initialize(black_box(&streamed)));
    let _ = black_box(filter.initialize_coarse(black_box(sample.imu)));
    let _ = black_box(filter.initialize_from(
        black_box(State::default()),
        Covariance::from_sigmas(black_box([0.5; STATES])),
        black_box(start),
    ));
    let _ = black_box(filter.time());

    let _ = black_box(filter.predict(black_box(rates)));

    let _ = black_box(filter.fuse_gnss_position(black_box(time), position, position_noise));
    let _ = black_box(filter.fuse_gnss_geodetic(black_box(time), fix, position_noise));
    let _ = black_box(filter.fuse_gnss_velocity(black_box(time), velocity, velocity_noise));
    let _ = black_box(filter.fuse_baro_altitude(
        black_box(time),
        Altitude::from_meters(black_box(60.0)),
        AltitudeNoise::from_sigma(black_box(2.0)),
    ));
    let _ = black_box(filter.fuse_mag_heading(
        black_box(time),
        MagField::body(black_box(0.22), black_box(0.0), black_box(0.44)),
        HeadingNoise::from_sigma(black_box(0.1)),
    ));
    let _ = black_box(filter.fuse_gnss_heading(
        black_box(time),
        Radians::from_radians(black_box(1.1)),
        HeadingNoise::clamped(black_box(0.01), SigmaBounds::at_least(black_box(0.1))),
    ));
    let _ =
        black_box(filter.fuse_course(black_box(time), HeadingNoise::from_sigma(black_box(0.05))));

    let _ = black_box(filter.reset_position_to(position, position_noise));
    let _ = black_box(filter.reset_velocity_to(velocity, velocity_noise));
    let _ = black_box(filter.set_baro_reference(
        Altitude::from_meters(black_box(100.0)),
        AltitudeNoise::from_sigma(black_box(0.1)),
    ));
    let _ = black_box(filter.set_origin(fix));
    let _ = black_box(filter.set_magnetic_declination(Radians::from_radians(black_box(0.1))));
    let _ = black_box(filter.magnetic_declination());
    let _ = black_box(filter.angular_rate());

    let _ = black_box(filter.state());
    let _ = black_box(filter.diagnostics());
    let _ = black_box(filter.covariance());
    let _ = black_box(filter.config());
    let _ = black_box(filter.origin());
    let _ = black_box(filter.geodetic_position());
    let _ = black_box(filter.baro_reference());
    let _ = black_box(filter.is_initialized());
    let _ = black_box(filter.is_aligned());
    let _ = black_box(filter.validity());
    let _ = black_box(filter.attitude_variance());
    let _ = black_box(filter.attitude_variance().to_enu());
    let _ = black_box(filter.predicted_validity());

    // The tangent plane is reachable through `fuse_gnss_geodetic` above, but it is also a
    // public type an application may hold on its own, and equations (43)-(44) are where
    // the f64 arithmetic lives.
    if let Some(origin) = black_box(LocalOrigin::new(fix)) {
        let _ = black_box(origin.geodetic());
        let _ = black_box(origin.to_ned(black_box(fix)));
        let _ = black_box(origin.to_geodetic(position));
    }
    let _ = black_box(LocalOrigin::placing(fix, position));
    // The magnetic model's lookup indexes a table, the shape `Index` bounds-checks.
    let _ = black_box(black_box(fix).magnetic_declination());

    // A receiver reports degrees, scaled integer degrees or radians, and an application
    // reads the fix back out in whichever of those it logs.
    let _ = black_box(Geodetic::from_radians(
        black_box(0.827),
        black_box(0.149),
        black_box(488.0),
    ));
    let _ = black_box(Geodetic::from_degrees_e7(
        black_box(473_977_420),
        black_box(85_455_940),
        black_box(488_000),
    ));
    let _ = black_box(fix.latitude_deg());
    let _ = black_box(fix.longitude_deg());
    let _ = black_box(fix.latitude_rad());
    let _ = black_box(fix.longitude_rad());
    let _ = black_box(fix.height());

    surface(
        black_box(filter.state()),
        black_box(*filter.diagnostics()),
        black_box(filter.covariance()),
    );
}

/// The rest of the public surface: accessors, constructors and the outcome enums.
///
/// Each is small enough to look obviously safe, which is the reason to link it rather than
/// the reason to skip it — `Covariance::get` indexes the same `nalgebra` matrix that
/// `Index` bounds-checks, and `Attitude::euler_angles` is two `atan2` calls and an `asin`.
fn surface(state: State, diagnostics: Diagnostics, covariance: &Covariance) {
    let _ = black_box(state.attitude.quaternion());
    let _ = black_box(state.attitude.euler_angles());
    let _ = black_box(Attitude::level());
    let _ = black_box(Attitude::body_to_ned(black_box(
        state.attitude.quaternion(),
    )));
    let _ = black_box(Attitude::ned_to_body(black_box(
        state.attitude.quaternion(),
    )));
    let _ = black_box(Attitude::flu_to_enu(black_box(state.attitude.quaternion())));
    let _ = black_box(Attitude::flu_to_nwu(black_box(state.attitude.quaternion())));
    let _ = black_box(black_box(state.attitude).as_ned_to_body());
    let _ = black_box(black_box(state.attitude).as_flu_to_enu());
    let _ = black_box(black_box(state.attitude).as_flu_to_nwu());

    let _ = black_box(state.position.x());
    let _ = black_box(state.position.y());
    let _ = black_box(state.position.z());
    let _ = black_box(state.velocity.vector());
    let _ = black_box(state.gyro_bias.to_array());
    let _ = black_box(Position::<Ned>::zero());
    let _ = black_box(Position::enu(black_box(1.0), black_box(2.0), black_box(3.0)).to_ned());
    let _ = black_box(black_box(state.position).to_enu());
    let _ = black_box(black_box(state.gyro_bias).to_flu());
    let _ = black_box(AngularRate::flu(
        black_box(0.1),
        black_box(0.2),
        black_box(0.3),
    ));
    let _ = black_box(AngularRate::body_deg_per_s(
        black_box(1.0),
        black_box(2.0),
        black_box(3.0),
    ));
    let _ = black_box(Velocity::<Ned>::from_vector(black_box(
        state.velocity.vector(),
    )));
    let _ = black_box(VelocityNoise::<Ned>::from_variance(
        black_box(0.1),
        black_box(0.2),
        black_box(0.3),
    ));
    let _ = black_box(AltitudeNoise::from_variance(black_box(4.0)).variance());
    let _ = black_box(Radians::from_degrees(black_box(30.0)));

    let _ = black_box(Covariance::from_matrix(black_box(*covariance.as_matrix())));
    let _ = black_box(covariance.as_matrix());
    let _ = black_box(covariance.get(ErrorState::PositionNorth, ErrorState::VelocityDown));
    let _ = black_box(covariance.variance(ErrorState::GyroBiasZ));
    let _ = black_box(ErrorState::GyroBiasZ.index());
    let _ = black_box(Covariance::zero());

    let _ = black_box(state.validity.all());
    let _ = black_box(state.validity.attitude());
    let _ = black_box(state.validity.navigation());
    let _ = black_box(state.status);

    for (name, source) in black_box(diagnostics.sources()) {
        let _ = black_box(name);
        let _ = black_box(source.has_been_used());
        let _ = black_box(source.accepted_within(Seconds::from_secs(black_box(1.0))));
        let _ = black_box(source.timeout(black_box(&Timeouts::default())));
        let _ = black_box(source.is_fresh(black_box(&Timeouts::default())));
        let _ = black_box(source.period());
        if let Some(innovation) = black_box(source.innovation) {
            let _ = black_box(innovation.values());
            let _ = black_box(innovation.variances());
        }
    }
    let _ = black_box(diagnostics.propagation);

    let outcome = black_box(Fusion::InvalidNoise);
    let _ = black_box(outcome.is_accepted());
    let _ = black_box(outcome.is_reset());
    let _ = black_box(outcome.refusal());
    let _ = black_box(outcome.test_ratio());
    let _ = black_box(Propagation::Propagated.is_propagated());
    let _ = black_box(Alignment::Static.is_static());

    show::<InitError>(InitError::InvalidInterval {
        interval: Seconds::from_secs(black_box(-0.5)),
    });
    show::<SampleRefusal>(SampleRefusal::InvalidStep {
        dt: Seconds::from_secs(black_box(-0.01)),
    });
    show::<InitError>(InitError::InvalidStep {
        dt: Seconds::from_secs(black_box(-0.5)),
    });
    show::<Fusion>(Fusion::OutOfHorizon {
        age: Seconds::from_secs(black_box(0.4)),
    });
    show::<Status>(state.status);
    show::<Validity>(state.validity);
    show::<Fusion>(outcome);
    show::<GnssFusion>(GnssFusion {
        horizontal: outcome,
        height: Fusion::Rejected {
            test_ratio: black_box(2.7),
        },
    });
    show::<Refusal>(Refusal::NotFinite);
    show::<Propagation>(Propagation::StepTooLong {
        dt: Seconds::from_secs(black_box(0.15)),
        limit: Seconds::from_secs(black_box(0.1)),
    });
    show::<Coarse>(Coarse::WindowTooShort {
        required: Seconds::from_secs(black_box(2.0)),
        provided: Seconds::from_secs(black_box(0.8)),
    });
    show::<Alignment>(Alignment::Coarse(Coarse::NotStationary {
        peak_gyro: RadiansPerSecond::from_rad_per_s(black_box(0.3)),
        peak_accel_deviation: MetersPerSecond2::from_m_per_s2(black_box(0.8)),
        span: Seconds::from_secs(black_box(2.0)),
        inertial_accel: None,
    }));
}

/// Format `value` into a sink the optimizer cannot see through.
///
/// The value goes through [`black_box`] first, so every arm of its `Display` is reachable
/// rather than the one the constant names. Named with a turbofish because `run.sh` checks
/// for `show::<T>(` against every `Display` impl in `src/`: core's float formatting reaches
/// `core::panicking`, so an impl printing an `f32` with `{}` fails here and nowhere else.
fn show<T: core::fmt::Display>(value: T) {
    use core::fmt::Write;
    let _ = write!(Sink, "{}", black_box(value));
}

/// A `core::fmt::Write` that keeps every byte it is handed alive.
struct Sink;

impl core::fmt::Write for Sink {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        black_box(s);
        Ok(())
    }
}
