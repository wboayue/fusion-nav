use bench::{
    SITE, altitude, imu_sample, magnetic_field, position, position_noise, sample_time,
    stationary_sample, velocity, velocity_noise,
};
use fusion_nav::prelude::*;

use super::*;

/// A `Config` in which no two numbers are equal and none is the default, so a codec that
/// writes one field where another belongs, or drops one, reads back a different `Config`.
fn distinct_config() -> Config {
    let mut n = 0.0f32;
    let mut next = || {
        n += 1.0;
        0.01 * n + 0.001
    };
    let seconds = |x: f32| Seconds::from_secs(x);
    Config {
        imu: ImuNoise {
            gyro_white: next(),
            accel_white: next(),
            gyro_bias_walk: next(),
            accel_bias_walk: next(),
        },
        gates: Gates {
            gnss_position: Gate::new(next()).unwrap(),
            gnss_height: Gate::new(next()).unwrap(),
            gnss_velocity: Gate::new(next()).unwrap(),
            baro_altitude: Gate::new(next()).unwrap(),
            mag_heading: Gate::new(next()).unwrap(),
            gnss_heading: Gate::new(next()).unwrap(),
            course: Gate::new(next()).unwrap(),
            stationary: Gate::new(next()).unwrap(),
            position_hold: Gate::new(next()).unwrap(),
        },
        timeouts: Timeouts {
            dead_reckoning_after: seconds(next()),
        },
        recovery: Recovery {
            gnss_position: Some(seconds(next())),
            gnss_height: None,
            gnss_velocity: Some(seconds(next())),
            baro_altitude: Some(seconds(next())),
            mag_heading: None,
            gnss_heading: Some(seconds(next())),
            course: Some(seconds(next())),
            yaw_estimator: Some(seconds(next())),
        },
        correlation: Correlation {
            gnss_position: Some(seconds(next())),
            gnss_height: Some(seconds(next())),
            gnss_velocity: None,
            baro_altitude: Some(seconds(next())),
            mag_heading: Some(seconds(next())),
            gnss_heading: None,
            course: Some(seconds(next())),
        },
        init: Initialization {
            min_duration: seconds(next()),
            max_gyro_rate: RadiansPerSecond::from_rad_per_s(next()),
            max_accel_deviation: MetersPerSecond2::from_m_per_s2(next()),
            sigma_position: Meters::from_meters(next()),
            sigma_velocity: MetersPerSecond::from_m_per_s(next()),
            sigma_tilt: Radians::from_radians(next()),
            sigma_yaw: Radians::from_radians(next()),
            sigma_accel_bias: MetersPerSecond2::from_m_per_s2(next()),
            sigma_gyro_bias: RadiansPerSecond::from_rad_per_s(next()),
        },
        accuracy: Accuracy {
            tilt: Radians::from_radians(next()),
            heading: Radians::from_radians(next()),
            position: Meters::from_meters(next()),
            velocity: MetersPerSecond::from_m_per_s(next()),
            horizon: seconds(next()),
        },
        gravity: 9.78 + 0.05 * next(),
        max_predict_dt: seconds(next()),
        coast: Some(Coast {
            acceleration: next(),
            rotation: next(),
        }),
        hold: Some(Hold {
            sigma: Meters::from_meters(next()),
        }),
        baro_offset_walk: next(),
        baro_reference_from_estimate: false,
        yaw_estimator: false,
    }
}

fn round_trip(record: &Record) -> Record {
    let mut buffer = [0u8; MAX_RECORD];
    let length = record.encode(&mut buffer).expect("fits");
    let decoded = Record::decode(&buffer[..length]).expect("decodes");
    let mut again = [0u8; MAX_RECORD];
    let length_again = decoded.encode(&mut again).expect("fits");
    assert_eq!(
        buffer[..length],
        again[..length_again],
        "{} re-encodes differently",
        record.name()
    );
    // A byte too many or too few is refused rather than read as a record.
    assert!(Record::decode(&buffer[..length - 1]).is_none());
    let mut longer = buffer;
    longer[length] = 0;
    assert!(Record::decode(&longer[..=length]).is_none());
    decoded
}

#[test]
fn a_config_survives_the_trace_field_for_field() {
    let config = distinct_config();
    if let Err(error) = Eskf::new(config) {
        panic!("the fixture is not a valid Config: {error:?}");
    }
    let Record::New(decoded) = round_trip(&Record::New(config)) else {
        panic!("decoded as another record");
    };
    assert_eq!(decoded, config);
    let Record::New(decoded) = round_trip(&Record::New(Config {
        coast: None,
        hold: None,
        ..config
    })) else {
        panic!("decoded as another record");
    };
    assert_eq!(decoded.coast, None);
    assert_eq!(decoded.hold, None);
}

#[test]
fn every_record_round_trips() {
    let t = Timestamp::from_micros(123_456_789);
    let fix = Geodetic::from_degrees(SITE.0, SITE.1, SITE.2);
    let antenna = Position::body(0.1, -0.2, 0.3);
    let heading = HeadingNoise::from_sigma(0.05);
    let state = State {
        velocity: velocity(),
        ..State::default()
    };
    let covariance = Covariance::from_sigmas(core::array::from_fn(|i| 0.1 + i as f32));
    let records = [
        Record::Nop,
        Record::New(Config::default()),
        Record::SetOrigin(fix),
        Record::SetMagneticDeclination(Radians::from_radians(0.07)),
        Record::SetBaroReference(altitude(), AltitudeNoise::from_sigma(0.3)),
        Record::ResetPositionTo(position(1.0), position_noise()),
        Record::ResetVelocityTo(velocity(), velocity_noise()),
        Record::WindowNew,
        Record::WindowPush(stationary_sample(3)),
        Record::WindowPush(StaticSample {
            mag: None,
            baro: None,
            velocity: Some(velocity()),
            ..stationary_sample(4)
        }),
        Record::Initialize,
        Record::InitializeCoarse(imu_sample(t)),
        Record::InitializeFrom(state, covariance, t),
        Record::Predict(imu_sample(t)),
        Record::FuseGnssPosition(t, position(2.0), position_noise(), antenna),
        Record::FuseGnssGeodetic(t, fix, position_noise(), antenna),
        Record::FuseGnssVelocity(t, velocity(), velocity_noise(), antenna),
        Record::FuseBaroAltitude(t, altitude(), AltitudeNoise::from_sigma(0.5)),
        Record::FuseMagHeading(t, magnetic_field(), heading),
        Record::FuseGnssHeading(t, Radians::from_radians(0.1), heading),
        Record::FuseCourse(t, heading),
        Record::FuseStationary(t, velocity_noise()),
        Record::PredictedValidity,
        Record::State,
        Record::GeodeticPosition,
    ];
    // Exhaustive, so a new record does not compile until it is in the list above as well as here.
    let listed = |record: &Record| match record {
        Record::Nop
        | Record::New(_)
        | Record::SetOrigin(_)
        | Record::SetMagneticDeclination(_)
        | Record::SetBaroReference(..)
        | Record::ResetPositionTo(..)
        | Record::ResetVelocityTo(..)
        | Record::WindowNew
        | Record::WindowPush(_)
        | Record::Initialize
        | Record::InitializeCoarse(_)
        | Record::InitializeFrom(..)
        | Record::Predict(_)
        | Record::FuseGnssPosition(..)
        | Record::FuseGnssGeodetic(..)
        | Record::FuseGnssVelocity(..)
        | Record::FuseBaroAltitude(..)
        | Record::FuseMagHeading(..)
        | Record::FuseGnssHeading(..)
        | Record::FuseCourse(..)
        | Record::FuseStationary(..)
        | Record::PredictedValidity
        | Record::State
        | Record::GeodeticPosition => record.tag(),
    };
    let mut tags: Vec<u8> = records.iter().map(listed).collect();
    tags.sort_unstable();
    tags.dedup();
    assert_eq!(
        tags,
        (0..=23).collect::<Vec<u8>>(),
        "every tag once, none skipped"
    );
    assert!(
        Record::decode(&[24]).is_none(),
        "a tag past the last decodes as nothing"
    );
    for record in &records {
        let decoded = round_trip(record);
        assert_eq!(decoded.name(), record.name());
    }
    let longest = records
        .iter()
        .map(|r| r.encode(&mut [0u8; MAX_RECORD]).unwrap())
        .max()
        .unwrap();
    assert!(
        longest <= MAX_RECORD && longest > MAX_RECORD / 2,
        "MAX_RECORD {MAX_RECORD} is far from the longest record, {longest}"
    );
}

/// A static start, a geodetic fix and two seconds aided by every source, recorded.
fn recorded_flight() -> Vec<u8> {
    let mut filter = Recorder::new(Config::default(), Some(Vec::new())).unwrap();
    let samples = (0..800).map(stationary_sample);
    assert_eq!(filter.initialize_on(samples), Ok(Alignment::Static));
    let (lat, lon, height) = SITE;
    let first = sample_time(801);
    assert!(filter.predict(imu_sample(first)).is_propagated());
    let fix = filter.fuse_gnss_geodetic(
        first,
        Geodetic::from_degrees(lat, lon, height),
        position_noise(),
        Position::zero(),
    );
    assert!(fix.is_accepted(), "{fix:?}");
    let mut accepted = 0;
    for n in 802..=1600 {
        let time = sample_time(n);
        assert!(filter.predict(imu_sample(time)).is_propagated());
        if n % 80 == 0 {
            let still = Velocity::ned(0.0, 0.0, 0.0);
            accepted += u32::from(
                filter
                    .fuse_gnss_velocity(time, still, velocity_noise(), Position::zero())
                    .is_accepted(),
            );
            accepted += u32::from(
                filter
                    .fuse_mag_heading(time, magnetic_field(), HeadingNoise::from_sigma(0.05))
                    .is_accepted(),
            );
        }
        if n % 20 == 0 {
            accepted += u32::from(
                filter
                    .fuse_baro_altitude(
                        time,
                        Altitude::from_meters(52.0),
                        AltitudeNoise::from_sigma(0.5),
                    )
                    .is_accepted(),
            );
        }
    }
    assert!(accepted > 50, "the flight fuses: {accepted}");
    filter.predicted_validity();
    filter.take_sink().unwrap()
}

#[test]
fn a_fresh_machine_makes_every_recorded_call_and_agrees_bit_for_bit() {
    assert!(replayed(&recorded_flight()) > 1000);
}

#[test]
fn a_digest_tells_two_filters_apart() {
    // The check above could pass on a digest that reads nothing; it must not.
    let trace = recorded_flight();
    let mut machine = Machine::default();
    let mut rest = trace.as_slice();
    let mut differed = false;
    while let Some((frame, after)) = Frame::split(rest) {
        rest = after;
        let mut record = Record::decode(frame.record).unwrap();
        if let Record::FuseBaroAltitude(t, altitude, noise) = record {
            record = Record::FuseBaroAltitude(
                t,
                Altitude::from_meters(altitude.as_meters() + 1e-3),
                noise,
            );
        }
        let returned = machine.execute(&record);
        differed |= frame.check(&machine, &returned) & flags::DIGEST == 0;
    }
    assert!(differed);
}

#[test]
fn a_frame_cut_short_is_not_a_frame() {
    let trace = recorded_flight();
    let (first, _) = Frame::split(&trace).unwrap();
    let length = 2 + first.record.len() + 2 + 8;
    assert_eq!(length, first.record.len() + FRAME_OVERHEAD);
    assert!(Frame::split(&trace[..length - 1]).is_none());
    assert!(Frame::split(&trace[..length]).is_some());
}

#[test]
fn outcomes_name_a_gnss_fix_by_both_halves() {
    let both = Outcome::of(&Returned::Gnss(GnssFusion {
        horizontal: Fusion::Accepted { test_ratio: 0.1 },
        height: Fusion::Rejected { test_ratio: 2.0 },
    }));
    assert_eq!(both.names(), ("accepted", Some("rejected")));
    let one = Outcome::of(&Returned::Fusion(Fusion::Reset));
    assert_eq!(one.names(), ("reset", None));
    let valid = Outcome::of(&Returned::Validity(Validity::NONE));
    assert_eq!(valid.names(), ("validity", None));
    let state = Outcome::of(&Returned::State(State {
        status: Status::DeadReckoning,
        ..State::default()
    }));
    assert_eq!(state.names(), ("state", None));
}

/// Re-execute `trace` on a fresh machine, requiring every outcome and digest; the call count.
fn replayed(trace: &[u8]) -> usize {
    let mut machine = Machine::default();
    let mut rest = trace;
    let mut calls = 0;
    while !rest.is_empty() {
        let (frame, after) = Frame::split(rest).expect("whole frames");
        rest = after;
        let record = Record::decode(frame.record).expect("decodes");
        let returned = machine.execute(&record);
        assert_eq!(
            frame.check(&machine, &returned),
            flags::OUTCOME | flags::DIGEST,
            "call {calls}, {}",
            record.name()
        );
        calls += 1;
    }
    calls
}

#[test]
fn a_rewound_run_forgets_the_calls_it_never_made() {
    let mut filter = Recorder::new(Config::default(), Some(Vec::new())).unwrap();
    assert!(
        filter
            .initialize_on((0..800).map(stationary_sample))
            .is_ok()
    );
    let snapshot = filter.clone();
    // Calls the rewound run never makes: they move the state, so a trace that kept them
    // would disagree with the machine replaying it.
    for n in 801..=900 {
        assert!(filter.predict(imu_sample(sample_time(n))).is_propagated());
    }
    let trace = filter.take_sink();
    let mut filter = snapshot;
    filter.resume(trace).unwrap();
    for n in 801..=820 {
        let still = imu_sample(sample_time(n));
        assert!(filter.predict(still).is_propagated());
    }
    // `New`, a window of 800 pushes, its start, and the twenty predictions after the rewind.
    assert_eq!(replayed(&filter.take_sink().unwrap()), 1 + 1 + 800 + 1 + 20);
}

/// Every `pub fn` on `Eskf` is a `Record`, so the board times it, or a read named here as one the
/// board does not time. Read off the source, as `panic-check/run.sh` reads it: a new entry point
/// fails here until it is one or the other.
#[test]
fn every_entry_point_is_recorded_or_named_as_untimed() {
    const UNTIMED: [&str; 14] = [
        // Constructors and plain field reads: no arithmetic to time.
        "new",
        "config",
        "diagnostics",
        "covariance",
        "time",
        "magnetic_declination",
        "baro_reference",
        "origin",
        "angular_rate",
        "is_initialized",
        "is_aligned",
        // Reads the trace does not carry: an integrator's choice, not the loop's.
        "alignment_of",
        "attitude_variance",
        "validity",
    ];
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../src");
    let mut files = vec![root.join("eskf.rs")];
    for entry in std::fs::read_dir(root.join("eskf")).unwrap() {
        files.push(entry.unwrap().path());
    }
    let recorded: Vec<&str> = [
        Record::Nop,
        Record::SetMagneticDeclination(Radians::ZERO),
        Record::WindowNew,
        Record::Initialize,
        Record::PredictedValidity,
        Record::State,
        Record::GeodeticPosition,
    ]
    .iter()
    .map(Record::name)
    .chain([
        "set_origin",
        "set_baro_reference",
        "reset_position_to",
        "reset_velocity_to",
        "initialize_coarse",
        "initialize_from",
        "predict",
        "fuse_gnss_position",
        "fuse_gnss_geodetic",
        "fuse_gnss_velocity",
        "fuse_baro_altitude",
        "fuse_mag_heading",
        "fuse_gnss_heading",
        "fuse_course",
        "fuse_stationary",
    ])
    .collect();
    let mut missing = Vec::new();
    for file in files {
        let text = std::fs::read_to_string(&file).unwrap();
        // Up to the test module: a test's helpers are not the API.
        let text = text.split("#[cfg(test)]").next().unwrap();
        for line in text.lines() {
            let line = line.trim_start();
            let Some(rest) = line
                .strip_prefix("pub fn ")
                .or_else(|| line.strip_prefix("pub const fn "))
            else {
                continue;
            };
            let name = rest.split(['(', '<']).next().unwrap();
            if !recorded.contains(&name) && !UNTIMED.contains(&name) {
                missing.push(format!("{}: {name}", file.display()));
            }
        }
    }
    assert!(
        missing.is_empty(),
        "neither recorded nor named untimed: {missing:?}"
    );
}
