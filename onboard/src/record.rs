//! One record per call into the filter, with every argument, so the board can make the call
//! the host made.
//!
//! A record carries the public types, encoded from their public fields and accessors and
//! rebuilt through their public constructors. Where a constructor does not round-trip a value
//! bit for bit (`Attitude::from_body_to_ned` renormalizes), the host runs the decoded record
//! too, through [`Recorder`](crate::Recorder), so both sides make the same call by
//! construction rather than by a codec that is never lossy.

use fusion_nav::prelude::*;
use fusion_nav::{Quaternion, STATES};

use crate::wire::{Reader, Writer};

/// The longest record: [`Record::InitializeFrom`], a state and the 15 × 15 covariance.
pub const MAX_RECORD: usize = 1024;

/// A call into the filter, or into the static window a start folds.
///
/// One variant carries a covariance and the rest a few dozen bytes, and the enum is sized to
/// it: a record is decoded into one slot per call, and the board has no allocator to box it in.
#[allow(clippy::large_enum_variant)]
#[derive(Clone, Copy, Debug)]
pub enum Record {
    /// Nothing: timed to measure what the timing itself costs.
    Nop,
    /// `Eskf::new`, replacing the filter.
    New(Config),
    SetOrigin(Geodetic),
    SetMagneticDeclination(Radians),
    SetBaroReference(Altitude, AltitudeNoise),
    ResetPositionTo(Position<Ned>, PositionNoise<Ned>),
    ResetVelocityTo(Velocity<Ned>, VelocityNoise<Ned>),
    /// An empty `StaticWindow`, replacing the one being folded.
    WindowNew,
    WindowPush(StaticSample),
    /// `Eskf::initialize` on the window folded since the last [`Record::WindowNew`].
    Initialize,
    InitializeCoarse(ImuSample),
    InitializeFrom(State, Covariance, Timestamp),
    Predict(ImuSample),
    FuseGnssPosition(Timestamp, Position<Ned>, PositionNoise<Ned>, Position<Body>),
    FuseGnssGeodetic(Timestamp, Geodetic, PositionNoise<Ned>, Position<Body>),
    FuseGnssVelocity(Timestamp, Velocity<Ned>, VelocityNoise<Ned>, Position<Body>),
    FuseBaroAltitude(Timestamp, Altitude, AltitudeNoise),
    FuseMagHeading(Timestamp, MagField<Body>, HeadingNoise),
    FuseGnssHeading(Timestamp, Radians, HeadingNoise),
    FuseCourse(Timestamp, HeadingNoise),
    FuseStationary(Timestamp, VelocityNoise<Ned>),
    PredictedValidity,
}

impl Record {
    /// The call's name, as the per-call CSV and the published tables spell it.
    pub const fn name(&self) -> &'static str {
        match self {
            Record::Nop => "nop",
            Record::New(_) => "new",
            Record::SetOrigin(_) => "set_origin",
            Record::SetMagneticDeclination(_) => "set_magnetic_declination",
            Record::SetBaroReference(..) => "set_baro_reference",
            Record::ResetPositionTo(..) => "reset_position_to",
            Record::ResetVelocityTo(..) => "reset_velocity_to",
            Record::WindowNew => "window_new",
            Record::WindowPush(_) => "window_push",
            Record::Initialize => "initialize",
            Record::InitializeCoarse(_) => "initialize_coarse",
            Record::InitializeFrom(..) => "initialize_from",
            Record::Predict(_) => "predict",
            Record::FuseGnssPosition(..) => "fuse_gnss_position",
            Record::FuseGnssGeodetic(..) => "fuse_gnss_geodetic",
            Record::FuseGnssVelocity(..) => "fuse_gnss_velocity",
            Record::FuseBaroAltitude(..) => "fuse_baro_altitude",
            Record::FuseMagHeading(..) => "fuse_mag_heading",
            Record::FuseGnssHeading(..) => "fuse_gnss_heading",
            Record::FuseCourse(..) => "fuse_course",
            Record::FuseStationary(..) => "fuse_stationary",
            Record::PredictedValidity => "predicted_validity",
        }
    }

    /// Encode into `buffer`, returning the length, or `None` if it does not fit.
    pub fn encode(&self, buffer: &mut [u8]) -> Option<usize> {
        let mut w = Writer::new(buffer);
        match *self {
            Record::Nop => w.u8(0),
            Record::New(config) => {
                w.u8(1);
                write_config(&mut w, &config);
            }
            Record::SetOrigin(origin) => {
                w.u8(2);
                write_geodetic(&mut w, origin);
            }
            Record::SetMagneticDeclination(declination) => {
                w.u8(3);
                w.f32(declination.as_radians());
            }
            Record::SetBaroReference(reference, noise) => {
                w.u8(4);
                w.f32(reference.as_meters());
                w.f32(noise.variance());
            }
            Record::ResetPositionTo(position, noise) => {
                w.u8(5);
                w.f32s(position.to_array());
                w.f32s(noise.variance());
            }
            Record::ResetVelocityTo(velocity, noise) => {
                w.u8(6);
                w.f32s(velocity.to_array());
                w.f32s(noise.variance());
            }
            Record::WindowNew => w.u8(7),
            Record::WindowPush(StaticSample {
                imu,
                mag,
                baro,
                velocity,
            }) => {
                w.u8(8);
                write_imu(&mut w, imu);
                write_option3(&mut w, mag.map(MagField::to_array));
                w.option_f32(baro.map(Altitude::as_meters));
                write_option3(&mut w, velocity.map(Velocity::to_array));
            }
            Record::Initialize => w.u8(9),
            Record::InitializeCoarse(imu) => {
                w.u8(10);
                write_imu(&mut w, imu);
            }
            Record::InitializeFrom(state, covariance, time) => {
                w.u8(11);
                write_state(&mut w, &state);
                for row in covariance.to_rows() {
                    w.f32s(row);
                }
                w.u64(time.as_micros());
            }
            Record::Predict(imu) => {
                w.u8(12);
                write_imu(&mut w, imu);
            }
            Record::FuseGnssPosition(time, position, noise, antenna) => {
                w.u8(13);
                w.u64(time.as_micros());
                w.f32s(position.to_array());
                w.f32s(noise.variance());
                w.f32s(antenna.to_array());
            }
            Record::FuseGnssGeodetic(time, fix, noise, antenna) => {
                w.u8(14);
                w.u64(time.as_micros());
                write_geodetic(&mut w, fix);
                w.f32s(noise.variance());
                w.f32s(antenna.to_array());
            }
            Record::FuseGnssVelocity(time, velocity, noise, antenna) => {
                w.u8(15);
                w.u64(time.as_micros());
                w.f32s(velocity.to_array());
                w.f32s(noise.variance());
                w.f32s(antenna.to_array());
            }
            Record::FuseBaroAltitude(time, altitude, noise) => {
                w.u8(16);
                w.u64(time.as_micros());
                w.f32(altitude.as_meters());
                w.f32(noise.variance());
            }
            Record::FuseMagHeading(time, field, noise) => {
                w.u8(17);
                w.u64(time.as_micros());
                w.f32s(field.to_array());
                w.f32(noise.variance());
            }
            Record::FuseGnssHeading(time, heading, noise) => {
                w.u8(18);
                w.u64(time.as_micros());
                w.f32(heading.as_radians());
                w.f32(noise.variance());
            }
            Record::FuseCourse(time, sideslip) => {
                w.u8(19);
                w.u64(time.as_micros());
                w.f32(sideslip.variance());
            }
            Record::FuseStationary(time, noise) => {
                w.u8(20);
                w.u64(time.as_micros());
                w.f32s(noise.variance());
            }
            Record::PredictedValidity => w.u8(21),
        }
        w.finish()
    }

    /// Decode one record that fills `bytes` exactly.
    pub fn decode(bytes: &[u8]) -> Option<Self> {
        let mut r = Reader::new(bytes);
        let record = match r.u8()? {
            0 => Record::Nop,
            1 => Record::New(read_config(&mut r)?),
            2 => Record::SetOrigin(read_geodetic(&mut r)?),
            3 => Record::SetMagneticDeclination(Radians::from_radians(r.f32()?)),
            4 => Record::SetBaroReference(
                Altitude::from_meters(r.f32()?),
                AltitudeNoise::from_variance(r.f32()?),
            ),
            5 => Record::ResetPositionTo(
                Position::from_array(r.f32s()?),
                noise3(r.f32s()?, PositionNoise::from_variance),
            ),
            6 => Record::ResetVelocityTo(
                Velocity::from_array(r.f32s()?),
                noise3(r.f32s()?, VelocityNoise::from_variance),
            ),
            7 => Record::WindowNew,
            8 => {
                let imu = read_imu(&mut r)?;
                let mag = read_option3(&mut r)?.map(MagField::from_array);
                let baro = r.option_f32()?.map(Altitude::from_meters);
                let velocity = read_option3(&mut r)?.map(Velocity::from_array);
                Record::WindowPush(StaticSample {
                    imu,
                    mag,
                    baro,
                    velocity,
                })
            }
            9 => Record::Initialize,
            10 => Record::InitializeCoarse(read_imu(&mut r)?),
            11 => {
                let state = read_state(&mut r)?;
                let mut rows = [[0.0; STATES]; STATES];
                for row in &mut rows {
                    *row = r.f32s()?;
                }
                let time = Timestamp::from_micros(r.u64()?);
                Record::InitializeFrom(state, Covariance::from_rows(rows), time)
            }
            12 => Record::Predict(read_imu(&mut r)?),
            13 => Record::FuseGnssPosition(
                Timestamp::from_micros(r.u64()?),
                Position::from_array(r.f32s()?),
                noise3(r.f32s()?, PositionNoise::from_variance),
                Position::from_array(r.f32s()?),
            ),
            14 => Record::FuseGnssGeodetic(
                Timestamp::from_micros(r.u64()?),
                read_geodetic(&mut r)?,
                noise3(r.f32s()?, PositionNoise::from_variance),
                Position::from_array(r.f32s()?),
            ),
            15 => Record::FuseGnssVelocity(
                Timestamp::from_micros(r.u64()?),
                Velocity::from_array(r.f32s()?),
                noise3(r.f32s()?, VelocityNoise::from_variance),
                Position::from_array(r.f32s()?),
            ),
            16 => Record::FuseBaroAltitude(
                Timestamp::from_micros(r.u64()?),
                Altitude::from_meters(r.f32()?),
                AltitudeNoise::from_variance(r.f32()?),
            ),
            17 => Record::FuseMagHeading(
                Timestamp::from_micros(r.u64()?),
                MagField::from_array(r.f32s()?),
                HeadingNoise::from_variance(r.f32()?),
            ),
            18 => Record::FuseGnssHeading(
                Timestamp::from_micros(r.u64()?),
                Radians::from_radians(r.f32()?),
                HeadingNoise::from_variance(r.f32()?),
            ),
            19 => Record::FuseCourse(
                Timestamp::from_micros(r.u64()?),
                HeadingNoise::from_variance(r.f32()?),
            ),
            20 => Record::FuseStationary(
                Timestamp::from_micros(r.u64()?),
                noise3(r.f32s()?, VelocityNoise::from_variance),
            ),
            21 => Record::PredictedValidity,
            _ => return None,
        };
        r.is_empty().then_some(record)
    }
}

fn write_option3(w: &mut Writer, value: Option<[f32; 3]>) {
    match value {
        None => w.u8(0),
        Some(components) => {
            w.u8(1);
            w.f32s(components);
        }
    }
}

fn read_option3(r: &mut Reader) -> Option<Option<[f32; 3]>> {
    match r.u8()? {
        0 => Some(None),
        1 => r.f32s().map(Some),
        _ => None,
    }
}

fn noise3<T>([x, y, z]: [f32; 3], from_variance: fn(f32, f32, f32) -> T) -> T {
    from_variance(x, y, z)
}

fn write_geodetic(w: &mut Writer, point: Geodetic) {
    w.f64(point.latitude_rad());
    w.f64(point.longitude_rad());
    w.f64(point.height());
}

fn read_geodetic(r: &mut Reader) -> Option<Geodetic> {
    Some(Geodetic::from_radians(r.f64()?, r.f64()?, r.f64()?))
}

fn write_imu(w: &mut Writer, imu: ImuSample) {
    let ImuSample {
        time,
        delta_angle,
        angle_interval,
        delta_velocity,
        velocity_interval,
    } = imu;
    w.u64(time.as_micros());
    w.f32s(delta_angle.to_array());
    w.f32(angle_interval.as_secs());
    w.f32s(delta_velocity.to_array());
    w.f32(velocity_interval.as_secs());
}

fn read_imu(r: &mut Reader) -> Option<ImuSample> {
    Some(ImuSample {
        time: Timestamp::from_micros(r.u64()?),
        delta_angle: DeltaAngle::from_array(r.f32s()?),
        angle_interval: Seconds::from_secs(r.f32()?),
        delta_velocity: DeltaVelocity::from_array(r.f32s()?),
        velocity_interval: Seconds::from_secs(r.f32()?),
    })
}

fn write_state(w: &mut Writer, state: &State) {
    let State {
        attitude,
        position,
        velocity,
        accel_bias,
        gyro_bias,
        status,
        validity,
    } = *state;
    let q = attitude.body_to_ned();
    w.f32s([q.w, q.x, q.y, q.z]);
    w.f32s(position.to_array());
    w.f32s(velocity.to_array());
    w.f32s(accel_bias.to_array());
    w.f32s(gyro_bias.to_array());
    w.u8(status_code(status));
    w.u8(validity_bits(validity));
}

fn read_state(r: &mut Reader) -> Option<State> {
    let [w, x, y, z] = r.f32s()?;
    Some(State {
        attitude: Attitude::from_body_to_ned(Quaternion { w, x, y, z }),
        position: Position::from_array(r.f32s()?),
        velocity: Velocity::from_array(r.f32s()?),
        accel_bias: Acceleration::from_array(r.f32s()?),
        gyro_bias: AngularRate::from_array(r.f32s()?),
        status: status_of(r.u8()?)?,
        validity: validity_of(r.u8()?),
    })
}

pub(crate) const fn status_code(status: Status) -> u8 {
    match status {
        Status::Healthy => 0,
        Status::Degraded => 1,
        Status::DeadReckoning => 2,
        Status::Aligning => 3,
    }
}

fn status_of(code: u8) -> Option<Status> {
    Some(match code {
        0 => Status::Healthy,
        1 => Status::Degraded,
        2 => Status::DeadReckoning,
        3 => Status::Aligning,
        _ => return None,
    })
}

pub(crate) const fn validity_bits(validity: Validity) -> u8 {
    let Validity {
        tilt,
        heading,
        horizontal_position,
        vertical_position,
        horizontal_velocity,
        vertical_velocity,
    } = validity;
    (tilt as u8)
        | (heading as u8) << 1
        | (horizontal_position as u8) << 2
        | (vertical_position as u8) << 3
        | (horizontal_velocity as u8) << 4
        | (vertical_velocity as u8) << 5
}

fn validity_of(bits: u8) -> Validity {
    let flag = |i: u8| bits & (1 << i) != 0;
    Validity {
        tilt: flag(0),
        heading: flag(1),
        horizontal_position: flag(2),
        vertical_position: flag(3),
        horizontal_velocity: flag(4),
        vertical_velocity: flag(5),
    }
}

/// Every field, destructured without `..`, so a field added to `Config` or to any struct in it
/// does not compile here until the trace carries it.
fn write_config(w: &mut Writer, config: &Config) {
    let Config {
        imu,
        gates,
        timeouts,
        recovery,
        correlation,
        init,
        accuracy,
        gravity,
        max_predict_dt,
        coast,
        hold,
        baro_offset_walk,
        baro_reference_from_estimate,
        yaw_estimator,
    } = *config;
    let ImuNoise {
        gyro_white,
        accel_white,
        gyro_bias_walk,
        accel_bias_walk,
    } = imu;
    w.f32s([gyro_white, accel_white, gyro_bias_walk, accel_bias_walk]);
    let Gates {
        gnss_position,
        gnss_height,
        gnss_velocity,
        baro_altitude,
        mag_heading,
        gnss_heading,
        course,
        stationary,
        position_hold,
    } = gates;
    w.f32s([
        gnss_position.threshold(),
        gnss_height.threshold(),
        gnss_velocity.threshold(),
        baro_altitude.threshold(),
        mag_heading.threshold(),
        gnss_heading.threshold(),
        course.threshold(),
        stationary.threshold(),
        position_hold.threshold(),
    ]);
    let Timeouts {
        dead_reckoning_after,
    } = timeouts;
    w.f32(dead_reckoning_after.as_secs());
    let Recovery {
        gnss_position,
        gnss_height,
        gnss_velocity,
        baro_altitude,
        mag_heading,
        gnss_heading,
        course,
        yaw_estimator: recover_yaw,
    } = recovery;
    for after in [
        gnss_position,
        gnss_height,
        gnss_velocity,
        baro_altitude,
        mag_heading,
        gnss_heading,
        course,
        recover_yaw,
    ] {
        w.option_f32(after.map(Seconds::as_secs));
    }
    let Correlation {
        gnss_position,
        gnss_height,
        gnss_velocity,
        baro_altitude,
        mag_heading,
        gnss_heading,
        course,
    } = correlation;
    for tau in [
        gnss_position,
        gnss_height,
        gnss_velocity,
        baro_altitude,
        mag_heading,
        gnss_heading,
        course,
    ] {
        w.option_f32(tau.map(Seconds::as_secs));
    }
    let Initialization {
        min_duration,
        max_gyro_rate,
        max_accel_deviation,
        sigma_position,
        sigma_velocity,
        sigma_tilt,
        sigma_yaw,
        sigma_accel_bias,
        sigma_gyro_bias,
    } = init;
    w.f32s([
        min_duration.as_secs(),
        max_gyro_rate.as_rad_per_s(),
        max_accel_deviation.as_m_per_s2(),
        sigma_position.as_meters(),
        sigma_velocity.as_m_per_s(),
        sigma_tilt.as_radians(),
        sigma_yaw.as_radians(),
        sigma_accel_bias.as_m_per_s2(),
        sigma_gyro_bias.as_rad_per_s(),
    ]);
    let Accuracy {
        tilt,
        heading,
        position,
        velocity,
        horizon,
    } = accuracy;
    w.f32s([
        tilt.as_radians(),
        heading.as_radians(),
        position.as_meters(),
        velocity.as_m_per_s(),
        horizon.as_secs(),
    ]);
    w.f32(gravity);
    w.f32(max_predict_dt.as_secs());
    match coast {
        None => w.u8(0),
        Some(Coast {
            acceleration,
            rotation,
        }) => {
            w.u8(1);
            w.f32s([acceleration, rotation]);
        }
    }
    w.option_f32(hold.map(|Hold { sigma }| sigma.as_meters()));
    w.f32(baro_offset_walk);
    w.bool(baro_reference_from_estimate);
    w.bool(yaw_estimator);
}

fn read_config(r: &mut Reader) -> Option<Config> {
    let [gyro_white, accel_white, gyro_bias_walk, accel_bias_walk] = r.f32s()?;
    let imu = ImuNoise {
        gyro_white,
        accel_white,
        gyro_bias_walk,
        accel_bias_walk,
    };
    let [p, h, v, b, m, g, c, s, hold_gate] = r.f32s()?;
    let gates = Gates {
        gnss_position: Gate::new(p)?,
        gnss_height: Gate::new(h)?,
        gnss_velocity: Gate::new(v)?,
        baro_altitude: Gate::new(b)?,
        mag_heading: Gate::new(m)?,
        gnss_heading: Gate::new(g)?,
        course: Gate::new(c)?,
        stationary: Gate::new(s)?,
        position_hold: Gate::new(hold_gate)?,
    };
    let timeouts = Timeouts {
        dead_reckoning_after: Seconds::from_secs(r.f32()?),
    };
    let mut seconds = || r.option_f32().map(|o| o.map(Seconds::from_secs));
    let recovery = Recovery {
        gnss_position: seconds()?,
        gnss_height: seconds()?,
        gnss_velocity: seconds()?,
        baro_altitude: seconds()?,
        mag_heading: seconds()?,
        gnss_heading: seconds()?,
        course: seconds()?,
        yaw_estimator: seconds()?,
    };
    let correlation = Correlation {
        gnss_position: seconds()?,
        gnss_height: seconds()?,
        gnss_velocity: seconds()?,
        baro_altitude: seconds()?,
        mag_heading: seconds()?,
        gnss_heading: seconds()?,
        course: seconds()?,
    };
    let [
        min_duration,
        rate,
        deviation,
        position,
        velocity,
        tilt,
        yaw,
        accel_bias,
        gyro_bias,
    ] = r.f32s()?;
    let init = Initialization {
        min_duration: Seconds::from_secs(min_duration),
        max_gyro_rate: RadiansPerSecond::from_rad_per_s(rate),
        max_accel_deviation: MetersPerSecond2::from_m_per_s2(deviation),
        sigma_position: Meters::from_meters(position),
        sigma_velocity: MetersPerSecond::from_m_per_s(velocity),
        sigma_tilt: Radians::from_radians(tilt),
        sigma_yaw: Radians::from_radians(yaw),
        sigma_accel_bias: MetersPerSecond2::from_m_per_s2(accel_bias),
        sigma_gyro_bias: RadiansPerSecond::from_rad_per_s(gyro_bias),
    };
    let [tilt, heading, position, velocity, horizon] = r.f32s()?;
    let accuracy = Accuracy {
        tilt: Radians::from_radians(tilt),
        heading: Radians::from_radians(heading),
        position: Meters::from_meters(position),
        velocity: MetersPerSecond::from_m_per_s(velocity),
        horizon: Seconds::from_secs(horizon),
    };
    let gravity = r.f32()?;
    let max_predict_dt = Seconds::from_secs(r.f32()?);
    let coast = match r.u8()? {
        0 => None,
        1 => {
            let [acceleration, rotation] = r.f32s()?;
            Some(Coast {
                acceleration,
                rotation,
            })
        }
        _ => return None,
    };
    let hold = r.option_f32()?.map(|sigma| Hold {
        sigma: Meters::from_meters(sigma),
    });
    Some(Config {
        imu,
        gates,
        timeouts,
        recovery,
        correlation,
        init,
        accuracy,
        gravity,
        max_predict_dt,
        coast,
        hold,
        baro_offset_walk: r.f32()?,
        baro_reference_from_estimate: r.bool()?,
        yaw_estimator: r.bool()?,
    })
}
