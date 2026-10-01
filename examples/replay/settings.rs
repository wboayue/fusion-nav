//! `--set <field>=<value>`: the `Config` fields a log can derive, named by their path from
//! `Config` as a struct literal writes them and `ConfigError` reports them.
//!
//! Two functions over one list: [`set`] writes a value into a `Config`, and [`settings`] reads
//! every one back out, so `--derive` can print the `--set` line equivalent to the `Config` it
//! prints and a test can hold the two to each other. [`settings`] destructures without `..`,
//! as `Config::validate` does, so a field added to a derived part of `Config` does not compile
//! until it has a name here.
//!
//! What a log cannot derive is left out on purpose: `accuracy` and `timeouts` are the
//! mission's, `init`'s tolerances are the window's policy, and `gates` take a percentile, not a
//! number. `--recovery off` stays the switch for `Recovery::OFF` as a whole.

use fusion_nav::Seconds;
use fusion_nav::prelude::*;

/// Write `value` into the field `name`, or say why not.
///
/// Numbers are SI, the field's own unit; `none` turns an optional field off; `coast=none` is
/// `Config::coast = None` and `correlation=white` is `Correlation::WHITE`. The result is not
/// validated here: `Eskf::new` refuses a value outside its bound, with the same field name.
pub fn set(config: &mut Config, name: &str, value: &str) -> Result<(), String> {
    let number = || -> Result<f32, String> {
        value
            .parse::<f32>()
            .map_err(|_| format!("--set {name}: `{value}` is not a number"))
    };
    let seconds = || number().map(Seconds::from_secs);
    let optional = || -> Result<Option<Seconds>, String> {
        if value == "none" {
            Ok(None)
        } else {
            seconds().map(Some)
        }
    };
    match name {
        "max_predict_dt" => config.max_predict_dt = seconds()?,
        "gravity" => config.gravity = number()?,
        "baro_offset_walk" => config.baro_offset_walk = number()?,
        "coast" if value == "none" => config.coast = None,
        "coast.acceleration" => config.coast.get_or_insert_default().acceleration = number()?,
        "coast.rotation" => config.coast.get_or_insert_default().rotation = number()?,
        "correlation" if value == "white" => config.correlation = Correlation::WHITE,
        "imu.gyro_white" => config.imu.gyro_white = number()?,
        "imu.accel_white" => config.imu.accel_white = number()?,
        "imu.gyro_bias_walk" => config.imu.gyro_bias_walk = number()?,
        "imu.accel_bias_walk" => config.imu.accel_bias_walk = number()?,
        _ => {
            if let Some(field) = name.strip_prefix("correlation.") {
                *per_source(&mut config.correlation, field, name)? = optional()?;
            } else if let Some(field) = name.strip_prefix("recovery.") {
                *per_source(&mut config.recovery, field, name)? = optional()?;
            } else {
                return Err(format!("--set: no field `{name}`"));
            }
        }
    }
    Ok(())
}

/// Every name [`set`] takes, with `config`'s value in the form `set` reads back.
pub fn settings(config: &Config) -> Vec<(String, String)> {
    let Config {
        imu:
            ImuNoise {
                gyro_white,
                accel_white,
                gyro_bias_walk,
                accel_bias_walk,
            },
        gates: _,
        timeouts: _,
        recovery,
        correlation,
        init: _,
        accuracy: _,
        gravity,
        max_predict_dt,
        coast,
        baro_offset_walk,
        baro_reference_from_estimate: _,
    } = *config;
    // `{}` on an `f32` prints the shortest text that parses back to the same value.
    let mut out = vec![
        ("imu.gyro_white".to_string(), gyro_white.to_string()),
        ("imu.accel_white".to_string(), accel_white.to_string()),
        ("imu.gyro_bias_walk".to_string(), gyro_bias_walk.to_string()),
        (
            "imu.accel_bias_walk".to_string(),
            accel_bias_walk.to_string(),
        ),
        ("gravity".to_string(), gravity.to_string()),
        (
            "max_predict_dt".to_string(),
            max_predict_dt.as_secs().to_string(),
        ),
        ("baro_offset_walk".to_string(), baro_offset_walk.to_string()),
    ];
    match coast {
        None => out.push(("coast".into(), "none".into())),
        Some(Coast {
            acceleration,
            rotation,
        }) => {
            out.push(("coast.acceleration".into(), acceleration.to_string()));
            out.push(("coast.rotation".into(), rotation.to_string()));
        }
    }
    let optional =
        |value: Option<Seconds>| value.map_or("none".into(), |s| s.as_secs().to_string());
    for (field, value) in correlation.fields() {
        out.push((format!("correlation.{field}"), optional(value)));
    }
    for (field, value) in recovery.fields() {
        out.push((format!("recovery.{field}"), optional(value)));
    }
    out
}

/// `Correlation` and `Recovery` share their shape: one optional duration per source, under the
/// source names `Gates` and `Diagnostics` use.
pub trait PerSource {
    fn fields(&self) -> [(&'static str, Option<Seconds>); 7];
    fn field_mut(&mut self, field: &str) -> Option<&mut Option<Seconds>>;
}

macro_rules! per_source_impl {
    ($type:ty) => {
        impl PerSource for $type {
            fn fields(&self) -> [(&'static str, Option<Seconds>); 7] {
                let Self {
                    gnss_position,
                    gnss_height,
                    gnss_velocity,
                    baro_altitude,
                    mag_heading,
                    gnss_heading,
                    course,
                } = *self;
                [
                    ("gnss_position", gnss_position),
                    ("gnss_height", gnss_height),
                    ("gnss_velocity", gnss_velocity),
                    ("baro_altitude", baro_altitude),
                    ("mag_heading", mag_heading),
                    ("gnss_heading", gnss_heading),
                    ("course", course),
                ]
            }

            fn field_mut(&mut self, field: &str) -> Option<&mut Option<Seconds>> {
                Some(match field {
                    "gnss_position" => &mut self.gnss_position,
                    "gnss_height" => &mut self.gnss_height,
                    "gnss_velocity" => &mut self.gnss_velocity,
                    "baro_altitude" => &mut self.baro_altitude,
                    "mag_heading" => &mut self.mag_heading,
                    "gnss_heading" => &mut self.gnss_heading,
                    "course" => &mut self.course,
                    _ => return None,
                })
            }
        }
    };
}
per_source_impl!(Correlation);
per_source_impl!(Recovery);

fn per_source<'a>(
    of: &'a mut impl PerSource,
    field: &str,
    name: &str,
) -> Result<&'a mut Option<Seconds>, String> {
    of.field_mut(field)
        .ok_or_else(|| format!("--set: no field `{name}`"))
}

/// The `--set` arguments that turn `Config::default()` into `config`: only the names whose
/// value differs, in [`settings`] order.
pub fn arguments(config: &Config) -> Vec<String> {
    let defaults = settings(&Config::default());
    settings(config)
        .into_iter()
        .zip(defaults)
        .filter(|(mine, default)| mine != default)
        .map(|((name, value), _)| format!("--set {name}={value}"))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A `Config` with every settable field away from its default, each to a value no other
    /// field holds, so a setter writing the wrong field is caught by the round trip.
    fn everything_moved() -> Config {
        let mut config = Config::default();
        config.imu = ImuNoise {
            gyro_white: 0.011,
            accel_white: 0.21,
            gyro_bias_walk: 3.1e-5,
            accel_bias_walk: 4.1e-4,
        };
        config.gravity = 9.7951;
        config.max_predict_dt = Seconds::from_secs(0.125);
        config.baro_offset_walk = 0.07;
        config.coast = Some(Coast {
            acceleration: 1.3,
            rotation: 0.17,
        });
        config.correlation = Correlation {
            gnss_position: Some(Seconds::from_secs(2.5)),
            gnss_height: None,
            gnss_velocity: Some(Seconds::from_secs(0.61)),
            baro_altitude: Some(Seconds::from_secs(0.33)),
            mag_heading: Some(Seconds::from_secs(2.9)),
            gnss_heading: Some(Seconds::from_secs(0.47)),
            course: Some(Seconds::from_secs(1.9)),
        };
        config.recovery = Recovery {
            gnss_position: Some(Seconds::from_secs(8.5)),
            gnss_height: Some(Seconds::from_secs(6.5)),
            gnss_velocity: None,
            baro_altitude: Some(Seconds::from_secs(4.5)),
            mag_heading: Some(Seconds::from_secs(9.5)),
            gnss_heading: Some(Seconds::from_secs(10.5)),
            course: Some(Seconds::from_secs(11.5)),
        };
        config
    }

    fn applied(arguments: &[String]) -> Config {
        let mut config = Config::default();
        for argument in arguments {
            let (name, value) = argument
                .strip_prefix("--set ")
                .and_then(|pair| pair.split_once('='))
                .expect("a --set argument");
            set(&mut config, name, value).expect("a name set takes");
        }
        config
    }

    #[test]
    fn the_arguments_for_a_config_set_that_config() {
        let config = everything_moved();
        let arguments = arguments(&config);
        // Every settable field moved, so every one is named: 7 scalars, the coast pair and
        // two per-source tables.
        assert_eq!(arguments.len(), 7 + 2 + 7 + 7, "{arguments:?}");
        assert_eq!(applied(&arguments), config);
    }

    #[test]
    fn the_defaults_need_no_arguments() {
        assert!(arguments(&Config::default()).is_empty());
    }

    #[test]
    fn the_switches_off_round_trip() {
        let config = Config {
            coast: None,
            correlation: Correlation::WHITE,
            ..Config::default()
        };
        assert_eq!(applied(&arguments(&config)), config);
        let mut white = Config::default();
        set(&mut white, "correlation", "white").expect("white");
        assert_eq!(white.correlation, Correlation::WHITE);
    }

    #[test]
    fn an_unknown_name_or_a_word_where_a_number_belongs_is_refused() {
        let mut config = Config::default();
        assert!(set(&mut config, "gravty", "9.8").is_err());
        assert!(set(&mut config, "correlation.gnss_pos", "1").is_err());
        assert!(set(&mut config, "recovery.course", "soon").is_err());
        assert!(set(&mut config, "coast", "some").is_err());
        assert_eq!(config, Config::default(), "a refusal wrote nothing");
    }
}
