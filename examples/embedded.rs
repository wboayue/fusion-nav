//! The integration loop on a microcontroller: `no_std`, no allocator, no `println!`.
//!
//! What the desktop examples cannot show. The filter never reads a clock, so every sample's
//! timestamp is the caller's timer. Sources arrive at their own rates from their own queues. Every
//! outcome is handled where it is returned, and what the application does about
//! `Status::DeadReckoning`, or about a recovery that stepped the state, is a policy the crate
//! leaves to it.
//!
//! [`Board`] stands in for a HAL: replace [`Hal`]'s body with your drivers. Logging goes
//! through `core::fmt::Write`, or through `defmt` with the `defmt` feature, and the same
//! format strings serve both, since every outcome implements `Display` and `defmt::Format`.
//! Neither route prints an `f32` through core's float formatting, which reaches
//! `core::panicking`; see the README, "The library cannot panic".
//!
//! Compiled for `thumbv7em-none-eabihf` and `thumbv6m-none-eabi` in CI, never run: there is
//! no board. On a host it compiles to an empty `main`, so `cargo clippy --all-targets` still
//! reads every line.
//!
//! ```text
//! cargo build --example embedded --target thumbv7em-none-eabihf [--features defmt]
//! ```
//!
//! A real firmware brings its runtime crate (`cortex-m-rt` or an RTOS) for the entry point
//! and a panic handler, and with `defmt` a transport such as `defmt-rtt` and the linker
//! argument `-Tdefmt.x`, which this example links without and so cannot be decoded.

#![cfg_attr(target_os = "none", no_std, no_main)]
#![cfg_attr(not(target_os = "none"), allow(dead_code))]

use fusion_nav::prelude::*;

const IMU_HZ: u32 = 400;

/// The window is collected at 50 Hz rather than the IMU's 400: a `StaticSample` is 80 bytes,
/// so the default 2 s `Initialization::min_duration` is 64 KB at full rate, more RAM than a
/// Cortex-M0 has, and 8 KB here. Each window sample sums eight IMU samples' increments, as an
/// integrating driver would, so the window still spans the 2 s it observed. A mean over 100
/// samples of a still vehicle is not what limits the alignment.
const WINDOW_DECIMATION: u32 = 8;
const WINDOW: usize = (2 * IMU_HZ / WINDOW_DECIMATION) as usize;

/// Health is reported once a second: `diagnostics()` is for logging, not the hot path.
const REPORT_EVERY: u32 = IMU_HZ;

/// One `log!` for both routes, so the call sites read the same with and without `defmt`.
#[cfg(not(feature = "defmt"))]
macro_rules! log {
    ($board:expr, $($arg:tt)*) => {{
        // A full log buffer is not the filter's problem, and not a reason to stop flying.
        let _ = writeln!($board, $($arg)*);
    }};
}

#[cfg(feature = "defmt")]
macro_rules! log {
    ($board:expr, $($arg:tt)*) => {{
        let _ = &$board;
        defmt::info!($($arg)*)
    }};
}

/// A GNSS solution as a receiver reports it: position and velocity, each with its own
/// accuracy, since `R` belongs to the fix rather than to the configuration.
struct Fix {
    position: Geodetic,
    position_noise: PositionNoise<Ned>,
    velocity: Velocity<Ned>,
    velocity_noise: VelocityNoise<Ned>,
}

/// The board: what an interrupt or DMA queue hands the loop, and where the loop's decisions
/// go. `Write` is the log's transport.
trait Board: core::fmt::Write {
    /// The next IMU sample, if one is queued, timed when it was *taken* rather than when the
    /// loop got to it, so the loop's own jitter never reaches the step. `Timestamp` counts
    /// microseconds in 64 bits: a 32-bit timer wraps every 71.6 minutes, and the HAL extends
    /// it by counting the wraps.
    fn imu(&mut self) -> Option<ImuSample>;
    /// The next GNSS solution, at 5 Hz.
    fn gnss(&mut self) -> Option<Fix>;
    /// The next barometric altitude, at 20 Hz.
    fn baro(&mut self) -> Option<Altitude>;
    /// The next calibrated magnetometer reading, at 20 Hz.
    fn mag(&mut self) -> Option<MagField<Body>>;
    /// The estimate stepped rather than moved: whatever holds a setpoint relative to it,
    /// a position hold for one, has to re-anchor or it chases the step.
    fn state_stepped(&mut self);
    /// Stop relying on horizontal position: land, or hand control back to the pilot.
    fn failsafe(&mut self);
}

fn run(board: &mut impl Board) -> ! {
    let mut filter = Eskf::new(Config::default());
    align(&mut filter, board);
    let mut ticks: u32 = 0;
    loop {
        let Some(imu) = board.imu() else {
            continue;
        };
        match filter.predict(imu) {
            Propagation::Propagated => {}
            // The state advanced across a gap on its estimated velocity, and the covariance
            // grew to say how little that is worth. Nothing to undo; worth a line in the log,
            // since a gap is the logger or the scheduler falling behind.
            coasted @ Propagation::Coasted { .. } => log!(board, "imu {}", coasted),
            // Refused, and the state is stale by that step. After a gap with `Config::coast` off
            // (`StepTooLong`), a sample the filter could not use (`NotFinite`,
            // `InvalidInterval`) or an overflow (`StateNotFinite`) the timers advanced, so
            // `Status` already reports the aiding that much staler; a duplicated timestamp
            // moved nothing. What is left is to say so.
            refused @ (Propagation::StepTooLong { .. }
            | Propagation::InvalidStep { .. }
            | Propagation::NotFinite
            | Propagation::InvalidInterval { .. }
            | Propagation::StateNotFinite) => log!(board, "imu {}", refused),
            // Not reachable once `align` has returned, and aligning again is the answer if it
            // were: a firmware that re-initializes in flight comes through here.
            Propagation::NotInitialized => align(&mut filter, board),
        }

        let mut stepped = false;
        if let Some(fix) = board.gnss() {
            let position = filter.fuse_gnss_geodetic(fix.position, fix.position_noise);
            stepped |= report(board, "gnss position", position.horizontal);
            stepped |= report(board, "gnss height", position.height);
            let velocity = filter.fuse_gnss_velocity(fix.velocity, fix.velocity_noise);
            stepped |= report(board, "gnss velocity", velocity);
        }
        if let Some(altitude) = board.baro() {
            let outcome = filter.fuse_baro_altitude(altitude, AltitudeNoise::from_sigma(2.0));
            stepped |= report(board, "baro", outcome);
        }
        if let Some(field) = board.mag() {
            let outcome = filter.fuse_mag_heading(field, HeadingNoise::from_sigma(0.05));
            stepped |= report(board, "mag heading", outcome);
        }
        if stepped {
            board.state_stepped();
        }

        ticks = ticks.wrapping_add(1);
        if ticks.is_multiple_of(REPORT_EVERY) {
            supervise(&filter, board);
        }
    }
}

/// Collect a still window and initialize from it, until initialization succeeds. The filter's
/// clock starts at the last sample's time, which the first step is measured from.
fn align(filter: &mut Eskf, board: &mut impl Board) {
    loop {
        let mut window = [StaticSample::default(); WINDOW];
        let (mut baro, mut mag) = (None, None);
        let mut filled = 0;
        let mut summed: Option<ImuSample> = None;
        let mut seen: u32 = 0;
        while filled < WINDOW {
            // A slower sensor's last reading is held across the samples it spans; the window
            // counts distinct readings, so holding it claims nothing.
            baro = board.baro().or(baro);
            mag = board.mag().or(mag);
            let Some(imu) = board.imu() else {
                continue;
            };
            let imu = summed.map_or(imu, |sum| accumulate(sum, imu));
            seen = seen.wrapping_add(1);
            if seen.is_multiple_of(WINDOW_DECIMATION) {
                window[filled] = StaticSample {
                    imu,
                    baro,
                    mag,
                    velocity: None,
                };
                filled += 1;
                summed = None;
            } else {
                summed = Some(imu);
            }
        }
        match filter.initialize(&window) {
            // A short or moving window still starts the filter, coarse: `Status::Aligning`
            // says so until the attitude converges, and the log says why.
            Ok(alignment) => {
                log!(board, "aligned {}", alignment);
                return;
            }
            Err(error) => log!(board, "initialization failed: {}", error),
        }
    }
}

/// `earlier` and `later` as one sample: the increments and intervals summed, timed at the
/// later one's end. What an integrating driver does between reads, less the coning correction
/// it would apply for a vehicle that is turning, which a still one is not.
fn accumulate(earlier: ImuSample, later: ImuSample) -> ImuSample {
    ImuSample {
        time: later.time,
        delta_angle: DeltaAngle::from_vector(
            earlier.delta_angle.vector() + later.delta_angle.vector(),
        ),
        angle_interval: Seconds::from_secs(
            earlier.angle_interval.as_secs() + later.angle_interval.as_secs(),
        ),
        delta_velocity: DeltaVelocity::from_vector(
            earlier.delta_velocity.vector() + later.delta_velocity.vector(),
        ),
        velocity_interval: Seconds::from_secs(
            earlier.velocity_interval.as_secs() + later.velocity_interval.as_secs(),
        ),
    }
}

/// Log what an operator needs from one outcome, and say whether it stepped the state.
///
/// One rejection needs nothing: that is what the gate is for. Sustained rejection shows up
/// in `Status` and is recovered by the filter itself, by default, through the `Reset` arm
/// below. An application that owns that decision sets `Config::recovery` to
/// `Recovery::OFF` and calls `reset_position_to` from its own failsafe instead; see the
/// README, "Recovery from gate lockout".
fn report(board: &mut impl Board, source: &str, outcome: Fusion) -> bool {
    match outcome {
        Fusion::Accepted { .. } | Fusion::Rejected { .. } => {}
        // The first fix after a coarse start, the first heading, or a recovery from lockout,
        // counted in `SourceHealth::recovered`.
        Fusion::Reset => log!(board, "{} {}", source, outcome),
        // Expected until there is something to measure against: a barometer before the first
        // fix of a start in motion. `Diagnostics` counts them.
        Fusion::NotInitialized | Fusion::NoReference => {}
        // A driver or a wire, not the flight: the sensor produced something no sensor can.
        Fusion::NotFinite | Fusion::InvalidNoise | Fusion::StateInvalid => {
            log!(board, "{} {}", source, outcome)
        }
    }
    outcome.is_reset()
}

/// Once a second: the summary, and the application's policy on it.
fn supervise(filter: &Eskf, board: &mut impl Board) {
    let state = filter.state();
    log!(board, "status {} valid {}", state.status, state.validity);
    match state.status {
        Status::Healthy | Status::Degraded | Status::Aligning => {}
        // Nothing is aiding, so the error grows. `Status` says that much and no more; how
        // long this vehicle can fly on it is `validity`, the covariance against
        // `Config::accuracy`, which is the mission's own bar.
        Status::DeadReckoning => {
            if !state.validity.horizontal_position {
                board.failsafe();
            }
        }
    }
}

/// Your board support. Every queue is empty and every decision is discarded.
struct Hal;

impl core::fmt::Write for Hal {
    fn write_str(&mut self, _: &str) -> core::fmt::Result {
        Ok(())
    }
}

impl Board for Hal {
    fn imu(&mut self) -> Option<ImuSample> {
        None
    }
    fn gnss(&mut self) -> Option<Fix> {
        None
    }
    fn baro(&mut self) -> Option<Altitude> {
        None
    }
    fn mag(&mut self) -> Option<MagField<Body>> {
        None
    }
    fn state_stepped(&mut self) {}
    fn failsafe(&mut self) {}
}

/// Where `defmt` frames go: RTT, a UART, a ring buffer. This one drops them.
#[cfg(feature = "defmt")]
mod logger {
    #[defmt::global_logger]
    struct Discard;

    unsafe impl defmt::Logger for Discard {
        fn acquire() {}
        unsafe fn flush() {}
        unsafe fn release() {}
        unsafe fn write(_: &[u8]) {}
    }
}

#[cfg(target_os = "none")]
#[panic_handler]
fn panicked(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

/// The entry point a runtime crate would provide.
#[cfg(target_os = "none")]
#[unsafe(no_mangle)]
pub extern "C" fn _start() -> ! {
    run(&mut Hal)
}

#[cfg(not(target_os = "none"))]
fn main() {}
