//! The host's half: a filter that writes down every call made into it, and the files it
//! writes them to.

use std::fs::File;
use std::io::{self, BufWriter, Write};
use std::path::Path;

use fusion_nav::prelude::*;

use crate::Frame;
use crate::machine::{Machine, Outcome, Returned};
use crate::record::{MAX_RECORD, Record};

/// Where a [`Recorder`] puts each call: its encoded record, and what the host got from it.
pub trait Sink {
    fn call(&mut self, bytes: &[u8], record: &Record, outcome: Outcome, digest: u64, label: &str);
}

/// The filter, behind every mutating call it offers.
///
/// Each call is encoded, decoded and made through [`Machine::execute`], with or without a
/// sink, so the host runs exactly the record the board will. Reads go through `Deref`; there
/// is no `DerefMut`, so a mutation that bypasses the trace does not compile.
pub struct Recorder<S> {
    machine: Machine,
    sink: Option<S>,
    label: &'static str,
}

impl<S: Sink> core::ops::Deref for Recorder<S> {
    type Target = Eskf;

    fn deref(&self) -> &Eskf {
        &self.machine.filter
    }
}

impl<S: Sink> Recorder<S> {
    /// `Eskf::new(config)`, recorded as the trace's first call.
    pub fn new(config: Config, sink: Option<S>) -> Result<Self, ConfigError> {
        let mut recorder = Self {
            machine: Machine::default(),
            sink,
            label: "",
        };
        match recorder.call(Record::New(config)) {
            Returned::New(result) => result.map(|()| recorder),
            other => unreachable!("Eskf::new returned {other:?}"),
        }
    }

    /// Name the calls that follow, until the next label: the per-call CSV carries it, so a
    /// trace built to reach a path can say which calls reach it.
    pub fn label(&mut self, label: &'static str) {
        self.label = label;
    }

    /// The window the last [`initialize_on`](Self::initialize_on) folded.
    pub fn window(&self) -> &StaticWindow {
        &self.machine.window
    }

    /// The sink, to finish it.
    pub fn into_sink(self) -> Option<S> {
        self.sink
    }

    fn call(&mut self, record: Record) -> Returned {
        let mut buffer = [0u8; MAX_RECORD];
        let length = record
            .encode(&mut buffer)
            .unwrap_or_else(|| unreachable!("{} exceeds MAX_RECORD", record.name()));
        let bytes = &buffer[..length];
        let decoded = Record::decode(bytes)
            .unwrap_or_else(|| unreachable!("{} does not decode", record.name()));
        let returned = self.machine.execute(&decoded);
        if let Some(sink) = &mut self.sink {
            let outcome = Outcome::of(&returned);
            let digest = self.machine.digest(outcome);
            sink.call(bytes, &decoded, outcome, digest, self.label);
        }
        returned
    }

    /// Fold `samples` into a new window and start on it: `StaticWindow::push` per sample, then
    /// `Eskf::initialize`, each recorded, so the board times the pushes too.
    pub fn initialize_on(
        &mut self,
        samples: impl IntoIterator<Item = StaticSample>,
    ) -> Result<Alignment, InitError> {
        self.call(Record::WindowNew);
        for sample in samples {
            match self.call(Record::WindowPush(sample)) {
                Returned::Push(Ok(())) => {}
                Returned::Push(Err(refusal)) => return Err(refusal.into()),
                other => unreachable!("StaticWindow::push returned {other:?}"),
            }
        }
        self.start(Record::Initialize)
    }

    pub fn initialize_coarse(&mut self, imu: ImuSample) -> Result<Alignment, InitError> {
        self.start(Record::InitializeCoarse(imu))
    }

    pub fn initialize_from(
        &mut self,
        state: State,
        covariance: Covariance,
        time: Timestamp,
    ) -> Result<Alignment, InitError> {
        self.start(Record::InitializeFrom(state, covariance, time))
    }

    fn start(&mut self, record: Record) -> Result<Alignment, InitError> {
        match self.call(record) {
            Returned::Start(result) => result,
            other => unreachable!("{} returned {other:?}", record.name()),
        }
    }

    fn boolean(&mut self, record: Record) -> bool {
        match self.call(record) {
            Returned::Bool(b) => b,
            other => unreachable!("{} returned {other:?}", record.name()),
        }
    }

    fn fusion(&mut self, record: Record) -> Fusion {
        match self.call(record) {
            Returned::Fusion(f) => f,
            other => unreachable!("{} returned {other:?}", record.name()),
        }
    }

    fn gnss(&mut self, record: Record) -> GnssFusion {
        match self.call(record) {
            Returned::Gnss(f) => f,
            other => unreachable!("{} returned {other:?}", record.name()),
        }
    }

    pub fn set_origin(&mut self, origin: Geodetic) -> bool {
        self.boolean(Record::SetOrigin(origin))
    }

    pub fn set_magnetic_declination(&mut self, declination: Radians) -> bool {
        self.boolean(Record::SetMagneticDeclination(declination))
    }

    pub fn set_baro_reference(&mut self, reference: Altitude, noise: AltitudeNoise) -> bool {
        self.boolean(Record::SetBaroReference(reference, noise))
    }

    pub fn reset_position_to(
        &mut self,
        position: Position<Ned>,
        noise: PositionNoise<Ned>,
    ) -> bool {
        self.boolean(Record::ResetPositionTo(position, noise))
    }

    pub fn reset_velocity_to(
        &mut self,
        velocity: Velocity<Ned>,
        noise: VelocityNoise<Ned>,
    ) -> bool {
        self.boolean(Record::ResetVelocityTo(velocity, noise))
    }

    pub fn predict(&mut self, imu: ImuSample) -> Propagation {
        match self.call(Record::Predict(imu)) {
            Returned::Propagation(p) => p,
            other => unreachable!("predict returned {other:?}"),
        }
    }

    pub fn fuse_gnss_position(
        &mut self,
        time: Timestamp,
        position: Position<Ned>,
        noise: PositionNoise<Ned>,
        antenna: Position<Body>,
    ) -> GnssFusion {
        self.gnss(Record::FuseGnssPosition(time, position, noise, antenna))
    }

    pub fn fuse_gnss_geodetic(
        &mut self,
        time: Timestamp,
        fix: Geodetic,
        noise: PositionNoise<Ned>,
        antenna: Position<Body>,
    ) -> GnssFusion {
        self.gnss(Record::FuseGnssGeodetic(time, fix, noise, antenna))
    }

    pub fn fuse_gnss_velocity(
        &mut self,
        time: Timestamp,
        velocity: Velocity<Ned>,
        noise: VelocityNoise<Ned>,
        antenna: Position<Body>,
    ) -> Fusion {
        self.fusion(Record::FuseGnssVelocity(time, velocity, noise, antenna))
    }

    pub fn fuse_baro_altitude(
        &mut self,
        time: Timestamp,
        altitude: Altitude,
        noise: AltitudeNoise,
    ) -> Fusion {
        self.fusion(Record::FuseBaroAltitude(time, altitude, noise))
    }

    pub fn fuse_mag_heading(
        &mut self,
        time: Timestamp,
        field: MagField<Body>,
        noise: HeadingNoise,
    ) -> Fusion {
        self.fusion(Record::FuseMagHeading(time, field, noise))
    }

    pub fn fuse_gnss_heading(
        &mut self,
        time: Timestamp,
        heading: Radians,
        noise: HeadingNoise,
    ) -> Fusion {
        self.fusion(Record::FuseGnssHeading(time, heading, noise))
    }

    pub fn fuse_course(&mut self, time: Timestamp, sideslip: HeadingNoise) -> Fusion {
        self.fusion(Record::FuseCourse(time, sideslip))
    }

    pub fn fuse_stationary(&mut self, time: Timestamp, noise: VelocityNoise<Ned>) -> Fusion {
        self.fusion(Record::FuseStationary(time, noise))
    }

    /// `Eskf::predicted_validity`, recorded: it reads, but it is an entry point with a cost of
    /// its own, up to 64 runs of (22).
    pub fn predicted_validity(&mut self) -> Validity {
        match self.call(Record::PredictedValidity) {
            Returned::Validity(v) => v,
            other => unreachable!("predicted_validity returned {other:?}"),
        }
    }
}

/// A trace on disk, `<path>`, and its per-call CSV beside it, `<path>.csv`.
///
/// Each frame is the record's length (`u16`), the record, the host's outcome (`u16`) and its
/// digest (`u64`), little-endian. The CSV names each call: its index, the call, the outcome and
/// the label, which the board's figures are joined back to by index.
pub struct TraceFile {
    trace: BufWriter<File>,
    calls: BufWriter<File>,
    index: u64,
    error: Option<io::Error>,
}

impl TraceFile {
    pub fn create(path: &Path) -> io::Result<Self> {
        let mut csv = path.as_os_str().to_owned();
        csv.push(".csv");
        let mut calls = BufWriter::new(File::create(csv)?);
        writeln!(calls, "index,call,outcome,height,label")?;
        Ok(Self {
            trace: BufWriter::new(File::create(path)?),
            calls,
            index: 0,
            error: None,
        })
    }

    /// Flush both files, reporting the first write that failed.
    pub fn finish(mut self) -> io::Result<u64> {
        if let Some(error) = self.error.take() {
            return Err(error);
        }
        self.trace.flush()?;
        self.calls.flush()?;
        Ok(self.index)
    }

    fn write(
        &mut self,
        bytes: &[u8],
        record: &Record,
        outcome: Outcome,
        digest: u64,
        label: &str,
    ) -> io::Result<()> {
        Frame {
            record: bytes,
            outcome,
            digest,
        }
        .write(&mut self.trace)?;
        let (kind, height) = outcome.names();
        writeln!(
            self.calls,
            "{},{},{},{},{}",
            self.index,
            record.name(),
            kind,
            height.unwrap_or(""),
            label
        )
    }
}

/// A trace in memory, frames only.
impl Sink for Vec<u8> {
    fn call(&mut self, bytes: &[u8], _: &Record, outcome: Outcome, digest: u64, _: &str) {
        let frame = Frame {
            record: bytes,
            outcome,
            digest,
        };
        // Writing to a `Vec` cannot fail.
        let _ = frame.write(self);
    }
}

impl Sink for TraceFile {
    fn call(&mut self, bytes: &[u8], record: &Record, outcome: Outcome, digest: u64, label: &str) {
        if self.error.is_none()
            && let Err(error) = self.write(bytes, record, outcome, digest, label)
        {
            self.error = Some(error);
        }
        self.index += 1;
    }
}
