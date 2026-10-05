//! Cost on a board (#41): every call a run makes into the filter, written down on the host and
//! made again on the target, where each is timed and its stack painted.
//!
//! A trace is a sequence of frames, each a [`Record`] beside the outcome the host got and a
//! [`digest`](Machine::digest) of the filter after it. The board makes each call through the
//! same [`Machine::execute`] and checks both, so a figure it reports is for the run the host
//! made, along the same path, bit for bit.

#![cfg_attr(target_os = "none", no_std)]

mod machine;
mod outcome;
mod record;
#[cfg(not(target_os = "none"))]
mod recorder;
mod wire;

pub use machine::{Arm, Machine, Returned};
pub use outcome::{KINDS, Kind, Outcome};
pub use record::{MAX_RECORD, Record};
#[cfg(not(target_os = "none"))]
pub use recorder::{Recorder, Sink, TraceFile};

/// One frame of a trace: a record and what the host got from it.
#[derive(Clone, Copy, Debug)]
pub struct Frame<'a> {
    pub record: &'a [u8],
    pub outcome: Outcome,
    pub digest: u64,
}

/// The board's verdict on a call, as bits: what `tools/onboard.py` reads off each result.
pub mod flags {
    /// The outcome matches the host's.
    pub const OUTCOME: u8 = 1;
    /// The digest matches the host's.
    pub const DIGEST: u8 = 2;
    /// A denormal was an input (`FPSCR.IDC`).
    pub const DENORMAL: u8 = 4;
    /// The call ran off the painted stack.
    pub const OVERFLOW: u8 = 8;
    /// The record did not decode.
    pub const UNDECODED: u8 = 16;
}

/// The bytes a frame takes beside its record: the length, the outcome and the digest.
pub const FRAME_OVERHEAD: usize = 2 + 2 + 8;

impl<'a> Frame<'a> {
    /// The first frame of `bytes`, and what follows it; `None` if `bytes` ends inside one.
    pub fn split(bytes: &'a [u8]) -> Option<(Self, &'a [u8])> {
        let (length, rest) = bytes.split_first_chunk::<2>()?;
        let (record, rest) = rest.split_at_checked(usize::from(u16::from_le_bytes(*length)))?;
        let (outcome, rest) = rest.split_first_chunk::<2>()?;
        let (digest, rest) = rest.split_first_chunk::<8>()?;
        let frame = Frame {
            record,
            outcome: Outcome(u16::from_le_bytes(*outcome)),
            digest: u64::from_le_bytes(*digest),
        };
        Some((frame, rest))
    }

    /// Whether `machine`, having made this frame's call and got `returned`, agrees with the
    /// host: [`flags::OUTCOME`] and [`flags::DIGEST`], set where each matches. The one check, so
    /// the board and `examples/verify.rs` report the same thing.
    pub fn check(&self, machine: &Machine, returned: &Returned) -> u8 {
        let outcome = Outcome::of(returned);
        let mut verdict = 0;
        if outcome == self.outcome {
            verdict |= flags::OUTCOME;
        }
        if machine.digest(outcome) == self.digest {
            verdict |= flags::DIGEST;
        }
        verdict
    }

    /// Append the frame to `out`.
    #[cfg(not(target_os = "none"))]
    pub fn write(&self, out: &mut impl std::io::Write) -> std::io::Result<()> {
        let length = u16::try_from(self.record.len()).map_err(std::io::Error::other)?;
        out.write_all(&length.to_le_bytes())?;
        out.write_all(self.record)?;
        out.write_all(&self.outcome.0.to_le_bytes())?;
        out.write_all(&self.digest.to_le_bytes())
    }
}

#[cfg(test)]
mod tests;
