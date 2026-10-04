//! Cost on a board (#41): every call a run makes into the filter, written down on the host and
//! made again on the target, where each is timed and its stack painted.
//!
//! A trace is a sequence of frames, each a [`Record`] beside the outcome the host got and a
//! [`digest`](Machine::digest) of the filter after it. The board makes each call through the
//! same [`Machine::execute`] and checks both, so a figure it reports is for the run the host
//! made, along the same path, bit for bit.

#![cfg_attr(target_os = "none", no_std)]

mod machine;
mod record;
#[cfg(not(target_os = "none"))]
mod recorder;
mod wire;

pub use machine::{KINDS, Machine, Outcome, Returned};
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
