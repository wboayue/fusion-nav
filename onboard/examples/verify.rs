//! Make every call in a trace again on the host, and check each outcome and digest: what the
//! board checks, run where a mismatch is cheap to read.
//!
//! ```text
//! cargo run --release -p onboard --example verify -- target/onboard/<id>.trace ...
//! ```
//!
//! A trace that fails here is the recorder's or the codec's fault; one that passes here and
//! fails on the board is a difference between the two targets' arithmetic.

use std::process::ExitCode;

use onboard::{Frame, Machine, Outcome, Record};

fn main() -> ExitCode {
    let mut failed = false;
    for path in std::env::args().skip(1) {
        let bytes = match std::fs::read(&path) {
            Ok(bytes) => bytes,
            Err(error) => {
                eprintln!("{path}: {error}");
                failed = true;
                continue;
            }
        };
        match verify(&bytes) {
            Ok(calls) => println!("{path}: {calls} calls agree"),
            Err(error) => {
                println!("{path}: {error}");
                failed = true;
            }
        }
    }
    if failed {
        ExitCode::FAILURE
    } else {
        ExitCode::SUCCESS
    }
}

fn verify(trace: &[u8]) -> Result<u64, String> {
    let mut machine = Machine::default();
    let mut rest = trace;
    let mut index = 0u64;
    while !rest.is_empty() {
        let (frame, after) = Frame::split(rest)
            .ok_or_else(|| format!("call {index}: the trace ends inside a frame"))?;
        rest = after;
        let record = Record::decode(frame.record)
            .ok_or_else(|| format!("call {index}: the record does not decode"))?;
        let outcome = Outcome::of(&machine.execute(&record));
        if outcome != frame.outcome {
            return Err(format!(
                "call {index}, {}: outcome {:?}, recorded {:?}",
                record.name(),
                outcome.names(),
                frame.outcome.names()
            ));
        }
        if machine.digest(outcome) != frame.digest {
            return Err(format!(
                "call {index}, {}: the digest differs",
                record.name()
            ));
        }
        index += 1;
    }
    Ok(index)
}
