#!/usr/bin/env python3
"""Time a trace on the board, and turn what it measured into data/onboard.txt's pins (#41).

    tools/onboard.py info                              the firmware's build line
    tools/onboard.py run --out DIR [--cold] TRACE ...  every call timed; DIR/<trace>.csv per trace
    tools/onboard.py fmodf --out DIR                   wrap_pi's fmodf across the exponent gap
    tools/onboard.py pin DIR ...                       data/onboard.txt's lines for those runs
    tools/onboard.py --self-test                       run the fixtures

A trace is `replay --trace` or `onboard/examples/paths.rs` output: frames of (`u16` length,
record, `u16` outcome, `u64` digest), with `<trace>.csv` beside it naming each call. `run` sends
the frames to the firmware (`onboard/src/firmware.rs`) in batches of whole frames, and writes one
row per call: the CSV's columns, then the board's cycles, stack bytes and flags. It refuses a run
in which any call's outcome or digest differs from the host's, naming the first, since a figure
for a call the board made differently is a figure for another run.

Every run is preceded by calls that do nothing (`Record::Nop`), timed the same way: their least
cycles are what the measurement itself costs, the two cycle-counter reads and the dispatch, and
`pin` subtracts them. A stack is painted from the call into `Machine::execute`, whose frame
`data/onboard.sh` reads off the ELF into `dispatch.txt` (a nop does not show it: LLVM sets up
the frame only on the paths that need it), and `pin` subtracts that. It is the one place the
board's raw figures become published ones (AGENTS.md, "one statistic, one implementation").

Standard library only, as `tools/anees.py` is: the port is a CDC device opened raw through
`termios`, so the tool has nothing to pin.
"""

import csv
import glob
import os
import re
import select
import struct
import sys
import termios
import tty
from pathlib import Path

PROTOCOL = 1
BATCH = 256 * 1024
RESULT = struct.Struct("<IIHBx")
FRAME_OVERHEAD = 2 + 2 + 8
NOPS = 256
# A frame for `Record::Nop` (tag 0), outcome "nothing" (0); its digest is the machine's, which
# the host does not know here, so calibration reads no digest flag.
NOP_FRAME = struct.pack("<HBHQ", 1, 0, 0, 0)

# The calls `Machine::execute` makes through an out-of-line arm, whose frame sits between the
# dispatcher's and the entry point's (`onboard/src/machine.rs`, `renew`, `new_window`, `seed`).
ARMS = {"new": "renew", "window_new": "new_window", "initialize_from": "seed"}

OUTCOME_MATCH, DIGEST_MATCH, DENORMAL, OVERFLOW, UNDECODED = 1, 2, 4, 8, 16


def die(message):
    sys.exit(f"onboard: {message}")


# --- the trace -------------------------------------------------------------------------------


def frames(trace):
    """The byte offsets at which each frame of `trace` ends, in order."""
    ends, at = [], 0
    while at < len(trace):
        if at + 2 > len(trace):
            raise ValueError(f"the trace ends inside a frame's length at byte {at}")
        (length,) = struct.unpack_from("<H", trace, at)
        at += 2 + length + 2 + 8
        if at > len(trace):
            raise ValueError("the trace ends inside a frame")
        ends.append(at)
    return ends


def batches(trace, ends, limit=BATCH):
    """`(start, end, count)` spans of whole frames, each at most `limit` bytes."""
    spans, start, count, previous = [], 0, 0, 0
    for end in ends:
        if end - previous > limit:
            raise ValueError(f"a frame of {end - previous} bytes exceeds a batch")
        if end - start > limit:
            spans.append((start, previous, count))
            start, count = previous, 0
        count += 1
        previous = end
    if count:
        spans.append((start, previous, count))
    return spans


def results(payload, count):
    """The board's per-call results: `(cycles, stack, outcome, flags)`."""
    if len(payload) != count * RESULT.size:
        raise ValueError(f"{len(payload)} result bytes for {count} calls")
    return [RESULT.unpack_from(payload, i * RESULT.size) for i in range(count)]


# --- the port --------------------------------------------------------------------------------


class Board:
    def __init__(self, path):
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
        tty.setraw(self.fd)
        attrs = termios.tcgetattr(self.fd)
        attrs[2] |= termios.CLOCAL
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        termios.tcflush(self.fd, termios.TCIOFLUSH)

    def write(self, data):
        view = memoryview(data)
        while view:
            n = os.write(self.fd, view)
            view = view[n:]

    def read(self, n, timeout=60.0):
        out = bytearray()
        while len(out) < n:
            ready, _, _ = select.select([self.fd], [], [], timeout)
            if not ready:
                raise TimeoutError(f"the board sent {len(out)} of {n} bytes")
            out += os.read(self.fd, n - len(out))
        return bytes(out)

    def line(self):
        out = bytearray()
        while not out.endswith(b"\n"):
            out += self.read(1)
        return out.decode().strip()

    def info(self):
        self.write(b"i")
        line = self.line()
        fields = dict(word.split("=", 1) for word in line.split()[1:] if "=" in word)
        if not line.startswith("onboard ") or fields.get("proto") != str(PROTOCOL):
            die(f"not this protocol's firmware: {line!r}")
        return line, fields

    def batch(self, data):
        self.write(b"b" + struct.pack("<I", len(data)) + data)
        if self.read(1) != b"R":
            die("the board did not answer a batch")
        (count,) = struct.unpack("<I", self.read(4))
        return count, self.read(count * RESULT.size, timeout=600.0)


def port(path):
    if path:
        return path
    candidates = sorted(glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/ttyACM*"))
    if len(candidates) != 1:
        die(f"name the port with --port: found {candidates or 'none'}")
    return candidates[0]


# --- commands --------------------------------------------------------------------------------


def run(board, trace_path, out_dir, cold):
    trace = Path(trace_path).read_bytes()
    calls = list(csv.DictReader(open(f"{trace_path}.csv", newline="")))
    ends = frames(trace)
    if len(ends) != len(calls):
        die(f"{trace_path}: {len(ends)} frames against {len(calls)} rows in its CSV")
    board.write(b"c" if cold else b"w")
    board.write(b"n")
    count, payload = board.batch(NOP_FRAME * NOPS)
    if count != NOPS:
        die(f"the board ran {count} of {NOPS} calibration calls")
    nops = results(payload, count)

    measured = []
    for start, end, count in batches(trace, ends):
        got, payload = board.batch(trace[start:end])
        if got != count:
            die(f"{trace_path}: the board ran {got} of {count} calls in a batch")
        measured += results(payload, count)
        print(f"\r{trace_path}: {len(measured)}/{len(ends)}", end="", file=sys.stderr)
    print(file=sys.stderr)

    name = Path(trace_path).name.removesuffix(".trace")
    out = Path(out_dir) / f"{name}.csv"
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["index", "call", "outcome", "height", "label", "cycles", "stack", "flags"])
        for row in nops:
            writer.writerow([-1, "nop", "nothing", "", "", row[0], row[1], row[3]])
        for call, (cycles, stack, _, flags) in zip(calls, measured):
            writer.writerow([call["index"], call["call"], call["outcome"], call["height"],
                             call["label"], cycles, stack, flags])
    for call, (_, _, _, flags) in zip(calls, measured):
        problem = refusal(flags)
        if problem:
            die(f"{trace_path}: call {call['index']}, {call['call']}: {problem}; {out} holds the rest")
    print(f"{out}: {len(measured)} calls")


def refusal(flags):
    """Why a call's figures cannot be published, or `None`."""
    if flags & UNDECODED:
        return "the board could not decode the record"
    if not flags & OUTCOME_MATCH:
        return "the board's outcome differs from the host's"
    if not flags & DIGEST_MATCH:
        return "the board's state differs from the host's after it"
    if flags & OVERFLOW:
        return "the call ran off the painted stack"
    return None


def fmodf(board, out_dir):
    board.write(b"f")
    if board.read(1) != b"F":
        die("the board did not answer the sweep")
    (count,) = struct.unpack("<I", board.read(4))
    rows = [struct.unpack("<iI", board.read(8)) for _ in range(count)]
    out = Path(out_dir) / "fmodf.csv"
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["exponent", "cycles"])
        writer.writerows(rows)
    print(f"{out}: {count} exponents")


# --- statistics ------------------------------------------------------------------------------


def pairs(text):
    """`{key: int}` for the `key=value` tokens of `text`."""
    return {k: int(v) for k, v in (t.split("=", 1) for t in text.split() if "=" in t)}


def slug(text):
    return re.sub(r"[^a-z0-9]+", "_", text.lower()).strip("_")


def summarize(rows, frames):
    """Keys for one run's per-call rows: per call, per call and outcome, per label.

    Cycles are net of the least a `nop` took, and stack of the dispatcher's frame and the arm's
    a call goes through (`frames`, by function, from `dispatch.txt`). Under LTO the dispatcher's
    frame holds whatever entry point it inlined, so `stack_raw` keeps it.
    """
    nops = [r for r in rows if r["call"] == "nop"]
    if not nops:
        raise ValueError("no calibration rows")
    overhead = min(int(r["cycles"]) for r in nops)
    keys = {"nop_cycles": overhead, "dispatch_stack": frames["execute"]}
    groups = {}
    for r in rows:
        if r["call"] == "nop":
            continue
        outcome = r["outcome"] + (f"_{r['height']}" if r["height"] else "")
        beneath = frames["execute"] + (frames[ARMS[r["call"]]] if r["call"] in ARMS else 0)
        cycles, raw = int(r["cycles"]) - overhead, int(r["stack"])
        names = [r["call"], f"{r['call']}.{outcome}"]
        if r["label"]:
            names.append(f"label.{slug(r['label'])}")
        for name in names:
            groups.setdefault(name, []).append((cycles, raw - beneath, int(r["flags"]), raw))
    for name, members in groups.items():
        cycles = [c for c, _, _, _ in members]
        keys[f"{name}.n"] = len(members)
        keys[f"{name}.min"] = min(cycles)
        keys[f"{name}.mean"] = round(sum(cycles) / len(cycles))
        keys[f"{name}.max"] = max(cycles)
        keys[f"{name}.stack"] = max(s for _, s, _, _ in members)
        keys[f"{name}.stack_raw"] = max(raw for _, _, _, raw in members)
        keys[f"{name}.denormal"] = sum(1 for _, _, f, _ in members if f & DENORMAL)
    return keys


def bound(runs):
    """The largest `max`, `stack` and `stack_raw` over runs, per key present in any run."""
    out = {}
    for keys in runs:
        for key, value in keys.items():
            if key.endswith((".max", ".stack", ".stack_raw")):
                out[key] = max(out.get(key, value), value)
    return out


def pin(dirs):
    """data/onboard.txt's lines for result directories named `<build>-<warm|cold>`."""
    lines = []
    for d in map(Path, dirs):
        info = (d / "info.txt").read_text().split()
        meta = dict(w.split("=", 1) for w in info[1:] if "=" in w)
        mode = "cold" if meta.get("cache") == "cold" else "warm"
        tag = f"{meta['build']}/{mode}"
        runs = {}
        for f in sorted(d.glob("*.csv")):
            if f.name == "fmodf.csv":
                continue
            frames = pairs((d / "dispatch.txt").read_text())
            runs[f.stem] = summarize(list(csv.DictReader(open(f, newline=""))), frames)
        keep = ("commit", "rustc", "opt", "lto", "cpu", "fpu", "sysclk", "cache", "fz")
        lines.append(f"{tag} " + " ".join(f"{k}={meta[k]}" for k in keep if k in meta))
        sweep = d / "fmodf.csv"
        if sweep.exists():
            rows = list(csv.DictReader(open(sweep, newline="")))
            cycles = [int(r["cycles"]) for r in rows]
            lines.append(f"{tag}/fmodf min={min(cycles)} max={max(cycles)}")
        for name, keys in runs.items():
            lines.append(f"{tag}/{name} " + " ".join(f"{k}={v}" for k, v in sorted(keys.items())))
        if runs:
            lines.append(f"{tag}/bound " + " ".join(
                f"{k}={v}" for k, v in sorted(bound(runs.values()).items())))
    return lines


# --- fixtures --------------------------------------------------------------------------------


def frame(record, outcome=0, digest=0):
    return struct.pack("<H", len(record)) + record + struct.pack("<HQ", outcome, digest)


def self_test():
    # Three frames of different lengths: the ends are where each closes, and a batch limit
    # that fits two puts the third in a batch of its own rather than splitting it.
    trace = frame(b"\x00") + frame(b"\x0c" + bytes(40)) + frame(b"\x01" + bytes(200))
    ends = frames(trace)
    assert ends == [13, 13 + 53, 13 + 53 + 213], ends
    assert batches(trace, ends, limit=220) == [(0, 66, 2), (66, 279, 1)]
    assert batches(trace, ends) == [(0, 279, 3)]
    for broken in (trace[:-1], trace + b"\x01"):
        try:
            frames(broken)
        except ValueError:
            pass
        else:
            raise AssertionError("a trace cut inside a frame was read")
    try:
        batches(trace, ends, limit=100)
    except ValueError:
        pass
    else:
        raise AssertionError("a frame larger than a batch was split")

    packed = RESULT.pack(1234, 5000, 0x1000, 3) + RESULT.pack(7, 8, 9, 2)
    assert results(packed, 2) == [(1234, 5000, 0x1000, 3), (7, 8, 9, 2)]
    assert refusal(OUTCOME_MATCH | DIGEST_MATCH) is None
    assert refusal(OUTCOME_MATCH | DIGEST_MATCH | DENORMAL) is None
    assert "state" in refusal(OUTCOME_MATCH)
    assert "outcome" in refusal(DIGEST_MATCH)
    assert "painted" in refusal(OUTCOME_MATCH | DIGEST_MATCH | OVERFLOW)

    # The overhead is the least nop, so a slow nop raises nothing; the frames are passed in, and
    # a nop's own stack (shallower: no prologue) is not the dispatcher's. The worst predict is in
    # the middle, so a reader that takes the first or the last row misses it.
    ok = OUTCOME_MATCH | DIGEST_MATCH
    rows = [
        {"call": "nop", "outcome": "nothing", "height": "", "label": "", "cycles": "20", "stack": "96", "flags": "1"},
        {"call": "nop", "outcome": "nothing", "height": "", "label": "", "cycles": "31", "stack": "104", "flags": "1"},
        {"call": "predict", "outcome": "propagated", "height": "", "label": "", "cycles": "1020", "stack": "5000", "flags": str(ok)},
        {"call": "predict", "outcome": "coasted", "height": "", "label": "a coast", "cycles": "64020", "stack": "7000", "flags": str(ok | DENORMAL)},
        {"call": "predict", "outcome": "propagated", "height": "", "label": "", "cycles": "1220", "stack": "5004", "flags": str(ok)},
        {"call": "fuse_gnss_position", "outcome": "accepted", "height": "rejected", "label": "", "cycles": "3020", "stack": "9000", "flags": str(ok)},
        {"call": "initialize_from", "outcome": "seeded", "height": "", "label": "", "cycles": "9020", "stack": "4000", "flags": str(ok)},
    ]
    keys = summarize(rows, pairs("execute=160 renew=5000 new_window=900 seed=1000"))
    # A seed's stack is beneath its arm's frame as well as the dispatcher's.
    assert keys["initialize_from.stack"] == 4000 - 160 - 1000
    assert keys["initialize_from.stack_raw"] == 4000
    assert keys["nop_cycles"] == 20 and keys["dispatch_stack"] == 160
    assert keys["predict.n"] == 3
    assert keys["predict.min"] == 1000 and keys["predict.max"] == 64000
    assert keys["predict.mean"] == round((1000 + 64000 + 1200) / 3)
    assert keys["predict.stack"] == 7000 - 160 and keys["predict.stack_raw"] == 7000
    assert keys["predict.propagated.max"] == 1200
    assert keys["predict.coasted.denormal"] == 1 and keys["predict.propagated.denormal"] == 0
    assert keys["label.a_coast.max"] == 64000
    assert keys["fuse_gnss_position.accepted_rejected.max"] == 3000

    other = dict(keys, **{"predict.max": 70000, "predict.stack": 10})
    worst = bound([keys, other])
    assert worst["predict.max"] == 70000 and worst["predict.stack"] == 7000 - 160
    assert "predict.mean" not in worst
    print("onboard self-test: ok")


def main(argv):
    if argv == ["--self-test"]:
        self_test()
        return
    if not argv:
        die(__doc__)
    command, rest = argv[0], argv[1:]
    options = {"--port": None, "--out": None}
    cold, positional = False, []
    it = iter(rest)
    for arg in it:
        if arg in options:
            options[arg] = next(it, None)
        elif arg == "--cold":
            cold = True
        else:
            positional.append(arg)
    if command == "pin":
        print("\n".join(pin(positional)))
        return
    board = Board(port(options["--port"]))
    if command == "info":
        print(board.info()[0])
        return
    if not options["--out"]:
        die(f"{command} wants --out DIR")
    line, _ = board.info()
    board.write(b"c" if cold else b"w")
    line, _ = board.info()
    out = Path(options["--out"])
    out.mkdir(parents=True, exist_ok=True)
    (out / "info.txt").write_text(line + "\n")
    if command == "run":
        for trace in positional:
            run(board, trace, out, cold)
    elif command == "fmodf":
        fmodf(board, out)
    else:
        die(f"unknown command {command}")


if __name__ == "__main__":
    main(sys.argv[1:])
