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
`pin` subtracts them. A stack is painted from the call into `Machine::execute`, and `pin`
subtracts `execute`'s frame and the out-of-line arm's a call goes through (the trace's `arm`
column), read off the ELF into `<build>.frames` by `onboard/build.sh`. A nop does not show that
frame: painting sees only the bytes written, and a call inlined into `execute` writes few. It is the one place the
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

PROTOCOL = 2
BATCH = 256 * 1024
RESULT = struct.Struct("<IIHBx")
FRAME_OVERHEAD = 2 + 2 + 8
NOPS = 256
# A frame for `Record::Nop` (tag 0), outcome "nothing" (0); its digest is the machine's, which
# the host does not know here, so calibration reads no digest flag.
NOP_FRAME = struct.pack("<HBHQ", 1, 0, 0, 0)

# `onboard::flags`, which `PROTOCOL` guards: the firmware's info line names its protocol.
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
        if fields.get("batch") != str(BATCH):
            die(f"the firmware takes batches of {fields.get('batch')} bytes, this tool {BATCH}")
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
        writer.writerow(["index", "call", "arm", "outcome", "height", "label", "cycles", "stack",
                         "flags"])
        for row in nops:
            writer.writerow([-1, "nop", "inline", "nothing", "", "", row[0], row[1], row[3]])
        for call, (cycles, stack, _, flags) in zip(calls, measured):
            writer.writerow([call["index"], call["call"], call["arm"], call["outcome"],
                             call["height"], call["label"], cycles, stack, flags])
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
    """`{key: int}` for the `key=value` tokens of `text`, its `#` lines aside."""
    lines = (line for line in text.splitlines() if not line.startswith("#"))
    return {k: int(v) for k, v in (t.split("=", 1) for line in lines for t in line.split() if "=" in t)}


def slug(text):
    return re.sub(r"[^a-z0-9]+", "_", text.lower()).strip("_")


def summarize(rows, frames):
    """Keys for one run's per-call rows: per call, per call and outcome, per label.

    Cycles are net of the least a `nop` took, and stack of `execute`'s frame and the arm's a call
    goes through (`frames`, by function, from `<build>.frames`); a call inlined into `execute` has
    no stack of its own, and none is published. Under LTO `execute`'s frame holds whatever entry
    point it inlined, so `stack_raw` keeps it.
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
        arm, raw = r["arm"], int(r["stack"])
        if arm == "inline":
            net = None
        elif arm not in frames:
            raise ValueError(f"{r['call']} goes through `{arm}`, which the build's frames lack")
        else:
            net = raw - frames["execute"] - (0 if arm == "execute" else frames[arm])
            if net < 0:
                raise ValueError(f"{r['call']}: {raw} bytes painted, under the frames above it")
        cycles = int(r["cycles"]) - overhead
        names = [r["call"], f"{r['call']}.{outcome}"]
        if r["label"]:
            names.append(f"label.{slug(r['label'])}")
        for name in names:
            groups.setdefault(name, []).append((cycles, net, int(r["flags"]), raw))
    for name, members in groups.items():
        cycles = [c for c, _, _, _ in members]
        keys[f"{name}.n"] = len(members)
        keys[f"{name}.min"] = min(cycles)
        keys[f"{name}.mean"] = round(sum(cycles) / len(cycles))
        keys[f"{name}.max"] = max(cycles)
        stacks = [s for _, s, _, _ in members if s is not None]
        if stacks:
            keys[f"{name}.stack"] = max(stacks)
        keys[f"{name}.stack_raw"] = max(raw for _, _, _, raw in members)
        keys[f"{name}.denormal"] = sum(1 for _, _, f, _ in members if f & DENORMAL)
    return keys


def bound(runs):
    """Every run taken together, per key present in any: the largest `max`, `stack` and
    `stack_raw`, the least `min` and `nop_cycles`, the summed `n` and `denormal`, and `mean`
    weighted by `n`."""
    out, weighted = {}, {}
    for keys in runs:
        for key, value in keys.items():
            if key.endswith((".max", ".stack", ".stack_raw")) or key == "dispatch_stack":
                out[key] = max(out.get(key, value), value)
            elif key.endswith(".min") or key == "nop_cycles":
                out[key] = min(out.get(key, value), value)
            elif key.endswith((".n", ".denormal")):
                out[key] = out.get(key, 0) + value
            elif key.endswith(".mean"):
                name = key[: -len(".mean")]
                total, count = weighted.get(name, (0, 0))
                weighted[name] = (total + value * keys[f"{name}.n"], count + keys[f"{name}.n"])
    for name, (total, count) in weighted.items():
        out[f"{name}.mean"] = round(total / count)
    raw = [v for k, v in out.items() if k.endswith(".stack_raw")]
    if raw:
        out["stack_raw_max"] = max(raw)
    return out


def microseconds(keys, sysclk):
    """`<name>.max_us` and `<name>.mean_us` beside each cycle count, at `sysclk` Hz, to 0.1 µs."""
    out = dict(keys)
    for key, value in keys.items():
        for kind in (".max", ".mean"):
            if key.endswith(kind):
                out[f"{key}_us"] = f"{value / sysclk * 1e6:.1f}"
    return out


def pin(dirs):
    """data/onboard.txt's lines for result directories named `<build>-<warm|cold>`.

    Besides each run and the bound over a build's runs, `common` is the bound over the runs
    every directory holds, so builds timed on fewer traces are compared on the same ones.
    """
    lines = []
    dirs = list(map(Path, dirs))
    names = [{f.stem for f in d.glob("*.csv")} - {"fmodf"} for d in dirs]
    shared = set.intersection(*names) if names else set()
    for d in dirs:
        info = (d / "info.txt").read_text().split()
        meta = dict(w.split("=", 1) for w in info[1:] if "=" in w)
        mode = "cold" if meta.get("cache") == "cold" else "warm"
        tag = f"{meta['build']}/{mode}"
        runs = {}
        for f in sorted(d.glob("*.csv")):
            if f.name == "fmodf.csv":
                continue
            frames = pairs((d / "frames.txt").read_text())
            runs[f.stem] = summarize(list(csv.DictReader(open(f, newline=""))), frames)
        keep = ("commit", "rustc", "opt", "lto", "cpu", "fpu", "sysclk", "cache", "fz")
        lines.append(f"{tag} " + " ".join(f"{k}={meta[k]}" for k in keep if k in meta))
        sweep = d / "fmodf.csv"
        if sweep.exists():
            rows = list(csv.DictReader(open(sweep, newline="")))
            cycles = [int(r["cycles"]) for r in rows]
            # The angles the filter forms itself are inside (−4π, 4π), exponents up to 3.
            own = [int(r["cycles"]) for r in rows if int(r["exponent"]) <= 3]
            lines.append(f"{tag}/fmodf min={min(cycles)} max={max(cycles)} filter_range={max(own)}")
        sysclk = int(meta["sysclk"])
        for name, keys in runs.items():
            keys = microseconds(keys, sysclk)
            lines.append(f"{tag}/{name} " + " ".join(f"{k}={v}" for k, v in sorted(keys.items())))
        if runs:
            keys = microseconds(bound(runs.values()), sysclk)
            lines.append(f"{tag}/bound " + " ".join(f"{k}={v}" for k, v in sorted(keys.items())))
            keys = microseconds(bound(v for n, v in runs.items() if n in shared), sysclk)
            keys["runs"] = ",".join(sorted(shared))
            lines.append(f"{tag}/common " + " ".join(f"{k}={v}" for k, v in sorted(keys.items())))
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

    # The overhead is the least nop, so a slow nop raises nothing; the frames are the build's, not
    # a nop's stack, which shows only what it wrote. The worst predict is in the middle, so a
    # reader that takes the first or the last row misses it.
    ok = OUTCOME_MATCH | DIGEST_MATCH
    rows = [
        {"call": "nop", "arm": "inline", "outcome": "nothing", "height": "", "label": "", "cycles": "20", "stack": "96", "flags": "1"},
        {"call": "nop", "arm": "inline", "outcome": "nothing", "height": "", "label": "", "cycles": "31", "stack": "104", "flags": "1"},
        {"call": "predict", "arm": "execute", "outcome": "propagated", "height": "", "label": "", "cycles": "1020", "stack": "5000", "flags": str(ok)},
        {"call": "predict", "arm": "execute", "outcome": "coasted", "height": "", "label": "a coast", "cycles": "64020", "stack": "7000", "flags": str(ok | DENORMAL)},
        {"call": "predict", "arm": "execute", "outcome": "propagated", "height": "", "label": "", "cycles": "1220", "stack": "5004", "flags": str(ok)},
        {"call": "fuse_gnss_position", "arm": "execute", "outcome": "accepted", "height": "rejected", "label": "", "cycles": "3020", "stack": "9000", "flags": str(ok)},
        {"call": "initialize_from", "arm": "seed", "outcome": "seeded", "height": "", "label": "", "cycles": "9020", "stack": "4000", "flags": str(ok)},
        {"call": "set_magnetic_declination", "arm": "inline", "outcome": "true", "height": "", "label": "", "cycles": "60", "stack": "36", "flags": str(ok)},
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
    # Inlined into `execute`, it painted less than `execute`'s frame: no stack is published.
    assert "set_magnetic_declination.stack" not in keys
    assert keys["set_magnetic_declination.stack_raw"] == 36
    built = pairs("execute=160 renew=5000 new_window=900 seed=1000")
    for arm, stack, why in (("rename", "4000", "lack"), ("execute", "120", "under")):
        bad = dict(rows[2], arm=arm, stack=stack)
        try:
            summarize(rows[:2] + [bad], built)
        except ValueError as error:
            assert why in str(error), error
        else:
            raise AssertionError(f"{arm} with {stack} painted was published")

    other = dict(keys, **{"predict.max": 70000, "predict.stack": 10, "predict.min": 900,
                          "predict.n": 1, "predict.mean": 900})
    worst = bound([keys, other])
    assert worst["predict.max"] == 70000 and worst["predict.stack"] == 7000 - 160
    assert worst["predict.min"] == 900 and worst["predict.n"] == 4
    # Weighted by count: three calls at their mean and one at 900, not the mean of two means.
    assert worst["predict.mean"] == round((keys["predict.mean"] * 3 + 900) / 4)
    assert worst["predict.coasted.denormal"] == 2
    # The deepest raw stack of any call, not of the first or last key: fuse_gnss_position's.
    assert worst["stack_raw_max"] == 9000
    us = microseconds({"predict.max": 112304, "predict.mean": 48081, "predict.n": 3}, 400_000_000)
    assert us["predict.max_us"] == "280.8" and us["predict.mean_us"] == "120.2"
    assert "predict.n_us" not in us
    # Two builds, one timed on a trace the other was not: `bound` is each build's own, and
    # `common` the shared trace's alone, so the slow trace the second build lacks stays out of
    # the first build's comparison line.
    import tempfile
    with tempfile.TemporaryDirectory() as tmp:
        header = "index,call,arm,outcome,height,label,cycles,stack,flags\n"
        nop = "-1,nop,inline,nothing,,,40,36,1\n"
        for build, traces in (("primary", {"a": 1040, "b": 9040}), ("shipped", {"a": 2040})):
            d = Path(tmp) / f"{build}-warm"
            d.mkdir()
            (d / "info.txt").write_text(
                f"onboard proto=1 build={build} commit=c sysclk=400000000 cache=warm fz=0\n")
            (d / "frames.txt").write_text(
                f"# {build} c\nexecute=160\nrenew=1\nnew_window=1\nseed=1\n")
            for name, cycles in traces.items():
                (d / f"{name}.csv").write_text(
                    header + nop + f"0,predict,execute,propagated,,,{cycles},5160,{ok}\n")
        out = {}
        for line in pin([Path(tmp) / "primary-warm", Path(tmp) / "shipped-warm"]):
            run, *words = line.split()
            out[run] = dict(w.split("=", 1) for w in words)
        assert out["primary/warm/bound"]["predict.max"] == "9000"
        assert out["primary/warm/common"]["predict.max"] == "1000"
        assert out["shipped/warm/common"]["predict.max"] == "2000"
        assert out["shipped/warm/common"]["predict.max_us"] == "5.0"
        assert out["primary/warm/a"]["predict.stack"] == "5000"
        assert out["primary/warm"]["sysclk"] == "400000000"
        assert out["primary/warm/common"]["runs"] == "a"
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
