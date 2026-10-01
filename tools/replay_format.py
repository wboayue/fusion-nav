"""The replay and truth CSVs every converter writes.

One writer, so the PX4 corpus, UrbanNav and INSANE cannot drift apart in a format
`examples/replay/main.rs` reads with one parser: the column list, the number formats and the
`t_meas_s` rule live here and nowhere else. Standard library only.
"""

import hashlib

REPLAY_COLUMNS = "t_s,source,v0,v1,v2,v3,v4,v5,var0,var1,var2,t_meas_s"

# `TRUTH_COLUMNS` in examples/replay/main.rs, which refuses a truth file whose header differs.
TRUTH_COLUMNS = ("t_s,pos_n,pos_e,pos_d,vel_n,vel_e,vel_d,roll,pitch,yaw,"
                 "ba_x,ba_y,ba_z,bg_x,bg_y,bg_z")


def write_replay(out, header, rows, t0, unit, delays=None):
    """Write `rows` in the order given, each `(timestamp, source, values, variances[, late])`.

    Timestamps are integers in ticks of `unit` seconds, rebased to `t0`, so a time is
    differenced before it is scaled and a long log keeps its microseconds. `t_meas_s` is a
    row's time less its source's delay in `delays`, ticks by source name, and blank where
    there is none: when the measurement was taken, where `t_s` is when it was logged. `late`,
    ticks, is how much later than its source's usual delay this row arrived, and comes off too.
    """
    delays = delays or {}
    with open(out, "w", newline="") as handle:
        for line in header:
            handle.write(f"# {line}\n")
        handle.write(REPLAY_COLUMNS + "\n")
        for timestamp, source, values, variances, *late in rows:
            values = list(values) + [None] * (6 - len(values))
            variances = list(variances) + [None] * (3 - len(variances))
            cells = [f"{(timestamp - t0) * unit:.6f}", source]
            cells += ["" if v is None else f"{v:.6g}" for v in values + variances]
            delay = delays.get(source, 0) + (late[0] if late else 0)
            cells.append(f"{(timestamp - delay - t0) * unit:.6f}" if delay else "")
            handle.write(",".join(cells) + "\n")


def write_truth(out, header, rows):
    """Write truth rows, each `(t_s, position_ned, velocity_ned, (roll, pitch, yaw))`.

    Radians, metres, seconds on the replay's own clock. The bias columns are left blank:
    a real vehicle's reference knows its trajectory and no bias of the IMU being scored,
    and `examples/replay/main.rs` then reports `none` rather than scoring against a zero.
    """
    with open(out, "w", newline="") as handle:
        for line in header:
            handle.write(f"# {line}\n")
        handle.write(TRUTH_COLUMNS + "\n")
        for t, position, velocity, attitude in rows:
            cells = [f"{t:.6f}"]
            cells += [f"{x:.6f}" for x in tuple(position) + tuple(velocity) + tuple(attitude)]
            cells += [""] * 6
            handle.write(",".join(cells) + "\n")


def source_tag(paths):
    """The first 12 hex digits of a sha256 over the inputs' own: the pairing check's tag.

    Written into both files' `# fusion-nav` line, which `examples/replay/main.rs` compares, so a
    truth file converted from other inputs is refused rather than scored.
    """
    digest = hashlib.sha256()
    for path in paths:
        digest.update(hashlib.sha256(path.read_bytes()).digest())
    return digest.hexdigest()[:12]
