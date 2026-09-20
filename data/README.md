# Replay data

`cargo run --example replay` reads a recorded flight from CSV and writes the estimate back out as
CSV — the normalized log format the [validation harness](../GOALS.md#harness-constraint) is built
on. The format itself is documented in `examples/replay.rs`.

## Synthetic log

`flight.csv` is a small synthetic log checked in so the example runs with no network. It is the
only log CI replays.

```console
$ cargo run --example replay                           # data/flight.csv -> target/replay.csv
$ cargo run --example replay -- <input.csv> <output.csv>
```

## PX4 corpus

Real flights come from [PX4 Flight Review](https://review.px4.io/). They are fetched rather than
committed: `logs/` is gitignored, and `manifest.txt` pins each log by sha256.

```console
$ data/fetch.sh                 # fetch + verify the manifest
$ data/fetch.sh --verify        # checksums only, no network
$ tools/ulog2replay.py data/logs/<log-id>.ulg -o data/logs/<log-id>.csv --reference
$ cargo run --example replay -- data/logs/<log-id>.csv
```

`--reference` writes EKF2's own solution and innovation test ratios to a second file, for a
side-by-side diff.

`data/fetch.sh --check` converts and replays the whole pinned corpus and asserts the per-log
expectations recorded beside each checksum against the `summary` line the replay example prints.
The keys are `rate=` and `window=` (the IMU rate and the samples it takes to cover
`min_duration`), `align=` and `alpha0=` (what initialization achieved, and whether it fixed a
barometric reference), `heading=` (`Validity::heading` **as initialization left it** — not as the
log ended, which would only restate `transitions=`), `resets=`, `refused=` and `invalid=` (adopted
measurements, and steps refused as too long or as not a step at all), and `epochs=`,
`transitions=` and `status=`. It needs `pyulog`, so it is a local tool rather than a CI job.

`data/fetch.sh --add <url> [name]` downloads a log once and appends a manifest line to commit; the
files stay out of the repo, the checksums do not. Each entry should cover something no other log
does.

Flight Review logs are [CC BY 4.0](https://review.px4.io/).
