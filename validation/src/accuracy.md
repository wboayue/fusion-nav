# Accuracy against truth

**How close does the estimate get when the true answer is known?** On the baseline simulated
flight the horizontal position is off by {{score mission pos_h}} m RMS and never more than
{{score mission pos_h_max}} m, the attitude by {{score mission tilt}}° of tilt and
{{score mission yaw}}° of heading. Every figure here is simulation: no real log in the corpus
carries truth, so this page says how the filter does against the sensors the simulator models,
not against a real vehicle's.

{{stamp}}

## Where the truth comes from

`examples/simulate.rs` flies each scenario from an analytic trajectory, so the position,
velocity, attitude and biases it writes beside the sensor log are exact. Its sensor noise is
deliberately not the filter's `Config`: scoring the filter against its own assumptions would
measure nothing. Each scenario exists because it covers something the others do not, and says
so in the `covers` column of the table in that file.

Every run below is the one `data/bench.sh` gates in CI, at the seed `data/scenarios.txt` pins,
and every number is copied off the `score` line `examples/replay.rs` printed for it. The error is
`truth ⊖ estimate` in the error state of `EQUATIONS.md` (2), computed in one place
(`error_state`); attitude is split into tilt, about the two horizontal axes, and heading, about
down, the way `Validity` states its claims.

## Every scenario

RMS error over every scored epoch, except `pos_h_max`, the single worst horizontal error.
Positions in metres, velocity in m/s, attitude in degrees.

{{table score static,mission,harsh_imu,gnss_outage,moving_start,baro_drift,gnss_latency,correlated,mag_disturbance,logging_dropout,flight pos_h,pos_h_max,pos_v,vel,tilt,yaw}}

`static` is 60 s on the ground; `mission` is a 180 s circuit with turns and climbs after 5 s of
standing still, and every scenario from `harsh_imu` to `logging_dropout` is that circuit with
one thing changed. `flight` is 14 s at 50 Hz, the file CI replays.

Three rows lose to the baseline, and each loss is its scenario's point:

- `gnss_outage` drifts to {{score gnss_outage pos_h_max}} m during 20 s without GNSS. That is
  dead reckoning, and [robustness](robustness.md) shows the error staying inside the band the
  filter claimed while it grew.
- `gnss_latency` fuses fixes 150 ms stale as if they were current, and pays
  {{score gnss_latency pos_h}} m RMS for it against the baseline's {{score mission pos_h}}. The
  filter has no measurement-delay model yet (#52), and the [honesty page](honesty.md) shows it
  is also overconfident here.
- `correlated` makes every aiding error slower than the filter assumes. Its heading is off by
  {{score correlated yaw}}° RMS against the baseline's {{score mission yaw}}°, and its position covariance is
  overconfident (#51).

## The baseline, over time

Each figure is drawn by `tools/replay_report.py` from the run's `<out>.error.csv`, which the harness writes from the same error vector the table reads. The
grey band is ±3σ of the filter's own covariance on the same axis, so an error inside the band is
one the filter admitted to.

{{figure mission error_position}}

{{figure mission error_velocity}}

{{figure mission error_attitude}}

The band is wide next to the error, and that is a finding rather than a margin to celebrate:
the simulator's IMU is one to two orders quieter than `ImuNoise::default()`, on purpose, so the
filter is pessimistic about every flight here. The [honesty page](honesty.md) measures by how
much.

## A start in motion

`moving_start` is airborne and turning from the first sample, so there is no still window to
level from. The filter starts coarse, adopts its first GNSS fix and heading whole, and converges
from there: {{score moving_start tilt}}° of tilt RMS and {{score moving_start yaw}}° of
heading over the whole flight, including the start.

{{figure moving_start error_attitude}}

## What this page cannot say

Simulation tests the filter against the errors the simulator models, and nothing else: no
vibration spectrum, no multipath, no clock drift between sensors. Real truth arrives with the
INSANE dataset (#9), which will add measured scalars here, and never its trajectory plots, whose
licence keeps them out of the repository.
