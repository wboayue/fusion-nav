# Accuracy against truth

**How close does the estimate get when the true answer is known?** On the baseline simulated
flight, horizontal position is off by {{score mission pos_h}} m RMS and never by more than
{{score mission pos_h_max}} m. Tilt is off by {{score mission tilt}}° RMS and heading by
{{score mission yaw}}°. These are simulated flights, because no real log carries the true
trajectory, so they show how the filter handles the errors the simulator models, not every
error a real vehicle has.

{{stamp}}

## Where the truth comes from

A simulator (`examples/simulate.rs`) flies each scenario along a trajectory defined by
equations, so the true position, velocity and attitude are known exactly at every instant. It
writes the sensor readings a vehicle on that path would produce, with noise, and the filter is
run on those readings alone. The simulator's noise is deliberately different from what the
filter is tuned for; a filter scored against its own assumptions would learn nothing.

Each scenario exists to test one thing no other scenario covers:

- `static`: 60 s sitting on the ground.
- `mission`: the **baseline**. 5 s still, then a 180 s circuit with turns and climbs.
- `moving_start`: a different flight, in the air and turning from the first sample.
- The rest (`harsh_imu`, `gnss_outage`, `baro_drift`, `gnss_latency`, `correlated`,
  `mag_disturbance`, `logging_dropout`): the baseline circuit with one thing changed, each
  described below.
- `flight`: 14 s at 50 Hz, with sensors appearing and dropping out. It is the log CI replays.

## Every scenario

{{table score @scenarios pos_h,pos_h_max,pos_v,vel,tilt,yaw}}

Columns: `pos_h` is horizontal position error and `pos_v` height error, in metres. `pos_h_max`
is the worst horizontal error at any moment. `vel` is velocity error in m/s. `tilt` is the
error in roll and pitch together, and `yaw` the error in heading, both in degrees. All are RMS
over the whole flight except `pos_h_max`. `seed` names the simulated flight, so anyone can
regenerate exactly this one.

Every departure from the baseline costs something, and what it costs is what the scenario is
for. Against `mission`:

- `harsh_imu` flies a poorly mounted, vibrating IMU. Attitude suffers:
  {{score harsh_imu tilt}}° of tilt against {{score mission tilt}}°, while position barely
  moves.
- `gnss_outage` loses GNSS for 20 s, and position drifts to {{score gnss_outage pos_h_max}} m
  at worst. With nothing to correct it, the filter is integrating acceleration, and the
  [robustness page](robustness.md#gnss-outage) shows the drift staying inside the band.
- `moving_start` starts in the air, turning, with no still moment to level from. That costs
  attitude at the start; [below](#a-start-in-motion), it converges.
- `baro_drift` has a barometer whose zero drifts. Height suffers:
  {{score baro_drift pos_v}} m RMS against {{score mission pos_v}} m, and the filter reports it
  ([robustness](robustness.md#a-drifting-barometer)).
- `gnss_latency` delivers every GNSS fix 150 ms late, and the filter uses each as if it were
  current. Position suffers most: {{score gnss_latency pos_h}} m RMS against
  {{score mission pos_h}} m. The filter has no model for delay yet (#52), and it is also
  overconfident here ([honesty](honesty.md#overconfident-fixes-that-arrive-late)).
- `correlated` makes every sensor's errors change more slowly than the filter assumes. Heading
  suffers most, {{score correlated yaw}}° against {{score mission yaw}}°, and the filter is
  overconfident about position (#51).
- `mag_disturbance` gives the magnetometer a 30° error for 10 s. The filter refuses those
  readings, so heading barely suffers: {{score mag_disturbance yaw}}° against
  {{score mission yaw}}°.
- `logging_dropout` loses 1.2 s of the log in a turn. The worst horizontal error,
  {{score logging_dropout pos_h_max}} m against {{score mission pos_h_max}} m, is at the end of
  the gap.

## The baseline, over time

How to read these figures: the orange line is the error, true value minus estimate, at every
moment. The grey band is ±3σ, three standard deviations of the uncertainty the filter reported
at that moment. While the line stays inside the band, the filter's error is one it admitted to.

{{figure mission error_position}}

{{figure mission error_velocity}}

{{figure mission error_attitude}}

The band is much wider than the error. That means the filter is pessimistic about these
flights, which is the safe direction but not free. The
[honesty page](honesty.md#pessimistic-by-design-of-the-test) explains why and measures it.

## A start in motion

`moving_start` is in the air and turning from the first sample, so the filter has no still
moment to find which way is down. It starts from a rough guess, takes its first GNSS fix and
magnetometer heading as they are, and corrects from there. Over the whole flight, including
that start, tilt is off by {{score moving_start tilt}}° RMS and heading by
{{score moving_start yaw}}°.

{{figure moving_start error_attitude}}

## What this page cannot say

A simulator models only the errors it was written with: no vibration spectrum of a real frame,
no GNSS reflections off buildings, no clock drift between sensors. Real ground truth will come
from the INSANE dataset (#9). Its licence allows publishing summary numbers here, but not
trajectory plots.

## Details

Every run on this page is the one CI checks (`data/bench.sh`), at the seed pinned in
`data/scenarios.txt`. Every number is copied from the `score` line that
`examples/replay.rs` printed for it. The error is `truth ⊖ estimate` in the error state of
`EQUATIONS.md` (2), computed in one place (`error_state`). Attitude is split into tilt, about the
two horizontal axes, and heading, about down, the same way the filter's `Validity` reports it.
The figures are drawn by `tools/replay_report.py` from the run's `<out>.error.csv`, which holds
the same error beside the filter's σ on the same axis.
