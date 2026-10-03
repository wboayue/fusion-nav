# Accuracy against truth

**How close does the estimate get when the true answer is known?** On the baseline simulated
flight, horizontal position is off by {{score mission pos_h}} m RMS and never by more than
{{score mission pos_h_max}} m. Tilt is off by {{score mission tilt}}° RMS and heading by
{{score mission yaw}}°. These are simulated flights, so they show how the filter handles the
errors the simulator models, not every error a real vehicle has. One real quadcopter is scored
too, [below](#a-real-quadcopter-against-rtk), for position, height and velocity.

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
- `bench`: 120 s still with no GNSS, the application telling the filter it is still.
- `hover_outage`: a hover drifting 27 m, with no GNSS for 90 s of it.
- The rest (`harsh_imu`, `gnss_outage`, `baro_drift`, `gnss_latency`, `correlated`,
  `mag_disturbance`, `logging_dropout`): the baseline circuit with one thing changed, each
  described below.
- `flight`: 14 s at 50 Hz, with sensors appearing and dropping out. It is the log CI replays.

## Every scenario

{{table score @scenarios pos_h,pos_h_max,pos_v,vel,tilt,yaw}}

Columns: `pos_h` is horizontal position error and `pos_v` height error, in meters. `pos_h_max`
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
- `hover_outage` is not a departure but the case `gnss_outage` cannot be: a long outage with
  the vehicle hardly moving. Nothing aids tilt for 90 s, and the position hold
  ([GOALS.md](../GOALS.md#holding-tilt-without-aiding)) keeps it at {{score hover_outage tilt}}°
  RMS, with position {{score hover_outage pos_h}} m RMS. `bench` holds tilt to
  {{score bench tilt}}° with no GNSS at all, from the application's word that it is still.
- `moving_start` starts in the air, turning, with no still moment to level from. That costs
  attitude at the start; [below](#a-start-in-motion), it converges.
- `baro_drift` has a barometer whose zero drifts. Height suffers:
  {{score baro_drift pos_v}} m RMS against {{score mission pos_v}} m, and the filter reports it
  ([robustness](robustness.md#a-drifting-barometer)).
- `gnss_latency` delivers every GNSS fix 150 ms late, and the filter uses each at the moment it
  describes. Position is {{score gnss_latency pos_h}} m RMS against {{score mission pos_h}} m,
  and the filter is honest about it ([honesty](honesty.md#fixes-that-arrive-late)).
- `correlated` makes every sensor's errors change more slowly than the filter assumes. Heading
  suffers most, {{score correlated yaw}}° against {{score mission yaw}}°, and the filter is
  overconfident about position (#195).
- `mag_disturbance` gives the magnetometer a 30° error for 10 s. The filter refuses those
  readings, so heading barely suffers: {{score mag_disturbance yaw}}° against
  {{score mission yaw}}°.
- `logging_dropout` loses 1.2 s of the log in a turn. The worst horizontal error,
  {{score logging_dropout pos_h_max}} m against {{score mission pos_h_max}} m, is at the end of
  the gap.

## The baseline, over time

How to read these figures: the orange line is the error, true value minus estimate, at every
moment. The gray band is ±3σ, three standard deviations of the uncertainty the filter reported
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

## A real quadcopter against RTK

A simulator models only the errors it was written with: no vibration spectrum of a real frame,
no bias in a real receiver, no gaps in a real log. The
[INSANE dataset](https://www.aau.at/en/smart-systems-technologies/control-of-networked-systems/datasets/insane-dataset/)
(University of Klagenfurt; Brommer et al., IROS 2022,
[arXiv:2210.09114](https://arxiv.org/abs/2210.09114)) flew a 3 kg quadcopter carrying two RTK
receivers 1.2 m apart, which place it to centimeters while their corrections hold. Three of its flights are replayed
here on the autopilot's own sensors, the ones a flight controller fuses: its IMU, its ordinary
GNSS receiver, its barometer and its magnetometer. The RTK receivers are only the truth.

| Flight | Horizontal (m RMS) | Its fixes, horizontal (m RMS) | Height (m RMS) | Its fixes, height (m RMS) | Velocity (m/s RMS) | Position NEES | Velocity NEES |
|---|---|---|---|---|---|---|---|
| Model airfield, Klagenfurt, 24 m climb | {{score insane-outdoor_1/raw pos_h}} | {{score insane-outdoor_1/raw rms_gnss_pos}} | {{score insane-outdoor_1/raw pos_v}} | {{score insane-outdoor_1/raw rms_gnss_hgt}} | {{score insane-outdoor_1/raw vel}} | {{score insane-outdoor_1/raw nees_pos}} | {{score insane-outdoor_1/raw nees_vel}} |
| Desert, a receiver that overstates its accuracy | {{score insane-mars_1/raw pos_h}} | {{score insane-mars_1/raw rms_gnss_pos}} | {{score insane-mars_1/raw pos_v}} | {{score insane-mars_1/raw rms_gnss_hgt}} | {{score insane-mars_1/raw vel}} | {{score insane-mars_1/raw nees_pos}} | {{score insane-mars_1/raw nees_vel}} |
| Desert, a long hover | {{score insane-mars_19/raw pos_h}} | {{score insane-mars_19/raw rms_gnss_pos}} | {{score insane-mars_19/raw pos_v}} | {{score insane-mars_19/raw rms_gnss_hgt}} | {{score insane-mars_19/raw vel}} | {{score insane-mars_19/raw nees_pos}} | {{score insane-mars_19/raw nees_vel}} |

Horizontal error here is mostly the receiver's, and the table sets the two side by side: the
fixes' own error against the truth at each fix's time. An ordinary receiver is off by a meter
or more for tens of seconds at a time, and no filter can remove an error its only position
source shares. What the filter owes is to know it, and it does: position NEES under 1 means
the reported uncertainty covers the error, conservatively (the
[honesty page](honesty.md#how-it-is-measured) says what NEES is). That rests on the filter
treating a receiver's error as lasting from one fix to the next
([EQUATIONS.md (24′)](../EQUATIONS.md)): fused as though each fix's error were new, the same
flights read far overconfident, and one of these receivers claims less error than it has.
It costs a little accuracy: the filter ends slightly further from the truth than the fixes
themselves are, where fused that way it would match them. Velocity is more conservative
still, because the dataset's GNSS velocity is horizontal only and states no accuracy, so none
is fused and velocity is observed only through the fixes.
Height is where the filter does better than its receiver, because it has a second height
source: the fixes' height errs by meters, and the barometer holds the estimate where they
wander. The barometers depart from the truth by meters too, during the airfield flight's climb and slowly
through the hover, which is the case the filter's estimated barometric offset exists for.

Attitude is not scored. INSANE builds its attitude truth from the RTK baseline and the same
magnetometer the filter fuses, and at rest that truth tilts gravity several degrees from
vertical, more than this filter's own tilt error; `tools/insane2replay.py` has the
measurements. The license allows publishing these numbers but not the converted data or plots
of it, and validating a commercial product against INSANE needs an arrangement with Klagenfurt.

## What this page cannot say

Attitude accuracy rests on the simulator alone: no real dataset found carries an attitude truth
better than this filter's own estimate.

## Details

Every run on this page is the one CI checks (`data/bench.sh`), at the seed pinned in
`data/scenarios.txt`. Every number is copied from the `score` line that
`examples/replay/main.rs` printed for it. The error is `truth ⊖ estimate` in the error state of
`EQUATIONS.md` (2), computed in one place (`error_state`). Attitude is split into tilt, about the
two horizontal axes, and heading, about down, the same way the filter's `Validity` reports it.
The figures are drawn by `tools/replay_report.py` from the run's `<out>.error.csv`, which holds
the same error beside the filter's σ on the same axis.
