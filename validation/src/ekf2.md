# Agreement with EKF2

**On real flights, does it agree with the estimator PX4 flies?** On the RTK baseline the
horizontal velocity agrees to {{agreement 89a498ce/raw vel_n_rms}} m/s RMS north and tilt to
{{agreement 89a498ce/raw tilt_diff_rms}}° RMS. The two part company where a log gives a
reason: a receiver that overstates its accuracy, a hand launch levelled wrong, and a barometer
the two filters trust differently. Each is shown below, beside one offset nothing yet explains.

{{stamp}}

**EKF2 is not truth.** Every number on this page is a distance between two estimators fed the
same log, so it measures agreement and never accuracy. A disagreement is a finding to explain,
not an error attributed to either side. No PX4 log carries truth; the
[accuracy page](accuracy.md) is where accuracy is measured.

The logs are from [PX4 Flight Review](https://review.px4.io), under CC BY 4.0. Each is pinned by
checksum in `data/manifest.txt`, whose note for it says what the log is and what it alone covers.

## How the comparison is made

`data/fetch.sh --compare` converts each log with EKF2's own solution beside it and replays it
twice. The difference is what GNSS noise is fused with:

- **`raw`**: each fix's variance as the receiver reports it. This is what `data/manifest.txt`
  pins, and what this crate does by default.
- **`px4`**: the same variances raised to EKF2's own floors, read from the parameters the log
  flew with. With these floors, EKF2 and this filter trust the receiver equally, so what remains
  is the filters.

The statistics are `tools/agreement.py`'s, and `data/README.md`, "Agreement with EKF2", defines
each one. In short:

- `pos_*_rms` (metres) and `vel_*_rms` (m/s) are the RMS of EKF2's estimate less this
  filter's, with EKF2's origin moved into this filter's frame.
- `tilt_diff_rms` is in degrees. `heading_diff_med` is the median absolute heading difference,
  in degrees.
- `rej_s_gnss_pos` is the seconds each filter spent with GNSS position over its own gate.

`none` means the log cannot answer: EKF2 reported no origin, or ran no GNSS.

## Every log

Fused `raw`:

{{table agreement @corpus/raw pos_n_rms,pos_e_rms,pos_d_rms,vel_n_rms,tilt_diff_rms,heading_diff_med,rej_s_gnss_pos,rej_s_gnss_pos_ekf2}}

Fused at EKF2's floors (`px4`):

{{table agreement @corpus/px4 pos_n_rms,pos_e_rms,pos_d_rms,vel_n_rms,tilt_diff_rms,heading_diff_med,rej_s_gnss_pos,rej_s_gnss_pos_ekf2}}

Four rows have a cause in the log rather than in either filter:

- `f16771dd` carries no GNSS, and EKF2 fused no magnetometer on it either (it was configured
  for vision height). With no aiding on heading its heading drifted freely, so the
  {{agreement f16771dd/raw heading_diff_med}}° is EKF2's unaided heading against this filter's
  magnetic one.
- `7592c9b2` is flown by LPE, PX4's older estimator, not EKF2, and reports no local position.
- `3949f175` is a simulation (PX4 SITL), whose declination parameter does not describe the
  field its simulator generated. It stays in the corpus for its log format.
- `a299e722`'s receiver contradicts its own velocities. Fused `raw`, this filter turns down a
  large share of them, which moves both estimates apart. At EKF2's floors the two come closer:
  heading differs by {{agreement a299e722/px4 heading_diff_med}}° against
  {{agreement a299e722/raw heading_diff_med}}°.

## The RTK baseline: `89a498ce`

A quadrotor mission with an RTK receiver. It reaches {{summary 89a498ce/raw extent}} m from
the start, at up to {{summary 89a498ce/raw speed_max}} m/s. Horizontal velocity and attitude
agree closely. Position does not, in one direction: EKF2 sits
{{agreement 89a498ce/raw pos_n_rms}} m RMS north of this filter, and north of its own
receiver's fixes too, which this filter follows. Nothing found so far explains that offset.
It is listed as open rather than attributed.

{{figure 89a498ce/raw track}}

{{figure 89a498ce/raw states_position_ned}}

## A receiver dishonest at speed: `093e806a`

A fixed-wing flight out to {{summary 093e806a/raw extent}} m at up to
{{summary 093e806a/raw speed_max}} m/s. Its receiver reports sub-metre accuracy while its own
velocities contradict its positions. Fused as reported, this filter spends
{{agreement 093e806a/raw rej_s_gnss_pos}} s over its position gate. At EKF2's floors it spends
{{agreement 093e806a/px4 rej_s_gnss_pos}} s. EKF2, which applies those floors, spends
{{agreement 093e806a/raw rej_s_gnss_pos_ekf2}} s over its own gate. So the choice of GNSS noise
policy accounts for most of the rejection disagreement between the two filters. On this log,
EKF2 rejects more than this filter does under either policy.

{{figure 093e806a/raw ratios}}

{{figure 093e806a/px4 ratios}}

## Two height references: `2c42096b`

This log is not a flight. It is a vehicle vibrating in place for two hours under a poor sky
view: it never moves more than {{summary 2c42096b/raw extent}} m, and its figures pin
behaviour, not accuracy. EKF2 was configured to take height from its barometer
(`height_reference_ekf2=` {{agreement 2c42096b/raw height_reference_ekf2}}). This filter fuses
both the barometer and GNSS height, and estimates the barometer's offset, so the slow part of
its height follows GNSS. The barometer drifts over the two hours. The two heights therefore
move in opposite directions. From the mean of the first 60 s to the mean of the last 60 s, up
positive, EKF2's height changes by {{agreement 2c42096b/raw climb_ekf2}} m and this filter's by
{{agreement 2c42096b/raw climb}} m. The figure draws EKF2 at this filter's origin, so the two
height traces are in one frame. Nothing here says which is right. Horizontally the two agree
to {{agreement 2c42096b/raw pos_n_rms}} m RMS north.

{{figure 2c42096b/raw states_position_ned}}

## A logging dropout at 30 m/s: `4b473e91`

A standard VTOL that hovers, transitions and cruises
{{summary 4b473e91/raw extent}} m out at up to {{summary 4b473e91/raw speed_max}} m/s. The
logger drops the IMU {{summary 4b473e91/raw coasted}} times, each longer than one step can
integrate. This filter coasts
each gap on its estimated velocity and accepts the next fix. With EKF2's origin moved onto this
filter's, the two tracks stay together through the cruise
({{agreement 4b473e91/raw pos_e_rms}} m RMS east).

{{figure 4b473e91/raw track}}

## Past the vertical: `285ee2e7`

A tailsitter, which flies forward pitched through 90°: it reaches
{{summary 285ee2e7/raw tilt_max}}° of tilt. The two filters' tilt agrees to
{{agreement 285ee2e7/raw tilt_diff_rms}}° RMS through the transitions. This filter's attitude
is split into tilt and heading so that neither is singular at 90° of pitch.

{{figure 285ee2e7/raw attitude}}

## Where this filter loses: `7ce66f0d`

A flying wing launched by hand. The window this filter levels from is the launch itself, not a
vehicle at rest, so it levels its tilt wrong. For the first minutes the estimate swings away
from EKF2 and the GNSS fixes, and the filter recovers by adopting a fix whole
{{summary 7ce66f0d/raw recovered}} times over the flight. After that the two positions track
each other, but this filter's `Status` spends most of the flight `Degraded` and ends there
(`status=` {{summary 7ce66f0d/raw status}}). Over the whole log the position differs from
EKF2's by {{agreement 7ce66f0d/raw pos_e_rms}} m RMS east and tilt by
{{agreement 7ce66f0d/raw tilt_diff_rms}}° RMS, and the filter turns down GNSS position for
{{agreement 7ce66f0d/raw rej_s_gnss_pos}} s where EKF2 turns down none. This is a loss, and its
cause is known: initialization cannot yet subtract the vehicle's own acceleration from the
window. That is equation (5′), in-motion levelling, which is #59.

{{figure 7ce66f0d/raw states_position_ned}}
