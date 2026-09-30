# Agreement with EKF2

**On real flights, does it agree with the estimator PX4 flies?** Mostly, and closely. On the log
with the most precise receiver (RTK), position agrees to {{agreement 89a498ce/raw pos_n_rms}} m
RMS north, velocity to {{agreement 89a498ce/raw vel_n_rms}} m/s RMS north and tilt to
{{agreement 89a498ce/raw tilt_diff_rms}}° RMS. Where the two differ, the log usually shows why: a
receiver that overstates its accuracy, a hand launch this filter levels wrong, and a barometer
the two filters trust differently.

{{stamp}}

**EKF2 is not the truth.** Real flight logs carry no true trajectory, so every number here is a
distance between two estimators fed the same sensor data. It measures agreement, never
accuracy, and a difference is a finding to explain, not a mistake by either side. Accuracy is
measured on the [accuracy page](accuracy.md), against simulation.

The logs are public, from [PX4 Flight Review](https://review.px4.io) under CC BY 4.0. Each is
pinned by checksum in `data/manifest.txt`, beside a note on what the log is and what it tests.

## How the comparison is made

Each log carries EKF2's own solution, recorded in flight. This filter is run on the same sensor
data twice, differing only in how much it trusts the GNSS receiver:

- **`raw`**: exactly as much as the receiver claims. This is this filter's default.
- **`px4`**: never more than EKF2 would. EKF2 applies minimum noise levels (floors) to what the
  receiver reports, set by the parameters in the log. With the same floors, the remaining
  difference is between the filters rather than their trust in the receiver.

## Every log

Replayed `raw`:

{{table agreement @corpus/raw pos_n_rms,pos_e_rms,pos_d_rms,vel_n_rms,tilt_diff_rms,heading_diff_med,rej_s_gnss_pos,rej_s_gnss_pos_ekf2}}

Replayed `px4`:

{{table agreement @corpus/px4 pos_n_rms,pos_e_rms,pos_d_rms,vel_n_rms,tilt_diff_rms,heading_diff_med,rej_s_gnss_pos,rej_s_gnss_pos_ekf2}}

Columns, each a difference between EKF2 and this filter:
- `pos_n_rms`, `pos_e_rms` and `pos_d_rms`: position north, east and down, in metres RMS, with
  EKF2's position converted into this filter's frame.
- `vel_n_rms`: velocity north, in m/s RMS.
- `tilt_diff_rms`: tilt, in degrees RMS.
- `heading_diff_med`: heading, the median difference in degrees.
- `rej_s_gnss_pos` and `rej_s_gnss_pos_ekf2`: how many seconds this filter and EKF2 each spent
  refusing GNSS positions as implausible.

`none` means the log cannot answer, because EKF2 reported no origin or had no GNSS.
`data/README.md`, "Agreement with EKF2", defines each statistic exactly.

Some rows are explained by the log, not by either filter:

- `f16771dd` has no GNSS, and EKF2 used no magnetometer on it either, so EKF2's heading drifted
  with nothing to correct it. The {{agreement f16771dd/raw heading_diff_med}}° is that drift
  against this filter's magnetometer heading.
- `7592c9b2` was flown with LPE, an older PX4 estimator, not EKF2, and reports no position.
- `3949f175` is a simulation (PX4 SITL) whose magnetic declination setting does not match the
  field its simulator generated. It is kept for its log format.
- `a299e722` has a receiver whose velocities contradict its own positions. Trusting it fully,
  this filter refuses many of its velocities and the two estimates drift apart, until its
  positions are refused too and adopted back. With EKF2's floors they stay together: east
  position differs by {{agreement a299e722/px4 pos_e_rms}} m RMS against
  {{agreement a299e722/raw pos_e_rms}} m. It is also the one log with a dual-antenna receiver, whose heading
  both estimators fuse; their headings differ by a median of
  {{agreement a299e722/px4 heading_diff_med}}°.

The sections below look closer at the logs that each test something the others do not, ending
with the one where this filter clearly does worse.

## The precise receiver: `89a498ce`

A quadrotor mission with an RTK receiver, flying {{summary 89a498ce/raw extent}} m out at up to
{{summary 89a498ce/raw speed_max}} m/s. The two agree closely on everything: position to
{{agreement 89a498ce/raw pos_n_rms}} m RMS north and {{agreement 89a498ce/raw pos_e_rms}} m east,
both following the same centimetre fixes.

Comparing positions needs the two filters in one frame. PX4 measures its local north and east on
a sphere, while this filter uses the exact tangent plane of the WGS 84 ellipsoid, and the two
differ by about 0.2 % of the distance from the origin, so EKF2's position is converted into this
filter's frame before any figure here is computed.

{{figure 89a498ce/raw track}}

{{figure 89a498ce/raw states_position_ned}}

## A receiver that overstates its accuracy: `093e806a`

A fixed-wing aircraft flying {{summary 093e806a/raw extent}} m out at up to
{{summary 093e806a/raw speed_max}} m/s. Its receiver claims sub-metre accuracy, but its own
velocities contradict its positions. How long each filter spent refusing its positions:

- this filter, trusting the receiver fully (`raw`): {{agreement 093e806a/raw rej_s_gnss_pos}} s;
- this filter, with EKF2's floors (`px4`): {{agreement 093e806a/px4 rej_s_gnss_pos}} s;
- EKF2: {{agreement 093e806a/raw rej_s_gnss_pos_ekf2}} s.

So how much the receiver is trusted explains most of the difference between the filters, and
EKF2 refuses more than this filter under either setting.

{{figure 093e806a/raw ratios}}

{{figure 093e806a/px4 ratios}}

## A receiver that jumps: `2b2ad123`

A quadrotor with a second RTK receiver, flying {{summary 2b2ad123/raw extent}} m out at up to
{{summary 2b2ad123/raw speed_max}} m/s. The receiver reports centimetre accuracy on every fix,
yet its position now and then slips one or two tenths of a second against its own velocity: at
10 m/s, a metre along the track and nothing across it, and the slip later comes straight back.
Trusting the receiver fully, this filter refuses
{{summary 2b2ad123/raw rejected_gnss_pos}} fixes, and every one sits on such a slip or on the
receiver's return from one. Half refuse the slip itself, and those refusals can be seen to be
right: a step that reverts within seconds was never the vehicle moving. The other half are the
cost of trusting the receiver: twice the filter followed a slip small enough to pass the gate,
then refused the receiver's return until it adopted a fix. With EKF2's
floors it refuses {{summary 2b2ad123/px4 rejected_gnss_pos}}, and EKF2 refuses none. Over the
whole flight the two filters agree to
{{agreement 2b2ad123/raw pos_n_rms}} m RMS north.

{{figure 2b2ad123/raw ratios}}

## Two ideas of height: `2c42096b`

Not a flight: a vehicle vibrating on the ground for two hours with poor GNSS reception, moving
at most {{summary 2c42096b/raw extent}} m. Its numbers show behaviour, not accuracy. EKF2 was
set to take its height from the barometer (its log records the height reference as
`{{agreement 2c42096b/raw height_reference_ekf2}}`). This filter uses the barometer and GNSS
height together, and trusts GNSS for slow changes. The barometer drifts over the two hours, so
the two heights move apart. From the average of the first minute to the average of the last,
EKF2's height changes by {{agreement 2c42096b/raw climb_ekf2}} m and this filter's by
{{agreement 2c42096b/raw climb}} m (up is positive). With no true height, nothing here says
which is right. Horizontally the two agree to {{agreement 2c42096b/raw pos_n_rms}} m RMS north.

{{figure 2c42096b/raw states_position_ned}}

## Gaps in the log at 30 m/s: `4b473e91`

A VTOL aircraft that hovers, transitions and cruises {{summary 4b473e91/raw extent}} m out at
up to {{summary 4b473e91/raw speed_max}} m/s. Its log loses the IMU
{{summary 4b473e91/raw coasted}} times, each gap too long to integrate as one step. This filter
carries its velocity through each gap and accepts the next GNSS fix, and the two tracks stay
together through the cruise ({{agreement 4b473e91/raw pos_e_rms}} m RMS east).

{{figure 4b473e91/raw track}}

## Past the vertical: `285ee2e7`

A tailsitter, which flies forward pitched through 90°, reaching
{{summary 285ee2e7/raw tilt_max}}° of tilt. The two filters' tilt agrees to
{{agreement 285ee2e7/raw tilt_diff_rms}}° RMS through the transitions between hover and forward
flight.

{{figure 285ee2e7/raw attitude}}

## Where this filter loses: `7ce66f0d`

A flying wing launched by hand. This filter finds which way is down by averaging the
accelerometer over its first moments, which only works when the vehicle is still. Here those
moments are the throw, so it starts with the wrong tilt. For the first minutes its position
swings away from EKF2's and from the GNSS fixes, and it resets itself to a fix
{{summary 7ce66f0d/raw recovered}} times over the flight. After that the positions track each
other, but the filter reports `Degraded` for most of the flight and ends there
({{summary 7ce66f0d/raw status}}). Over the whole log, position differs from EKF2's by
{{agreement 7ce66f0d/raw pos_e_rms}} m RMS east and tilt by
{{agreement 7ce66f0d/raw tilt_diff_rms}}° RMS, and this filter spends
{{agreement 7ce66f0d/raw rej_s_gnss_pos}} s refusing GNSS positions where EKF2 spends
{{agreement 7ce66f0d/raw rej_s_gnss_pos_ekf2}} s.

This is a loss with a known cause. Starting while moving needs the vehicle's own acceleration
taken out of that first average, which is equation (5′), in-motion levelling, and #59.

{{figure 7ce66f0d/raw states_position_ned}}
