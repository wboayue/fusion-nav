# Equations

> **Status: design only.** No implementation exists yet. The code references in
> [Equation-to-code mapping](#equation-to-code-mapping) describe the intended layout, not
> existing symbols.

This document is the normative mathematical description of `fusion-nav`. Equations are numbered
so that the implementation can cite them directly; see
[Readable mathematics](GOALS.md#3-readable-mathematics) for why that matters.

See [DESIGN.md](DESIGN.md) for the architecture and [GOALS.md](GOALS.md) for positioning.

## Notation and conventions

### Frames

Frame labels appear only as subscripts on vectors: $`a_b`$ is a body-frame vector, $`a_n`$ a
navigation-frame vector.

| symbol | meaning |
| ------ | ------- |
| $`n`$ | navigation frame, North-East-Down |
| $`b`$ | body frame |

The navigation frame is down-positive, so gravity has a **positive** $`z`$ component.

The attitude error is defined as a **local** (body-frame) perturbation. Every Jacobian below
follows from that choice; switching to a global perturbation changes signs throughout.

### States and measurements

| symbol | meaning | units |
| ------ | ------- | ----- |
| $`p, v`$ | position and velocity, navigation frame | m, m s⁻¹ |
| $`q`$ | unit quaternion rotating body to navigation, Hamilton convention, scalar first | — |
| $`R(q)`$ | rotation matrix equivalent of $`q`$, body to navigation | — |
| $`\beta_a, \beta_g`$ | accelerometer and gyroscope bias, body frame | m s⁻², rad s⁻¹ |
| $`a_m, \omega_m`$ | raw accelerometer and gyroscope measurements, body frame | m s⁻², rad s⁻¹ |
| $`g`$ | gravity vector in the navigation frame | m s⁻² |
| $`m_n`$ | reference magnetic field, navigation frame | normalized |
| $`D_m`$ | magnetic declination | rad |
| $`\alpha`$ | barometric altitude above its own reference, positive **up** | m |

Biases are written $`\beta`$ rather than $`b`$ so that they never collide with the body-frame
subscript.

### Filter quantities

| symbol | meaning | dimension |
| ------ | ------- | --------- |
| $`\delta x`$ | error state | 15 |
| $`P`$ | error covariance | 15 × 15 |
| $`F`$ | discrete state transition matrix | 15 × 15 |
| $`Q`$ | discrete process noise | 15 × 15 |
| $`H`$ | measurement Jacobian, $`H = \left.\partial h / \partial \delta x\right\rvert_{\hat{x}}`$ | dim(z) × 15 |
| $`R_m`$ | measurement noise covariance | dim(z) × dim(z) |
| $`y`$ | innovation | dim(z) |
| $`S`$ | innovation covariance | dim(z) × dim(z) |
| $`K`$ | Kalman gain | 15 × dim(z) |
| $`G`$ | reset Jacobian | 15 × 15 |
| $`w_a, w_g`$ | accelerometer and gyroscope white noise | — |
| $`w_{\beta a}, w_{\beta g}`$ | bias random-walk driving noise | — |

### Operators

| symbol | meaning |
| ------ | ------- |
| $`\hat{x}`$ | estimated (nominal) quantity |
| $`[\,u\,]_\times`$ | skew-symmetric matrix of $`u`$, so that $`[\,u\,]_\times v = u \times v`$ |
| $`\otimes`$ | quaternion product |
| $`\mathrm{Exp}(\phi)`$ | rotation vector to quaternion, $`\mathrm{Exp}(\phi) = [\cos\tfrac{\lVert\phi\rVert}{2},\ \tfrac{\phi}{\lVert\phi\rVert}\sin\tfrac{\lVert\phi\rVert}{2}]`$ |
| $`R\{\phi\}`$ | rotation **matrix** of the rotation vector $`\phi`$, equal to $`R(\mathrm{Exp}(\phi))`$ |
| $`\mathrm{wrap}(\cdot)`$ | angle wrapped to $`(-\pi, \pi]`$ |
| $`e_3`$ | $`[0, 0, 1]^\mathsf{T}`$ |

### Gravity

$`g = [0, 0, \gamma]^\mathsf{T}`$ with $`\gamma`$ the local gravity magnitude. The default is the
WGS-84 standard value 9.80665 m s⁻², but $`\gamma`$ varies by roughly 0.5 % between the equator
and the poles and falls by about 3 µm s⁻² per metre of altitude. It is a configuration
parameter, not a literal.

## State definitions

The nominal state has 16 components:

**(1)**

```math
x = \begin{bmatrix} p & v & q & \beta_a & \beta_g \end{bmatrix}^\mathsf{T}
```

The error state has 15:

**(2)**

```math
\delta x = \begin{bmatrix} \delta p & \delta v & \delta\theta & \delta\beta_a & \delta\beta_g \end{bmatrix}^\mathsf{T} \in \mathbb{R}^{15}
```

True state as nominal composed with error:

**(3)**

```math
p = \hat{p} + \delta p, \quad v = \hat{v} + \delta v, \quad q = \hat{q} \otimes \delta q, \quad \beta_a = \hat{\beta}_a + \delta\beta_a, \quad \beta_g = \hat{\beta}_g + \delta\beta_g
```

with the small-angle approximation

**(4)**

```math
\delta q \approx \begin{bmatrix} 1 & \tfrac{1}{2}\delta\theta \end{bmatrix}^\mathsf{T}
```

Using a three-component $`\delta\theta`$ rather than four quaternion states keeps the covariance
non-singular and preserves the unit-norm constraint on $`q`$ automatically.

## Initialization

The filter is initialized from a quasi-static interval: the vehicle stationary, with gravity the
only specific force. Initialization quality dominates early-flight performance, so the static
assumption must be validated rather than assumed.

Roll and pitch follow from the accelerometer. With $`f = a_m`$ averaged over the interval:

**(5)**

```math
\phi_0 = \mathrm{atan2}(-f_y,\ -f_z), \qquad \theta_0 = \mathrm{atan2}\left(f_x,\ \sqrt{f_y^2 + f_z^2}\right)
```

The signs follow from the down-positive convention: a level, stationary accelerometer reads
$`f = [0, 0, -\gamma]^\mathsf{T}`$.

Yaw follows from the magnetometer, levelled by the roll and pitch just computed. With
$`R_0 = R_y(\theta_0) R_x(\phi_0)`$ and $`m_b`$ the averaged magnetometer reading:

**(6)**

```math
\tilde{m} = R_0\, m_b, \qquad \psi_0 = D_m - \mathrm{atan2}(\tilde{m}_E,\ \tilde{m}_N)
```

The nominal state is then initialized as

**(7)**

```math
\hat{q}_0 = q_{ZYX}(\psi_0, \theta_0, \phi_0), \qquad \hat{p}_0 = 0, \qquad \hat{v}_0 = 0, \qquad \hat{\beta}_{a,0} = 0, \qquad \hat{\beta}_{g,0} = \overline{\omega_m}
```

with $`q_{ZYX}`$ the quaternion of the yaw-pitch-roll sequence
$`R = R_z(\psi) R_y(\theta) R_x(\phi)`$, matching $`R_0`$ in (6).

The gyroscope bias is observable at rest and is initialized to the measured average. The
accelerometer bias is not separable from attitude error at rest and is initialized to zero.

The initial covariance is diagonal:

**(8)**

```math
P_0 = \mathrm{diag}\left( \sigma_{p,0}^2 I,\quad \sigma_{v,0}^2 I,\quad \mathrm{diag}(\sigma_{\text{tilt},0}^2, \sigma_{\text{tilt},0}^2, \sigma_{\psi,0}^2),\quad \sigma_{\beta a,0}^2 I,\quad \sigma_{\beta g,0}^2 I \right)
```

Yaw uncertainty $`\sigma_{\psi,0}`$ is set much larger than tilt uncertainty
$`\sigma_{\text{tilt},0}`$: roll and pitch come from gravity and are well determined, whereas yaw
comes from the magnetometer and inherits its calibration error.

## Nominal state propagation

Bias-corrected IMU measurements:

**(9)**

```math
\omega = \omega_m - \hat{\beta}_g
```

**(10)**

```math
a_b = a_m - \hat{\beta}_a
```

Specific force rotated into the navigation frame and gravity added:

**(11)**

```math
a_n = R(\hat{q})\, a_b + g
```

Continuous-time kinematics:

**(12)**

```math
\dot{p} = v, \qquad \dot{v} = a_n, \qquad \dot{q} = \tfrac{1}{2}\, q \otimes \begin{bmatrix} 0 \\ \omega \end{bmatrix}, \qquad \dot{\beta}_a = 0, \qquad \dot{\beta}_g = 0
```

Discrete integration over $`\Delta t`$:

**(13)**

```math
\hat{p} \leftarrow \hat{p} + \hat{v}\,\Delta t + \tfrac{1}{2} a_n \Delta t^2
```

**(14)**

```math
\hat{v} \leftarrow \hat{v} + a_n \Delta t
```

**(15)**

```math
\hat{q} \leftarrow \hat{q} \otimes \mathrm{Exp}(\omega\,\Delta t)
```

> **Order matters.** Equation (13) uses the **pre-update** velocity. Applying (14) first and then
> (13) adds a spurious $`a_n \Delta t^2`$ to position on every propagation step — a bias that
> integrates without bound. Implementations must evaluate (13) before (14), or compute both from
> a saved copy of $`\hat{v}`$.

Biases are modelled as random walks and are unchanged by propagation. The quaternion is
renormalized after (15).

## Error-state dynamics

Linearized continuous-time error dynamics, local attitude error:

**(16)**

```math
\delta\dot{p} = \delta v
```

**(17)**

```math
\delta\dot{v} = -R(\hat{q})\,[\,a_b\,]_\times\,\delta\theta \;-\; R(\hat{q})\,\delta\beta_a \;-\; R(\hat{q})\,w_a
```

**(18)**

```math
\delta\dot{\theta} = -[\,\omega\,]_\times\,\delta\theta \;-\; \delta\beta_g \;-\; w_g
```

**(19)**

```math
\delta\dot{\beta}_a = w_{\beta a}, \qquad \delta\dot{\beta}_g = w_{\beta g}
```

Equation (17) is the coupling that motivates the whole filter: an attitude error rotates the
measured specific force incorrectly, which integrates into velocity and then position.

## Covariance propagation

Discrete state transition matrix. Every block is first order in $`\Delta t`$ except the attitude
block, which is exact:

**(20)**

```math
F = \begin{bmatrix}
I & I\Delta t & 0 & 0 & 0 \\
0 & I & -R(\hat{q})[\,a_b\,]_\times \Delta t & -R(\hat{q})\Delta t & 0 \\
0 & 0 & R\{\omega \Delta t\}^\mathsf{T} & 0 & -I\Delta t \\
0 & 0 & 0 & I & 0 \\
0 & 0 & 0 & 0 & I
\end{bmatrix}
```

The attitude block $`R\{\omega\Delta t\}^\mathsf{T}`$ may be approximated as
$`I - [\,\omega\,]_\times \Delta t`$ where the cost of the exact form is not justified; that
approximation is the usual source of small attitude-covariance error at high rotation rates.

Discrete process noise, impulse form:

**(21)**

```math
Q = \mathrm{diag}\left( 0,\quad \sigma_a^2 \Delta t^2 I,\quad \sigma_g^2 \Delta t^2 I,\quad \sigma_{\beta a}^2 \Delta t\, I,\quad \sigma_{\beta g}^2 \Delta t\, I \right)
```

The velocity block of (21) is the rotated accelerometer noise $`R \Sigma_a R^\mathsf{T}`$. Writing
it as $`\sigma_a^2 I`$ is exact only when the accelerometer noise is **isotropic**, since
$`R (\sigma_a^2 I) R^\mathsf{T} = \sigma_a^2 I`$ for orthogonal $`R`$. Real IMUs are not
isotropic — the z axis is typically noisier. Either use a per-axis $`\Sigma_a`$ and carry the
rotation, or set $`\sigma_a`$ to the worst axis and document the conservatism.

Covariance propagation:

**(22)**

```math
P \leftarrow F P F^\mathsf{T} + Q
```

## Measurement update

For an observation $`z`$ with model $`h(x)`$, Jacobian $`H`$, and noise covariance $`R_m`$:

**(23)**

```math
y = z - h(\hat{x})
```

**(24)**

```math
S = H P H^\mathsf{T} + R_m
```

**(25)**

```math
K = P H^\mathsf{T} S^{-1}
```

**(26)**

```math
\delta\hat{x} = K y
```

Because the error state is reset to zero after every update, its prior is always zero and (26)
has no prior term.

Covariance update in Joseph form, which preserves symmetry and positive definiteness under
finite precision:

**(27)**

```math
P \leftarrow (I - KH)\,P\,(I - KH)^\mathsf{T} + K R_m K^\mathsf{T}
```

Joseph form costs more than $`P \leftarrow (I - KH)P`$ but is the appropriate default for `f32`
arithmetic on an embedded target.

## Observation models

### GNSS position

**(28)**

```math
z = p_{\text{GNSS}}, \qquad h(x) = p, \qquad H = \begin{bmatrix} I_3 & 0 & 0 & 0 & 0 \end{bmatrix}
```

Where the filter has no position estimate at all — a coarse initialization, with the vehicle
moving through somewhere it cannot name — the first fix is adopted rather than fused:

```math
\hat{p} \leftarrow z, \qquad P_{pp} \leftarrow R, \qquad P_{p\ast} \leftarrow 0
```

which is $`\lim_{P_{pp} \to \infty}`$ of the update above, taken exactly. The correlations go to
zero because the new error is the measurement's and has nothing to do with what preceded it.
Applies once, to a quantity never established; it is not a recovery mechanism. See
[gate lockout](#gate-lockout).

### GNSS velocity

**(29)**

```math
z = v_{\text{GNSS}}, \qquad h(x) = v, \qquad H = \begin{bmatrix} 0 & I_3 & 0 & 0 & 0 \end{bmatrix}
```

Velocity observations matter disproportionately: velocity error grows linearly from accelerometer
and attitude error, so constraining it directly also constrains those states through the
covariance.

The same adoption applies to the first velocity solution after a coarse start, and matters more
there: a static initialization knows the vehicle is at rest, a coarse one knows nothing at all.

### Barometric altitude

The barometer reports altitude $`\alpha`$ above its own reference, positive up, while the
navigation frame is down-positive:

**(30)**

```math
z = -(\alpha - \alpha_0), \qquad h(x) = p_D, \qquad H = \begin{bmatrix} e_3^\mathsf{T} & 0 & 0 & 0 & 0 \end{bmatrix}
```

$`\alpha_0`$ is the barometric altitude recorded during initialization, which fixes the barometer
reference to the navigation origin. **It is a constant, not a state.** The 15-state vector (2) has
no barometer bias, so slow drift in the barometric reference — from weather, from ground effect,
from sensor warm-up — is not estimated and appears directly as vertical position error. Where
that matters, the options are to re-establish $`\alpha_0`$ on the ground, to lean on GNSS height
for the low-frequency component, or to extend the state vector, which is out of scope for this
filter.

### Magnetometer, three-axis

With $`m_n`$ the reference field in the navigation frame, the predicted body-frame measurement is

**(31)**

```math
\hat{m}_b = R(\hat{q})^\mathsf{T} m_n
```

Perturbing with a local attitude error, $`R = R(\hat{q})\,\mathrm{Exp}(\delta\theta)`$, and using
$`[\,u\,]_\times v = -[\,v\,]_\times u`$:

**(32)**

```math
m_b \approx \hat{m}_b + [\,\hat{m}_b\,]_\times\, \delta\theta
```

so

**(33)**

```math
z = m_b^{\text{meas}}, \qquad h(x) = \hat{m}_b, \qquad H = \begin{bmatrix} 0 & 0 & [\,\hat{m}_b\,]_\times & 0 & 0 \end{bmatrix}
```

Three-axis fusion constrains all three attitude components from the magnetometer. That is a
liability rather than a benefit here: hard- and soft-iron errors and local field anomalies
corrupt roll and pitch, which the accelerometer already determines well, and `fusion-nav` carries
no magnetic-field states to absorb them. See
[Magnetometer without magnetic-field states](GOALS.md#magnetometer-without-magnetic-field-states).

### Magnetometer, heading only

This is the **default**. It constrains yaw alone, leaving roll and pitch to gravity.

Rotate the measurement into the navigation frame with the current attitude estimate:

**(34)**

```math
\tilde{m}_n = R(\hat{q})\, m_b^{\text{meas}}
```

If the attitude estimate were exact, the horizontal part of $`\tilde{m}_n`$ would make an angle
$`D_m`$ with North. Any residual is yaw error, so the innovation is formed directly rather than
as a difference of two angles:

**(35)**

```math
y = -\,\mathrm{wrap}\big(\mathrm{atan2}(\tilde{m}_{n,E},\ \tilde{m}_{n,N}) - D_m\big)
```

A local body-frame error $`\delta\theta`$ corresponds to the navigation-frame rotation vector
$`R(\hat{q})\,\delta\theta`$, and yaw is rotation about the navigation down axis, so

**(36)**

```math
H = \begin{bmatrix} 0 & 0 & e_3^\mathsf{T} R(\hat{q}) & 0 & 0 \end{bmatrix}
```

with $`R_m = \sigma_\psi^2`$ scalar. Equation (36) treats yaw as the down-axis component of the
rotation vector, which is exact at zero tilt and degrades as $`1/\cos\theta`$; at the tilt angles
flight controllers operate at, the error is not significant, and the approximation avoids a
singularity at 90° pitch.

Note that (35) wraps the angle **before** it is used, so the innovation is always in
$`(-\pi, \pi]`$ and a heading near ±180° does not produce a spurious 2π innovation.

## Innovation gating

The normalized innovation squared

**(37)**

```math
\epsilon = y^\mathsf{T} S^{-1} y
```

is compared against a threshold $`\gamma`$ from the chi-square distribution with
$`\dim(z)`$ degrees of freedom. The measurement is rejected when $`\epsilon > \gamma`$.

| dim(z) | observation | 95 % | 99 % |
| ------ | ----------- | ---- | ---- |
| 1 | barometric altitude, magnetic heading | 3.84 | 6.63 |
| 3 | GNSS position, GNSS velocity, three-axis magnetometer | 7.81 | 11.34 |

Rather than reporting $`\epsilon`$ directly, the filter exposes the dimensionless **test ratio**

**(38)**

```math
r = \frac{\epsilon}{\gamma}
```

so that $`r > 1`$ means rejected regardless of the degrees of freedom of the observation. One
number is then comparable across GNSS position, barometric altitude, and magnetic heading, and
directly comparable with the innovation test ratios PX4 publishes in its logs — which is what
makes replay comparison against EKF2 a like-for-like check rather than an approximate one.

This single mechanism covers GNSS glitches, barometer transients, and magnetic interference.

### Gate lockout

Gating is self-sealing. If the **filter** is wrong rather than the measurement — a poor
initialization, an unmodelled bias, a divergence — then correct measurements are inconsistent
with the state, every one of them is rejected, and the filter locks itself out of the very data
that would correct it. It then dead-reckons on the IMU alone while continuing to report a
solution whose covariance says it is confident.

A filter that gates without a route out of this state is more dangerous than one that does not
gate at all.

`fusion-nav` therefore tracks, per observation source, the time since a measurement was last
accepted and the number of consecutive rejections, and reports an aggregate status alongside the
state estimate. It does **not** reset itself: recovery policy belongs to the application, which
is the only layer that knows whether to reset the affected states, degrade the flight mode, or
alert the operator. PX4, for comparison, resets its states to the measurement after 7 s of
horizontal fusion timeout and 5 s for height.

The design obligation is that the degraded condition cannot be missed, not that the filter hides
it by recovering silently.

## Error injection and reset

The estimated error is composed into the nominal state:

**(39)**

```math
\hat{p} \leftarrow \hat{p} + \delta\hat{p}, \qquad \hat{v} \leftarrow \hat{v} + \delta\hat{v}, \qquad \hat{q} \leftarrow \hat{q} \otimes \mathrm{Exp}(\delta\hat{\theta})
```

**(40)**

```math
\hat{\beta}_a \leftarrow \hat{\beta}_a + \delta\hat{\beta}_a, \qquad \hat{\beta}_g \leftarrow \hat{\beta}_g + \delta\hat{\beta}_g
```

The error state is then reset to zero and the covariance transformed by the reset Jacobian:

**(41)**

```math
\delta x \leftarrow 0, \qquad P \leftarrow G P G^\mathsf{T}, \qquad G = \mathrm{diag}\left(I, I, I - [\tfrac{1}{2}\delta\hat{\theta}]_\times, I, I\right)
```

The attitude block of $`G`$ is frequently approximated as $`I`$. That is acceptable for small
corrections and should be an explicit, documented choice rather than an omission.

## Numerical conditioning

Symmetry is enforced after every covariance operation:

**(42)**

```math
P \leftarrow \tfrac{1}{2}\left(P + P^\mathsf{T}\right)
```

Diagonal variances are floored at a small positive value to prevent a state from becoming
unobservably certain and then unrecoverable. With `f32` these are not optional refinements;
they are what keeps a 15-state filter stable over a long flight.

## Geodetic origin

The navigation frame is a local tangent plane about an origin $`(\varphi_0, \lambda_0, h_0)`$,
which the filter holds so that a geodetic fix and the estimate are relative to the same point.
With the WGS84 radii of curvature at the origin,

```math
M = \frac{a(1 - e^2)}{(1 - e^2 \sin^2\varphi_0)^{3/2}}, \qquad
N = \frac{a}{\sqrt{1 - e^2 \sin^2\varphi_0}}
```

a fix $`(\varphi, \lambda, h)`$ is, in NED meters,

**(43)**

```math
p_N = (\varphi - \varphi_0)(M + h_0), \qquad
p_E = \operatorname{wrap}(\lambda - \lambda_0)(N + h_0)\cos\varphi_0, \qquad
p_D = -(h - h_0)
```

A first-order expansion about the origin: exact there, with an error that grows as
$`p_N p_E / R`$ — 0.2 m at 1 km by 1 km at mid latitudes. Its inverse is exact, so converting the
estimate back to latitude and longitude returns what (43) was given. Undefined at the poles, where
$`\cos\varphi_0 = 0`$.

The first geodetic fix $`z_g`$ places the origin. A filter that already has a position estimate
$`\hat{p}`$ — it has been navigating relative to its own start — places it so the fix lands on
the estimate:

**(44)**

```math
(\varphi_0, \lambda_0, h_0) = \text{(43)}^{-1}_{z_g}\!\left(-\hat{p}\right)
```

that is, the point $`-\hat{p}`$ from the fix, with (43) taken about the fix. The first fix then
carries no information about position, which is correct: before it, the filter's absolute
position was unknown, not wrong. Without an estimate (a coarse start) the origin is the fix and
the fix is adopted as $`\hat{p} = 0`$, per (28).

## Equation-to-code mapping

Intended layout. Each implementing function cites its equation numbers in a doc comment.

| equations | concept | module | function |
| --------- | ------- | ------ | -------- |
| (1)–(4) | state definitions | `state.rs` | `State`, `ErrorState` |
| (5)–(8) | static initialization | `init.rs` | `classify`, `attitude_sigmas`, `initial_covariance`; unbuilt: `level_from_accel`, `heading_from_mag` |
| (30) `α₀` | barometric reference | `init.rs` | `baro_reference` |
| (9)–(11) | bias correction, gravity | `propagate.rs` | `ImuSample`; unbuilt: `corrected_imu` |
| (12)–(15) | nominal propagation | `propagate.rs` | `propagate_nominal` |
| (16)–(19) | error dynamics | `propagate.rs` | `error_dynamics` |
| (20) | state transition matrix | `propagate.rs` | `transition_matrix` |
| (21) | discrete process noise | `propagate.rs` | `process_noise` |
| (22) | covariance propagation | `propagate.rs` | `propagate_covariance` |
| (23)–(27) | generic update, Joseph form | `update.rs` | `update` |
| (28) | GNSS position | `observation/gnss.rs` | `position_jacobian` |
| (29) | GNSS velocity | `observation/gnss.rs` | `velocity_jacobian` |
| (30) | barometric altitude | `observation/baro.rs` | `altitude_jacobian` |
| (31)–(33) | magnetometer, three-axis | `observation/mag.rs` | `field_jacobian` |
| (34)–(36) | magnetometer, heading only | `observation/mag.rs` | `heading_innovation`, `heading_jacobian` |
| (37)–(38) | innovation gating, test ratio | `update.rs` | `gate`, `test_ratio` |
| — | per-source health tracking | `health.rs` | `SourceHealth`, `Status` |
| (39)–(41) | injection and reset | `update.rs` | `inject`, `reset` |
| (42) | symmetry enforcement | `math.rs` | `enforce_symmetry` |
| (43) | local tangent plane | `geodetic.rs` | `LocalOrigin::to_ned`, `to_geodetic` |
| (44) | origin placement | `geodetic.rs` | `LocalOrigin::placing`; committed by `Eskf::fuse_gnss_geodetic` |
| — | skew, quaternion exponential, angle wrap | `math.rs` | `skew`, `exp_quat`, `wrap_pi` |

## Deferred

Measurement latency is not modelled. Every observation above is fused as though it were
simultaneous with the current state, which is not true of GNSS. See
[Measurement latency](GOALS.md#measurement-latency).

## References

* J. Solà, *Quaternion kinematics for the error-state Kalman filter*, [arXiv:1711.02508](https://arxiv.org/abs/1711.02508) — the primary source for the error-state formulation and the Jacobians above
* P. D. Groves, *Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems*, 2nd ed.
* J. A. Farrell, *Aided Navigation: GPS with High Rate Sensors*
* F. L. Markley and J. L. Crassidis, *Fundamentals of Spacecraft Attitude Determination and Control*
* [PX4 EKF2](https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf) — reference for practical behavior, not for derivations
