# Equations

> **Status: design only.** No implementation exists yet. The code references in
> [Equation-to-code mapping](#equation-to-code-mapping) describe the intended layout, not
> existing symbols.

This document is the normative mathematical description of `fusion-nav`. Equations are numbered
so that the implementation can cite them directly; see
[Readable mathematics](GOALS.md#3-readable-mathematics) for why that matters.

See [README.md](README.md) for the architecture and [GOALS.md](GOALS.md) for positioning.

## Notation and conventions

| symbol | meaning |
| ------ | ------- |
| $`n`$ | navigation frame, North-East-Down |
| $`b`$ | body frame |
| $`q`$ | unit quaternion rotating body to navigation, Hamilton convention, scalar first |
| $`R(q)`$ | rotation matrix equivalent of $`q`$, body to navigation |
| $`p, v`$ | position and velocity, navigation frame |
| $`b_a, b_g`$ | accelerometer and gyroscope bias, body frame |
| $`a_m, \omega_m`$ | raw accelerometer and gyroscope measurements, body frame |
| $`g`$ | gravity vector in the navigation frame, $`g = [0, 0, +9.81]^\mathsf{T}`$ |
| $`\hat{x}`$ | estimated (nominal) quantity |
| $`\delta x`$ | error-state quantity |
| $`[\,u\,]_\times`$ | skew-symmetric matrix of $`u`$ |
| $`\otimes`$ | quaternion product |
| $`\mathrm{Exp}(\theta)`$ | rotation vector to quaternion, $`\mathrm{Exp}(\theta) = [\cos\tfrac{\lVert\theta\rVert}{2},\ \tfrac{\theta}{\lVert\theta\rVert}\sin\tfrac{\lVert\theta\rVert}{2}]`$ |

Down is positive, so gravity has a **positive** $`z`$ component in the navigation frame.

The attitude error is defined as a **local** (body-frame) perturbation. Every Jacobian below
follows from that choice; switching to a global perturbation changes signs throughout.

## State definitions

The nominal state has 16 components:

**(1)**

```math
x = \begin{bmatrix} p & v & q & b_a & b_g \end{bmatrix}^\mathsf{T}
```

The error state has 15:

**(2)**

```math
\delta x = \begin{bmatrix} \delta p & \delta v & \delta\theta & \delta b_a & \delta b_g \end{bmatrix}^\mathsf{T} \in \mathbb{R}^{15}
```

True state as nominal composed with error:

**(3)**

```math
p = \hat{p} + \delta p, \quad v = \hat{v} + \delta v, \quad q = \hat{q} \otimes \delta q, \quad b_a = \hat{b}_a + \delta b_a, \quad b_g = \hat{b}_g + \delta b_g
```

with the small-angle approximation

**(4)**

```math
\delta q \approx \begin{bmatrix} 1 & \tfrac{1}{2}\delta\theta \end{bmatrix}^\mathsf{T}
```

Using a three-component $`\delta\theta`$ rather than four quaternion states keeps the covariance
non-singular and preserves the unit-norm constraint on $`q`$ automatically.

## Nominal state propagation

Bias-corrected IMU measurements:

**(5)**

```math
\omega = \omega_m - \hat{b}_g
```

**(6)**

```math
a_b = a_m - \hat{b}_a
```

Specific force rotated into the navigation frame and gravity added:

**(7)**

```math
a_n = R(\hat{q})\, a_b + g
```

Continuous-time kinematics:

**(8)**

```math
\dot{p} = v, \qquad \dot{v} = a_n, \qquad \dot{q} = \tfrac{1}{2}\, q \otimes \begin{bmatrix} 0 \\ \omega \end{bmatrix}, \qquad \dot{b}_a = 0, \qquad \dot{b}_g = 0
```

Discrete integration over $`\Delta t`$:

**(9)**

```math
\hat{p} \leftarrow \hat{p} + \hat{v}\,\Delta t + \tfrac{1}{2} a_n \Delta t^2
```

**(10)**

```math
\hat{v} \leftarrow \hat{v} + a_n \Delta t
```

**(11)**

```math
\hat{q} \leftarrow \hat{q} \otimes \mathrm{Exp}(\omega\,\Delta t)
```

Biases are modelled as random walks and are unchanged by propagation. The quaternion is
renormalized after (11).

## Error-state dynamics

Linearized continuous-time error dynamics, local attitude error:

**(12)**

```math
\delta\dot{p} = \delta v
```

**(13)**

```math
\delta\dot{v} = -R(\hat{q})\,[\,a_b\,]_\times\,\delta\theta \;-\; R(\hat{q})\,\delta b_a \;-\; R(\hat{q})\,n_a
```

**(14)**

```math
\delta\dot{\theta} = -[\,\omega\,]_\times\,\delta\theta \;-\; \delta b_g \;-\; n_g
```

**(15)**

```math
\delta\dot{b}_a = n_{ba}, \qquad \delta\dot{b}_g = n_{bg}
```

where $`n_a, n_g`$ are accelerometer and gyroscope white noise and $`n_{ba}, n_{bg}`$ drive the bias
random walks.

Equation (13) is the coupling that motivates the whole filter: an attitude error rotates the
measured specific force incorrectly, which integrates into velocity and then position.

## Covariance propagation

Discrete state transition matrix, first order in $`\Delta t`$:

**(16)**

```math
F = \begin{bmatrix}
I & I\Delta t & 0 & 0 & 0 \\
0 & I & -R(\hat{q})[\,a_b\,]_\times \Delta t & -R(\hat{q})\Delta t & 0 \\
0 & 0 & R\{\omega \Delta t\}^\mathsf{T} & 0 & -I\Delta t \\
0 & 0 & 0 & I & 0 \\
0 & 0 & 0 & 0 & I
\end{bmatrix}
```

The attitude block $`R\{\omega\Delta t\}^\mathsf{T}`$ is the rotation matrix of the incremental
rotation, transposed. It may be approximated as $`I - [\,\omega\,]_\times \Delta t`$ where the
cost of the exact form is not justified; the approximation is the usual source of small
attitude-covariance error at high rotation rates.

Discrete process noise, impulse form:

**(17)**

```math
Q = \mathrm{diag}\left( 0,\quad \sigma_a^2 \Delta t^2 I,\quad \sigma_g^2 \Delta t^2 I,\quad \sigma_{ba}^2 \Delta t\, I,\quad \sigma_{bg}^2 \Delta t\, I \right)
```

Covariance propagation:

**(18)**

```math
P \leftarrow F P F^\mathsf{T} + Q
```

## Measurement update

For an observation $`z`$ with model $`h(x)`$, measurement Jacobian $`H = \left.\frac{\partial h}{\partial \delta x}\right|_{\hat{x}}`$, and noise covariance $`R_m`$:

**(19)**

```math
y = z - h(\hat{x})
```

**(20)**

```math
S = H P H^\mathsf{T} + R_m
```

**(21)**

```math
K = P H^\mathsf{T} S^{-1}
```

**(22)**

```math
\delta\hat{x} = K y
```

Covariance update in Joseph form, which preserves symmetry and positive definiteness under
finite precision:

**(23)**

```math
P \leftarrow (I - KH)\,P\,(I - KH)^\mathsf{T} + K R_m K^\mathsf{T}
```

Joseph form costs more than $`P \leftarrow (I - KH)P`$ but is the appropriate default for `f32`
arithmetic on an embedded target.

## Observation models

### GNSS position

**(24)**

```math
z = p_{\text{GNSS}}, \qquad h(x) = p, \qquad H = \begin{bmatrix} I_3 & 0 & 0 & 0 & 0 \end{bmatrix}
```

### GNSS velocity

**(25)**

```math
z = v_{\text{GNSS}}, \qquad h(x) = v, \qquad H = \begin{bmatrix} 0 & I_3 & 0 & 0 & 0 \end{bmatrix}
```

Velocity observations matter disproportionately: velocity error grows linearly from accelerometer
and attitude error, so constraining it directly also constrains those states through the
covariance.

### Barometric altitude

Barometric height $`h_{\text{baro}}`$ is measured up; the navigation frame is down-positive, so

**(26)**

```math
z = -h_{\text{baro}}, \qquad h(x) = p_D, \qquad H = \begin{bmatrix} e_3^\mathsf{T} & 0 & 0 & 0 & 0 \end{bmatrix}
```

with $`e_3 = [0, 0, 1]^\mathsf{T}`$. The barometer observes height relative to an arbitrary
reference; the offset between that reference and the navigation origin must be established at
initialization or treated as slowly varying.

### Magnetometer

With $`m_n`$ the reference field in the navigation frame, the predicted body-frame measurement is

**(27)**

```math
\hat{m}_b = R(\hat{q})^\mathsf{T} m_n
```

Perturbing with a local attitude error, $`R = R(\hat{q})\,\mathrm{Exp}(\delta\theta)`$, gives

**(28)**

```math
m_b \approx \hat{m}_b + [\,\hat{m}_b\,]_\times\, \delta\theta
```

so

**(29)**

```math
z = m_b^{\text{meas}}, \qquad h(x) = \hat{m}_b, \qquad H = \begin{bmatrix} 0 & 0 & [\,\hat{m}_b\,]_\times & 0 & 0 \end{bmatrix}
```

Full three-axis fusion per (29) makes the filter sensitive to hard- and soft-iron error, which
`fusion-nav` does not estimate; see
[Magnetometer without magnetic-field states](GOALS.md#magnetometer-without-magnetic-field-states).
Projecting the measurement to a heading angle and fusing that single scalar is the more robust
option and should be the default.

## Innovation gating

The normalized innovation squared

**(30)**

```math
\epsilon = y^\mathsf{T} S^{-1} y
```

is compared against a threshold $`\gamma`$ drawn from the chi-square distribution with degrees of
freedom equal to $`\dim(z)`$. The measurement is rejected when $`\epsilon > \gamma`$.

This single mechanism covers GNSS glitches, barometer transients, and magnetic interference.
Rejection must be counted and exposed: a filter that silently discards every measurement is
indistinguishable from one that is working.

## Error injection and reset

The estimated error is composed into the nominal state:

**(31)**

```math
\hat{p} \leftarrow \hat{p} + \delta\hat{p}, \qquad \hat{v} \leftarrow \hat{v} + \delta\hat{v}, \qquad \hat{q} \leftarrow \hat{q} \otimes \mathrm{Exp}(\delta\hat{\theta})
```

**(32)**

```math
\hat{b}_a \leftarrow \hat{b}_a + \delta\hat{b}_a, \qquad \hat{b}_g \leftarrow \hat{b}_g + \delta\hat{b}_g
```

The error state is then reset to zero and the covariance transformed by the reset Jacobian:

**(33)**

```math
\delta x \leftarrow 0, \qquad P \leftarrow G P G^\mathsf{T}, \qquad G = \mathrm{diag}\left(I, I, I - [\tfrac{1}{2}\delta\hat{\theta}]_\times, I, I\right)
```

The attitude block of $`G`$ is frequently approximated as $`I`$. That is acceptable for small
corrections and should be an explicit, documented choice rather than an omission.

## Numerical conditioning

Symmetry is enforced after every covariance operation:

**(34)**

```math
P \leftarrow \tfrac{1}{2}\left(P + P^\mathsf{T}\right)
```

Diagonal variances are floored at a small positive value to prevent a state from becoming
unobservably certain and then unrecoverable. With `f32` these are not optional refinements;
they are what keeps a 15-state filter stable over a long flight.

## Equation-to-code mapping

Intended layout. Each implementing function cites its equation numbers in a doc comment.

| equations | concept | module | function |
| --------- | ------- | ------ | -------- |
| (1)–(4) | state definitions | `state.rs` | `NominalState`, `ErrorState` |
| (5)–(7) | bias correction, gravity | `propagate.rs` | `corrected_imu` |
| (8)–(11) | nominal propagation | `propagate.rs` | `propagate_nominal` |
| (12)–(15) | error dynamics | `propagate.rs` | `error_dynamics` |
| (16) | state transition matrix | `propagate.rs` | `transition_matrix` |
| (17) | discrete process noise | `propagate.rs` | `process_noise` |
| (18) | covariance propagation | `propagate.rs` | `propagate_covariance` |
| (19)–(23) | generic update, Joseph form | `update.rs` | `update` |
| (24) | GNSS position | `observation/gnss.rs` | `position_jacobian` |
| (25) | GNSS velocity | `observation/gnss.rs` | `velocity_jacobian` |
| (26) | barometric altitude | `observation/baro.rs` | `altitude_jacobian` |
| (27)–(29) | magnetometer | `observation/mag.rs` | `field_jacobian`, `heading_jacobian` |
| (30) | innovation gating | `update.rs` | `gate` |
| (31)–(33) | injection and reset | `update.rs` | `inject`, `reset` |
| (34) | symmetry enforcement | `math.rs` | `enforce_symmetry` |
| — | skew, quaternion exponential | `math.rs` | `skew`, `exp_quat` |

## References

* J. Solà, *Quaternion kinematics for the error-state Kalman filter*, [arXiv:1711.02508](https://arxiv.org/abs/1711.02508) — the primary source for the error-state formulation and the Jacobians above
* P. D. Groves, *Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems*, 2nd ed.
* J. A. Farrell, *Aided Navigation: GPS with High Rate Sensors*
* F. L. Markley and J. L. Crassidis, *Fundamentals of Spacecraft Attitude Determination and Control*
* [PX4 EKF2](https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf) — reference for practical behavior, not for derivations
