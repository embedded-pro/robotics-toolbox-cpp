# Odometry Fusion (Wheel and Gyroscope EKF on SE(2))

## Overview & Motivation

Wheel odometry measures distance well but heading poorly: a few millimetres of wheel-radius error or a little slip turn into large heading drift. A MEMS gyroscope measures rotation well but has a bias that drifts with temperature, and integrating that bias produces a steadily growing heading error.

This filter combines the two in the way each sensor is good at: **the gyroscope for rotation, the wheels for distance**. It estimates the gyroscope bias from the agreement between the wheel and gyroscope yaw rates, and refuses that comparison when the wheels slip. Optional absolute heading (compass, dual-antenna GNSS) and position (GNSS, scan matching) measurements go through the same update path.

## Mathematical Theory

### State and Motion Model

The state is the planar pose and the gyroscope bias, $x = (p_x, p_y, \theta, b_g)$. Given the forward speed $v$ from a wheel model and the raw gyroscope rate $\omega_g$, the corrected yaw rate is $\omega = \omega_g - b_g$. The mean moves along the exact SE(2) arc:

$$T_{k+1} = T_k \exp\!\big((v, 0, \omega)\,\Delta t\big), \qquad b_{g,k+1} = b_{g,k}$$

### Covariance Propagation

The Jacobians use the midpoint heading $\theta_m = \theta + \omega\Delta t/2$. With respect to the state:

$$F = \begin{pmatrix} 1 & 0 & -v\Delta t\sin\theta_m & \tfrac{1}{2}v\Delta t^2\sin\theta_m \\ 0 & 1 & v\Delta t\cos\theta_m & -\tfrac{1}{2}v\Delta t^2\cos\theta_m \\ 0 & 0 & 1 & -\Delta t \\ 0 & 0 & 0 & 1 \end{pmatrix}$$

The inputs $(v, \omega_g)$ enter through $G = \partial f / \partial(v, \omega_g)$. The speed noise grows with speed, $\sigma_v = \sigma_{v,0} + k_v |v|$, and the bias follows a random walk:

$$P \leftarrow F P F^T + G\,\mathrm{diag}(\sigma_v^2, \sigma_g^2)\,G^T + \mathrm{diag}(0, 0, 0, \sigma_b^2 \Delta t)$$

### Measurement Updates

Every update is a residual-form Kalman update with a Joseph-form covariance and an optional χ² gate. The heading is re-wrapped afterwards.

| Update            | Residual                                 | Jacobian $H$          | Noise                     | Gate      |
|-------------------|------------------------------------------|-----------------------|---------------------------|-----------|
| Wheel yaw rate    | $\omega_w - (\omega_g - b_g)$            | $(0, 0, 0, -1)$       | $\sigma_w^2 + \sigma_g^2$ | slip gate |
| Absolute heading  | $\operatorname{wrap}(\theta_m - \theta)$ | $(0, 0, 1, 0)$        | $\sigma_\theta^2$         | none      |
| Absolute position | $p_m - (p_x, p_y)$                       | first two rows of $I$ | $\Sigma_p$                | none      |

Both yaw rates measure the same true rotation, so their difference observes the bias. The gyroscope sample appears in the predicted rate, which is why its variance is added to the wheel noise.

### Slip Detection

When the wheels slip, for example spinning on a smooth floor or with a stuck caster, the wheel yaw rate disagrees with the gyroscope far beyond its noise. The normalised innovation squared then exceeds the χ²(1) gate. The update is rejected, the state is untouched, and the returned status lets the caller flag the slip.

## Complexity Analysis

| Operation       | Time   | Space  | Notes                                                   |
|-----------------|--------|--------|---------------------------------------------------------|
| Predict         | $O(1)$ | $O(1)$ | One SE(2) exponential and a $4 \times 4$ congruence     |
| Scalar update   | $O(1)$ | $O(1)$ | One $1 \times 1$ Cholesky and a $4 \times 1$ gain       |
| Position update | $O(1)$ | $O(1)$ | One $2 \times 2$ Cholesky and a $4 \times 2$ gain       |
| State           | —      | $O(1)$ | 20 scalars of state and covariance plus the noise block |

## Step-by-Step Walkthrough

Noise floor $0.01$ m/s, speed gain $5$ %, gyroscope noise $0.01$ rad/s, bias random walk $10^{-4}$, wheel yaw-rate noise $0.02$ rad/s, slip gate $6.63$ (99 %), starting at the origin with variances $(0.01, 0.01, 0.01, 0.0025)$:

1. **Straight drive**: one metre at $1$ m/s. The along-track variance barely grows, but the cross-track variance doubles to $0.0206$ through the heading uncertainty, which itself grows to $0.0125$ from the bias.
2. **Arc**: $1$ m/s and $0.5$ rad/s for $2$ s end at $(1.683, 0.919, 1.0)$, exactly the SE(2) exponential of the twist.
3. **Standstill**: a gyroscope with a $0.02$ rad/s bias, compared with the zero wheel yaw rate at $100$ Hz, gives a bias estimate of $0.019992$ after $5$ s. The heading error stays below $10^{-4}$ rad instead of drifting to $0.1$ rad.
4. **Slip**: the wheels report $1$ rad/s while the gyroscope reads zero. The NIS is $333$, far beyond the gate, so the update is rejected.
5. **Heading across $\pm\pi$**: an estimate of $3.1$ rad corrected by a compass reading of $-3.1$ rad moves through the seam to $-3.117$ rad.
6. **Position fix**: a GNSS fix at $(0.1, -0.05)$ with the same variance as the estimate pulls the position halfway and halves its variance, with NIS $0.625$.

## Pitfalls & Edge Cases

- **Bias observability**: the bias is only observable through the wheel comparison or absolute heading. Long stretches of slip leave it unobserved, and its variance grows with the random walk.
- **Gate locking**: after a long slip the filter may disagree with the wheels for a while. Monitor the gating rate, and raise the speed noise gain while slipping.
- **Wheel model errors**: a miscalibrated track width biases the wheel yaw rate, which the filter then reads as gyroscope bias. Calibrate the wheel model first.
- **Lateral slip**: the motion model has no lateral velocity. Skid-steer platforms turning hard violate it; an accelerometer test of $a_y \approx v\omega$ can flag that.
- **Time alignment**: the wheel and gyroscope samples must refer to the same interval, or the bias estimate absorbs the delay.

## Variants & Generalizations

- **Error-state filters**: propagate the error on the SE(2) manifold for large uncertainties.
- **Accelerometer aiding**: lateral-slip detection and pitch compensation on slopes.
- **Wheel calibration states**: estimating wheel radii and track width online.
- **Smoothing**: fixed-lag smoothers for post-processing logged runs.

## Applications

- Dead reckoning for indoor service robots and warehouse vehicles.
- Heading stabilisation for agricultural and outdoor robots between GNSS fixes.
- The motion model for landmark-based and Monte Carlo localization.

## Connections to Other Algorithms

- [Planar Odometry](../mobile/PlanarOdometry.md): the exact SE(2) arc integration used for the mean.
- [Differential-Drive Kinematics](../mobile/DifferentialDrive.md) and [Skid-Steer Kinematics](../mobile/SkidSteerKinematics.md): provide the wheel speed and yaw rate.
- The residual-form Kalman update, SE(2) transform and angle wrapping come from [numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp).

## References & Further Reading

- Borenstein, J. and Feng, L. (1996). "Gyrodometry: a new method for combining data from gyros and odometry in mobile robots." *IEEE International Conference on Robotics and Automation*, 423–428.
- Thrun, S., Burgard, W. and Fox, D. (2005). *Probabilistic Robotics*. MIT Press. Chapters 3 and 5.
- Barfoot, T.D. (2017). *State Estimation for Robotics*. Cambridge University Press. Chapter 4.
