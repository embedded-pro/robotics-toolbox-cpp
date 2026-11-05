# Planar Odometry on SE(2)

## Overview & Motivation

Every wheeled robot estimates where it is by **dead reckoning**: it integrates its measured body velocity, from wheel encoders and sometimes a gyro, into a pose. The result drifts, but it is smooth, fast and always available, and it feeds every localization filter as the prediction step.

The usual way to integrate, Euler or Runge–Kutta on $(\dot{x}, \dot{y}, \dot{\theta})$, treats the heading as constant within a step. When the robot turns, that biases the position. A constant body twist held for one period moves the robot along a **circular arc**. Integrating with the SE(2) exponential lands exactly on that arc, so turning inside a step introduces no error.

## Mathematical Theory

### Exact Arc Integration

With the pose $T_k \in SE(2)$ and the body twist $\xi = (v_x, v_y, \omega)$ held over a period $\Delta t$:

$$T_{k+1} = T_k \exp(\xi\,\Delta t)$$

The exponential moves the body by $V(\omega\Delta t)\,(v_x, v_y)\,\Delta t$ in its own frame and turns it by $\omega\Delta t$. For $\omega = 0$ it is a straight segment. The update is exact for any step size as long as the twist is constant, so one step of $2\pi$ s along a radius-2 arc gives the same pose as a thousand small steps.

### Encoder Increments

The wheel models map wheel speeds linearly to body twists. Applied to wheel-angle increments instead, they give the body displacement $\Delta = \xi\,\Delta t$ directly, which is integrated as $\exp(\Delta)$ with unit time. No division by a jittery sample period is needed.

### Renormalisation

The rotation is a unit complex number. Rounding in each composition makes $\cos^2 + \sin^2$ drift by about one unit in the last place, and long runs accumulate it. Rescaling to unit length after every step keeps the rotation valid indefinitely.

### Odometry Increments

Motion models in localization filters consume the body-frame motion between two odometry readings:

$$\Delta = \log\left(T_{\text{from}}^{-1}\, T_{\text{to}}\right)$$

This is the twist that, held for unit time, carries one reading into the other. It requires the heading change between readings to stay below $\pi$.

### Increment Noise

The noise of an increment grows with the distance travelled $\Delta s$ and the rotation $\Delta\theta$:

$$\sigma_{xy}^2 = \alpha_3\,\Delta s^2 + \alpha_4\,\Delta\theta^2, \qquad \sigma_\theta^2 = \alpha_1\,\Delta\theta^2 + \alpha_2\,\Delta s^2$$

Applying Thrun's odometry noise model to the body-frame increment, rather than to a rotate–translate–rotate decomposition, keeps it continuous when the robot reverses.

## Complexity Analysis

| Operation           | Time   | Space  | Notes                                            |
|---------------------|--------|--------|--------------------------------------------------|
| Integrate one step  | $O(1)$ | $O(1)$ | One exponential, one composition, one hypotenuse |
| Increment           | $O(1)$ | $O(1)$ | One inverse, one composition, one logarithm      |
| Increment deviation | $O(1)$ | $O(1)$ | Two square roots                                 |
| State               | —      | $O(1)$ | Four scalars                                     |

## Step-by-Step Walkthrough

1. **Straight line**: four steps of $(1, 0, 0)$ for $0.25$ s each end at $(1, 0, 0)$.
2. **Quarter arc in one step**: the twist $(0.5, 0, 0.25)$ describes a radius-2 circle. One step of $2\pi$ s lands exactly at $(2, 2, \pi/2)$.
3. **Full circle**: seven steps of $8\pi/7$ s, or a thousand steps of $8\pi/1000$ s, both close the circle back at the origin.
4. **Increment**: starting at $(1, 2, 0.3)$ and integrating $(0.4, 0.1, -0.6)$ for $0.5$ s, the recovered increment is $(0.2, 0.05, -0.3)$.
5. **Noise**: with $\alpha = (0.05, 0.01, 0.05, 0.01)$, an increment of $0.5$ m and $0.1$ rad has standard deviations $0.112$ m in position and $0.055$ rad in heading. Driving backwards by the same distance gives the same values.

## Pitfalls & Edge Cases

- **Changing twist**: the update is exact only for a constant twist. For a twist that changes during the step, use the mean of the twists at both ends.
- **Slow sampling**: increments through the logarithm wrap if the robot turns more than half a revolution between readings. Sample faster.
- **Unbounded drift**: odometry error grows without limit. Wheel-radius and track-width errors dominate it and must be calibrated in the wheel model, not here.
- **Skipping renormalisation**: long runs without it let the rotation lose unit length and distort every later pose.
- **Zero period**: a step of zero length leaves the pose unchanged.

## Variants & Generalizations

- **Gyro-aided odometry**: replace the wheel-derived turn rate with a gyroscope's, which suffers far less from slip.
- **Odometry fusion**: an extended Kalman filter combining wheel and inertial measurements.
- **Three-dimensional odometry**: the same exponential update on SE(3) for legged or aerial platforms.
- **Higher-order integration**: Magnus-series updates for twists that vary within a step.

## Applications

- Dead reckoning for differential-drive, car-like, omnidirectional and skid-steer robots.
- The prediction step of landmark, Monte Carlo and scan-matching localization.
- Short-term pose tracking between absolute fixes such as GNSS or visual markers.

## Connections to Other Algorithms

- The SE(2) exponential, logarithm and composition come from [numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp).

## References & Further Reading

- Thrun, S., Burgard, W. and Fox, D. (2005). *Probabilistic Robotics*. MIT Press. Chapter 5.
- Borenstein, J. and Feng, L. (1996). "Measurement and correction of systematic odometry errors in mobile robots." *IEEE Transactions on Robotics and Automation*, 12(6), 869–880.
- Solà, J., Deray, J. and Atchuthan, D. (2018). "A micro Lie theory for state estimation in robotics." arXiv:1812.01537.
