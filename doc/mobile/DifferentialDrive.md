# Differential-Drive Kinematics

## Overview & Motivation

Most indoor robots, from vacuum cleaners to warehouse carriers, drive on **two independently powered wheels on one axle**, with passive casters for balance. Steering comes purely from the speed difference between the wheels: equal speeds drive straight, opposite speeds spin in place.

The kinematics are linear and cheap, but two details matter in practice. The wheels cannot slide sideways, so the robot moves like a unicycle with no lateral velocity. And when a command exceeds the motor limits, how it is saturated decides whether the robot still follows the planned curve or veers off it.

## Mathematical Theory

### Forward Kinematics

With wheel radius $r$, track width $B$ between the contact points, and wheel angular speeds $\omega_L$, $\omega_R$, the body twist at the axle midpoint is

$$v = r\,\frac{\omega_L + \omega_R}{2}, \qquad v_y = 0, \qquad \omega = r\,\frac{\omega_R - \omega_L}{B}$$

The lateral velocity is zero because the wheels roll without slipping sideways (the nonholonomic constraint).

### Inverse Kinematics

$$\omega_L = \frac{v - \omega B/2}{r}, \qquad \omega_R = \frac{v + \omega B/2}{r}$$

### Saturation That Preserves the Path

When either wheel would exceed $\omega_{\max}$, both are scaled by the same factor:

$$k = \frac{\omega_{\max}}{\max(|\omega_L|, |\omega_R|)}, \qquad (\omega_L, \omega_R) \leftarrow k\,(\omega_L, \omega_R)$$

Because $v$ and $\omega$ scale together, the curvature $\omega / v$ is unchanged. The robot follows the same arc, only more slowly. Clipping each wheel separately would change the curvature.

### Speed Envelope

The feasible commands form a diamond in the $(v, \omega)$ plane:

$$\frac{|v|}{v_{\max}} + \frac{|\omega|}{\omega_{\text{body},\max}} \le 1, \qquad v_{\max} = r\,\omega_{\max}, \qquad \omega_{\text{body},\max} = \frac{2 r\,\omega_{\max}}{B}$$

Saturating by a common factor projects an infeasible command onto the diamond along the ray through the origin.

## Complexity Analysis

| Operation         | Time   | Space  | Notes                                  |
|-------------------|--------|--------|----------------------------------------|
| Forward / inverse | $O(1)$ | $O(1)$ | A few multiplications, no trigonometry |
| Saturation        | $O(1)$ | $O(1)$ | One maximum and one division           |
| State             | —      | $O(1)$ | Radius, track width and wheel limit    |

## Step-by-Step Walkthrough

A robot with $r = 5$ cm, $B = 30$ cm and $\omega_{\max} = 20$ rad/s:

1. **Forward**: wheels at $(10, 14)$ rad/s give $v = 0.6$ m/s and $\omega = 0.667$ rad/s.
2. **Spin in place**: $(v, \omega) = (0, 2)$ needs the wheels at $(-6, 6)$ rad/s.
3. **Saturation**: $(1.2, 2)$ needs $(18, 30)$ rad/s, beyond the limit. Scaling by $20/30$ gives $(12, 20)$ rad/s, which is $(0.8, 1.333)$. The curvature $\omega / v = 1.667$ is the same before and after.
4. **Envelope corners**: the top linear speed is $1$ m/s and the top turn rate is $6.667$ rad/s.

## Pitfalls & Edge Cases

- **Per-wheel clipping**: saturating wheels independently bends the path and makes path trackers overshoot. Scale both wheels together.
- **Calibration**: odometry accuracy depends directly on $r$ (possibly different left and right) and $B$. Calibrate them, for example with the UMBmark square-path test.
- **Slip**: on carpet, ramps or under acceleration the no-slip assumption breaks. Use a skid-steer model or fuse a gyroscope.
- **Reversing**: negative $v$ maps symmetrically. The saturation uses absolute values.
- **Zero command**: scaling with both wheels at rest returns zeros without dividing.

## Variants & Generalizations

- **Skid-steer**: four or more wheels or tracks that must slip to turn, modelled with instantaneous centres of rotation.
- **Dynamic models**: motor torque, inertia and traction limits instead of pure kinematics.
- **Unicycle control**: path trackers designed on the $(v, \omega)$ model and mapped to wheels through the inverse kinematics.

## Applications

- Indoor service and logistics robots.
- Educational and research platforms.
- The mobile base of mobile manipulators.

## Connections to Other Algorithms

- [Planar Odometry](PlanarOdometry.md): integrates the body twist, or wheel-angle increments mapped through the forward kinematics, into a pose.

## References & Further Reading

- Siegwart, R., Nourbakhsh, I.R. and Scaramuzza, D. (2011). *Introduction to Autonomous Mobile Robots*, 2nd ed. MIT Press. Chapter 3.
- Lynch, K.M. and Park, F.C. (2017). *Modern Robotics*. Cambridge University Press. Chapter 13.
- Borenstein, J. and Feng, L. (1996). "Measurement and correction of systematic odometry errors in mobile robots." *IEEE Transactions on Robotics and Automation*, 12(6), 869–880.
