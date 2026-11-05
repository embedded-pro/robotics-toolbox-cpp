# Kanayama Trajectory Tracking

## Overview & Motivation

Path trackers such as pure pursuit follow a geometric path at whatever speed the robot happens to drive. Many tasks also fix **when** the robot should be where: a delivery robot keeping to a schedule, several robots moving in formation, or a base that must arrive in sync with an arm. These need **trajectory tracking**, where a reference pose and its velocity are given at every instant.

The Kanayama controller (Kanayama et al., 1990) is the classical solution for unicycle-type bases such as differential-drive and skid-steer robots. It expresses the tracking error in the robot's own frame and adds a feedback correction to the reference velocity. A Lyapunov function proves that the error converges to zero from any initial error while the reference keeps moving forward.

## Mathematical Theory

### Tracking Error

With the robot pose $T$ and the reference pose $T_r$ in SE(2), the reference seen from the robot is

$$T_e = T^{-1} T_r, \qquad e = (e_x,\; e_y,\; e_\theta)$$

$e_x$ is the along-track error, $e_y$ the cross-track error and $e_\theta \in (-\pi, \pi]$ the heading error, all in the robot frame.

### Control Law

Given the reference speed $v_r$ and turn rate $\omega_r$, with positive gains $k_x$, $k_y$ and $k_\theta$:

$$v = v_r \cos e_\theta + k_x\,e_x, \qquad \omega = \omega_r + v_r\left(k_y\,e_y + k_\theta \sin e_\theta\right)$$

With zero error the command is exactly the feedforward $(v_r, \omega_r)$.

### Stability

The candidate function

$$V = e_x^2 + e_y^2 + \frac{2\,(1 - \cos e_\theta)}{k_y}$$

is positive definite on $e_\theta \in (-\pi, \pi)$. Along the error dynamics of the unicycle it satisfies

$$\dot{V} = -k_x\,e_x^2 - \frac{v_r\,k_\theta}{k_y}\sin^2 e_\theta \le 0$$

whenever $v_r > 0$, so the error never grows and converges to zero.

### Tuning

Linearised about a straight reference, the lateral loop is a second-order system with natural frequency $v_r\sqrt{k_y}$ and damping ratio $k_\theta / (2\sqrt{k_y})$. The critically damped choice is

$$k_\theta = 2\sqrt{k_y}$$

## Complexity Analysis

| Operation      | Time   | Space  | Notes                                            |
|----------------|--------|--------|--------------------------------------------------|
| Tracking error | $O(1)$ | $O(1)$ | One SE(2) inverse and composition, one $\arctan$ |
| Control law    | $O(1)$ | $O(1)$ | One sine and one cosine                          |
| State          | —      | $O(1)$ | Three gains                                      |

## Step-by-Step Walkthrough

Gains $k_x = 1$ s⁻¹, $k_y = 4$ m⁻² and $k_\theta = 4$ m⁻¹ (critically damped), with reference speed $v_r = 0.5$ m/s:

1. **Zero error**: the command is the feedforward, $(0.5, 0.1)$ for a reference turning at $0.1$ rad/s.
2. **Along-track error**: a reference $0.2$ m ahead raises the speed to $0.5 + 0.2 = 0.7$ m/s.
3. **Cross-track error**: a reference $0.1$ m to the left gives $\omega = 0.5 \cdot 4 \cdot 0.1 = 0.2$ rad/s, turning towards it.
4. **Heading error**: a reference rotated $0.3$ rad gives $v = 0.5\cos 0.3 = 0.478$ m/s and $\omega = 0.5 \cdot 4 \sin 0.3 = 0.591$ rad/s.
5. **Robot frame**: a robot at $(1, 1)$ facing north with the reference at $(1, 2)$ sees an along-track error of $1$ m, not a lateral one, and speeds up to $1.5$ m/s.
6. **Closed loop**: starting at $(-0.3, 0.5, -0.4)$ with $V = 0.379$, the error falls below $10^{-4}$ within $20$ s, and $V$ decreases at every $10$ ms step.

## Pitfalls & Edge Cases

- **Stationary reference**: with $v_r = 0$ the cross-track error has no effect on the command. A unicycle cannot move sideways, so point stabilisation needs a time-varying or discontinuous law.
- **Reversed reference**: at $e_\theta = \pi$ the law drives backwards at $-v_r + k_x e_x$. The output stays bounded, but $V$ is no longer a valid Lyapunov function there.
- **Saturation**: the proof assumes the command is applied as computed. Saturate with a curvature-preserving scaling and keep the reference within about 70 % of the actuator limits.
- **Reference consistency**: $(v_r, \omega_r)$ must be the actual velocity of the reference pose, or the controller tracks a steady error.

## Variants & Generalizations

- **Samson's and de Luca's laws**: alternative Lyapunov designs with different error coordinates.
- **Dynamic extension**: backstepping through the robot's dynamics for torque-controlled bases.
- **Car-like vehicles**: the same structure with the steering angle as input and its limits.
- **Model-predictive tracking**: explicit handling of velocity and acceleration limits.

## Applications

- Time-parameterised trajectory following for differential-drive and skid-steer robots.
- Formation control, where each robot tracks a reference derived from a leader.
- Mobile manipulation, where the base must keep pace with an arm trajectory.

## Connections to Other Algorithms

- [Differential-Drive Kinematics](DifferentialDrive.md): maps $(v, \omega)$ to wheel speeds and saturates them while preserving curvature.
- [Skid-Steer Kinematics](SkidSteerKinematics.md): another unicycle-type base the controller drives.
- [Planar Odometry](PlanarOdometry.md): provides the pose the controller uses.
- [Pure-Pursuit Path Tracking](PurePursuit.md): the geometric alternative when timing does not matter.

## References & Further Reading

- Kanayama, Y., Kimura, Y., Miyazaki, F. and Noguchi, T. (1990). "A stable tracking control method for an autonomous mobile robot." *IEEE International Conference on Robotics and Automation*, 384–389.
- Siciliano, B., Sciavicco, L., Villani, L. and Oriolo, G. (2009). *Robotics: Modelling, Planning and Control*. Springer. Chapter 11.
- Samson, C. and Ait-Abderrahim, K. (1991). "Feedback control of a nonholonomic wheeled cart in Cartesian space." *IEEE International Conference on Robotics and Automation*, 1136–1141.
