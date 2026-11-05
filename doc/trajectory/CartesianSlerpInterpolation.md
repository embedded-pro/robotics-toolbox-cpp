# Cartesian Path with SLERP Orientation

## Overview & Motivation

Many tool motions are specified in Cartesian space rather than joint space: move straight along a weld seam, insert a peg along its axis, approach a part without sweeping through the fixture. A joint-space spline between the end poses produces a curved, unpredictable tool path.
A **Cartesian path** keeps the tool on a straight line, while its orientation rotates steadily from the start attitude to the goal attitude.

Orientation is the subtle part. Interpolating rotation matrices or Euler angles element-wise leaves the rotation group or produces uneven, gimbal-dependent motion.
**Spherical linear interpolation (SLERP)** of unit quaternions moves along the shortest great-circle arc at constant angular velocity about a fixed axis. Shoemake (1985) introduced it, and it is the standard way to blend attitudes.
Coupling position and orientation through one scalar time law makes both arrive together and gives the controller a consistent velocity feed-forward.

## Mathematical Theory

### Path Parameter

A scalar progress $s(t) \in [0, 1]$, produced by any time law (cubic or quintic polynomial, trapezoidal, S-curve), drives both channels. Its rate $\dot{s}$ scales every velocity.

### Position

$$p(s) = p_0 + s\,(p_1 - p_0), \qquad v = \dot{s}\,(p_1 - p_0)$$

### Orientation

With unit quaternions $q_0$, $q_1$ of the start and goal rotations, sign-fixed so that $q_0 \cdot q_1 \ge 0$ (both $q$ and $-q$ represent the same rotation, and the sign picks the short arc):

$$q(s) = \frac{\sin\left((1-s)\Omega\right)}{\sin\Omega}\,q_0 + \frac{\sin(s\Omega)}{\sin\Omega}\,q_1, \qquad \cos\Omega = q_0 \cdot q_1$$

For nearly identical rotations ($\cos\Omega \to 1$) the formula is ill-conditioned, so a normalized linear blend replaces it.

### Angular Velocity

The relative rotation $R_1 R_0^T$ has rotation vector $\phi\,\hat{k}$ (axis $\hat{k}$, angle $\phi \le \pi$). SLERP rotates about that fixed axis at a rate proportional to $\dot{s}$, so the angular velocity in the base frame is

$$\omega = \dot{s}\,\phi\,\hat{k}$$

The rotation vector is obtained from the relative quaternion $d = q_1 q_0^{*}$ as $\phi\,\hat{k} = 2\,\mathrm{atan2}(\|d_v\|, d_w)\,d_v/\|d_v\|$. This form is well-conditioned near $\phi \to 0$, where $\arccos$ loses precision, and it falls back to $2\,d_v$ as $\|d_v\| \to 0$. The twist $(v;\omega)$ is the feed-forward for task-space controllers.

## Complexity Analysis

| Phase    | Time   | Space  | Notes                                                                                  |
|----------|--------|--------|----------------------------------------------------------------------------------------|
| Planning | $O(1)$ | $O(1)$ | Two matrix-to-quaternion conversions, one quaternion product, one atan2                |
| Sampling | $O(1)$ | $O(1)$ | One time-law sample, a vector blend, one SLERP and one quaternion-to-matrix conversion |

## Step-by-Step Walkthrough

From the identity attitude at the origin to a $90°$ rotation about $z$ at $(1, 0, 0)$, with a linear time law ($s = t$, $\dot{s} = 1$, duration $1$ s):

1. $q_0 = (1, 0, 0, 0)$ and $q_1 = (\cos 45°, 0, 0, \sin 45°)$, so $q_0 \cdot q_1 > 0$ and no sign fix is needed. The relative rotation vector is $(0, 0, \pi/2)$.
2. At $t = 0.5$: the position is $(0.5, 0, 0)$, and SLERP gives $q = (\cos 22.5°, 0, 0, \sin 22.5°) = (0.92388, 0, 0, 0.38268)$, a $45°$ rotation.
3. The twist is constant: $v = (1, 0, 0)$ m/s and $\omega = (0, 0, \pi/2)$ rad/s.
4. With a goal of $270°$ about $z$, the quaternions have a negative dot product. After the sign fix the arc goes $-90°$, the midpoint is $-45°$ and $\omega = (0, 0, -\pi/2)$.
5. With a rest-to-rest quintic law over $2$ s, $\dot{s}(1) = 15/16 = 0.9375$, so the mid-path twist is $(0.9375, 0, 0;\ 0, 0, 1.4726)$, and both the velocity and the angular velocity vanish at the ends.

## Pitfalls & Edge Cases

- **Quaternion double cover**: without the sign fix, SLERP takes the long way round ($270°$ instead of $90°$), and the twist no longer matches the motion.
- **Twist consistency**: compute the rotation vector **after** the sign fix, from the same pair of quaternions that SLERP blends.
- **Near-identical orientations**: $\sin\Omega \to 0$ makes the SLERP weights ill-conditioned; use the normalized linear blend there.
- **$180°$ rotations**: both arcs are equally short and the choice is arbitrary but consistent; plan through an intermediate pose if a specific direction matters.
- **Joint-space feasibility**: a straight Cartesian line can pass through a singularity or outside the workspace even when both ends are reachable. Check the path with inverse kinematics and manipulability before execution.
- **Time-law limits act on $s$**: the Cartesian speed $|v| = \dot{s}|\Delta p|$ and angular speed $|\omega| = \dot{s}\phi$ follow from the limits on $\dot{s}$; scale them per segment.

## Variants & Generalizations

- **Screw (SE(3)) interpolation**: $T(s) = T_0 \exp(s \log(T_0^{-1} T_1))$ moves along a helix, coupling translation and rotation. It is natural for some tasks but not a straight line.
- **SQUAD and quaternion splines**: smooth blending through several attitudes with continuous angular velocity.
- **Via-point blending**: polynomial blends between consecutive Cartesian segments avoid stopping at each corner.
- **Separate time laws**: decoupled position and orientation schedules, with arrival synchronized by scaling the shorter one.

## Applications

- Straight-line tool moves: welding, gluing, insertion, approach and retreat.
- Camera and sensor pointing with smooth, constant-axis attitude changes.
- Feed-forward twists for operational-space and impedance control.
- Teleoperation and teaching pendants that move the tool in Cartesian jog modes.

## Connections to Other Algorithms

- [Cubic Spline Trajectory](CubicSplineTrajectory.md): the joint-space counterpart for via points.
- [Geometric Jacobian](../kinematics/GeometricJacobian.md): maps the Cartesian twist to joint rates, and its transpose maps task forces back to torques.
- [Inverse Kinematics](../kinematics/InverseKinematics.md): converts sampled poses into joint targets.
- The quaternion SLERP, rigid transforms and polynomial time laws come from [numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp).

## References & Further Reading

- Shoemake, K. (1985). "Animating rotation with quaternion curves." *Proc. SIGGRAPH*, 19(3), 245–254.
- Lynch, K.M. and Park, F.C. (2017). *Modern Robotics*. Cambridge University Press. Chapter 9.
- Biagiotti, L. and Melchiorri, C. (2008). *Trajectory Planning for Automatic Machines and Robots*. Springer. Chapter 8.
