# Parallel Manipulator Kinematics (Stewart–Gough and Delta)

## Overview & Motivation

A **parallel manipulator** connects its moving platform to the base through several kinematic chains at once. Load is shared between the legs, so parallel robots are stiff, light and fast: flight simulators and machining hexapods use the **Stewart–Gough platform**, and pick-and-place lines use the **Delta robot** at several hundred picks per minute.

Their kinematics mirror serial arms. **Inverse** kinematics, from platform pose to actuator values, is easy: each leg is independent. **Forward** kinematics is hard in general, with many assembly modes and no closed form for the hexapod.
This module provides closed-form inverse kinematics for both mechanisms, the hexapod's leg Jacobian with Newton forward kinematics on SE(3), and closed-form forward kinematics for the Delta.

## Mathematical Theory

### Stewart–Gough Platform

Six prismatic legs connect base anchors $b_i$ (base frame) to platform anchors $p_i$ (platform frame). For a platform pose $(R, p)$:

$$\ell_i = \left\| R\,p_i + p - b_i \right\|$$

**Leg Jacobian.** With the unit leg vector $n_i$ and the rotated anchor $a_i = R\,p_i$, the leg rates for a platform twist $(v; \omega)$, expressed in the base frame at the platform origin, are

$$\dot{\ell}_i = n_i^T v + (a_i \times n_i)^T \omega, \qquad \dot{\ell} = J\begin{bmatrix} v \\ \omega \end{bmatrix}$$

$J$ is the platform's inverse Jacobian. By statics, $J^T$ maps leg forces to the platform wrench.

**Forward kinematics.** Given measured lengths $L$, solve $\ell(T) - L = 0$ for the pose with Newton's method on SE(3). Each step is $\delta = -J^{-1}(\ell(T) - L)$, retracted as

$$R \leftarrow \exp([\delta_\omega]_\times)\,R, \qquad p \leftarrow p + \delta_v$$

A damping $\lambda$ turns the step into Levenberg–Marquardt, $\delta = -J^T(JJ^T + \lambda^2 I)^{-1}(\ell - L)$, which stays finite where $J$ is singular. A hexapod has up to 40 assembly modes, and the initial guess selects which one is found.

### Delta Robot

Three revolute shoulders at azimuths $\phi_i = 0°, 120°, 240°$ on a base circle of radius $R$ drive upper arms of length $r_f$. Parallelogram forearms of length $r_e$ keep the effector parallel to the base, attached at radius $r$ on the effector. The effector therefore translates only.

**Inverse kinematics.** In arm $i$'s frame, with $X = x' + r - R$, the elbow must lie both on the shoulder circle and on the forearm sphere around the attachment point:

$$a\cos\theta + b\sin\theta = c, \qquad a = 2r_f X, \quad b = -2r_f z, \quad c = X^2 + y'^2 + z^2 + r_f^2 - r_e^2$$

so $\theta = \operatorname{atan2}(b, a) - \arccos\left(c / \sqrt{a^2 + b^2}\right)$, the elbow-out root. The point is unreachable when $|c| > \sqrt{a^2 + b^2}$.

**Forward kinematics.** Shifting each elbow inward by the effector radius turns the problem into intersecting three spheres of radius $r_e$ centred at

$$C_i = R_z(\phi_i)\begin{bmatrix} R - r + r_f\cos\theta_i \\ 0 \\ -r_f\sin\theta_i \end{bmatrix}$$

Trilateration in the frame spanned by the three centres gives two mirror solutions. The lower one is the physical effector. No solution exists when the spheres do not meet.

## Complexity Analysis

| Operation            | Time             | Space   | Notes                                      |
|----------------------|------------------|---------|--------------------------------------------|
| Hexapod inverse      | $O(1)$           | $O(1)$  | Six vector norms                           |
| Hexapod leg Jacobian | $O(1)$           | $O(1)$  | Six unit vectors and cross products        |
| Hexapod forward      | $O(k \cdot 6^3)$ | $O(36)$ | $k$ Newton iterations, one 6×6 solve each  |
| Delta inverse        | $O(1)$           | $O(1)$  | Three `atan2`/`acos` pairs                 |
| Delta forward        | $O(1)$           | $O(1)$  | Closed-form trilateration, one square root |

## Step-by-Step Walkthrough

**Hexapod.** Base anchors sit on a unit circle and platform anchors on a radius-$0.6$ circle, paired at $\pm 15°$ and $\pm 45°$ around three sectors.

1. At the home pose, platform $0.8$ m above the base, all six legs measure $0.980$ m. Raising it to $0.9$ m lengthens every leg to $1.063$ m.
2. A tilted pose, yaw $0.1$ rad and roll $0.05$ rad at $(0.05, -0.03, 0.85)$, gives legs from $0.956$ to $1.083$ m.
3. Newton forward kinematics recovers that pose from a guess at yaw $0.15$ rad within six iterations.
4. The same home lengths solved from a guess below the base converge to the mirror assembly at $z = -0.8$ m.
5. At yaw $90°$ the leg Jacobian is singular. With damping $\lambda = 10^{-3}$ the leg residual still falls below $10^{-4}$ m, while the yaw itself is poorly determined.

**Delta.** The geometry is $R = 0.1$, $r = 0.03$, $r_f = 0.2$ and $r_e = 0.45$ m.

1. With all arms horizontal, the effector hangs on the axis at $z = -\sqrt{0.45^2 - 0.27^2} = -0.36$ m.
2. The point $(0.05, -0.03, -0.40)$ requires arm angles $(0.0425, 0.3712, 0.2057)$ rad.
3. Forward and inverse kinematics round-trip over a grid of $4^3$ arm configurations to $10^{-4}$ rad.
4. The point $(0, 0, -0.9)$ is beyond reach, so the inverse returns no solution.

## Pitfalls & Edge Cases

- **Assembly modes**: hexapod forward kinematics converges to the mode nearest the guess. Seed it with the previous cycle's pose.
- **Singularities**: where the leg Jacobian loses rank, the platform gains an uncontrollable motion and Newton steps blow up. Use damping and avoid such poses in the workspace design.
- **Delta branch**: the elbow-out root is selected. Mechanisms built elbow-in need the other root.
- **Reach boundary**: on the boundary, $|c| = \sqrt{a^2 + b^2}$ and the two roots coincide. The arccos argument is clamped against rounding.
- **Effector convention**: the Delta effector lies below the base ($z < 0$). Mounting it upright flips the sign convention.

## Variants & Generalizations

- **Other parallel mechanisms**: 3-RPR planar platforms, hexaglides, and the 4-axis Delta with a rotary wrist.
- **Closed-form hexapod forward kinematics**: special geometries with coincident anchors admit polynomial solutions.
- **Dynamics**: virtual-work and Lagrangian formulations built on the leg Jacobian.
- **Calibration**: identification of anchor positions from measured leg lengths and poses.

## Applications

- Motion simulators, vibration isolation and precision positioning (hexapods).
- High-speed pick-and-place and packaging (Delta robots).
- Machine-tool spindle heads and telescope secondary-mirror alignment.

## Connections to Other Algorithms

- [Pose Inverse Kinematics](PoseInverseKinematics.md): the same Levenberg–Marquardt pattern applied to a serial arm.
- [Geometric Jacobian](GeometricJacobian.md): the serial counterpart of the leg Jacobian.
- [Analytical Inverse Kinematics (OPW)](AnalyticalIkOpw.md): another closed-form solver with several solution branches.
- The SE(3) types and the Levenberg–Marquardt solver come from [numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp).

## References & Further Reading

- Stewart, D. (1965). "A platform with six degrees of freedom." *Proceedings of the Institution of Mechanical Engineers*, 180(1), 371–386.
- Clavel, R. (1988). "DELTA, a fast robot with parallel geometry." *Proceedings of the 18th International Symposium on Industrial Robots*, 91–100.
- Merlet, J.-P. (2006). *Parallel Robots*, 2nd ed. Springer.
- Husty, M.L. (1996). "An algorithm for solving the direct kinematics of general Stewart–Gough platforms." *Mechanism and Machine Theory*, 31(4), 365–379.
