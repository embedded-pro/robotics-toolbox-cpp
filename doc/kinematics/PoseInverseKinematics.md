# Pose Inverse Kinematics

## Overview & Motivation

Position-only inverse kinematics places the tool point but ignores how the tool is oriented. Grasping, welding, screwing and camera pointing all need the full **pose**: position and orientation together, six constraints for a six-joint arm.

This solver finds joint angles that reach a target pose. It works on any arm that exposes a geometric Jacobian and a tool pose (link chain, Denavit–Hartenberg or product of exponentials). It iterates damped least-squares steps on a properly weighted log-map pose error.
Damping adapts to the arm's manipulability, so iterations are fast Newton steps away from singularities and stay bounded near them. Step clamping and joint limits keep every iterate safe to command.

## Mathematical Theory

### Pose Error

For a target $T^* = (R^*, p^*)$ and the current tool pose $T(q) = (R, p)$, the error is

$$e = \begin{bmatrix} p^* - p \\ \log\left(R^* R^T\right)^\vee \end{bmatrix}$$

The rotation part is the true log map: a rotation vector $\phi\hat{n}$ with $\phi \le \pi$, taken the short way round. The vector part of an error quaternion, $2\sin(\phi/2)\hat{n}$, is only a small-angle approximation; at $\phi = 1$ rad it is already $4\%$ short.

### Weighting

Position errors are in metres and rotation errors in radians. A **characteristic length** $\rho$, of the order of the arm's reach or tool length, makes them commensurate:

$$W = \operatorname{diag}(1, 1, 1, \rho, \rho, \rho)$$

The stopping test is $\|We\| < \epsilon$, and the step is computed on the weighted Jacobian $WJ$.

### Damped Least-Squares Step

Each iteration solves

$$\Delta q = J^T W \left(W J J^T W + \lambda^2 I\right)^{-1} W e$$

then clamps $\|\Delta q\| \le \Delta_{\max}$ and projects $q + \Delta q$ onto the joint limits. With $\lambda = 0$ and an invertible $J$, the weights cancel and the step is Newton's $J^{-1}e$.

**Damping does not bias the answer.** At a fixed point, $(WJ)^T y = 0$ with $WJ$ of full row rank forces $y = 0$ and hence $e = 0$. For a reachable, non-singular target the iteration therefore converges to the exact pose, and damping only slows it from quadratic to linear convergence.

### Adaptive Damping

With the weighted manipulability $w = \sqrt{\det(WJJ^TW)}$ and a threshold $w_0$ (Nakamura–Hanafusa, Chiaverini):

$$\lambda^2 = \begin{cases} \lambda_0^2 \left(1 - (w/w_0)^2\right) & w < w_0 \\ 0 & w \ge w_0 \end{cases}$$

Damping is zero away from singularities and rises smoothly to $\lambda_0$ as $w \to 0$. With $w_0 = 0$, the damping is the constant $\lambda_0$.

## Complexity Analysis

| Step                   | Time         | Space   | Notes                                                    |
|------------------------|--------------|---------|----------------------------------------------------------|
| Tool pose and Jacobian | $O(n)$       | $O(n)$  | Supplied by the arm model                                |
| Weighted normal matrix | $O(36n)$     | $O(36)$ | $WJ\,(WJ)^T$, $6 \times 6$                               |
| Damped solve           | $O(6^3)$     | $O(36)$ | One LU factorization                                     |
| Adaptive damping       | $O(36n+6^3)$ | $O(36)$ | One more normal matrix and LU determinant when $w_0 > 0$ |
| Final error            | $O(n)$       | $O(1)$  | One extra tool pose after the last update                |

## Step-by-Step Walkthrough

A PUMA-560 (standard DH) with target pose $T^* = T(q^\ast)$, $q^\ast = (0.3, -0.4, 0.5, 0.6, -0.7, 0.8)$, has the tool at $p = (0.402, -0.033, 0.264)$ m. The solver starts from $q^\ast + 0.1$ with $\rho = 0.3$ m.

1. **Constant damping $\lambda_0 = 0.05$**: $\|We\| < 10^{-4}$ after 3 iterations.
2. **Heavier damping $\lambda_0 = 0.2$**: 14 iterations to $10^{-5}$ instead of 4, but the same joint angles to within $10^{-3}$. Damping costs speed, not accuracy.
3. **One weighted step**: with $J = I$, $\rho = 0.1$ and $\lambda = 0.1$, the error $(0.1, 0, 0, 0, 0, 0.1)$ gives $\Delta q_i = w_i^2 e_i / (w_i^2 + \lambda^2)$, i.e. $0.0990$ on the position row and $0.05$ on the rotation row.
4. **Wrist singularity**: starting at $\theta_5 = 0$, where the wrist axes align, adaptive damping with $w_0 = 10^{-3}$ and $\Delta_{\max} = 0.2$ converges in 8 iterations.
5. **Unreachable target**: moving the target to $(3, 0, 0)$ m, beyond the arm's reach, stops at the iteration limit with a finite configuration and a residual of about $3.8$.

## Pitfalls & Edge Cases

- **Approximate orientation errors**: quaternion-vector or Euler-angle differences distort large rotations and can stall convergence. Use the log map.
- **Unweighted error**: mixing metres and radians unscaled makes the tolerance and the damping meaningless for one of them. Pick $\rho$ from the arm's dimensions.
- **Antipodal orientations**: at $\phi = \pi$ the rotation axis is ambiguous. Either sign is valid, and the iteration picks one.
- **Joint limits**: clamping is a projected iteration. A target that needs a joint beyond its limit ends with a non-zero residual rather than a violation.
- **Redundant arms**: with seven or more joints the same code returns the minimum-norm step, which drifts in the null space. Add a null-space objective to steer it.
- **Warm start**: seed $q_0$ from the previous control cycle. Far-away seeds may converge to a different solution branch (elbow up or down, wrist flipped).

## Variants & Generalizations

- **Levenberg–Marquardt with trust region**: adapt $\lambda$ from the actual versus predicted error reduction.
- **Selectively damped least squares**: damp each singular direction separately.
- **Quadratic-programming IK**: handle joint limits, velocity limits and collision constraints as inequalities.
- **Analytical solutions**: closed forms for arms with spherical wrists, used to seed or replace the iteration.

## Applications

- Converting Cartesian waypoints into joint targets for motion planning.
- Task-space teleoperation and jogging.
- Grasp and tool-placement queries during planning.
- Calibration routines that drive the tool to measured poses.

## Connections to Other Algorithms

- [Inverse Kinematics](InverseKinematics.md): the position-only damped least-squares solver.
- [Geometric Jacobian](GeometricJacobian.md): supplies the Jacobian through the shared provider interface.
- [Denavit-Hartenberg Parameters](DenavitHartenberg.md): the PUMA-560 model used in the walkthrough.
- [Manipulability Index](ManipulabilityIndex.md): the measure driving the adaptive damping.
- [Redundancy Resolution](RedundancyResolution.md): steers the null-space motion of redundant arms.
- The pose error and the Levenberg–Marquardt iteration come from [numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp).

## References & Further Reading

- Nakamura, Y. and Hanafusa, H. (1986). "Inverse kinematic solutions with singularity robustness for robot manipulator control." *ASME Journal of Dynamic Systems, Measurement, and Control*, 108(3), 163–171.
- Wampler, C.W. (1986). "Manipulator inverse kinematic solutions based on vector formulations and damped least-squares methods." *IEEE Transactions on Systems, Man, and Cybernetics*, 16(1), 93–101.
- Chiaverini, S. (1997). "Singularity-robust task-priority redundancy resolution for real-time kinematic control of robot manipulators." *IEEE Transactions on Robotics and Automation*, 13(3), 398–410.
- Buss, S.R. (2004). "Introduction to inverse kinematics with Jacobian transpose, pseudoinverse and damped least squares methods." Technical report, University of California, San Diego.
