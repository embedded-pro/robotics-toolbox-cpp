# Cubic Spline Trajectory

## Overview & Motivation

Teach-in programs, planners and resampled optimal profiles hand a controller a list of **via points**: joint vectors $q_k$ that should be reached at times $t_k$.
Stopping at every via point wastes time and excites the structure; fitting one high-order polynomial through all of them rings between the knots. A **cubic spline** is the standard middle ground.
Between consecutive knots each joint follows a cubic polynomial, and the pieces join with continuous position, velocity **and** acceleration ($C^2$).

Taking time as the abscissa makes the spline a trajectory directly. Its slope is the joint velocity and its curvature is the joint acceleration. Planning is a single linear-time pass, and every servo tick costs one interval search plus a cubic evaluation per joint. Both are bounded and deterministic, which suits embedded controllers.

## Mathematical Theory

### Piecewise Cubic

For $K$ knots $t_0 < t_1 < \dots < t_{K-1}$ with values $q_k \in \mathbb{R}^n$, segment $k$ ($t \in [t_k, t_{k+1}]$, $h_k = t_{k+1} - t_k$, $\tau = t - t_k$) is written in terms of the knot accelerations $M_k = \ddot{q}(t_k)$:

$$q(t) = q_k + \beta_k \tau + \frac{M_k}{2}\tau^2 + \frac{M_{k+1} - M_k}{6 h_k}\tau^3, \qquad \beta_k = \frac{q_{k+1} - q_k}{h_k} - \frac{h_k}{6}\left(2M_k + M_{k+1}\right)$$

This form interpolates the knots and has continuous acceleration by construction. Matching velocities at the interior knots gives one equation per knot.

### Tridiagonal System

$$h_{k-1} M_{k-1} + 2(h_{k-1} + h_k) M_k + h_k M_{k+1} = 6\left(\frac{q_{k+1} - q_k}{h_k} - \frac{q_k - q_{k-1}}{h_{k-1}}\right), \quad 1 \le k \le K-2$$

Two end conditions close the system:

- **Clamped**: prescribed end velocities $v_0, v_f$, giving $2h_0 M_0 + h_0 M_1 = 6\left(\frac{q_1 - q_0}{h_0} - v_0\right)$ and the mirror equation at the end. With $v_0 = v_f = 0$ the move is **rest-to-rest**, the default for a trajectory.
- **Natural**: $M_0 = M_{K-1} = 0$. End accelerations vanish, and the end velocities follow from the data.

The matrix is strictly diagonally dominant, so the Thomas algorithm solves it stably in $O(K)$ without pivoting. The joints share the same matrix and differ only in the right-hand side, so the matrix is factored once and back-substituted per joint.

### Trajectory State

At time $t$ the trajectory reports the triple $(q, \dot{q}, \ddot{q})$, all from the same segment:

$$\dot{q}(t) = \beta_k + M_k \tau + \frac{M_{k+1} - M_k}{2h_k}\tau^2, \qquad \ddot{q}(t) = M_k + \frac{M_{k+1} - M_k}{h_k}\tau$$

Requests outside $[t_0, t_{K-1}]$ are clamped to the end states.

## Complexity Analysis

| Phase    | Time            | Space       | Notes                                                 |
|----------|-----------------|-------------|-------------------------------------------------------|
| Planning | $O(nK)$         | $O(nK)$     | One tridiagonal factorization, $n$ back-substitutions |
| Sampling | $O(\log K + n)$ | $O(1)$      | Binary search for the segment, then a cubic per joint |
| Storage  | —               | $K(2 + 2n)$ | Knot times, spans, values and knot accelerations      |

Storage is sized by the compile-time capacity, so the memory footprint is fixed no matter how many via points a plan uses.

## Step-by-Step Walkthrough

One joint, knots $t = (0, 1, 2.5, 3, 4.5)$ s, positions $q = (0, 1, -0.5, 0.3, 2)$ rad, clamped zero end velocities.

1. Spans $h = (1, 1.5, 0.5, 1.5)$; secant slopes $(1, -1, 1.6, 1.133)$.
2. Right-hand side: $6(1 - 0) = 6$, $6(-1 - 1) = -12$, $6(1.6 + 1) = 15.6$, $6(1.133 - 1.6) = -2.8$, $6(0 - 1.133) = -6.8$.
3. Solving the $5 \times 5$ tridiagonal system gives $M = (5.665, -5.330, 5.991, -0.737, -1.898)$ rad/s².
4. At $t = 0$: $\tau = 0$, so $q = 0$, $\dot{q} = \beta_0 = 1 - \tfrac{1}{6}(2 \cdot 5.665 - 5.330) = 0$, which is rest as requested. The start acceleration is $5.665$ rad/s², not zero.
5. For two knots with $v_0 = 0.5$, $v_f = -0.5$ over 2 s, the system reduces to $M = (1, -2)$, the same as the cubic point-to-point polynomial with those boundary velocities.

## Pitfalls & Edge Cases

- **Acceleration steps at the ends**: clamped zero velocities start and stop at rest, but the end accelerations are generally non-zero, so the acceleration jumps at start and stop. Natural ends remove the jump but leave the end velocities free. Pick per application, or blend into the spline with a higher-order segment.
- **Overshoot between knots**: the spline minimizes curvature, not excursion. Joint and velocity limits can be violated between knots, so check sampled states or stretch all knot times by $\lambda \ge 1$ (velocities scale by $1/\lambda$, accelerations by $1/\lambda^2$).
- **Knot timing**: badly chosen times (a long hop scheduled in a short interval) produce large accelerations. A good default spaces knot times in proportion to the largest joint displacement per segment, or to the chord length.
- **Nearly coincident knots**: spans close to zero make the system ill-conditioned and the slopes huge. Times must be strictly increasing with a margin above floating-point resolution, and plans that violate this are rejected.
- **Capacity**: plans with fewer than two knots or more than the compile-time capacity are rejected rather than truncated.

## Variants & Generalizations

- **Periodic splines**: wrap-around end conditions for cyclic motions (gait cycles, conveyor tracking).
- **Quintic splines**: $C^4$ continuity and prescribed end accelerations, removing the acceleration steps at the ends.
- **Smoothing splines**: trade exact interpolation for smoothness when via points are noisy (teach-in by demonstration).
- **B-splines**: local control and convex-hull bounds, useful when limits must be guaranteed between knots.
- **Time-optimal knot timing**: choose the $t_k$ to minimize duration under velocity and acceleration limits, an outer optimization around the same interpolant.

## Applications

- Joint-space execution of teach-in programs and planner waypoints without stopping at every point.
- Resampling time-optimal or optimization-based trajectories into a compact, smooth reference.
- Smooth references for computed-torque and PD controllers, which need continuous acceleration feed-forward.

## Connections to Other Algorithms

- The interpolant, the tridiagonal (Thomas) solve and the cubic and quintic point-to-point profiles come from the shared primitives of [numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp). With two knots the spline reduces to the cubic point-to-point profile.
- [Inverse Kinematics](../kinematics/InverseKinematics.md): converts Cartesian via points into the joint-space knots this trajectory interpolates.
- [Recursive Newton-Euler](../dynamics/RecursiveNewtonEuler.md): turns the sampled $(q, \dot{q}, \ddot{q})$ into feed-forward joint torques.

## References & Further Reading

- Biagiotti, L. and Melchiorri, C. (2008). *Trajectory Planning for Automatic Machines and Robots*. Springer. Chapter 4.
- de Boor, C. (2001). *A Practical Guide to Splines*. Revised ed. Springer.
- Siciliano, B. et al. (2009). *Robotics: Modelling, Planning and Control*. Springer. Chapter 4.
