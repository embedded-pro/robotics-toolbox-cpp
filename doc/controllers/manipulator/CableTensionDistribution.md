# Cable Tension Distribution

## Overview & Motivation

**Cable-driven parallel robots** move a platform with several cables wound on winches: stadium cameras, large-scale 3D printers, rehabilitation devices and wind-tunnel model supports. They reach huge workspaces with tiny moving mass. The catch is that cables can only **pull**. A tension that would have to be negative means the cable goes slack and control is lost.

With more cables than platform degrees of freedom, infinitely many tension sets produce the same platform wrench. The controller must pick one that stays within $[t_{\min}, t_{\max}]$ (taut, but below the winch limit) at every control cycle, deterministically and quickly.
This module computes the structure matrix from the cable geometry, distributes tensions with Pott's closed-form method and its improved variant, and provides an exact quadratic-programming path for the boundary of the workspace.

## Mathematical Theory

### Structure Matrix

For base anchors $a_i$ (world frame), platform anchors $b_i$ (platform frame) and platform pose $(R, p)$, cable $i$ pulls the platform along the unit vector

$$u_i = \frac{a_i - p - R\,b_i}{\|a_i - p - R\,b_i\|}$$

and the cables balance the required platform wrench $w$ when

$$A\,t = w, \qquad A = \begin{bmatrix} u_1 & \cdots & u_m \\ R b_1 \times u_1 & \cdots & R b_m \times u_m \end{bmatrix}$$

For point-mass platforms only the force rows are kept, and the platform anchors are ignored.

### Closed-Form Distribution

With the mid tension $t_m = (t_{\min} + t_{\max})/2$, the tension vector closest to $t_m\mathbf{1}$ that balances $w$ is

$$t = t_m\mathbf{1} + A^T(AA^T)^{-1}(w - A\,t_m\mathbf{1})$$

It keeps every cable as far from slack and from overload as the wrench allows (Pott, Bruckmann and Mikelsons, 2009). With exactly as many cables as wrench dimensions, it reduces to the unique $A^{-1}w$.

### Improved Closed Form

When some tension leaves its bounds, the worst-violating cable is clamped to its bound, its contribution moves to the wrench side, and the reduced system is solved again (Pott, 2014). This repeats at most $m - n + 1$ times. If fewer than $n$ free cables remain, the wrench is declared infeasible for this method.

### Exact Distribution

Close to the boundary of the wrench-feasible workspace, the improved closed form can fail although a solution exists. The exact path solves the strictly convex quadratic program

$$\min_t \tfrac12\|t - t_m\mathbf{1}\|^2 \quad \text{s.t.} \quad A\,t = w, \quad t_{\min} \le t \le t_{\max}$$

with a dual active-set method. No result means the wrench is truly outside the feasible set.

## Complexity Analysis

| Method               | Time                          | Space         | Notes                                                   |
|----------------------|-------------------------------|---------------|---------------------------------------------------------|
| Structure matrix     | $O(mn)$                       | $O(mn)$       | One unit vector and cross product per cable             |
| Improved closed form | $O((m - n + 1)(n^2 m + n^3))$ | $O(mn + n^2)$ | At most $m - n + 1$ passes, one $n \times n$ solve each |
| Exact QP             | $O(k \cdot m^3)$              | $O(m^2)$      | Active-set iterations $k$, bounded by the solver's cap  |

## Step-by-Step Walkthrough

A planar point mass hangs from three cables anchored $120°$ apart at radius 2, with $t_{\min} = 10$ and $t_{\max} = 200$, so $t_m = 105$. The cable directions are $(0, 1)$ and $(\mp 0.866, -0.5)$.

1. **No wrench**: the three tensions are the pretension $(105, 105, 105)$, which already balances because the directions sum to zero.
2. **Upward wrench** $(0, 60)$: $t = (145, 85, 85)$, the balancing set closest to $t_m\mathbf{1}$.
3. **Upper bound** $(0, 150)$: the plain closed form asks for $205$ on the top cable. Clamping it at $200$ and re-solving the two lower cables gives $(200, 50, 50)$, which is also the exact optimum.
4. **Lower bound** $(0, -150)$: the top cable would go slack at $5$. It is clamped to $10$, and the others carry $160$ each.
5. **Infeasible** $(0, 300)$: even at $200$ on the top cable, the lower cables would need negative tension, so no distribution exists.
6. **Spatial boundary case**: on an eight-cable crossed platform, a wrench near the workspace boundary defeats the improved closed form. The exact QP finds tensions with cable 0 at $t_{\max}$ and cable 3 at $t_{\min}$.

## Pitfalls & Edge Cases

- **Slack cables**: $t_{\min}$ must be positive. A slack cable leaves the platform under-constrained and lets it swing.
- **Degenerate geometry**: collinear or coincident cables make $AA^T$ singular. The distribution then reports no solution rather than infinite tensions.
- **Zero-length cables**: a platform anchor on its base anchor has no direction and is rejected.
- **Wrench reference point**: the wrench is about the platform origin in the world frame. Include gravity as $-m g$ plus the dynamic wrench from the motion controller.
- **Boundary behaviour**: call the exact path only when the closed form fails, since it is slower and its cost varies.
- **Cable sag and elasticity**: the massless straight-line model ignores sagging long cables. Use catenary models for large installations.

## Variants & Generalizations

- **Puncture and barycentric methods**: alternative closed-form choices inside the feasible polytope.
- **Linear-programming distributions**: minimum-norm or minimum-maximum tensions for energy or safety criteria.
- **Stiffness-optimal tensions**: tension choices that maximize platform stiffness.
- **Dynamic workspace analysis**: wrench-feasible workspace computation from the same structure matrix.

## Applications

- Large-volume camera systems and robotic cranes.
- Cable-driven rehabilitation and haptic devices.
- Large 3D printers, façade cleaning and warehouse automation.

## Connections to Other Algorithms

- [Parallel Manipulator Kinematics](../../kinematics/ParallelManipulatorKinematics.md): the rigid-leg counterparts, whose leg Jacobian plays the role of the structure matrix.
- [Hybrid Position/Force Control](HybridPositionForceControl.md): another controller that splits a wrench between constraints.
- The linear solver and the dual active-set QP come from [numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp).

## References & Further Reading

- Pott, A., Bruckmann, T. and Mikelsons, L. (2009). "Closed-form force distribution for parallel wire robots." *Computational Kinematics*, Springer, 25–34.
- Pott, A. (2014). "An improved force distribution algorithm for over-constrained cable-driven parallel robots." *Computational Kinematics*, Springer, 139–146.
- Goldfarb, D. and Idnani, A. (1983). "A numerically stable dual method for solving strictly convex quadratic programs." *Mathematical Programming*, 27, 1–33.
- Pott, A. (2018). *Cable-Driven Parallel Robots: Theory and Application*. Springer.
