# Dubins Shortest Paths

## Overview & Motivation

A car cannot turn on the spot. Its turning radius is bounded below, and without reversing it can only drive forward. Planning for such a vehicle needs the shortest path between two poses that respects this **curvature bound**, both as a local steering function inside sampling planners and as a distance metric between poses.

Dubins (1957) proved a remarkable result: the shortest forward-only path between any two planar poses is one of just **six words** of at most three segments. Each segment is either an arc at the minimum turning radius ($C$, turning left $L$ or right $R$) or a straight line ($S$). Every word has a closed-form solution, so the planner evaluates all six and keeps the shortest.

The same header defines a general **car path** type, a short list of steering segments with signed lengths, which the Reeds–Shepp planner reuses for paths that also reverse.

## Mathematical Theory

### Normalised Problem

With turning radius $\rho$, the problem is scaled to unit radius and rotated so that the goal lies on the positive $x$ axis. With the displacement $\Delta$ between the two positions:

$$d = \frac{\|\Delta\|}{\rho}, \qquad \theta = \operatorname{atan2}(\Delta_y, \Delta_x), \qquad \alpha = \operatorname{mod}_{2\pi}(\theta_0 - \theta), \qquad \beta = \operatorname{mod}_{2\pi}(\theta_1 - \theta)$$

When the two positions coincide, $\theta$ is taken as the start heading.

### The Six Words

Each word gives three normalised lengths $(t, p, q)$: two arcs and a straight segment ($CSC$), or three arcs ($CCC$). For LSL, for example:

$$p = \left\|\left(d + \sin\alpha - \sin\beta,\; \cos\beta - \cos\alpha\right)\right\|, \qquad \tau = \operatorname{atan2}(\cos\beta - \cos\alpha,\; d + \sin\alpha - \sin\beta)$$

$$t = \operatorname{mod}_{2\pi}(\tau - \alpha), \qquad q = \operatorname{mod}_{2\pi}(\beta - \tau)$$

RSR follows by symmetry. The crossing words LSR and RSL need the two turning circles to be far enough apart:

$$p^2 = d^2 - 2 + 2\cos(\alpha - \beta) \pm 2d(\sin\alpha + \sin\beta) \ge 0$$

The $CCC$ words RLR and LRL need the turning circles close enough for a third circle to touch both:

$$\cos(2\pi - p) = \frac{6 - d^2 + 2\cos(\alpha - \beta) \pm 2d(\sin\alpha - \sin\beta)}{8}, \qquad |\cos(2\pi - p)| \le 1$$

The total length is $\rho\,(t + p + q)$, and the shortest valid word wins. On a tie, the first word in the order LSL, RSR, LSR, RSL, RLR, LRL is kept.

### Single-Precision Boundary Cases

Goals reachable by a single arc, such as a quarter turn to $(\rho, \rho, \pi/2)$, place the crossing discriminants exactly on zero. In single precision they come out slightly negative. A strict test would then reject the correct word and return a path five times too long.
The existence tests therefore accept a tolerance of $10^{-5}$ (in units of $\rho$), and angles within $10^{-5}$ of $2\pi$ snap to zero rather than adding a spurious full loop.

### Sampling a Car Path

A segment of signed length $\ell$ and curvature $\kappa \in \{1/\rho, 0, -1/\rho\}$ moves the pose by the SE(2) exponential of the twist $\operatorname{sign}(\ell)\,(1, 0, \kappa)$ over $|\ell|$. Reversing flips the yaw rate too. Sampling at arc length $s$ composes whole segments, then the remaining part of the current one.

## Complexity Analysis

| Operation     | Time   | Space  | Notes                                                |
|---------------|--------|--------|------------------------------------------------------|
| Shortest path | $O(1)$ | $O(1)$ | Six closed forms, about twenty trigonometric calls   |
| Sample        | $O(1)$ | $O(1)$ | At most one SE(2) exponential per segment            |
| Car path      | —      | $O(1)$ | Up to five segments with a steering tag and a length |

## Step-by-Step Walkthrough

From the origin facing along $x$, with $\rho = 1$ m:

1. **LSL**: to $(4, 4, \pi/2)$, a $45°$ left arc, $3\sqrt{2} = 4.243$ m straight and another $45°$ arc, $5.813$ m in total.
2. **RSL**: to $(3, -2, \pi/3)$, a right arc of $1.380$ rad, $0.897$ m straight and a left arc of $2.427$ rad, $4.703$ m in total. Mirroring the goal to $(3, 2, -\pi/3)$ gives LSR with the same lengths.
3. **RLR**: a goal close by at $(0.2, 0.6, -2.5)$ needs three arcs, $0.752$, $4.975$ and $0.439$, $6.167$ m in total.
4. **Turning on the spot**: a quarter turn without moving needs LRL, $0.424 + 5.560 + 0.424 = 6.409$ m. The best $CSC$ word, LSL, would take $9.268$ m.
5. **Float boundary**: the single quarter arc to $(1, 1, \pi/2)$ has length $\pi/2$. A strict existence test in single precision would return $7.854$ m instead.
6. **Radius scaling**: with $\rho = 2$ m, the goal $(8, 8, \pi/2)$ costs $11.627$ m, exactly twice the first case.
7. **Sampling**: halfway along the first path, the pose is $(2.207, 1.793)$ heading $\pi/4$, in the middle of the straight segment.

## Pitfalls & Edge Cases

- **Discontinuous metric**: a slightly rotated goal can switch the shortest word and add almost a full loop. Planners that need a continuous metric should allow reversing and use Reeds–Shepp paths.
- **Strict float comparisons**: exact zero tests drop valid words on boundary goals. Keep the tolerances.
- **Curvature discontinuity**: the path jumps between $\pm 1/\rho$ and $0$, which a real steering actuator cannot follow. Use clothoid transitions or a slightly larger radius for tracking.
- **Coincident poses**: start equal to goal gives a zero-length path.
- **Obstacles**: Dubins paths ignore obstacles. Use them as the local steering function inside a sampling planner.

## Variants & Generalizations

- **Reeds–Shepp paths**: shortest paths when reversing is allowed, with 48 words.
- **Continuous-curvature paths**: clothoid arcs that respect a bounded steering rate.
- **Dubins airplane**: an altitude dimension with a climb-rate bound.
- **Dubins traveling salesman**: tours through many poses for fixed-wing aircraft and survey vehicles.

## Applications

- Steering function and distance metric for RRT* and lattice planners of car-like robots.
- Path planning for fixed-wing aircraft, boats and agricultural vehicles that cannot reverse.
- Lower bounds and heuristics for planning with nonholonomic constraints.

## Connections to Other Algorithms

- [RRT-Connect](RrtConnect.md): a sampling planner where Dubins paths can replace straight-line steering.
- [Car-Like Kinematics](../mobile/CarLikeKinematics.md): the vehicle model with the minimum turning radius.
- [Pure-Pursuit Path Tracking](../mobile/PurePursuit.md): tracks a sampled Dubins path.
- The SE(2) exponential used for sampling comes from [numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp).

## References & Further Reading

- Dubins, L.E. (1957). "On curves of minimal length with a constraint on average curvature, and with prescribed initial and terminal positions and tangents." *American Journal of Mathematics*, 79(3), 497–516.
- Shkel, A.M. and Lumelsky, V. (2001). "Classification of the Dubins set." *Robotics and Autonomous Systems*, 34(4), 179–202.
- LaValle, S.M. (2006). *Planning Algorithms*. Cambridge University Press. Chapter 15.3.
