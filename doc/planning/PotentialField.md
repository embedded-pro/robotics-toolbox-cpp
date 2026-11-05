# Artificial Potential Fields

## Overview & Motivation

A robot often needs a velocity command **right now**, using only what its sensors see, rather than a complete collision-free path. Examples are a mobile base nudging around a person, or a manipulator's elbow dodging a moving obstacle.
**Artificial potential fields** (Khatib, 1986) answer with a single gradient step. The goal is an attractive well, each nearby obstacle an expanding hill, and the robot follows the negative gradient of their sum.

The method is constant-time per obstacle, needs no map and no search, and reacts instantly to new proximity readings, which makes it a classic reactive layer below a global planner. Its well-known weakness is **local minima**, where attraction and repulsion cancel short of the goal. Here they are detected and reported, so the caller can switch strategy.

## Mathematical Theory

### Attractive Potential

With the error $e = g - p$ and distance $d = \|e\|$, a quadratic well near the goal blends into a cone farther away:

$$U_{\text{att}} = \begin{cases} \tfrac12 k_a d^2 & d \le d^* \\ k_a d^* d - \tfrac12 k_a d^{*2} & d > d^* \end{cases} \qquad F_{\text{att}} = -\nabla U_{\text{att}} = \begin{cases} k_a\,e & d \le d^* \\ k_a d^*\,\dfrac{e}{d} & d > d^* \end{cases}$$

The quadratic part gives a pull proportional to the distance, so the robot slows smoothly into the goal. The conic part caps the pull at $k_a d^*$, so a far goal cannot overwhelm the repulsion.

### Repulsive Potential

For an obstacle at surface distance $\rho$, with unit direction $n$ pointing from the obstacle towards the robot:

$$U_{\text{rep}} = \begin{cases} \tfrac12 k_r \left(\dfrac{1}{\rho} - \dfrac{1}{\rho_0}\right)^2 & \rho < \rho_0 \\ 0 & \rho \ge \rho_0 \end{cases} \qquad F_{\text{rep}} = k_r \left(\frac{1}{\rho} - \frac{1}{\rho_0}\right)\frac{1}{\rho^2}\,n$$

The repulsion is zero beyond the influence distance $\rho_0$, grows without bound as $\rho \to 0$, and contributions from several obstacles add vectorially.

### Command and Status

The total force $F = F_{\text{att}} + \sum F_{\text{rep}}$ is used as a velocity, clamped to a maximum speed. The result is classified as:

- **reached** when $d$ is within the goal tolerance;
- **stalled** when $\|F\|$ falls below a small threshold away from the goal, which is a local minimum or saddle;
- **moving** otherwise.

### Local Minima

With an obstacle exactly between the robot and the goal, the forces are collinear and opposite. For $k_a d^* = 1$, $k_r = 0.1$ and $\rho_0 = 1$, they balance where $0.1(1/\rho - 1)/\rho^2 = 1$, at $\rho^* = 0.393$: an equilibrium short of the goal. No local rule removes such points in general, which is why the field is paired with a global planner or an escape behaviour.

## Complexity Analysis

| Case          | Time   | Space  | Notes                                           |
|---------------|--------|--------|-------------------------------------------------|
| $M$ obstacles | $O(M)$ | $O(1)$ | One division chain per obstacle inside $\rho_0$ |

## Step-by-Step Walkthrough

Gains $k_a = 1$, $d^* = 1$, $k_r = 0.1$, $\rho_0 = 1$, speed limit $0.5$, robot at the origin:

1. Goal at $(0.4, 0.3, 0)$: $d = 0.5 \le d^*$, so $F = (0.4, 0.3, 0)$. The magnitude $0.5$ is within the limit, and the robot moves straight at the goal.
2. Goal at $(3, 4, 0)$: $d = 5 > d^*$, the conic pull $(0.6, 0.8, 0)$ has magnitude $1$, and it is clamped to $(0.3, 0.4, 0)$.
3. Add an obstacle $0.5$ m away, pushing along $-y$: $F_{\text{rep}} = 0.1 \cdot (2 - 1)/0.25 = 0.4$ along $-y$. The total $(0.6, 0.4, 0)$ is clamped to $(0.416, 0.277, 0)$, so the robot veers away from the obstacle.
4. Goal at $(3, 0, 0)$ with an obstacle at $\rho = 0.393$ pushing along $-x$: both forces have magnitude $1$ and cancel. The status is **stalled**.
5. With the same obstacle at $\rho = 0.3$, the repulsion of $2.59$ wins and the robot backs away at the speed limit.

## Pitfalls & Edge Cases

- **Local minima and saddles**: detection is cheap, escape is not. Use a random walk, wall following, or intermediate goals from a global planner.
- **Oscillation in corridors**: opposing walls create lateral forces that alternate sign. Damping the command or a lower $k_r$ helps.
- **Goals near obstacles (GNRON)**: with the repulsion above, a goal inside $\rho_0$ of an obstacle may be unreachable. Variants scale the repulsion by the goal distance.
- **Contact**: as $\rho \to 0$ the repulsion diverges. The speed clamp keeps commands finite, but the caller must stop at contact; signed distances below zero are not valid input.
- **Units and tuning**: $k_a$ has units of 1/s, and $k_r$ of m³/s for a velocity output. Tune the speed limit and $\rho_0$ for the robot's stopping distance.

## Variants & Generalizations

- **Navigation functions** (Rimon and Koditschek): potentials with a unique minimum on sphere worlds, at the cost of global knowledge.
- **Harmonic potentials**: solutions of Laplace's equation have no interior local minima, but require a grid solve.
- **Vortex and circulation fields**: tangential components steer around obstacles instead of pushing directly away.
- **Manipulator control points**: evaluate the field at several points on the arm and map each force through the transposed point Jacobian into joint torques or rates.
- **Elastic bands**: deform a global path with repulsive forces instead of commanding velocity directly.

## Applications

- Reactive obstacle avoidance for mobile robots and drones between global replanning cycles.
- Collision avoidance for manipulator links in shared workspaces.
- Local refinement and smoothing of sampled paths.
- Teleoperation assistance that pushes the operator away from obstacles.

## Connections to Other Algorithms

- [Primitive Distance Queries](PrimitiveDistance.md): produce the proximity readings (distance and direction) fed into the repulsive term.
- [Geometric Jacobian](../kinematics/GeometricJacobian.md): maps the task-space field force at a control point to joint space.
- [Redundancy Resolution](../kinematics/RedundancyResolution.md): a repulsive term in the null space keeps a redundant arm clear of obstacles without moving the tool.

## References & Further Reading

- Khatib, O. (1986). "Real-time obstacle avoidance for manipulators and mobile robots." *International Journal of Robotics Research*, 5(1), 90–98.
- Koren, Y. and Borenstein, J. (1991). "Potential field methods and their inherent limitations for mobile robot navigation." *Proc. IEEE ICRA*, 1398–1404.
- Rimon, E. and Koditschek, D.E. (1992). "Exact robot navigation using artificial potential functions." *IEEE Transactions on Robotics and Automation*, 8(5), 501–518.
- Choset, H. et al. (2005). *Principles of Robot Motion*. MIT Press. Chapter 4.
