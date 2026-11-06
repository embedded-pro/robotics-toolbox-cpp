# Time-Optimal Path Parameterization (TOPP-RA)

## Overview & Motivation

A path planner answers *where* the robot goes: a collision-free curve $q(s)$ in joint space, with $s$ running from 0 to 1. It does not say *how fast*. Running the curve at a constant, conservative speed wastes cycle time. Running it too fast asks the drives for torque they cannot deliver, and the arm leaves the path.

**Time-optimal path parameterization** (TOPP) keeps the geometry fixed and finds the fastest timing law $s(t)$ that respects every joint's velocity and torque limits. The result accelerates as hard as the weakest drive allows, cruises where a speed limit binds, and brakes at the last possible moment.

The classical phase-plane method (Bobrow et al., 1985) searches for switching points between maximum acceleration and maximum deceleration, which is fragile at singular points. **TOPP-RA** (Pham and Pham, 2018) instead propagates *sets of admissible speeds* along a grid.
It takes a fixed number of closed-form steps per grid stage, with bounded memory and deterministic run time, which suits an embedded planner.

## Mathematical Theory

### Reduction to the Path Coordinate

Along the path, $\dot q = q'\,\dot s$ and $\ddot q = q'\,\ddot s + q''\,\dot s^2$. With $x = \dot s^2$ and $u = \ddot s$, the manipulator equation $\tau = M(q)\ddot q + C(q, \dot q)\dot q + g(q)$ becomes affine in $(x, u)$:

$$\tau(s) = a(s)\,u + b(s)\,x + c(s), \qquad a = M q', \quad b = M q'' + C(q, q')\,q', \quad c = g(q)$$

Three inverse-dynamics passes give the coefficients: $c = \mathrm{ID}(q, 0, 0)$, $a = \mathrm{ID}(q, 0, q') - c$ and $b = \mathrm{ID}(q, q', q'') - c$. The term $C(q, q'\dot s)\,q'\dot s = \dot s^2\,C(q, q')\,q'$ is quadratic in $\dot s$, so it lands in $b$.

A joint speed limit $|q'_j \dot s| \le \dot q_{\max,j}$ becomes an upper bound on $x$:

$$x \le x_{\max}(s) = \min_{j:\,q'_j \ne 0} \left(\frac{\dot q_{\max,j}}{q'_j}\right)^2$$

### Discretization

The grid is $s_i = i/N$, $i = 0, \ldots, N$, with step $\Delta = 1/N$. Each stage holds $u_i$ constant, so $x$ is linear in $s$:

$$x_{i+1} = x_i + 2\Delta\,u_i$$

The constraints at gridpoint $i$ are the $2n$ torque rows $\tau_{\min} \le a_i u + b_i x + c_i \le \tau_{\max}$ and the speed rows $0 \le x \le x_{\max,i}$. All of them have the form $\alpha x + \beta u \le \gamma$.

### Controllable Sets (Backward Pass)

The controllable set $K_i = [l_i, h_i]$ holds the speeds $x$ at $s_i$ from which the robot can still finish the path and stop at $s = 1$. It starts at $K_N = [0, 0]$ and runs backwards:

$$K_i = \{\, x \;:\; \exists u,\; (x, u) \text{ satisfies stage } i,\; x + 2\Delta u \in K_{i+1} \,\}$$

The feasible region of stage $i$ is a polygon in the $(x, u)$ plane, and $K_i$ is its shadow on the $x$ axis. Every row with $\beta > 0$ gives an upper bound on $u$, and every row with $\beta < 0$ a lower bound. Fourier–Motzkin elimination combines each pair $(p, n)$ into a constraint on $x$ alone:

$$(\beta_p \alpha_n - \beta_n \alpha_p)\,x \le \beta_p \gamma_n - \beta_n \gamma_p$$

Together with the rows that do not involve $u$, these give the exact minimum and maximum of $x$ over the polygon, without an iterative LP solver. The path is infeasible if some $K_i$ is empty or if $0 \notin K_0$, since the robot must start from rest.

### Greedy Forward Pass

Starting from $x_0 = 0$, each stage takes the largest admissible control, a one-dimensional LP:

$$u_i = \min_{\beta > 0} \frac{\gamma - \alpha x_i}{\beta}, \qquad x_{i+1} = \operatorname{clamp}(x_i + 2\Delta u_i,\; l_{i+1},\; h_{i+1})$$

The control $u_i$ is then recomputed from the clamped $x_{i+1}$, so each stage ends exactly at $s_{i+1}$. Staying inside $K_{i+1}$ guarantees that the next stage is feasible too, so the pass never gets stuck.

### Timing and Sampling

With constant $u_i$, the stage takes exactly

$$\Delta t_i = \frac{2\Delta}{\dot s_i + \dot s_{i+1}}, \qquad \dot s_i = \sqrt{x_i}$$

which stays finite at the rest endpoints, unlike $\int ds / \sqrt{x}$. Inside stage $i$, at time $\tau$ after its start, $s = s_i + \dot s_i \tau + u_i \tau^2 / 2$, $\dot s = \dot s_i + u_i \tau$ and $\ddot s = u_i$. The joint state follows from the path derivatives.

## Complexity Analysis

| Operation         | Time            | Space          | Notes                                                         |
|-------------------|-----------------|----------------|---------------------------------------------------------------|
| Path coefficients | $O(N n)$        | $O(N n)$       | Three RNEA passes per gridpoint through the adapter           |
| Backward pass     | $O(N n^2)$      | $O(n)$         | All pairs of the $2n + 4$ rows of each stage                  |
| Forward pass      | $O(N n)$        | $O(1)$         | One 1-D LP per stage                                          |
| Sample            | $O(\log N + n)$ | $O(1)$         | Binary search over stage start times, three path queries      |
| Profile store     | —               | $O(N(3n + 6))$ | About 3 KB for 2 joints and 64 gridpoints in single precision |

For $N$ stages and $n$ joints. Planning is deterministic: no iteration count depends on the data.

## Step-by-Step Walkthrough

A single joint with unit inertia and $|\tau| \le 1$ moves a distance $L$ rest to rest. The continuous optimum is bang-bang, with time $2\sqrt{L}$. The planner uses 64 gridpoints, so 63 stages:

1. **Bang-bang**: for $L = 1$ the profile accelerates at $+1$ to the midpoint and brakes at $-1$. The total time is $2.000064$ against $2$ exact. The odd stage count leaves one coast stage at the midpoint, so the peak speed is $0.992$ instead of $1$.
2. **Longer path**: $L = 4$ takes $4.000128$, with the acceleration at exactly $\pm 1$ in the first and last 45 % of the motion.
3. **Speed limit**: with $\dot q_{\max} = 0.5$ and $|\tau| \le 100$, one stage ramps up, the speed holds at $0.5$, and one stage ramps down. The total is $2 + 4/63 = 2.063492$.
4. **Coupled two-joint path**: on $q(s) = (s,\; 0.4s - 0.4s^2)$ with position-dependent coefficients, the profile takes $2.256496$. Joint speeds peak at 58 % and 18 % of their limits. Torques hold exactly at gridpoints and exceed their bounds by at most $0.018$ between them.
5. **Infeasible holding torque**: a gravity torque of $2$ against a bound of $1$ forces $u \le -1$ on every stage. Then $0 \notin K_0$, and the planner reports failure.

## Pitfalls & Edge Cases

- **Collocation**: constraints hold exactly at gridpoints only, with an $O(\Delta)$ excursion between them. Refine the grid or shrink the bounds by a margin.
- **Viscous friction**: it is linear in $\dot s$, not quadratic, so it breaks the affine form. Use a friction-free model plus a torque margin.
- **Real-time use**: planning calls the path geometry and the dynamics through virtual interfaces. Plan offline, then resample the result into a [Cubic Spline Trajectory](CubicSplineTrajectory.md) for the servo loop.
- **Zero-length path**: if $q' \equiv 0$, the planner succeeds with a total time of $0$.
- **Stationary joints and zero inertia**: $q'_j = 0$ drops the joint from $x_{\max}$, and $a_{ij} = 0$ turns its torque rows into pure $x$ bounds. No division by zero occurs.
- **Stalls**: a zero speed limit pins $K_1$ to $\{0\}$, and the profile cannot leave rest. The planner reports failure instead of dividing by zero.

## Variants & Generalizations

- **Phase-plane TOPP**: integrates maximum-acceleration and maximum-deceleration curves between switching points (Bobrow; Pham 2014).
- **Convex-optimization TOPP**: solves the whole profile as one second-order cone program, at a higher cost.
- **Jerk-limited TOPP**: adds third-order limits, at the price of a nonconvex problem.
- **Second-order constraints**: contact stability, friction cones and Cartesian speed limits fit the same $\alpha x + \beta u \le \gamma$ rows.

## Applications

- Cycle-time optimization of industrial manipulators along fixed welding, gluing or machining paths.
- Speed scheduling of mobile manipulators and vehicles along planned curves.
- Retiming the output of sampling-based planners such as [RRT-Connect](../planning/RrtConnect.md) before execution.

## Connections to Other Algorithms

- [Chain Dynamics Model](../dynamics/ChainDynamicsModel.md): the inverse-dynamics model behind the path coefficients.
- [Cubic Spline Trajectory](CubicSplineTrajectory.md): resamples the optimal profile for the servo loop.
- [Cartesian Path with SLERP Orientation](CartesianSlerpInterpolation.md): a task-space path that inverse kinematics maps to $q(s)$.
- The S-curve and trapezoidal profiles in [numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp) are the heuristic alternative when no dynamics model is available.

## References & Further Reading

- Pham, H. and Pham, Q.-C. (2018). "A New Approach to Time-Optimal Path Parameterization Based on Reachability Analysis." *IEEE Transactions on Robotics*, 34(3), 645–659.
- Bobrow, J.E., Dubowsky, S. and Gibson, J.S. (1985). "Time-Optimal Control of Robotic Manipulators Along Specified Paths." *International Journal of Robotics Research*, 4(3), 3–17.
- Pham, Q.-C. (2014). "A General, Fast, and Robust Implementation of the Time-Optimal Path Parameterization Algorithm." *IEEE Transactions on Robotics*, 30(6), 1533–1540.
