# Dynamic Window Approach

## Overview & Motivation

A global planner gives a mobile robot a route through a known map, but people, carts and furniture that moved since the map was built must be avoided on the fly. The **dynamic window approach** (Fox, Burgard and Thrun, 1997) is a classic reactive local planner for this. It runs every control cycle and directly selects the next velocity command.

It searches in **velocity space** rather than in position space. Only the speeds and turn rates the motors can reach within one control period are considered (the *dynamic window*), so every candidate is physically achievable. Each candidate is then checked against the obstacles along its circular arc.
The candidates the robot could still stop for are scored on progress towards the goal, distance to obstacles and speed, and the best one is commanded.

## Mathematical Theory

### Dynamic Window

From the current speed $v_0$ and turn rate $\omega_0$, with acceleration limits $a$ and $\alpha$ and control period $\Delta t$, the reachable velocities are

$$v \in \left[\max(v_{\min}, v_0 - a\Delta t),\; \min(v_{\max}, v_0 + a\Delta t)\right], \qquad \omega \in \left[\max(-\omega_{\max}, \omega_0 - \alpha\Delta t),\; \min(\omega_{\max}, \omega_0 + \alpha\Delta t)\right]$$

The window is sampled on a regular $N_v \times N_\omega$ grid that includes its corners.

### Arc Clearance

A constant command $(v, \omega)$ moves the robot centre on a circle of radius $R = v/\omega$ about $c = (0, R)$ in the body frame. For a disc robot of radius $r$ and an obstacle point $p$, the clearance is the arc length travelled before the disc first touches $p$. If $p$ stays farther than $r$ from that circle, the clearance is the cap $d_{\max}$.
Otherwise the contact interval about $c$ has half-angle

$$\Delta = \arccos\frac{R^2 + \|p - c\|^2 - r^2}{2|R|\,\|p - c\|}$$

and the clearance is $|R|\,(\varphi - \Delta)$, where $\varphi$ is the angle from the start to the obstacle around $c$ in the direction of travel. A straight command uses $p_x - \sqrt{r^2 - p_y^2}$ for obstacles in its path, and rotation in place sweeps no new area.

### Admissibility

A command is admissible if the robot can brake to a stop before the nearest obstacle on its arc:

$$v \le \sqrt{2\,a\,d(v, \omega)}$$

Standstill is always admissible, so no admissible command means the robot is moving too fast to stop. The caller then brakes at full deceleration.

### Objective

For each admissible sample, the pose after the prediction time $T_p$ comes from the SE(2) exponential of the twist $(v, 0, \omega)$. The heading term rewards facing the goal from there:

$$h = \pi - \left|\operatorname{wrap}\left(\operatorname{atan2}(g - p_{T_p}) - \theta_{T_p}\right)\right|$$

Heading, clearance and speed are each normalised by their maximum over the admissible samples, and the command maximises

$$G = w_h\,\hat{h} + w_c\,\hat{d} + w_v\,\hat{v}$$

On a tie, the first sample in grid order wins.

## Complexity Analysis

| Operation     | Time                | Space             | Notes                                           |
|---------------|---------------------|-------------------|-------------------------------------------------|
| Window        | $O(1)$              | $O(1)$            | Four clamps                                     |
| Arc clearance | $O(1)$              | $O(1)$            | At most one arccosine and two arctangents       |
| Command       | $O(N_v N_\omega M)$ | $O(N_v N_\omega)$ | $M$ obstacle points, one exponential per sample |

An $11 \times 21$ grid against a 360-beam scan is about 83 000 arc tests per cycle. Decimating the scan to the nearest point per sector reduces this considerably.

## Step-by-Step Walkthrough

A disc robot of radius $0.2$ m with $v \le 1$ m/s, $|\omega| \le 2$ rad/s, $a = 1$ m/s², $\alpha = 3$ rad/s², a $0.25$ s period, a $1$ s prediction, a $2$ m clearance cap, weights $(1, 1, 0.5)$ and a $5 \times 11$ grid:

1. **Straight clearance**: driving straight, an obstacle at $(2, 0.1)$ is touched after $2 - \sqrt{0.03} = 1.827$ m. One behind the robot never is.
2. **Turning clearance**: on the unit circle ($v = \omega = 1$), an obstacle at $(1, 1)$ is touched after $\pi/2 - \arccos 0.98 = 1.370$ m.
3. **Window**: at $(0.5, 0)$ the window is $[0.25, 0.75]$ m/s by $[-0.75, 0.75]$ rad/s. Near the limits it is clipped, to $[0.7, 1]$ by $[1.15, 2]$ at $(0.95, 1.9)$.
4. **Free space**: with the goal straight ahead, the robot accelerates to $0.75$ m/s without turning.
5. **Goal to the left**: with the goal at $(0, 3)$, it accelerates and turns left at the window's edge, $(0.75, 0.75)$.
6. **Obstacle slightly left**: a point at $(0.45, 0.05)$ leaves 44 of the 55 samples admissible. The robot slows to $0.375$ m/s and turns right at $0.75$ rad/s. Mirroring the obstacle mirrors the turn.
7. **Too fast to stop**: a wall $0.3$ m ahead admits no command at $0.75$ m/s. From standstill, the robot stays put.

## Pitfalls & Edge Cases

- **Local minima**: the planner is reactive and can stall in U-shaped traps. Feed it intermediate goals from a global planner or a path rather than the final goal.
- **Footprint**: a disc that is too tight lets corners scrape. Inflate the radius by a safety margin.
- **Body frame**: obstacle points must be in the body frame. Transform map obstacles with the current pose before each call.
- **Computation budget**: the cost grows with samples times obstacle points. Decimate the scan or shrink the grid on small targets.
- **Turn-rate admissibility**: the original paper also bounds $\omega$ by $\sqrt{2\alpha d}$, which mixes metres and radians. For a disc robot braking along its arc, the speed bound is the one that matters.

## Variants & Generalizations

- **Global dynamic window**: adds a navigation function to the objective to escape local minima.
- **Trajectory rollout**: simulates each candidate over the horizon instead of using a constant arc.
- **Velocity obstacles and ORCA**: reciprocal avoidance of moving agents.
- **Timed elastic band and model-predictive control**: optimisation-based local planners.

## Applications

- Local planners in indoor navigation stacks for service and logistics robots.
- Reactive avoidance of people in museums, hospitals and offices.
- Fallback obstacle avoidance under a global path tracker.

## Connections to Other Algorithms

- [Differential-Drive Kinematics](../mobile/DifferentialDrive.md): turns the selected command into wheel speeds.
- [Grid Search (A* and Dijkstra)](GridSearch.md): provides intermediate goals that avoid local minima.
- [Pure-Pursuit Path Tracking](../mobile/PurePursuit.md): provides a look-ahead point on a planned path to use as the goal.
- The SE(2) exponential and angle wrapping come from [numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp).

## References & Further Reading

- Fox, D., Burgard, W. and Thrun, S. (1997). "The dynamic window approach to collision avoidance." *IEEE Robotics & Automation Magazine*, 4(1), 23–33.
- Brock, O. and Khatib, O. (1999). "High-speed navigation using the global dynamic window approach." *IEEE International Conference on Robotics and Automation*, 341–346.
- Siegwart, R., Nourbakhsh, I.R. and Scaramuzza, D. (2011). *Introduction to Autonomous Mobile Robots*, 2nd ed. MIT Press. Chapter 6.
