# Pure-Pursuit Path Tracking

## Overview & Motivation

A mobile robot that must follow a planned path needs a steering law that turns position errors into a smooth command. **Pure pursuit** (Coulter, 1992) is the simplest robust choice and still the default in many navigation stacks: pick the point one **look-ahead distance** ahead on the path, then drive the circular arc that reaches it.

The output is a **curvature**, so the same tracker serves a unicycle ($\omega = v\kappa$) and a car-like vehicle ($\delta = \arctan(L\kappa)$). Its behaviour is governed by one intuitive parameter. A short look-ahead tracks tightly but oscillates at speed, and a long one is smooth but cuts corners.

The tracker works on a bounded **planar polyline path**, stored with cumulative arc lengths, which supports closest-point projection and interpolation by arc length.

## Mathematical Theory

### Planar Path

A path of waypoints $p_0, \ldots, p_{K}$ stores the cumulative arc lengths $s_0 = 0$, $s_{k+1} = s_k + \|p_{k+1} - p_k\|$. The projection of a query point onto segment $k$, with unit direction $u_k$, is

$$t = \operatorname{clamp}\left((q - p_k)\cdot u_k,\; 0,\; \ell_k\right), \qquad c = p_k + t\,u_k$$

Its signed cross-track distance $u_{k,x}(q - p_k)_y - u_{k,y}(q - p_k)_x$ is positive when the query lies left of the path. The point at arc length $s$ is found by binary search over the cumulative lengths, then linear interpolation.

### Monotone Progress

The closest point is searched only over a short window of segments starting at the current one, and the progress along the path never decreases. A path that doubles back on itself, such as a U-turn or a parking loop, therefore cannot capture the robot onto its return leg.

### Look-Ahead Distance

The look-ahead is scheduled on speed:

$$L = \operatorname{clamp}\left(L_{\min} + g\,|v|,\; L_{\min},\; L_{\max}\right)$$

### Goal Point

The goal is the first point where the path **leaves** the circle $\|x - q\| = L$ beyond the current progress. On segment $p_k + t\,d$, this is the larger root of

$$\|d\|^2 t^2 + 2\,(p_k - q)\cdot d\;t + \|p_k - q\|^2 - L^2 = 0, \qquad 0 \le t \le 1$$

If no segment crosses the circle, the goal falls back. It is the last waypoint when that lies within the look-ahead, which handles the end of the path. Otherwise it is the path point at $s = \min(\text{progress} + L,\; \text{length})$, which steers a robot that has strayed far from the path back towards it.

### Steering Arc

With the goal expressed in the body frame as $b = (b_x, b_y)$, the circle tangent to the current heading that passes through it has curvature

$$\kappa = \frac{2\,b_y}{b_x^2 + b_y^2}$$

The arc does not depend on the direction of travel, so reversing along a path listed in driving order works with a negative speed.

## Complexity Analysis

| Operation       | Time         | Space  | Notes                                              |
|-----------------|--------------|--------|----------------------------------------------------|
| Path assignment | $O(K)$       | $O(K)$ | Cumulative arc lengths for $K$ waypoints           |
| Projection      | $O(w)$       | $O(1)$ | $w$ segments in the search window                  |
| Point at length | $O(\log K)$  | $O(1)$ | Binary search and interpolation                    |
| Tracker step    | $O(K)$ worst | $O(1)$ | Goal search usually ends after one or two segments |

## Step-by-Step Walkthrough

The path runs $(0,0) \to (2,0) \to (4,2) \to (4,5)$, with arc lengths $0$, $2$, $4.83$ and $7.83$ m. The tracker uses $L_{\min} = 1$ m, $L_{\max} = 3$ m, gain $0.5$ s and a two-segment search window.

1. **Projection**: the point $(3.5, 0.5)$ projects onto $(3, 1)$ on the diagonal segment, $3.41$ m along the path, $0.71$ m to its right.
2. **Goal on the current segment**: at $(0.5, -0.2)$ heading east at $1$ m/s, $L = 1.5$ m. The goal is $(1.99, 0)$ and the curvature $2 \cdot 0.2/1.5^2 = 0.178$.
3. **Goal on the next segment**: from $(1.8, 0)$, the circle leaves the path on the diagonal at $(2.96, 0.96)$, which needs curvature $0.85$.
4. **Far off the path**: from $(1, -3)$, the circle does not reach the path, so the goal falls back to $(2, 0)$ with curvature $0.6$.
5. **U-turn**: on a path that runs out and back $1$ m apart, a robot at $(1, 0.6)$ stays on the outbound leg (goal $(1.8, 0)$). An unbounded search would have jumped to the closer return leg.
6. **Closed loop**: a unicycle starting $0.5$ m off the path at $0.8$ m/s reaches within $0.1$ m of the end in under $600$ steps of $20$ ms.

## Pitfalls & Edge Cases

- **Look-ahead tuning**: too short oscillates, too long cuts corners. Scale it with speed.
- **Curvature limits**: the commanded curvature may exceed what the vehicle can steer. Saturate it, and slow down on sharp curvature.
- **Steering lag**: a rate-limited steering actuator makes pure pursuit overshoot at speed. Increase the look-ahead or model the actuator.
- **Self-intersecting paths**: an unbounded closest-point search jumps between legs. Keep the search window small and progress monotone.
- **Degenerate paths**: fewer than two points, or repeated points, are rejected when the path is assigned.

## Variants & Generalizations

- **Regulated pure pursuit**: scales speed down with curvature and proximity to obstacles (Macenski et al., 2023).
- **Adaptive look-ahead**: look-ahead scheduled on curvature as well as speed.
- **Vector pursuit**: also uses the path heading at the goal point.
- **Stanley and model-predictive control**: alternatives for higher speeds and tighter tracking.

## Applications

- Global-path following in indoor and outdoor navigation stacks.
- Agricultural row following and autonomous-vehicle low-speed manoeuvring.
- Teaching geometric path tracking.

## Connections to Other Algorithms

- [Differential-Drive Kinematics](DifferentialDrive.md): turns the curvature into wheel speeds with $\omega = v\kappa$.
- [Car-Like Kinematics](CarLikeKinematics.md): turns the curvature into a steering angle, with steering limits.
- [Planar Odometry](PlanarOdometry.md): provides the pose the tracker uses.

## References & Further Reading

- Coulter, R.C. (1992). *Implementation of the Pure Pursuit Path Tracking Algorithm*. Technical Report CMU-RI-TR-92-01, Carnegie Mellon University.
- Snider, J.M. (2009). *Automatic Steering Methods for Autonomous Automobile Path Tracking*. Technical Report CMU-RI-TR-09-08, Carnegie Mellon University.
- Macenski, S., Singh, S., Martín, F. and Ginés, J. (2023). "Regulated pure pursuit for robot path tracking." *Autonomous Robots*, 47, 685–694.
