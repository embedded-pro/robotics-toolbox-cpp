# Stanley Path Tracking

## Overview & Motivation

The **Stanley** steering law steered Stanford's autonomous car to victory in the 2005 DARPA Grand Challenge. It is a path tracker for front-steered vehicles that measures its error at the **front axle**, the point the steered wheels act on directly.
It then combines two intuitive corrections: turn the wheels parallel to the path, and turn them further towards the path by an amount that depends on the cross-track distance and the speed.

Pure pursuit chases a look-ahead point and tends to cut corners. Stanley instead uses the path heading at the closest point, so it tracks tightly at moderate speed. For small errors the cross-track distance decays exponentially, at a rate set by one gain and independent of speed.

## Mathematical Theory

### Front-Axle Error

With the rear-axle pose $T$ and wheel base $L$, the front axle is at $p_f = T\,(L, 0)$. Its projection onto the path gives the signed cross-track error $e$ (positive when the axle lies left of the path) and the path heading $\theta_p$. The heading error is

$$\psi = \operatorname{wrap}(\theta_p - \theta)$$

### Steering Law

With gain $k$ and softening speed $k_s$:

$$\delta = \psi - \arctan\frac{k\,e}{k_s + v}$$

The first term aligns the wheels with the path. The second steers back towards it: an axle left of the path ($e > 0$) steers right.

### Convergence

With the wheels aligned to the path ($\psi = 0$), the front axle moves sideways at $\dot{e} = -v \sin\left(\arctan\frac{k e}{v}\right)$. For small errors this is

$$\dot{e} \approx -k\,e$$

so the error decays exponentially at rate $k$, whatever the speed.

### Softening and Actuator Limits

Without $k_s$, the arctangent jumps to $\pm\pi/2$ at low speed for any error. A softening near $1$ m/s keeps low-speed steering gentle. The output is clamped to $\pm\delta_{\max}$ and then rate-limited to $\pm\dot{\delta}_{\max}\Delta t$ around the previously applied angle.

### Monotone Progress

The closest point is searched over a short window of path segments starting at the current one, and progress along the path never decreases. A path that doubles back on itself cannot capture the vehicle onto its return leg.

## Complexity Analysis

| Operation        | Time   | Space  | Notes                                      |
|------------------|--------|--------|--------------------------------------------|
| Front-axle error | $O(w)$ | $O(1)$ | Projection over $w$ segments in the window |
| Steering law     | $O(1)$ | $O(1)$ | One arctangent, a clamp and a rate limit   |
| State            | —      | $O(1)$ | Path and vehicle references, progress      |

## Step-by-Step Walkthrough

A car with wheel base $2.5$ m and $\delta_{\max} = 0.6$ rad follows a straight road along the $x$ axis, with $k = 2.5$ s⁻¹, $k_s = 1$ m/s and $v = 5$ m/s:

1. **Lateral offset**: $0.5$ m left of the road with zero heading error, $\delta = -\arctan(1.25/6) = -0.205$ rad.
2. **Heading error**: on the road but heading $0.2$ rad to the left, the front axle sits at $(2.450, 0.497)$. Both terms steer right, $\delta = -0.2 - \arctan(1.242/6) = -0.404$ rad.
3. **Saturation**: $3$ m right of the road, the law asks for $0.896$ rad, which is clamped to $0.6$ rad.
4. **Rate limit**: with $\dot{\delta}_{\max} = 0.5$ rad/s and a $0.1$ s cycle, the $-0.205$ rad request from rest only reaches $-0.05$ rad.
5. **Standstill**: at $v = 0$ the softening keeps the request finite, $-\arctan(1.25) = -0.896$ rad before the clamp.
6. **Closed loop**: starting $1$ m off the road at $5$ m/s, the cross-track error falls below $1$ mm within $4$ s.

## Pitfalls & Edge Cases

- **Rear-axle reference**: measuring the error at the rear axle makes the law unstable at speed. Always use the front axle.
- **Steering rate**: the actuator's slew rate dominates at speed. Feed back the applied steering angle every cycle.
- **Reversing**: the law assumes forward driving. Reverse with the rear axle as reference and a mirrored law, or use pure pursuit.
- **High speed**: the kinematic law ignores tyre slip and steering delay. The original controller adds yaw-rate damping and servo-delay terms.
- **Path discontinuities**: a corner in the polyline makes the heading term jump. Smooth sharp corners or limit the steering rate.

## Variants & Generalizations

- **Dynamic Stanley**: yaw-rate damping, steering-servo delay compensation and curvature feedforward for high speed.
- **Rear-axle Stanley**: the mirrored law for reversing.
- **Model-predictive control**: explicit handling of actuator limits and vehicle dynamics.

## Applications

- Autonomous-car lane keeping and route following at moderate speed.
- Agricultural vehicles following rows and headland turns.
- Outdoor delivery and inspection robots with car-like steering.

## Connections to Other Algorithms

- [Car-Like Kinematics](CarLikeKinematics.md): wheel base, steering angle and rate limits, and the bicycle twist used to simulate the vehicle.
- [Pure-Pursuit Path Tracking](PurePursuit.md): provides the planar path and its projection, and is the look-ahead alternative.
- [Planar Odometry](PlanarOdometry.md): provides the rear-axle pose.
- The angle wrapping comes from [numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp).

## References & Further Reading

- Thrun, S. et al. (2006). "Stanley: The robot that won the DARPA Grand Challenge." *Journal of Field Robotics*, 23(9), 661–692.
- Hoffmann, G.M., Tomlin, C.J., Montemerlo, M. and Thrun, S. (2007). "Autonomous automobile trajectory tracking for off-road driving: controller design, experimental validation and racing." *American Control Conference*, 2296–2301.
- Snider, J.M. (2009). *Automatic Steering Methods for Autonomous Automobile Path Tracking*. Technical Report CMU-RI-TR-09-08, Carnegie Mellon University.
