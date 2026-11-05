# Car-Like Kinematics (Kinematic Bicycle and Ackermann)

## Overview & Motivation

Cars, tractors, forklifts and many outdoor robots steer with their front wheels. They cannot turn in place or move sideways. Their motion is limited by the steering angle, and the steering angle itself is limited in range and rate.

The **kinematic bicycle** model merges the two front wheels into one virtual wheel at steering angle $\delta$ and the two rear wheels into one, with no tyre slip. It is accurate at low lateral acceleration (below roughly $0.4\,g$), and it is the model behind classical path trackers such as pure pursuit and Stanley.
The **Ackermann** geometry maps the virtual steering angle back to the two real front wheels so that all four wheels turn about one common centre.

## Mathematical Theory

### Bicycle Model

With wheel base $L$ and the reference point at the rear-axle midpoint, the body twist for rear-axle speed $v$ and steering angle $\delta$ is

$$\xi = \left(v,\; 0,\; \frac{v \tan\delta}{L}\right), \qquad \kappa = \frac{\tan\delta}{L}$$

The rear-axle midpoint has no lateral velocity, which makes it the natural reference for pure pursuit. A negative speed reverses, and with it the sign of the yaw rate.

### Other Reference Points

A point a distance $a$ ahead of the rear axle (the front axle at $a = L$, the centre of gravity at $a = l_r$) moves with

$$\xi_a = (v,\; a\,\omega,\; \omega), \qquad \omega = \frac{v\tan\delta}{L}$$

This is the SE(2) adjoint of the rear-axle twist, so it agrees exactly with composing poses.

### Steering for a Curvature

$$\delta = \operatorname{clamp}\left(\arctan(L\kappa),\; -\delta_{\max},\; \delta_{\max}\right)$$

### Steering Actuator Limits

A requested angle is first clamped to $\pm\delta_{\max}$, then limited in rate relative to the previous command:

$$\delta_k = \operatorname{clamp}\left(\delta_{\text{req}},\; \delta_{k-1} - \dot{\delta}_{\max}\Delta t,\; \delta_{k-1} + \dot{\delta}_{\max}\Delta t\right)$$

### Ackermann Geometry

For both front wheels to roll about the turning centre on the rear-axle line at radius $R = L/\tan\delta$, with track width $W$:

$$\delta_{\text{inner}} = \arctan\frac{L\tan\delta}{L - \tfrac{W}{2}\tan\delta}, \qquad \delta_{\text{outer}} = \arctan\frac{L\tan\delta}{L + \tfrac{W}{2}\tan\delta}$$

This form stays finite at $\delta = 0$. The inner wheel must stay below $90°$, which requires $|\delta| < \arctan(2L/W)$.

## Complexity Analysis

| Operation              | Time   | Space  | Notes                                       |
|------------------------|--------|--------|---------------------------------------------|
| Twist and curvature    | $O(1)$ | $O(1)$ | One tangent                                 |
| Steering for curvature | $O(1)$ | $O(1)$ | One arctangent and a clamp                  |
| Ackermann angles       | $O(1)$ | $O(1)$ | One tangent, two arctangents                |
| State                  | —      | $O(1)$ | Wheel base, track width and steering limits |

## Step-by-Step Walkthrough

A vehicle with $L = 2.5$ m, $W = 1.5$ m, $\delta_{\max} = 0.6$ rad and $\dot{\delta}_{\max} = 0.5$ rad/s:

1. **Twist**: at $10$ m/s and $\delta = 0.1$ rad, the yaw rate is $10 \cdot \tan(0.1)/2.5 = 0.401$ rad/s.
2. **Front axle**: a point $1.2$ m ahead of the rear axle also moves sideways at $1.2 \cdot 0.401 = 0.482$ m/s, matching the SE(2) adjoint.
3. **Curvature**: $\kappa = 0.1$ needs $\delta = 0.245$ rad. $\kappa = 0.5$ would need $0.896$ rad, so it saturates at $0.6$.
4. **Rate limit**: from $0.1$ rad, a request of $0.5$ rad reaches only $0.2$ rad in $0.2$ s. From $0.55$ rad, a request of $-0.9$ rad is clamped to $-0.6$, then rate-limited to $0.05$ rad after one second.
5. **Ackermann**: at $\delta = 0.3$ rad, the inner wheel steers $0.329$ rad and the outer $0.276$ rad, and both point at the same centre $8.08$ m away.
6. **Turning radius**: the minimum turning radius is $2.5/\tan(0.6) = 3.65$ m.

## Pitfalls & Edge Cases

- **High lateral acceleration**: beyond about $0.4\,g$, tyre slip makes the kinematic model wrong. Use a dynamic bicycle model with cornering stiffness.
- **Reference point mismatch**: pure pursuit uses the rear axle, Stanley the front axle. Mixing them biases the tracker.
- **Ignoring the rate limit**: a real steering actuator lags. Trackers that assume instant steering overshoot at speed.
- **Steering near $90°$**: the Ackermann inner angle diverges as $\tan\delta \to 2L/W$. The maximum steering angle must stay below that.
- **Reversing**: negative speeds flip the yaw rate for the same steering angle.

## Variants & Generalizations

- **Dynamic bicycle model**: lateral tyre forces with slip angles for high-speed driving.
- **Four-wheel steering**: front and rear steering angles with a movable turning centre.
- **Trailers**: articulated kinematics for tractor–trailer combinations.
- **Curvature-rate models**: clothoid-based planning that respects the steering rate.

## Applications

- Autonomous cars, agricultural vehicles and outdoor delivery robots.
- Path tracking with pure pursuit, Stanley or model-predictive control.
- Feasibility checks for curvature-constrained planners such as Dubins and Reeds–Shepp.

## Connections to Other Algorithms

- [Differential-Drive Kinematics](DifferentialDrive.md): the turn-in-place alternative with the same unicycle twist form.
- [Planar Odometry](PlanarOdometry.md): integrates the rear-axle twist into a pose.
- The SE(2) adjoint used for other reference points comes from [numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp).

## References & Further Reading

- Rajamani, R. (2012). *Vehicle Dynamics and Control*, 2nd ed. Springer. Chapter 2.
- Paden, B., Čáp, M., Yong, S.Z., Yershov, D. and Frazzoli, E. (2016). "A survey of motion planning and control techniques for self-driving urban vehicles." *IEEE Transactions on Intelligent Vehicles*, 1(1), 33–55.
- Kong, J., Pfeiffer, M., Schildbach, G. and Borrelli, F. (2015). "Kinematic and dynamic vehicle models for autonomous driving control design." *IEEE Intelligent Vehicles Symposium*, 1094–1099.
