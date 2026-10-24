# Car-Like Kinematics — Overview

## What it is
The kinematic model of a front-steered vehicle, from cars and delivery robots to tractors. The two
front wheels are replaced by one virtual wheel in the middle (the "bicycle"), and the model gives how
fast the vehicle turns for a speed and steering angle. It also returns the individual left and right
wheel angles real steering geometry needs.

## Why it matters (embedded)
Path trackers for car-like vehicles (pure pursuit, Stanley) produce a steering angle every cycle, which
must respect the steering limit and the actuator's slew rate. At low speeds the bicycle model is exact
enough for control and costs a single tangent; the Ackermann angles drive steer-by-wire systems with
separate actuators per wheel.

## How it works (intuition)
A car cannot slide sideways, so every wheel rolls perpendicular to a line through the turning centre,
which lies on the extension of the rear axle. The steering angle sets how far away that centre is: the
turn radius is the wheelbase divided by the tangent of the steering angle. On a real axle the inner wheel
must steer more than the outer one so that both point around the same centre; that is the Ackermann
condition. Steering commands are clipped twice, once to the maximum angle and once to how far the
actuator can move in one control period.

## Key parameters
- **wheelbase `L`** and **front track width `W`**.
- **max steering angle** (sets the minimum turning radius) and **max steering rate**.

## Reference
R. Rajamani, *Vehicle Dynamics and Control*, 2nd ed. (2012), Ch. 2; B. Paden, M. Čáp, S. Z. Yong,
D. Yershov, E. Frazzoli, "A survey of motion planning and control techniques for self-driving urban
vehicles," *IEEE T-IV*, 1(1), 2016.

## See also
`PurePursuit` (W5), `StanleyController` (W6), `DubinsPlanner` and `ReedsSheppPlanner` (W8, W10: paths
at the minimum turning radius), `math::SE2Transform` (NT66).
