# Pure Pursuit — Overview

## What it is
A path-following controller for wheeled robots and cars. It picks a point a fixed distance ahead on the
path, the "carrot", and steers along the circular arc that would take the vehicle there. The output is
a curvature, which a differential-drive robot turns into a turn rate and a car into a steering angle.
The same module provides the bounded polyline path type the other trackers and planners use.

## Why it matters (embedded)
Pure pursuit is the workhorse tracker of warehouse robots, agricultural vehicles and autonomous carts:
it needs no model beyond geometry, has one intuitive tuning knob (how far ahead to look), and its cost
per cycle is a short segment search and a few multiplications. The path itself lives in fixed-size
storage, so long missions run without allocation.

## How it works (intuition)
Draw a circle around the robot with radius equal to the look-ahead distance; where the path leaves that
circle is the target. Exactly one arc starts at the robot, tangent to its heading, and passes through
the target, and its curvature is twice the target's sideways offset divided by the squared distance.
Looking farther ahead smooths the motion and cuts corners; looking closer tracks tightly but can
oscillate, so the distance grows with speed. The tracker remembers how far along the path it has come
and only searches forward, so a path that doubles back cannot pull it onto the wrong leg.

## Key parameters
- **look-ahead** — minimum, maximum, and the speed gain between them.
- **search window** — how many segments ahead the closest-point search may look.
- **goal tolerance** — distance to the last waypoint that counts as arrived.

## Reference
R. C. Coulter, "Implementation of the pure pursuit path tracking algorithm," CMU-RI-TR-92-01, 1992;
S. Macenski, S. Singh, F. Martín, J. Ginés, "Regulated pure pursuit for robot path tracking,"
*Autonomous Robots*, 47, 2023.

## See also
`DifferentialDrive` (W1), `CarLikeKinematics` (W2), `StanleyController` (W6, shares the path type),
`math::SE2Transform` (NT66).
