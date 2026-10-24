# Stanley Controller — Overview

## What it is
A steering law for car-like vehicles that keeps the front axle on a path. It combines two corrections:
turn the wheels parallel to the path, and turn them further towards the path the farther the axle has
drifted from it. This is the controller that steered the vehicle that won the 2005 DARPA Grand Challenge.

## Why it matters (embedded)
Autonomous cars, agricultural tractors and industrial tugs need lane- or row-keeping that is accurate
at speed, insensitive to speed changes and trivially cheap — one projection onto the path and one
arctangent per cycle. Stanley meets all three and is easy to tune with a single gain.

## How it works (intuition)
The error is measured at the front axle, where the steered wheels act. The heading part rotates the
wheels to match the path direction. The cross-track part adds an angle whose tangent is the gain times
the distance from the path divided by the speed: at high speed a given offset calls for a smaller
correction, so the vehicle returns to the path at the same rate whatever its speed. A small softening
term keeps the law gentle when crawling, and every command is clipped to the steering limit and the
actuator's slew rate.

## Key parameters
- **gain `k`** — how fast the cross-track error decays (per second).
- **softening `k_s`** — low-speed regularisation.
- **steering limits** — from the vehicle model (W2).

## Reference
S. Thrun et al., "Stanley: The robot that won the DARPA Grand Challenge," *J. Field Robotics*, 23(9),
2006; G. M. Hoffmann, C. J. Tomlin, M. Montemerlo, S. Thrun, "Autonomous automobile trajectory tracking
for off-road driving," *American Control Conference*, 2007.

## See also
`PurePursuit` (W5, the path type), `CarLikeKinematics` (W2), `math::SE2Transform` (NT66).
