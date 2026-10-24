# Dubins Paths — Overview

## What it is
The shortest way for a forward-only vehicle with a limited turning radius — a car that cannot reverse, a
fixed-wing aircraft at constant altitude, a boat — to get from one position and heading to another.
The answer is always a combination of at most three pieces: turn, go straight, turn (or turn, turn the
other way, turn).

## Why it matters (embedded)
Dubins paths are the local connector inside sampling-based planners for car-like robots and drones, the
basis of UAV loiter and waypoint transitions, and a fast admissible distance for search heuristics. The
exact answer comes from six closed formulas in constant time, so it fits in a control loop on a
microcontroller.

## How it works (intuition)
L. E. Dubins proved that only six patterns can be optimal: left–straight–left, right–straight–right,
left–straight–right, right–straight–left, and the two three-turn patterns. Each pattern's piece lengths
follow from simple geometry: the straight piece is a tangent between the two turning circles, or a third
circle touches both. The planner computes all six, discards the impossible ones and keeps the shortest.
Floating-point care matters at the edges, where a pattern is just barely possible.

## Key parameters
- **turning radius `ρ`** — the vehicle's minimum radius (W2's `MinTurningRadius`).
- **start and goal poses**.

## Reference
L. E. Dubins, "On curves of minimal length with a constraint on average curvature, and with prescribed
initial and terminal positions and tangents," *American J. Mathematics*, 79(3), 1957; A. M. Shkel,
V. Lumelsky, "Classification of the Dubins set," *Robotics and Autonomous Systems*, 34(4), 2001.

## See also
`ReedsSheppPlanner` (W10, with reversing), `CarLikeKinematics` (W2), `PurePursuit` (W5, to track the
sampled path), `math::SE2Transform` (NT66).
