# Cartesian Path + Orientation (SLERP) Interpolation — Overview

## What it is
A task-space motion generator: it moves the end-effector along a **straight line** in
Cartesian position while blending orientation with **SLERP** (spherical linear interpolation of
unit quaternions). A single scalar progress variable `s(t) ∈ [0, 1]` drives both channels so they
start and finish together.

## Why it matters (embedded)
Many manipulator tasks are defined in the workspace, not joint space — draw a straight bead, keep a
tool square to a surface, approach along an axis. Interpolating position and orientation directly in
Cartesian space produces predictable, collision-friendly paths that joint-space interpolation cannot
guarantee.

## How it works (intuition)
Position is a plain linear blend between the two endpoints. Orientation is trickier: naively
averaging quaternions leaves the sphere and changes speed. SLERP instead walks the **great-circle
arc** on the unit sphere at constant angular rate, giving the shortest, smoothest rotation. Feeding
`s(t)` from a trapezoidal or polynomial time law shapes how fast the pose advances along the path.
Because the path is fixed and only `s` moves, the end-effector twist is simply `ṡ` times the path
tangent: linear velocity `ṡ·(p1 − p0)` and angular velocity `ṡ·θ` about the fixed axis of the relative
rotation — a free feed-forward term for task-space controllers.

## Key parameters
- **Start / goal pose** — rigid transforms (position + rotation); orientation handled as unit quaternions.
- **Time law `s(t)`** — any 0 → 1 profile (polynomial, trapezoidal, S-curve); it fixes the duration and
  bounds the linear and angular speeds through `ṡ`.

## Reference
K. Lynch, F. Park, *Modern Robotics* (2017), Ch. 9 (trajectory generation);
K. Shoemake, "Animating rotation with quaternion curves," *SIGGRAPH* 1985 (SLERP).

## See also
`Quaternion` from [numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp)
(provides `Slerp`), `SE3Transform` (M6, pose and twist conventions),
`TrapezoidalProfile` / `PolynomialTrajectory` / `SCurveProfile` (the `s(t)` drivers).
