# Cable Tension Distribution — Overview

## What it is
The force-allocation step for a cable-driven parallel robot: given a desired wrench (force + torque)
on the moving platform, compute a set of cable tensions — each between a positive minimum and a
maximum — that produce exactly that wrench. Because cables can only *pull*, and there are usually more
cables than platform degrees of freedom, this is a constrained problem, not a plain linear solve.

## Why it matters (embedded)
Cable robots — warehouse cranes, camera rigs (SkyCam), tendon-driven hands, large 3D printers —
actuate through tension only. Every control cycle must hand the winches a feasible, bounded tension
vector; a negative or over-limit request is physically impossible and can slacken a cable or snap
it. A closed-form allocator with a known worst-case cost fits the same real-time loop that computes
the wrench.

## How it works (intuition)
From the platform pose, each cable's direction is the unit vector from its platform attachment point
to its winch; together with the attachment offsets (moment arms) these form the **structure matrix**
`A`, and the cables apply `A·t` to the platform. With more cables than DOF, many tension sets give the
same wrench — the extra freedom is internal *pretension*. The closed form takes the tension set closest
to the mid-range value `(tMin + tMax)/2` that still reproduces the wrench, which keeps every cable
comfortably away from slack and overload. If some cable still falls outside its range, the improved
method pins the worst one at its limit, subtracts its pull from the wrench, and re-solves with the
remaining cables — repeating at most once per cable, and reporting "not feasible" if too few cables are
left.

## Key parameters
- **baseAnchors, platformAnchors** — winch exit points (world) and attachment points (platform frame).
- **tMin** — minimum tension (> 0) so cables never go slack.
- **tMax** — maximum tension set by winch/cable strength.

## Reference
A. Pott, T. Bruckmann, L. Mikelsons, "Closed-form Force Distribution for Parallel Wire Robots,"
*Computational Kinematics*, 2009. A. Pott, "An Improved Force Distribution Algorithm for
Over-Constrained Cable-Driven Parallel Robots," *Computational Kinematics*, 2014.

## See also
`SE3Transform` (M6, platform pose and wrench convention); `ParallelManipulatorKinematics` (M23,
platform pose from actuator lengths); `solvers::TrySolveSystem` (the small wrench-space solve, from
[numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp)).
