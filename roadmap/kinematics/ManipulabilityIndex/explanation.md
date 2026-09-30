# Manipulability Index — Overview

## What it is
A single number that scores how well an arm can move in its current posture. Yoshikawa's measure is
the volume of the "velocity ellipsoid" — the set of tool velocities reachable with unit-norm joint
rates. Big means dexterous; zero means the arm is at a singularity. Its companions are the ellipsoid's
axis lengths (the singular values) and their ratio (the condition number).

## Why it matters (embedded)
Singularities are where Jacobian-based controllers blow up: joint rates rocket toward infinity for a
finite tool motion. A cheap scalar that flags "you are getting close" lets a real-time controller slow
down, raise damping in its inverse, or switch strategy *before* the actuators saturate. It is also the
objective a redundant arm maximizes in its null space.

## How it works (intuition)
Map the unit sphere of joint velocities through the Jacobian and it becomes an ellipsoid; its axes are
the singular values and its volume their product. If there are at least as many joints as measured
task directions, the volume is `√det(J·Jᵀ)`. If there are fewer joints — a two-joint planar arm measured
in 3-D position — the ellipsoid is flat in the task space, `det(J·Jᵀ)` is always zero, and the
meaningful volume is that of the lower-dimensional ellipsoid, `√det(Jᵀ·J)`. Linear and angular rows
have different units, so for full-pose tasks the translational and rotational volumes are reported
separately rather than multiplied into one unit-dependent number.

## Key parameters
- **Jacobian provider and task dimension** — which rows are measured (position only, planar, or full pose).
- **singularity threshold `eps`** — below which `NearSingular` trips.

## Reference
T. Yoshikawa, "Manipulability of Robotic Mechanisms," *Int. J. Robotics Research*, 4(2), 1985.

## See also
`GeometricJacobian` / `JacobianProvider` (M8, the input), `SingularValueDecomposition` and
`LuDecomposition` ([numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp)),
`PoseInverseKinematics` (M13, adaptive damping), `RedundancyResolution` (M14, a null-space objective).
