# Parallel Manipulator Kinematics — Overview

## What it is
Kinematics for robots whose moving platform is held by several chains in parallel: the six-legged
**Stewart–Gough** hexapod, driven by leg lengths, and the pick-and-place **Delta**, driven by three
rotary arms whose parallelogram forearms keep the effector level. Inverse kinematics maps a platform
pose to actuator values; forward kinematics maps actuator values back to the pose.

## Why it matters (embedded)
Parallel robots are stiff, fast and accurate — the workhorses of high-speed packaging and motion
platforms. Their servo layer needs actuator commands from a desired pose on every tick, and both
mechanisms make that a short, closed-form, per-chain computation. Forward kinematics, needed for state
estimation, is cheap for the Delta and a few Newton steps for the hexapod.

## How it works (intuition)
For the **hexapod**, each leg is independent in the inverse problem: move the platform anchor into the
base frame and measure its distance to the base anchor. The forward problem couples all legs, so it is
solved by Newton's method on the pose: predict the leg lengths, compare, and correct position and
orientation using the leg Jacobian (each row is the leg direction and its moment about the platform
origin). Several poses share the same leg lengths, so the initial guess picks the assembly mode;
near singular poses a little damping keeps the correction finite.
For the **Delta**, each arm's elbow moves on a circle, and the forearm end must lie on a sphere around
the effector attachment. Rotating the target into the arm's plane reduces that to one equation
`a·cos θ + b·sin θ = c`, solved with one `atan2` and one `acos`. Going forward, each elbow (shifted
inward by the effector radius) is the centre of a sphere of forearm length, and the effector centre is
the lower of the two points where the three spheres meet.

## Key parameters
- **Hexapod:** base and platform anchors; Newton tolerance, damping and iteration limit.
- **Delta:** base radius, effector radius, upper-arm length, forearm length.

## Reference
J.-P. Merlet, *Parallel Robots*, 2nd ed. (2006); R. Clavel, "Delta, a Fast Robot with Parallel
Geometry," *Int. Symp. on Industrial Robots*, 1988.

## See also
`SE3Transform` (M6, platform pose and retraction), `LuDecomposition`
([numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp), the Newton step),
`ManipulabilityIndex` (M11, singularity monitoring).
