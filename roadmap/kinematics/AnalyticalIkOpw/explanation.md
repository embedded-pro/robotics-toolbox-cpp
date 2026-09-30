# Analytical IK for ortho-parallel 6R arms with a spherical wrist (OPW) — Overview

## What it is
A **closed-form** inverse-kinematics solver for the six-revolute layout used by most industrial arms:
the second and third axes are parallel to each other and perpendicular to the first, and the last
three axes meet at one point (a spherical wrist). Seven lengths — shoulder offset, lateral offset,
elbow offset, and the base, upper-arm, forearm and wrist lengths — plus a zero offset and a direction
sign per joint describe the arm. It returns every posture — up to eight — that reaches a flange pose.

## Why it matters (embedded)
Closed-form IK is exact, has no convergence loop and runs in constant time — ideal for a hard real-time
controller that cannot afford an iterative solver's worst case. Getting *all* solutions lets a planner
pick the posture that respects joint limits or stays close to the current one. The seven-parameter
description is read straight off a datasheet drawing, avoiding DH frame-placement mistakes.

## How it works (intuition)
Pieper's result is the theoretical basis: when the last three axes intersect, position and
orientation decouple. The wrist centre sits a fixed distance behind the flange along the approach axis,
and it depends only on the first three joints. The base joint turns the arm plane toward the wrist
centre — facing it or facing away, correcting for the lateral offset — giving two shoulder branches.
Inside that plane the shoulder offset shifts the triangle's corner, and the forearm offset turns the
forearm into a slightly longer virtual link at a fixed angle; the law of cosines on that triangle gives
elbow-up and elbow-down. With the arm placed, the remaining rotation is exactly a Z-Y-Z rotation of the
wrist, read off with `atan2` in two mirror-image flavours. When the wrist's middle joint is straight,
the first and last wrist axes line up and only their sum is determined, so one is fixed by convention.

## Key parameters
- **a1, a2, b, c1, c2, c3, c4** — shoulder, elbow and lateral offsets; base, upper-arm, forearm and wrist lengths.
- **joint offsets and signs** — map the manufacturer's joint zero and direction onto the model.
- **branch selection** — shoulder front/back, elbow up/down, wrist flip.

## Reference
M. Brandstötter, A. Angerer, M. Hofbaur, "An Analytical Solution of the Inverse Kinematics Problem of
Industrial Serial Manipulators with an Ortho-parallel Basis and a Spherical Wrist," *Proc. Austrian
Robotics Workshop*, 2014; D. L. Pieper, "The Kinematics of Manipulators Under Computer Control," PhD
thesis, Stanford University, 1968.

## See also
`SE3Transform` (M6, the flange pose), `PoseInverseKinematics` (M13, the iterative fallback for other
layouts), `DenavitHartenberg` (M7), `Cordic` ([numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp),
fixed-cost trig).
