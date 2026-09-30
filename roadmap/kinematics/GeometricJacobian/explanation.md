# Geometric Jacobian — Overview

## What it is
The matrix that maps joint rates to the tool's velocity: for an `N`-joint arm it is `6×N`, the top
three rows giving the linear velocity of the tool point and the bottom three its angular velocity,
both expressed in the base frame. Its transpose maps a force and moment applied at the tool to joint
torques. The companion term `J̇q̇` is the tool acceleration produced by joint velocities alone.

## Why it matters (embedded)
It is the workhorse of manipulator control: velocity control, force control, singularity detection,
pose inverse kinematics, impedance and operational-space control all need it, and task-space
controllers additionally need `J̇q̇`. Building it from one generic "frame chain" means a single,
well-tested routine serves every kinematic description in the library.

## How it works (intuition)
Each joint contributes one column. A revolute joint spins everything outboard about its axis `z`, so
the tool gains angular velocity `z` and linear velocity `z × r`, where `r` runs from the joint to the
tool. A prismatic joint slides the tool along `z`. `J̇q̇` follows from differentiating those columns
while the axes and lever arms are carried along by the moving links.

## Key parameters
- **Frame chain** — joint origins, axes and types plus the tool pose, in the base frame.
- **Ordering** — `(v; ω)`, linear first, as everywhere in the library.

## Reference
B. Siciliano et al., *Robotics: Modelling, Planning and Control* (2009), Ch. 3 (geometric Jacobian);
K. M. Lynch, F. C. Park, *Modern Robotics* (2017), Ch. 5 (for the space/body Jacobian variants).

## See also
`ChainPoseKinematics` (M30), `DenavitHartenberg` (M7) and `ProductOfExponentials` (M15) produce the
frame chain; `ManipulabilityIndex` (M11) scores the Jacobian; `PoseInverseKinematics` (M13),
`RedundancyResolution` (M14) and the task-space controllers (M17–M19) use it.
