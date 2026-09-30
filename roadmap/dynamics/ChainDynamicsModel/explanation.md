# Chain Dynamics Model — Overview

## What it is
An adapter that turns a robot described link by link into the "manipulator equation"
`M(q)q̈ + C(q,q̇)q̇ + g(q) = τ` that model-based controllers speak: it answers "what is the mass
matrix?", "what are the Coriolis/centrifugal torques?", "what are the gravity torques?" and "what
torque produces this acceleration?".

## Why it matters (embedded)
Gravity compensation, computed-torque, impedance and operational-space controllers all need these
terms. Without an adapter every controller would re-derive them or require hand-written analytic
models; with it, one link description drives all of them, and the one-call inverse-dynamics path
keeps computed-torque control at linear cost per cycle.

## How it works (intuition)
The recursive Newton-Euler algorithm already computes joint torques for any state; zeroing the
acceleration and gravity isolates the velocity-dependent torques, zeroing velocity and acceleration
isolates gravity, and passing everything gives the full inverse dynamics in one pass. The composite
rigid body algorithm supplies the mass matrix.

## Key parameters
- **Link array** — the same inertial and geometric description the dynamics algorithms use.
- **Gravity vector** — expressed in the base frame.

## Reference
R. Featherstone, *Rigid Body Dynamics Algorithms* (2008), Ch. 5–6; M. W. Spong, S. Hutchinson,
M. Vidyasagar, *Robot Modeling and Control* (2020), Ch. 6.

## See also
`RecursiveNewtonEuler`, `CompositeRigidBodyAlgorithm` (M28), `EulerLagrangeSolver`,
`PdGravityCompensation` (M5), `ComputedTorqueControl` (M12), `CoriolisMatrixAndRegressor` (M31).
