# Composite Rigid Body Algorithm — Overview

## What it is
An `O(n²)` algorithm that builds the joint-space mass (inertia) matrix `M(q)` of a serial arm — the
matrix in `M(q)q̈ + C(q,q̇)q̇ + g(q) = τ` that says how joint accelerations turn into joint torques.

## Why it matters (embedded)
Several controllers need `M(q)` itself rather than just torques: operational-space control reflects
it into task space, momentum observers integrate `M(q)q̇`, and some forward-dynamics schemes factor
it. Building it column by column with inverse dynamics costs `O(n²)` too but with a larger constant;
CRBA does it directly and cheaply.

## How it works (intuition)
Treat everything outboard of a joint as one rigid "composite" body. Accelerating that joint by one
unit with all other joints held still requires a wrench equal to the composite inertia times the
joint's motion. Projecting that wrench onto the joint's own axis gives the diagonal entry; carrying
it inward and projecting on each ancestor axis gives the off-diagonal entries of the same column.

## Key parameters
- **Link inertial parameters** — mass, center of mass, inertia tensor.
- **Joint axes and offsets** — the chain geometry.

## Reference
R. Featherstone, *Rigid Body Dynamics Algorithms* (2008), Ch. 6; M. W. Walker, D. E. Orin,
"Efficient Dynamic Computer Simulation of Robotic Mechanisms," *ASME J. Dyn. Sys. Meas. Control*,
104(3), 1982.

## See also
`ArticulatedBodyAlgorithm` (shares the spatial algebra), `RecursiveNewtonEuler` (column-by-column
alternative and cross-check), `ChainDynamicsModel` (M29), `OperationalSpaceControl` (M18),
`MomentumObserver` (M16).
