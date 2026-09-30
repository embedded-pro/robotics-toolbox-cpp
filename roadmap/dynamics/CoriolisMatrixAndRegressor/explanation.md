# Coriolis Matrix and Inertial Regressor — Overview

## What it is
Two views of the same rigid-body dynamics. The **Coriolis matrix** `C(q,q̇)` is the matrix whose
product with the joint velocity gives the centrifugal and Coriolis torques — chosen in the specific
(Christoffel) form that makes `Ṁ − 2C` skew-symmetric. The **inertial regressor** `Y` rewrites the
torques as a known matrix of motion terms times an unknown vector of inertial parameters (masses,
first moments, inertia-tensor entries).

## Why it matters (embedded)
Adaptive controllers learn payloads online by adjusting the parameter vector, which only works with
the regressor form and with a skew-symmetric `Ṁ − 2C`. Collision-detection observers need `Cᵀq̇`.
Offline identification of a real robot's inertial parameters is a least-squares fit on the
regressor. All of these fall out of one modified recursive pass, at a few times the cost of plain
inverse dynamics.

## How it works (intuition)
Run the recursive Newton-Euler sweep with two velocities side by side — the actual joint velocity and
a "reference" one. The actual velocity decides how the bodies rotate; the reference velocity is what
those rotating bodies are asked to carry. Splitting each body's gyroscopic term symmetrically between
the two produces the Coriolis matrix with the passivity property. Because every body's contribution
is linear in its ten inertial parameters, replacing the numbers by the columns that multiply them
gives the regressor.

## Key parameters
- **Reference velocity/acceleration** — the Slotine–Li reference motion (or the actual motion).
- **Parameter vector** — ten numbers per link, inertia taken about the joint origin.

## Reference
S. Echeandia, P. M. Wensing, "Numerical Methods to Compute the Coriolis Matrix and Christoffel
Symbols for Rigid-Body Systems," *J. Computational and Nonlinear Dynamics*, 16(9), 2021;
G. Niemeyer, J.-J. Slotine, "Performance in Adaptive Manipulator Control," *Int. J. Robotics
Research*, 10(2), 1991; C. Atkeson, C. An, J. Hollerbach, *Int. J. Robotics Research*, 5(3), 1986.

## See also
`SlotineLiAdaptiveControl` (M20), `MomentumObserver` (M16), `DynamicParameterIdentification` (M22),
`CompositeRigidBodyAlgorithm` (M28), `RecursiveNewtonEuler`.
