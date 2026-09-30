# Inverse Dynamics with an External Tool Wrench — Overview

## What it is
Inverse dynamics that also accounts for a force and moment applied to the tool by the environment —
a payload's weight, a contact force, a tool reaction.

## Why it matters (embedded)
Force-controlled tasks and payload handling need torques that include the contact wrench, and
collision observers need a model to compare against. Adding the wrench inside the recursion keeps
the cost linear and avoids building the Jacobian just to map one wrench.

## How it works (intuition)
The backward pass of the recursive Newton-Euler algorithm accumulates, from the tip inward, the
wrench each joint must transmit. An external wrench on the last link simply offsets that
accumulation at the tip: the environment already supplies part of what the joints would otherwise
have to provide.

## Key parameters
- **Wrench** — force and moment on the tool, in the base frame, with a fixed sign convention.
- **Tool offset** — where the wrench acts, in the last link's frame.

## Reference
J. Y. S. Luh, M. W. Walker, R. P. C. Paul, "On-Line Computational Scheme for Mechanical Manipulators,"
*ASME J. Dyn. Sys. Meas. Control*, 102(2), 1980; B. Siciliano et al., *Robotics* (2009), Ch. 7.

## See also
`RecursiveNewtonEuler`, `GeometricJacobian` (M8, `Jᵀw` equivalence), `ImpedanceControl` (M17),
`HybridPositionForceControl` (M19), `MomentumObserver` (M16).
