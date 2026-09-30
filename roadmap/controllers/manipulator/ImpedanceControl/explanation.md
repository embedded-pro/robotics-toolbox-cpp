# Impedance Control — Overview

## What it is
Instead of commanding a position, impedance control makes the end-effector *behave* like a chosen
spring–damper (and, optionally, mass). You program the stiffness, damping, and — with a force sensor —
the inertia the robot presents to the world: a tunable "softness" rather than a rigid trajectory.

## Why it matters (embedded)
Rigid position control shatters on contact: the tiniest position error against a hard surface
generates enormous force. Impedance control lets a robot push, insert, wipe, and physically interact
with people *safely*, with predictable and adjustable compliance. It is the foundation of
collaborative robots, assembly, and teleoperation.

## How it works (intuition)
Measure the pose error between where the tip is and where it should be (orientation as a rotation
vector, never a difference of angles). The simple law turns that error and the velocity error into
the force of a virtual spring–damper, maps it to joint torques with the Jacobian *transpose*, and
adds the gravity torque. Pushed by the environment, the tip settles where the spring balances the
push — it yields along the force by exactly the programmed compliance. It does **not** change the
inertia you feel: that is still the arm's own task-space inertia `Λ(q)`, which varies with posture.
Because it uses `Jᵀ` (never an inverse) and leaves the arm's natural energy exchange intact, it stays
passive and safe near singularities. To make the tip also *feel* like a chosen mass, the second law
measures the contact wrench, computes the tip acceleration the target mass–spring–damper would have,
and realises it through `Λ(q)` with full gravity/Coriolis compensation — the operational-space
machinery. If the chosen mass equals `Λ(q)`, the sensor drops out and it collapses back to the
simple law plus feed-forward.

## Key parameters
- **K (stiffness), D (damping)** — the target spring–damper per task axis.
- **Md (target inertia)** — used only by the inertia-shaping law; needs a measured contact wrench.
- **model, jacobian** — injected dynamics (`g`; plus `M`, `Cq̇` for shaping) and task Jacobian
  provider (`J`, `J̇q̇`, tool pose).
- **fExternal** — wrench exerted by the environment on the tool, from a wrist force/torque sensor.

## Reference
N. Hogan, "Impedance Control: An Approach to Manipulation, Parts I–III,"
*ASME J. Dynamic Systems, Measurement, and Control*, 1985.

## See also
`OperationalSpaceControl` (the `Λ` machinery behind inertia shaping); `HybridPositionForceControl`
(partition force/motion axes); `ComputedTorqueControl` (rigid tracking counterpart).
