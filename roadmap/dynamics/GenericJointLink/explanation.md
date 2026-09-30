# Generic Joint Link — Overview

## What it is
One link description that covers both revolute (rotating) and prismatic (sliding) joints and also
carries joint limits and the actuator's reflected inertia (armature).

## Why it matters (embedded)
Real machines are rarely all-revolute: SCARA arms, gantries, linear axes and hydraulic rams contain
sliding joints. Limits are needed by every planner and inverse-kinematics solver, and the armature of
geared actuators often dominates the effective inertia of small arms — ignoring it makes
model-based control markedly worse.

## How it works (intuition)
Every one-degree-of-freedom joint moves along a screw axis: a revolute joint spends its motion in
the angular channel, a prismatic joint in the linear channel. The kinematic and dynamic recursions
only need to know which channel the joint drives; for prismatic joints the torque projection becomes
a force projection and an extra Coriolis term appears when the slider rides on a rotating link.
Armature adds a rotor inertia that only the joint itself feels.

## Key parameters
- **type** — revolute or prismatic.
- **axis** — unit rotation axis or slide direction.
- **limits** — position, velocity and effort bounds.
- **armature** — reflected actuator inertia.

## Reference
J. J. Craig, *Introduction to Robotics: Mechanics and Control*, 4th ed., Ch. 3 and 6;
R. Featherstone, *Rigid Body Dynamics Algorithms* (2008), Ch. 4 (joint models).

## See also
`RecursiveNewtonEuler`, `ArticulatedBodyAlgorithm`, `CompositeRigidBodyAlgorithm` (M28),
`ChainPoseKinematics` (M30), `DenavitHartenberg` (M7).
