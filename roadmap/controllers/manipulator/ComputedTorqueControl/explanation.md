# Computed-Torque Control — Overview

## What it is
The workhorse model-based *tracking* law for manipulators. An inverse-dynamics feedforward plus a
PD correction linearises and decouples the arm into independent unit double integrators:
`τ = M(q)(q̈_d + Kd·ė + Kp·e) + C(q,q̇)q̇ + g(q)`.

## Why it matters (embedded)
A fixed PID tuned at one posture misbehaves at another because a robot's inertia and gravity change
with configuration. Computed-torque erases that variation with a single inverse-dynamics call — one
recursive Newton–Euler pass evaluated at the commanded acceleration, which never forms the mass
matrix. A *single* gain set therefore tracks fast trajectories across the entire workspace — no gain
scheduling, no lookup tables — at a deterministic `O(n)` cost (with diagonal gains).

## How it works (intuition)
Work out the acceleration you actually want: the desired trajectory acceleration plus a PD term that
corrects position and velocity error. Then ask the dynamics model, "what joint torque produces
exactly that acceleration *right now*?" The answer, `M(q)·a + C(q,q̇)q̇ + g(q)`, cancels the arm's
inertial coupling, Coriolis, and gravity, leaving clean, identical second-order error dynamics on
every joint.

## Key parameters
- **model** — injected inverse-dynamics model returning `M(q)q̈ + C(q,q̇)q̇ + g(q)` in one call
  (`ChainDynamicsModel`, M29).
- **Kp, Kd** — error gains for the linearised double integrator; pick `Kd = 2√Kp` for critical damping.

## Reference
M. Spong, S. Hutchinson, M. Vidyasagar, *Robot Modeling and Control*, Ch. 8; Luh, Walker, Paul (1980).

## See also
`PdGravityCompensation` (set-point-only special case); `SlotineLiAdaptiveControl` (adapts unknown
parameters); `ChainDynamicsModel` (M29, the one-pass RNEA model); general-plant feedback linearization in
[numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp).
