# Time-Optimal Path Parameterization (TOPP) — Overview

## What it is
Given a **fixed geometric path** (the shape the end-effector or joints must follow) and the robot's
**actuator limits**, TOPP computes the fastest possible timing law `s(t)` to traverse that path
without violating any velocity or torque bound. It answers "how fast can I run *this* curve?" — the
geometry is frozen, only the speed along it is optimized.

## Why it matters (embedded)
Separating *where* to go (path planning) from *how fast* to go (parameterization) is a powerful
decomposition. Once a collision-free path exists, TOPP squeezes maximum throughput out of the
hardware — critical for cycle-time-bound industrial robots — while provably respecting drive
saturation, so the plan never asks for torque the motors cannot deliver. The reachability variant
used here is a fixed number of closed-form steps per grid stage: bounded memory, no iterative solver,
deterministic planning time.

## How it works (intuition)
Reparameterize the dynamics in terms of the scalar path coordinate `s`: with `x = ṡ²` and `u = s̈`,
every joint torque becomes affine, `τ = a(s)·u + b(s)·x + c(s)`, and a joint speed limit becomes an upper
bound on `x`. On a grid of `s`, holding `u` constant per stage makes `x` evolve linearly
(`x_{i+1} = x_i + 2Δ·u_i`). TOPP-RA then works on **sets of admissible speeds**:
- a **backward pass** computes, from the end at rest, the *controllable set* `K_i = [lo_i, hi_i]` — the
  speeds at `s_i` from which the rest of the path can still be completed. Each `K_i` is the range of `x`
  over a small polygon in the `(x, u)` plane (stage constraints plus "land inside `K_{i+1}`"), i.e. two
  tiny **2-variable** linear programs, solved exactly by eliminating `u`;
- a **forward pass** starts at rest and greedily picks the largest acceleration that keeps the next
  speed inside `K_{i+1}` — a 1-D linear program per stage.

The result is bang-bang-like (maximum acceleration, then maximum deceleration, with speed-limited
plateaus), and the stage times follow exactly from the average of the speeds at both ends. It avoids
the fragile switch-point search of the classical phase-plane (Bobrow) and numerical-integration methods.

## Key parameters
- **Path geometry** — `q(s)` and its first/second derivatives along `s ∈ [0, 1]`.
- **Velocity & torque limits** — per-joint bounds; the path coefficients `a, b, c` come from inverse
  dynamics (three RNEA passes per grid point).
- **Grid resolution** — number of `s` samples; constraints hold exactly at grid points, with an
  `O(Δ)` excursion between them.

## Reference
H. Pham, Q.-C. Pham, "A New Approach to Time-Optimal Path Parameterization Based on Reachability
Analysis," *IEEE Trans. Robotics* 34(3), 2018 (TOPP-RA — the algorithm specified here).
Background: J. Bobrow, S. Dubowsky, J. Gibson, "Time-Optimal Control of Robotic Manipulators Along
Specified Paths," *IJRR* 4(3), 1985 (phase-plane method); Q.-C. Pham, "A General, Fast, and Robust
Implementation of the Time-Optimal Path Parameterization Algorithm," *IEEE Trans. Robotics* 30(6), 2014
(numerical-integration TOPP, not TOPP-RA).

## See also
`CartesianSlerpInterpolation` (task-space path, mapped to `q(s)` through IK),
`ChainDynamicsModel` / `InverseDynamicsModel` (M29, torque coefficients),
`CubicSplineTrajectory` (resample the result for the servo loop), `SCurveProfile` (heuristic
limit-respecting alternative).
