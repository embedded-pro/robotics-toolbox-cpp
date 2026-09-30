# Hybrid Position/Force Control — Implementation Pseudocode

> Roadmap ref: #M19 (Tier 4) · Target: `robotics/controllers/manipulator` · Namespace `controllers` · Type: `float` (templated on `T`, instantiated for `float` only)

Force convention: `fMeasured` and `fd` are the wrench the **tool applies on the environment** (base
frame, `(f; n)` about the tool point) — i.e. `−fExternal` of `ImpedanceControl`.

## Data structures

```
template<typename T, std::size_t Dof, std::size_t TaskDim>   # static_assert(std::is_floating_point_v<T>); instantiated for float
class HybridPositionForceControl:                             # TaskDim = 6 (pose) or ≤ 3 (position; 2 = planar xy)
    const dynamics::EulerLagrangeDynamics<T, Dof>&       model      # C q̇, g — typically ChainDynamicsModel (M29)
    const kinematics::JacobianProvider<T, TaskDim, Dof>& jacobian   # J, tool pose — ChainTaskJacobian (M8)
    math::SquareMatrix<T, TaskDim> Rc        # base ← constraint rotation: blkdiag(R, R) for 6, R for 3, 2-D rotation for 2
    math::SquareMatrix<T, TaskDim> S         # diagonal selection in the constraint frame: 1 = motion, 0 = force
    Gains gains                              # { Kp, Kd (motion); Kf, Ki, Kdf (force); integralLimit }
    math::Vector<T, TaskDim> forceIntegral   # ∫eF dt in the constraint frame (STATE)
    T dt
```

## Interface

```
struct Gains { SquareMatrix Kp, Kd, Kf, Ki, Kdf; TaskVector integralLimit; }   # integralLimit ≥ 0

HybridPositionForceControl(const dynamics::EulerLagrangeDynamics<T,Dof>& model,
                           const kinematics::JacobianProvider<T,TaskDim,Dof>& jacobian,
                           const SquareMatrix& Rc, const SquareMatrix& S, const Gains& gains, T dt)

JointVector ComputeTorque(const JointVector& q, const JointVector& qDot,
                          const kinematics::SE3Transform<T>& desiredPose, const TaskVector& xdDot,
                          const TaskVector& fMeasured, const TaskVector& fd)      # hot path
void Reset()                                                                      # clears force integral
```

## Algorithm (pseudocode)

```
function ComputeTorque(q, qDot, desiredPose, xdDot, fMeasured, fd):   # OPTIMIZE_FOR_SPEED; gains.* unqualified
    J    = jacobian.Jacobian(q)
    xDot = J·qDot
    # --- base-frame errors (TaskError.hpp, see ImpedanceControl) rotated into the constraint frame ---
    eX    = Rcᵀ·TaskError(desiredPose, jacobian.ToolPose(q))
    eXDot = Rcᵀ·(xdDot − xDot)
    vC    = Rcᵀ·xDot
    fdC   = Rcᵀ·fd
    eF    = fdC − Rcᵀ·fMeasured
    # --- motion subspace: PD ---
    Fmotion = Kp·eX + Kd·eXDot
    # --- force subspace: PI + velocity damping, integral clamped (anti-windup) ---
    forceIntegral = clamp(forceIntegral + (I − S)·eF·dt, −integralLimit, +integralLimit)   # element-wise
    Fforce        = fdC + Kf·eF + Ki·forceIntegral − Kdf·vC
    # --- complementary partition in the constraint frame, rotate back, map to joints ---
    F = Rc·(S·Fmotion + (I − S)·Fforce)
    return Jᵀ·F + model.ComputeCoriolisTerms(q, qDot) + model.ComputeGravityTerms(q)
```

## Complexity & memory

- Time: `O(TaskDim·Dof)` for `J·q̇` and `Jᵀ·F`; `O(TaskDim²)` for the rotations; model terms `O(Dof)` each.
- Memory: `O(TaskDim²)` gains + `O(TaskDim)` integral state; no heap.

## Numerical / embedded notes

- `S` is diagonal with 0/1 entries (asserted in the constructor), so `S² = S` and `S·(I−S) = 0`: the
  motion and force loops act on disjoint **constraint-frame** axes and never fight (Raibert–Craig).
  `Rc` must be orthonormal; it aligns those axes with the contact surface (peg-in-hole: normal =
  force-controlled, insertion/tangential = motion-controlled). Rotating with `Rc` only (no translation)
  keeps the wrench reference point at the tool.
- Force loop = **PI + damping**: the integral removes steady force error on a stiff environment; the
  `−Kdf·vC` term damps motion along force axes (contact impacts, stiff-surface oscillation). The integral
  accumulates only on force axes and is clamped to `±integralLimit` (anti-windup); call `Reset()` on
  contact loss.
- **Stability caution (An & Hollerbach, 1987):** kinematic hybrid control without dynamic compensation
  can be unstable. This law compensates `g(q)` and `C(q,q̇)q̇` and maps
  with `Jᵀ` only (no inverse, so it passes through singularities); for full dynamic decoupling of the
  subspaces use the `Λ`-weighted form of `OperationalSpaceControl` (Khatib 1987).
- The injected interfaces are a virtual seam for StrictMock tests; hard real-time builds may template
  the controller on the concrete `ChainDynamicsModel` / `ChainTaskJacobian` to avoid virtual calls
  (AGENTS.md: no virtual calls in real-time paths).
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `robotics/controllers/manipulator/HybridPositionForceControl.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `ComputeTorque`, and
  `extern template class HybridPositionForceControl<float, 2, 2>;` / `<float, 6, 6>` under
  `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `robotics/controllers/manipulator/HybridPositionForceControl.cpp` →
  `template class HybridPositionForceControl<float, 2, 2>;` / `<float, 6, 6>`
- Test: `robotics/controllers/manipulator/test/TestHybridPositionForceControl.cpp`
- Doc: `doc/controllers/manipulator/HybridPositionForceControl.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `robotics_add_coverage_sources`;
  `TestHybridPositionForceControl.cpp` → the `_test` target.
- New module: create `robotics/controllers/manipulator/CMakeLists.txt` via `robotics_add_header_library(...)`,
  add a `test/` subdir, register it in `robotics/CMakeLists.txt`, and add a
  `doc/controllers/manipulator/` folder.
- Depends on: M6 (`SE3Transform`), M8 (`JacobianProvider`), M29 (`ChainDynamicsModel`); `TaskError.hpp`
  (specified in M17, deployed with whichever of M17/M18/M19 lands first).
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
