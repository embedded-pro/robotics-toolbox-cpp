# Pose Inverse Kinematics (Weighted Damped Least Squares) — Implementation Pseudocode

> Roadmap ref: #M13 (Tier 3) · Target: `robotics/kinematics` · Namespace `kinematics` · Type: `float` (templated on `T`, instantiated for `float` only)

Model-agnostic: the arm is a `JacobianProvider<T, 6, Dof>` (M8), so the same solver serves the link
chain (`ChainTaskJacobian`, M30/M8), DH (`DhTaskJacobian`, M7) and PoE (M15). The iteration itself is
upstream `optimization::LevenbergMarquardt` (NT64); this item supplies the IK problem — weighted log-map
pose residual, weighted Jacobian, joint-limit retraction — and the singularity-robust damping policy.

## Data structures

```cpp
template<typename T, std::size_t Dof>          # static_assert(std::is_floating_point_v<T>); instantiated for float
struct PoseIkConfig:
    T           damping                  # λ₀
    T           manipulabilityThreshold  # w₀; 0 ⇒ constant damping λ₀ (adaptive otherwise)
    T           characteristicLength     # ρ (m): W = diag(1,1,1, ρ,ρ,ρ) makes rad commensurate with m
    T           tolerance                # on ‖W·e‖ (m)
    T           maxStep                  # Δmax on ‖Δq‖; +∞ disables
    std::size_t maxIterations
    JointVector lowerLimits, upperLimits # ∓∞ disables

template<typename T, std::size_t Dof>
struct PoseIkResult:
    JointVector q
    T           finalError               # ‖W·PoseError(target, ToolPose(q))‖ at the returned q
    std::size_t iterations
    bool        converged

template<typename T, std::size_t Dof>          # implements optimization::LeastSquaresProblem<T, JointVector, 6, Dof>
class PoseIkProblem:
    const JacobianProvider<T, 6, Dof>& arm;  math::SE3Transform<T> target;  T ρ;  JointVector lower, upper

template<typename T, std::size_t Dof>          # implements optimization::DampingPolicy<T, 6, Dof>
class ManipulabilityDamping:
    T λ₀, w₀                                   # w₀ = 0 ⇒ constant λ₀²

template<typename T, std::size_t Dof>
class PoseInverseKinematics:
    const JacobianProvider<T, 6, Dof>& arm      # Jacobian(q), ToolPose(q); injected
    PoseIkConfig<T, Dof>               config
```

## Interface

```text
PoseInverseKinematics(const JacobianProvider<T, 6, Dof>& arm, const PoseIkConfig<T, Dof>& config)
PoseIkResult<T, Dof>  Solve(const SE3Transform<T>& target, const JointVector& q0) const   # hot path
```

## Algorithm (pseudocode)

```text
# --- the IK problem handed to the upstream solver (residual r = −W·e, so δ = Jᵀ W (W J Jᵀ W + λ²I)⁻¹ W e) ---
function PoseIkProblem::Residual(q):
    e = math::SE3Transform<T>::PoseError(target, arm.ToolPose(q))   # (Δp; true log-map rotation vector), NT53
    return −(e.linear; ρ·e.angular)
function PoseIkProblem::Jacobian(q):  return arm.Jacobian(q) with rows 3–5 scaled by ρ      # W·J
function PoseIkProblem::Retract(q, δ): return clamp(q + δ, lower, upper)                    # projected iteration

function ManipulabilityDamping::DampingSquared(JW):   # Nakamura–Hanafusa / Chiaverini adaptive form
    if w₀ == 0: return λ₀²
    A0 = JW·JWᵀ                                           # 6×6
    w = LuDecomposition(A0) singular ? 0 : sqrt(max(0, Determinant(A0)))   # weighted manipulability
                                                         # (≡ 0 when Dof < 6 or near-singular ⇒ λ = λ₀)
    return w < w₀ ? λ₀²·(1 − (w / w₀)²) : 0

function Solve(target, q0):                     # OPTIMIZE_FOR_SPEED
    problem = PoseIkProblem{ arm, target, ρ, lowerLimits, upperLimits }
    damping = ManipulabilityDamping{ λ₀, w₀ }
    r = optimization::LevenbergMarquardt{ problem, damping, { tolerance, maxStep, maxIterations } }.Solve(q0)
    return { r.x, r.residualNorm, r.iterations, r.converged }   # residual recomputed at the returned q
```

## Complexity & memory

- Per iteration: one `ToolPose` + `Jacobian` (`O(Dof)`), `O(36·Dof)` for `JW·JWᵀ`, `O(6³)` LU (in the
  upstream wide-branch step), plus one more `JW·JWᵀ` and 6×6 LU in the damping policy when `w₀ > 0`.
- One extra `ToolPose` after the loop for the final error (upstream).
- Memory: one `6×Dof` Jacobian and a `6×6` system; stack only.

## Numerical / embedded notes

- **Damping does not bias the answer:** at a fixed point `JWᵀ·y = 0` with `JW` of full row rank forces
  `y = 0`, hence `e = 0`. For a reachable, non-singular target iterative DLS converges to the exact pose;
  `λ` only slows convergence (linear instead of quadratic). The residual is non-zero only for
  unreachable targets, at singularities (DLS returns the damped least-squares compromise) or when a
  joint limit is active.
- **Units:** position error is in m, rotation error in rad. `ρ` (≈ the arm's reach or tool length) converts
  radians to metres in both the stopping test and the step. With `λ = 0` and invertible `J`, `W` cancels
  (`Δq = J⁻¹e`); it matters for damping, redundancy and unreachable targets.
- **Orientation error** is `math::SE3Transform::PoseError` — the true log map, `|eR| ≤ π`, short way round.
  `2·vec(error quaternion)` equals `2 sin(φ/2)·n̂`, which is only a small-angle approximation of `φ·n̂`
  (0.959 vs 1 rad at φ = 1 rad) and must not be used.
- **Adaptive damping:** `λ = 0` away from singularities (fast Newton convergence), rising smoothly to
  `λ₀` as `w → 0`. Step clamping bounds the joint jump per iteration; joint limits are enforced by
  clamping (projected iteration).
- Upstream `LuDecomposition` reports singular below a relative pivot of `1e-6`; keep `λ₀² ≫ 1e-6·‖W·J‖²`
  (assert `λ₀ > 0`) so the damped system never trips it — a tripped `A0` simply means `w = 0`, `λ = λ₀`.
- Seed `q0` from the previous control cycle for warm-started, few-iteration convergence.
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Header: `robotics/kinematics/PoseInverseKinematics.hpp` — `#pragma once`,
  `OPTIMIZE_FOR_SPEED` on `Solve`, and
  `extern template class PoseInverseKinematics<float, 6>;` / `<float, 7>` under
  `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `robotics/kinematics/PoseInverseKinematics.cpp` → the same instantiations.
- Test: `robotics/kinematics/test/TestPoseInverseKinematics.cpp`
- Doc: `doc/kinematics/PoseInverseKinematics.md` (per `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `robotics_add_coverage_sources`;
  `TestPoseInverseKinematics.cpp` → the `_test` target.
- Depends on: NT53 (`math::SE3Transform::PoseError`), NT64 (`optimization::LevenbergMarquardt`),
  M8 (`JacobianProvider`); upstream `solvers::LuDecomposition` (damping policy).
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
