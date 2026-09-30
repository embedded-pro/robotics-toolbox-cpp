# Pose Inverse Kinematics (Weighted Damped Least Squares) — Implementation Pseudocode

> Roadmap ref: #M13 (Tier 3) · Target: `robotics/kinematics` · Namespace `kinematics` · Type: `float` (templated on `T`, instantiated for `float` only)

Model-agnostic: the arm is a `JacobianProvider<T, 6, Dof>` (M8), so the same solver serves the link
chain (`ChainTaskJacobian`, M30/M8), DH (`DhTaskJacobian`, M7) and PoE (M15).

## Data structures

```
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

template<typename T, std::size_t Dof>
class PoseInverseKinematics:
    const JacobianProvider<T, 6, Dof>& arm      # Jacobian(q), ToolPose(q); injected
    PoseIkConfig<T, Dof>               config
```

## Interface

```
PoseInverseKinematics(const JacobianProvider<T, 6, Dof>& arm, const PoseIkConfig<T, Dof>& config)
PoseIkResult<T, Dof>  Solve(const SE3Transform<T>& target, const JointVector& q0) const   # hot path
```

## Algorithm (pseudocode)

```
function WeightedError(target, q):
    e = SE3Transform::PoseError(target, arm.ToolPose(q))   # (Δp; true log-map rotation vector), M6
    return (e.linear; ρ·e.angular)

function Solve(target, q0):                     # OPTIMIZE_FOR_SPEED
    q = q0
    for iter in 0..maxIterations-1:
        eW = WeightedError(target, q)
        if ‖eW‖ < tolerance: return { q, ‖eW‖, iter, true }
        JW = arm.Jacobian(q) with rows 3–5 scaled by ρ          # W·J
        A0 = JW·JWᵀ                                              # 6×6
        λ² = Damping(A0)
        y  = LuDecomposition(A0 + λ²·I₆).Solve(eW)              # never invert explicitly
        Δq = JWᵀ·y                                               # = Jᵀ W (W J Jᵀ W + λ² I)⁻¹ W e
        if ‖Δq‖ > maxStep: Δq = Δq · maxStep / ‖Δq‖
        q = clamp(q + Δq, lowerLimits, upperLimits)
    eFinal = ‖WeightedError(target, q)‖                          # recomputed at the returned q
    return { q, eFinal, maxIterations, eFinal < tolerance }

function Damping(A0):                           # Nakamura–Hanafusa / Chiaverini adaptive form
    if manipulabilityThreshold == 0: return λ₀²
    w = sqrt(max(0, Determinant(A0)))            # weighted manipulability (≡ 0 when Dof < 6 ⇒ λ = λ₀)
    return w < w₀ ? λ₀²·(1 − (w / w₀)²) : 0
```

## Complexity & memory

- Per iteration: one `ToolPose` + `Jacobian` (`O(Dof)`), `O(36·Dof)` for `JW·JWᵀ`, `O(6³)` LU.
- One extra `ToolPose` after the loop for the final error.
- Memory: one `6×Dof` Jacobian and a `6×6` system; stack only.

## Numerical / embedded notes

- **Damping does not bias the answer:** at a fixed point `JWᵀ·y = 0` with `JW` of full row rank forces
  `y = 0`, hence `e = 0`. For a reachable, non-singular target iterative DLS converges to the exact pose;
  `λ` only slows convergence (linear instead of quadratic). The residual is non-zero only for
  unreachable targets or at singularities, where DLS returns the damped least-squares compromise.
- **Units:** position error is in m, rotation error in rad. `ρ` (≈ the arm's reach or tool length) converts
  radians to metres in both the stopping test and the step. With `λ = 0` and invertible `J`, `W` cancels
  (`Δq = J⁻¹e`); it matters for damping, redundancy and unreachable targets.
- **Orientation error** is `SE3Transform::PoseError` — the true log map, `|eR| ≤ π`, short way round.
  `2·vec(error quaternion)` equals `2 sin(φ/2)·n̂`, which is only a small-angle approximation of `φ·n̂`
  (0.959 vs 1 rad at φ = 1 rad) and must not be used.
- **Adaptive damping:** `λ = 0` away from singularities (fast Newton convergence), rising smoothly to
  `λ₀` as `w → 0`. Step clamping bounds the joint jump per iteration; joint limits are enforced by
  clamping (projected iteration).
- Seed `q0` from the previous control cycle for warm-started, few-iteration convergence.
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Header: `robotics/kinematics/PoseInverseKinematics.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Solve`, and
  `extern template class PoseInverseKinematics<float, 6>;` / `<float, 7>` under
  `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `robotics/kinematics/PoseInverseKinematics.cpp` → the same instantiations.
- Test: `robotics/kinematics/test/TestPoseInverseKinematics.cpp`
- Doc: `doc/kinematics/PoseInverseKinematics.md` (per `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `robotics_add_coverage_sources`;
  `TestPoseInverseKinematics.cpp` → the `_test` target.
- Depends on: M6 (`SE3Transform::PoseError`), M8 (`JacobianProvider`); upstream `solvers::LuDecomposition`.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
