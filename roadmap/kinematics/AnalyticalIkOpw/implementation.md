# Analytical IK — OPW (Ortho-Parallel 6R with Spherical Wrist) — Implementation Pseudocode

> Roadmap ref: #M21 (Tier 4) · Target: `robotics/kinematics` · Namespace `kinematics` · Type: `float` (templated on `T`, instantiated for `float` only)

Closed-form IK of Brandstötter, Angerer & Hofbaur (2014) for the industrial 6R layout (KUKA, ABB,
Fanuc, Stäubli, …): axes 2 and 3 parallel and orthogonal to axis 1, spherical wrist. Seven lengths plus
per-joint offsets and signs describe the arm; no DH table and no frame convention to pin.

## Data structures

```
template<typename T>                            # static_assert(std::is_floating_point_v<T>); instantiated for float
struct OpwParameters:                           # model zero pose: upper arm and forearm point along +z
    T a1                  # axis 1 → axis 2, radial (x)
    T a2                  # forearm offset ⊥ forearm, axis 3 → wrist line (x at zero pose)
    T b                   # lateral offset of the arm plane (y)
    T c1                  # base → axis 2, along axis 1 (z)
    T c2                  # upper arm, axis 2 → axis 3
    T c3                  # forearm, axis 3 → wrist centre
    T c4                  # wrist centre → flange, along axis 6
    std::array<T, 6>      offsets              # θ = sign·q − offset  (model angle from joint value)
    std::array<int8_t, 6> signs                # ±1

template<typename T>
struct OpwSolution:
    math::Vector<T, 6> q                        # joint values, wrapped to (−π, π]
    bool               wristSingular            # sin θ5 ≈ 0: only θ4 ± θ6 is determined

template<typename T>
using OpwSolutions = std::array<std::optional<OpwSolution<T>>, 8>
    # slot j (0..3): arm branch j with wrist θ5 ≥ 0; slot j + 4: same arm, flipped wrist
    # arm j: 0/1 shoulder front (elbow a/b), 2/3 shoulder back (elbow a/b)

template<typename T>
class AnalyticalIkOpw:
    OpwParameters<T> parameters
```

Forward model (defines the parameters; `Forward` implements it):
`T(q) = Rz(θ1)·Trans(a1, b, c1)·Ry(θ2)·Trans(0, 0, c2)·Ry(θ3)·Trans(a2, 0, c3)·Rz(θ4)·Ry(θ5)·Rz(θ6)·Trans(0, 0, c4)`.

## Interface

```
explicit AnalyticalIkOpw(const OpwParameters<T>& parameters)
OpwSolutions<T>  Solve(const SE3Transform<T>& flange) const        # all real branches; hot path
SE3Transform<T>  Forward(const JointVector& q) const                # closed-form FK (round trips, tests)
```

## Algorithm (pseudocode)

```
function Forward(q):
    θ = signs ⊙ q − offsets;  k = √(a2² + c3²);  ψ3 = atan2(a2, c3)
    x = c2·sin θ2 + k·sin(θ2 + θ3 + ψ3) + a1;  z = c2·cos θ2 + k·cos(θ2 + θ3 + ψ3)
    C = (x·cos θ1 − b·sin θ1, x·sin θ1 + b·cos θ1, z + c1)          # wrist centre
    R = Rz(θ1)·Ry(θ2 + θ3)·Rz(θ4)·Ry(θ5)·Rz(θ6)
    return { R, C + c4·R·ẑ }

function Solve(flange):                         # OPTIMIZE_FOR_SPEED — closed form, no iteration
    (R, p) = flange;  C = p − c4·R·ẑ
    ρ² = Cx² + Cy² − b²;  if ρ² < 0: return all empty
    n = √ρ² − a1;  z = Cz − c1;  k² = a2² + c3²;  ψ3 = atan2(a2, c3)
    θ1ᶠ = atan2(Cy, Cx) − atan2(b, n + a1)                 # shoulder front
    θ1ᵇ = atan2(Cy, Cx) + atan2(b, n + a1) − π             # shoulder back
    s1² = n² + z²;           s2² = (n + 2·a1)² + z²
    β1 = acosV((s1² + c2² − k²) / (2·s1·c2));  γ1 = acosV((s1² − c2² − k²) / (2·c2·k))
    β2 = acosV((s2² + c2² − k²) / (2·s2·c2));  γ2 = acosV((s2² − c2² − k²) / (2·c2·k))
    arm[0] = (θ1ᶠ,  atan2(n, z) − β1,            γ1 − ψ3)
    arm[1] = (θ1ᶠ,  atan2(n, z) + β1,           −γ1 − ψ3)
    arm[2] = (θ1ᵇ, −atan2(n + 2·a1, z) − β2,      γ2 − ψ3)
    arm[3] = (θ1ᵇ, −atan2(n + 2·a1, z) + β2,     −γ2 − ψ3)
    for j in 0..3 with arm[j] valid (its β, γ defined):
        (θ1, θ2, θ3) = arm[j]
        W = Transpose(Rz(θ1)·Ry(θ2 + θ3)) · R              # = Rz(θ4)·Ry(θ5)·Rz(θ6)
        s5 = √max(0, 1 − W₂₂²)
        if s5 > ε:                                          # regular wrist: two branches
            w  = (atan2(W₁₂, W₀₂), atan2(s5, W₂₂), atan2(W₂₁, −W₂₀))
            slot[j]     = Emit(arm[j], w, false)
            slot[j + 4] = Emit(arm[j], (w0 + π, −w1, w2 − π), false)
        else:                                               # axes 4 and 6 aligned
            θ4 = 0                                          # tie-break; caller may redistribute
            w = W₂₂ > 0 ? (θ4, 0, atan2(W₁₀, W₁₁) − θ4)     # θ4 + θ6 fixed
                        : (θ4, π, θ4 − atan2(−W₁₀, W₁₁))    # θ4 − θ6 fixed
            slot[j] = Emit(arm[j], w, true);  slot[j + 4] = empty
    return slot

function acosV(c):  |c| > 1 + tol ⇒ invalid (branch unreachable);  else acos(clamp(c, −1, 1))
function Emit(arm, w, singular):  q = signs ⊙ ((arm, w) + offsets), each wrapped to (−π, π]
```

## Complexity & memory

- `O(1)`: a fixed set of `sqrt`/`acos`/`atan2` for the arm (`k`, `ψ3` precomputed at construction),
  then 3 `atan2` and one 3×3 product per arm branch.
- Memory: 8 optional solutions on the stack; no iteration, no Jacobian, no linear solve.

## Numerical / embedded notes

- **Why not a planar two-link elbow:** `a1`, `b` and the forearm offset `a2` break the planar
  law-of-cosines triangle used for offset-free arms; OPW folds them into `n`, `n + 2a1` and the virtual
  forearm length `k = √(a2² + c3²)` with angle `ψ3`, which is why it covers KUKA/ABB/Fanuc arms.
- **Reachability:** `|acos argument| > 1` prunes that arm branch; `ρ² < 0` (wrist centre inside the
  `b`-cylinder) prunes all. Generic poses yield 8 or 4 solutions (shoulder-back branch often out of reach).
- **Boundary precision:** at full stretch `acos` of ≈ 1 amplifies rounding (`δθ ≈ √(2ε)`, ≈ 5e-4 rad in
  `float`); tolerate it in tests, never feed `acos` an unclamped argument.
- **Wrist singularity** (`sin θ5 ≈ 0`): the two wrist branches coincide; one solution is emitted with
  `θ4 = 0` and `wristSingular = true`. **Shoulder singularity** (`Cx = Cy = 0`, `b = 0`): `atan2(0, 0) = 0`,
  any `θ1` is valid — returned as `θ1 = 0`.
- Joint limits are not applied; the caller filters/chooses (e.g. nearest to the current `q`).
- The target is the **flange** pose; for a tool use `target · tool⁻¹`.
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Header: `robotics/kinematics/AnalyticalIkOpw.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Solve`, and
  `extern template class AnalyticalIkOpw<float>;` under `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `robotics/kinematics/AnalyticalIkOpw.cpp` → `template class AnalyticalIkOpw<float>;`
- Test: `robotics/kinematics/test/TestAnalyticalIkOpw.cpp`
- Doc: `doc/kinematics/AnalyticalIkOpw.md` (per `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `robotics_add_coverage_sources`;
  `TestAnalyticalIkOpw.cpp` → the `_test` target.
- Depends on: M6 (`SE3Transform`).
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
