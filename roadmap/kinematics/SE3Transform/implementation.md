# SE(3) Transform, Twists and Wrenches — Implementation Pseudocode

> Roadmap ref: #M6 (Tier 2) · Target: `robotics/kinematics` · Namespace `kinematics` · Type: `float` (templated on `T`, instantiated for `float` only)

numerical-toolbox provides `Matrix`, `Quaternion` and the 3D geometry helpers but no rigid-body
transform, so this type lives here, in `kinematics`. It fixes the **one convention used by every
other spec**: 6-vectors are ordered **linear part first** — twists `(v; ω)`, wrenches `(f; n)`.

## Data structures

```cpp
template<typename T> using Vector3 = math::Vector<T, 3>
template<typename T> using Matrix3 = math::SquareMatrix<T, 3>
template<typename T> using Vector6 = math::Vector<T, 6>      # twist (v; ω) or wrench (f; n)
template<typename T> using Matrix6 = math::SquareMatrix<T, 6>

template<typename T>                  # static_assert(std::is_floating_point_v<T>); instantiated for float
struct SE3Transform:
    Matrix3<T> R                      # rotation, orthonormal, det = +1
    Vector3<T> p                      # translation
```

## Interface

```text
static SE3Transform Identity()
SE3Transform operator*(const SE3Transform& rhs) const        # composition
SE3Transform Inverse() const
Vector3<T>   Apply(const Vector3<T>& point) const            # R·x + p
Vector3<T>   Rotate(const Vector3<T>& direction) const       # R·x
Matrix6<T>   Adjoint() const                                 # twist map for (v; ω)
Vector6<T>   TransformTwist(const Vector6<T>& twist) const   # Adjoint()·twist
Vector6<T>   TransformWrench(const Vector6<T>& wrench) const # dual map, see below

static SE3Transform Exp(const Vector6<T>& twist, T theta)    # e^{[ξ]θ}; hot path for PoE
Vector6<T>          Log() const                               # inverse of Exp with theta = 1
static Vector6<T>   PoseError(const SE3Transform& target, const SE3Transform& current)
```

## Algorithm (pseudocode)

```text
function operator*(rhs):   return { R·rhs.R, R·rhs.p + p }
function Inverse():        return { Rᵀ, −Rᵀ·p }

function Adjoint():                         # V_A = Ad_T · V_B for V = (v; ω)
    return [[ R,  Skew(p)·R ],
            [ 0,  R         ]]

function TransformWrench(w = (f; n)):       # power-consistent dual of Adjoint
    fA = R·f
    nA = R·n + CrossProduct(p, R·f)
    return (fA; nA)

function Exp(ξ = (v; ω), θ):                # OPTIMIZE_FOR_SPEED
    if ‖ω‖ < ε:                             # prismatic / pure translation
        return { I, v·θ }
    # general twist: fold the magnitude of ω into θ so the axis is unit
    s = ‖ω‖;  ω̂ = ω / s;  v̂ = v / s;  φ = θ·s
    R = RotationAboutAxis(ω̂, φ)             # reuse Geometry3D (Rodrigues)
    G = I·φ + (1 − cos φ)·Skew(ω̂) + (φ − sin φ)·Skew(ω̂)²
    return { R, G·v̂ }

function Log():                             # returns (v; ω) with ‖ω‖ = rotation angle
    c = clamp((trace(R) − 1) / 2, −1, 1);  φ = acos(c)
    if φ < ε:                               # small angle: first-order
        ω = Vee(R − Rᵀ) / 2;  return (p; ω)
    if π − φ < ε:                           # near π: sin φ → 0, use the diagonal form
        ω̂ = axis from the largest of sqrt((R_ii + 1) / 2), signs from the off-diagonals
    else:
        ω̂ = Vee(R − Rᵀ) / (2 sin φ)
    G⁻¹ = I/φ − Skew(ω̂)/2 + (1/φ − cot(φ/2)/2)·Skew(ω̂)²
    return (G⁻¹·p·φ; ω̂·φ)

function PoseError(target, current):        # base-frame error, matches the geometric Jacobian (M8)
    eP = target.p − current.p
    eR = (SE3Transform{ target.R · current.Rᵀ, 0 }).Log().angular   # rotation vector, |eR| ≤ π
    return (eP; eR)
```

## Complexity & memory

- Compose / inverse / apply: `O(1)` — a handful of 3×3 products.
- `Exp` / `Log`: `O(1)` — one `sin`/`cos` pair (plus `acos` for `Log`).
- Memory: 12 scalars per transform; `Matrix6` only when an adjoint is requested. No heap.

## Numerical / embedded notes

- **Convention (normative for all specs):** twists are `(v; ω)` and wrenches `(f; n)`; the geometric
  Jacobian (M8) has linear rows first; `PoseError` returns `(position; rotation-vector)`.
- `Log` must be the true logarithm: `2·vec(quaternion)` equals `2 sin(φ/2)·ω̂`, which is only a
  small-angle approximation of `φ·ω̂`. For `φ > π` the rotation is re-expressed the short way, so
  `|eR| ≤ π`.
- Handle both singular regions of `Log` (`φ → 0` and `φ → π`) explicitly; `acos` near ±1 loses
  precision in `float`, so derive `φ` from `atan2(‖Vee(R − Rᵀ)‖/2, (trace R − 1)/2)` when accuracy
  near 0 matters.
- Composition drifts off SO(3) over long chains of products in `float`; re-orthonormalize (Gram–Schmidt
  or via `Quaternion` normalization) when a transform is integrated over time, not after single
  compositions.
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Header: `robotics/kinematics/SE3Transform.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Exp`/`operator*`, and
  `extern template struct SE3Transform<float>;` under `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `robotics/kinematics/SE3Transform.cpp` → `template struct SE3Transform<float>;`
- Test: `robotics/kinematics/test/TestSE3Transform.cpp`
- Doc: `doc/kinematics/SE3Transform.md` (per `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `robotics_add_coverage_sources`;
  `TestSE3Transform.cpp` → the `_test` target.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
