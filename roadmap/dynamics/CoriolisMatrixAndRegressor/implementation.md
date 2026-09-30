# Coriolis Matrix and Inertial Regressor — Implementation Pseudocode

> Roadmap ref: #M31 (Tier 3) · Target: `robotics/dynamics` · Namespace `dynamics` · Type: `float` (templated on `T`, instantiated for `float` only)

Plain RNEA only yields the Coriolis *vector* `C(q,q̇)q̇`. Passivity-based control (Slotine–Li, M20)
needs `C(q,q̇)q̇_r` for a reference velocity `q̇_r ≠ q̇`, the momentum observer (M16) needs `Cᵀ(q,q̇)q̇`,
and identification (M22) plus adaptive control need the dynamics written linearly in the inertial
parameters, `Y·π`. One modified RNEA pass provides all three, with the **Christoffel-consistent**
`C` (so `Ṁ − 2C` is skew-symmetric).

## Data structures

```cpp
# internal spatial algebra uses Featherstone ordering, motion (ω; v), force (n; f), link frames,
# exactly like the shipped ArticulatedBodyAlgorithm; nothing here is exposed as a public 6-vector.

template<typename T, std::size_t NumLinks>        # static_assert(std::is_floating_point_v<T>); instantiated for float
class ModifiedRecursiveNewtonEuler:               # stateless
    JointVector InverseDynamics(links, q, q̇, q̇r, q̈r, gravity) const   # = M q̈r + C(q,q̇) q̇r + g

template<typename T, std::size_t NumLinks>
class CoriolisMatrix:                             # stateless
    SquareMatrix<T, NumLinks> Compute(links, q, q̇) const               # Christoffel C(q, q̇)
    JointVector              TransposeTimesVelocity(links, q, q̇) const # Cᵀ(q, q̇) q̇

template<typename T, std::size_t Dof, std::size_t NumParams>
class InertialRegressor:                          # interface, virtual ~InertialRegressor() = default
    virtual Matrix<T, Dof, NumParams> Compute(q, q̇, q̇r, q̈r) const = 0  # Y with Y·π = M q̈r + C q̇r + g

template<typename T, std::size_t NumLinks>
class ChainInertialRegressor : public InertialRegressor<T, NumLinks, 10 * NumLinks>:
    const LinkArray& links;  Vector3 gravity
    static Vector<T, 10 * NumLinks> Parameters(const LinkArray& links)
        # per link π_i = (m, m·c_x, m·c_y, m·c_z, I_xx, I_xy, I_xz, I_yy, I_yz, I_zz),
        # I = inertia about the JOINT origin: I_c + m·Skew(c)·Skew(c)ᵀ (link frame)
```

## Algorithm (pseudocode)

```cpp
# helpers (6-vectors in (ω; v) / (n; f) ordering, 6×6 in 3×3 blocks):
#   crm(v) = [[ω×, 0], [v×, ω×]]        crf(v) = −crm(v)ᵀ = [[ω×, v×], [0, ω×]]
#   barcrf(h) for force h = (n; f):   barcrf(h)·w = crf(w)·h  ⇒ barcrf(h) = [[−n×, −f×], [−f×, 0]]
#   X_i = link-i-from-parent motion transform (same as ABA);  S_i = (z_i; 0)
#   bodyC(I, v) = ½·( crf(v)·I − I·crm(v) + barcrf(I·v) )       # skew-compatible body Coriolis

function InverseDynamics(links, q, q̇, q̇r, q̈r, g):             # OPTIMIZE_FOR_SPEED
    v_−1 = vr_−1 = 0;  a_−1 = (0; −g)                           # gravity as base acceleration
    for i in 0..N−1:
        v_i  = X_i·v_{i−1}  + S_i·q̇[i]
        vr_i = X_i·vr_{i−1} + S_i·q̇r[i]
        a_i  = X_i·a_{i−1}  + S_i·q̈r[i] + crm(v_i)·S_i·q̇r[i]    # J_i q̈r + J̇_i q̇r (J̇ built with q̇)
        f_i  = I_i·a_i + bodyC(I_i, v_i)·vr_i
    for i = N−1 .. 0:
        τ[i] = S_iᵀ·f_i
        if i > 0: f_{i−1} += X_iᵀ·f_i
    return τ

function CoriolisMatrix.Compute(links, q, q̇):                  # O(N²): one pass per column
    for j: column j = InverseDynamics(links, q, q̇, e_j, 0, g = 0)
function TransposeTimesVelocity(links, q, q̇):  return Transpose(Compute(links, q, q̇))·q̇

# regressor: bodyC and I·v are linear in π_i, via A(v)·π_i = I·v with v = (ω; u):
#   A(v) = [[ 0₃ₓ₁, −u×,  L(ω) ],
#           [ u,     ω×,   0₃ₓ₆ ]],   L(ω) = [[ωx, ωy, ωz, 0, 0, 0], [0, ωx, 0, ωy, ωz, 0], [0, 0, ωx, 0, ωy, ωz]]
function ChainInertialRegressor.Compute(q, q̇, q̇r, q̈r):         # O(N²)
    forward pass for v_i, vr_i, a_i as above
    for i in 0..N−1:
        K = A(a_i) + ½·( crf(v_i)·A(vr_i) − A(crm(v_i)·vr_i) + crf(vr_i)·A(v_i) )   # 6×10
        for j = i .. 0:                                          # carry towards the base
            Y[j, 10i .. 10i+9] = S_jᵀ·K
            if j > 0: K = X_jᵀ·K
    return Y
```

## Complexity & memory

- `InverseDynamics`: `O(N)`, about twice an RNEA pass.
- `CoriolisMatrix::Compute` and the regressor: `O(N²)`.
- Memory: `N` spatial velocities/accelerations and one `6×10` block; stack only; no recursion.

## Numerical / embedded notes

- With `q̇r = q̇, q̈r = q̈` the pass reduces to ordinary inverse dynamics — test against RNEA.
- `C` is the Christoffel form: `C(q,q̇)q̇` equals the RNEA bias and `Ṁ − 2C` is skew-symmetric; both
  were verified numerically against finite-difference Christoffel symbols (residual ~1e-11 in double).
- The 10-parameter-per-link vector is not identifiable as a whole; M22 reduces it to base parameters.
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Headers: `robotics/dynamics/ModifiedRecursiveNewtonEuler.hpp`, `CoriolisMatrix.hpp`,
  `InertialRegressor.hpp` (interface), `ChainInertialRegressor.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on the passes, and `extern template`
  declarations for `<float, 2>` and `<float, 3>` under `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD`.
- Coverage: matching `.cpp` files with the same instantiations.
- Tests: `robotics/dynamics/test/TestCoriolisMatrix.cpp`, `TestChainInertialRegressor.cpp`
- Doc: `doc/dynamics/CoriolisMatrixAndRegressor.md` (per `doc/TEMPLATE.md`)
- Shares the spatial helpers factored out for M28 (CRBA) and the shipped ABA.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
