# Composite Rigid Body Algorithm (CRBA) — Implementation Pseudocode

> Roadmap ref: #M28 (Tier 2) · Target: `robotics/dynamics` · Namespace `dynamics` · Type: `float` (templated on `T`, instantiated for `float` only)

Computes the joint-space mass matrix `M(q)` of a serial chain described by the shipped link model.
`M(q)` is needed by operational-space control (M18), the momentum observer (M16), the chain dynamics
model (M29) and any forward-dynamics path that solves `M q̈ = τ − h`.

## Data structures

```
template<typename T, std::size_t NumLinks>      # static_assert(std::is_floating_point_v<T>); instantiated for float
class CompositeRigidBodyAlgorithm:              # stateless
    using MassMatrix = math::SquareMatrix<T, NumLinks>
    # reuses the spatial-inertia block form {rot, cross, lin} of ArticulatedBodyAlgorithm:
    #   I = [[rot, cross], [crossᵀ, lin]] about the joint origin, link frame, acting on (ω; v)
```

## Interface

```
MassMatrix Compute(const LinkArray& links, const JointVector& q) const          # hot path
```

## Algorithm (pseudocode)

```
function Compute(links, q):                     # OPTIMIZE_FOR_SPEED
    for i: R[i] = RotationAboutAxis(links[i].jointAxis, q[i])
    for i: Ic[i] = RigidBodyInertia(links[i])   # rot = I_c + m·Skew(c)ᵀSkew(c), cross = m·Skew(c), lin = m·I
    # 1. composite inertias, tip → base (same transform as ABA's TransformInertiaToParent)
    for i = N−1 .. 1:
        Ic[i−1] += TransformInertiaToParent(Ic[i], R[i], links[i].parentToJoint)
    # 2. one column per joint: the composite body's wrench for a unit joint acceleration,
    #    carried towards the base and projected on every ancestor axis
    for i = N−1 .. 0:
        z = links[i].jointAxis
        n = Ic[i].rot · z;   f = Ic[i].crossᵀ · z          # wrench (n; f) about joint i, frame i
        M[i][i] = z · n
        j = i
        while j > 0:
            f = R[j] · f;   n = R[j] · n + CrossProduct(links[j].parentToJoint, f)
            j = j − 1
            M[i][j] = M[j][i] = links[j].jointAxis · n
    return M
```

## Complexity & memory

- `O(N²)`: `O(N)` composite-inertia pass plus `N(N−1)/2` wrench transforms (each `O(1)`).
- Memory: `N` spatial inertias (three 3×3 blocks each) + the `N×N` result; stack only; no recursion.

## Numerical / embedded notes

- Factor the spatial helpers now private to `ArticulatedBodyAlgorithm` (rigid-body inertia, inertia
  and wrench transforms) into a shared internal header so ABA and CRBA cannot drift apart.
- `M` is symmetric positive definite for valid links; fill both triangles from one computation so the
  result is exactly symmetric in `float`.
- Joint armature / reflected rotor inertia (M1 fields) is added to the diagonal: `M[i][i] += armature[i]`.
- To solve `M q̈ = b`, prefer a Cholesky factorization (numerical-toolbox `CholeskyDecomposition`).
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Header: `robotics/dynamics/CompositeRigidBodyAlgorithm.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Compute`, and
  `extern template class CompositeRigidBodyAlgorithm<float, 1>;` / `<float, 2>` / `<float, 3>` under
  `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `robotics/dynamics/CompositeRigidBodyAlgorithm.cpp` → the same instantiations.
- Test: `robotics/dynamics/test/TestCompositeRigidBodyAlgorithm.cpp`
- Doc: `doc/dynamics/CompositeRigidBodyAlgorithm.md` (per `doc/TEMPLATE.md`)
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
