# Product of Exponentials (Screw-Theory FK) — Implementation Pseudocode

> Roadmap ref: #M15 (Tier 3) · Target: `robotics/kinematics` · Namespace `kinematics` · Type: `float` (templated on `T`, instantiated for `float` only)

Screws follow the library convention (M6): `S = (v; ω)`, linear part first.

## Data structures

```
template<typename T, std::size_t N>             # static_assert(std::is_floating_point_v<T>); instantiated for float
class ProductOfExponentials:
    std::array<Vector6<T>, N>  screws           # space-frame screw axes Sᵢ = (vᵢ; ωᵢ) at q = 0
    SE3Transform<T>            home             # M: tool pose at q = 0
```

Screw construction (asserted at construction):
- revolute about unit `ω` through point `a`: `S = (−ω × a; ω)` (pitch 0 ⇒ `ω·v = 0`);
- prismatic along unit `v`: `S = (v; 0)`.

## Interface

```
ProductOfExponentials(const std::array<Vector6<T>, N>& screws, const SE3Transform<T>& home)
SE3Transform<T>   Compute(const JointVector& q) const          # tool pose; hot path
Matrix<T, 6, N>   SpaceJacobian(const JointVector& q) const    # Lynch–Park space Jacobian, (v; ω) rows
FrameChain<T, N>  Frames(const JointVector& q) const           # M30 frame chain for M8
static Matrix<T, 6, N> SpaceToGeometric(const Matrix<T, 6, N>& Js, const Vector3<T>& pTool)
```

## Algorithm (pseudocode)

```
function Compute(q):                            # OPTIMIZE_FOR_SPEED
    A = Identity
    for i in 0..N-1: A = A * SE3Transform::Exp(screws[i], q[i])      # M6
    return A * home                             # e^{[S₁]q₁}···e^{[Sₙ]qₙ}·M

function SpaceJacobian(q):
    A = Identity
    for i in 0..N-1:
        column i = A.Adjoint() · screws[i]      # Ad(e^{[S₁]q₁}···e^{[Sᵢ₋₁]qᵢ₋₁})·Sᵢ, Ad = [[R, Skew(p)R],[0, R]]
        A = A * SE3Transform::Exp(screws[i], q[i])
    return Js

function Frames(q):                             # same loop, one pass
    A = Identity
    for i in 0..N-1:
        (v; ω) = A.Adjoint() · screws[i]        # current screw of joint i in the base frame
        if ‖ω‖ > 0:  jointAxes[i] = ω/‖ω‖;  jointOrigins[i] = (ω × v)/‖ω‖²;  jointTypes[i] = Revolute
                                                # = foot of the perpendicular from the base origin to the axis
        else:        jointAxes[i] = v;  jointOrigins[i] = A.p;    jointTypes[i] = Prismatic
        A = A * SE3Transform::Exp(screws[i], q[i])
    frames.tool = A * home
    return frames

function SpaceToGeometric(Js, pTool):           # J_geo = [[I, −Skew(pTool)], [0, I]] · J_space
    for each column (v; ω): (v − Skew(pTool)·ω ; ω)  = (v + ω × pTool ; ω)
```

## Complexity & memory

- `Compute`: `O(N)` — one `Exp` and one compose per joint.
- `SpaceJacobian` / `Frames`: `O(N)` — one adjoint–vector product per joint, reusing the running product.
- Memory: `N` screws + `M` + one accumulator; stack only.

## Numerical / embedded notes

- **Which Jacobian:** the space Jacobian's linear part is the velocity of the (virtual) body point that
  currently coincides with the **base origin**, not of the tool. The geometric Jacobian (M8) — tool-point
  velocity, used by M11/M13/M14 and the controllers — is `J_geo = [[I, −Skew(p_tool)],[0, I]]·J_space`,
  equal column by column to `GeometricJacobian::Compute(Frames(q))`.
- No per-link frames: every screw is written once in the base frame (the main advantage over DH).
- `SE3Transform::Exp` handles the prismatic (`ω = 0`) branch and folds a non-unit `ω` into the angle;
  still author unit `ω` so `qᵢ` is a true angle. Pitched screws (`ω·v ≠ 0`) cannot be expressed as a
  `FrameChain` and are rejected at construction.
- Body form (`T = M·e^{[B₁]q₁}···`, `Bᵢ = Ad(M⁻¹)·Sᵢ`) is not needed: `J_body = Ad(T⁻¹)·J_space`.
- Upstream `math::MatrixExponential` ([numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp))
  on the 4×4 `[S]θ` is a valid cross-check of `Exp` in tests, not a runtime dependency.
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Header: `robotics/kinematics/ProductOfExponentials.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Compute`, and
  `extern template class ProductOfExponentials<float, 1>;` / `<float, 2>` / `<float, 6>` under
  `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `robotics/kinematics/ProductOfExponentials.cpp` → the same instantiations.
- Test: `robotics/kinematics/test/TestProductOfExponentials.cpp`
- Doc: `doc/kinematics/ProductOfExponentials.md` (per `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `robotics_add_coverage_sources`;
  `TestProductOfExponentials.cpp` → the `_test` target.
- Depends on: M6 (`SE3Transform::Exp`, `Adjoint`), M30 (`FrameChain`), M8 (`GeometricJacobian`).
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
