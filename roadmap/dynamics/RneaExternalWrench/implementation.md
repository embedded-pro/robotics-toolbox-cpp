# Inverse Dynamics with an External Tool Wrench — Implementation Pseudocode

> Roadmap ref: #M32 (Tier 1) · Target: `robotics/dynamics` · Namespace `dynamics` · Type: `float` (templated on `T`, instantiated for `float` only)

Extends the shipped `RecursiveNewtonEuler` so contact forces at the tool enter inverse dynamics:
`τ = M q̈ + C q̇ + g − Jᵀ w_ext`. Needed by force control (M17, M19), by payload/contact estimation,
and to cross-check the momentum observer (M16).

## Data structures

```cpp
template<typename T>                                   # static_assert(std::is_floating_point_v<T>); instantiated for float
struct ToolWrench:
    Vector3<T> force                                   # f, exerted BY the environment ON the tool, base frame
    Vector3<T> moment                                  # n, about the tool point, base frame
    Vector3<T> toolOffset                              # tool point in the last link frame (same as FK)
```

## Interface

```text
# new overload next to the shipped InverseDynamics(links, q, q̇, q̈, gravity):
JointVector InverseDynamics(const LinkArray& links, const JointVector& q, const JointVector& qDot,
                            const JointVector& qDDot, const Vector3& gravity,
                            const ToolWrench<T>& wrench) const                    # hot path
```

## Algorithm (pseudocode)

```text
function InverseDynamics(links, q, q̇, q̈, g, w):       # OPTIMIZE_FOR_SPEED
    states = ComputeForwardPass(links, q, q̇, q̈, g)      # unchanged
    Rlast = R_0 · R_1 ··· R_{N−1}                        # base ← last link (product of states[i].R)
    fExt = Rlastᵀ · w.force                              # into the last link frame
    nExt = Rlastᵀ · w.moment + CrossProduct(w.toolOffset, fExt)   # moment about the last joint origin
    # backward pass: the environment pushes on the last link, so the joints need less wrench
    force[N−1]  −= fExt
    torque[N−1] −= nExt
    ... continue the shipped backward pass unchanged ...
    return τ
```

## Complexity & memory

- `O(N)`; one extra rotation product chain (already available from the forward pass) and two cross
  products. No additional storage.

## Numerical / embedded notes

- Sign convention (normative): `w` is the wrench the environment applies to the robot. Holding a
  0.5 kg payload with gravity `(0, 0, −9.81)` is `w.force = (0, 0, −4.905)`.
- Equivalence to the Jacobian-transpose form: `τ(w) − τ(0) = −Jᵀ·(f; n)` with the geometric
  Jacobian (M8) at the same tool point — the key test.
- Orders `(f; n)` linear first, as everywhere (M6).
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Extend `robotics/dynamics/RecursiveNewtonEuler.hpp` (overload) and add `ToolWrench` to it; the
  coverage `.cpp` already instantiates `<float, 1..3>`.
- Test: new cases in `robotics/dynamics/test/TestRecursiveNewtonEuler.cpp`
- Doc: extend `doc/dynamics/RecursiveNewtonEuler.md` (external forces section).
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
