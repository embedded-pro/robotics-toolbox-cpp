# Denavit-Hartenberg Parameters — Implementation Pseudocode

> Roadmap ref: #M7 (Tier 2) · Target: `robotics/kinematics` · Namespace `kinematics` · Type: `float` (templated on `T`, instantiated for `float` only)

Produces the tool pose and the M30 `FrameChain` from a DH table, so the geometric Jacobian (M8) and
everything built on `JacobianProvider` work on DH-described arms unchanged.

## Data structures

```cpp
enum class JointType : uint8_t { Revolute, Prismatic }      # shared with M30 / M1
enum class DhConvention : uint8_t { Standard, Modified }     # distal (classic) vs proximal (Craig)

template<typename T>                           # static_assert(std::is_floating_point_v<T>); instantiated for float
struct DhLink:                                 # Standard row i: (aᵢ, αᵢ, dᵢ, θᵢ); Modified row i: (aᵢ₋₁, αᵢ₋₁, dᵢ, θᵢ)
    T         a          # link length along x
    T         alpha      # link twist about x
    T         d          # constant offset along z (added to q when Prismatic)
    T         theta      # constant offset about z (added to q when Revolute)
    JointType type

template<typename T, std::size_t N>
class DenavitHartenberg:
    std::array<DhLink<T>, N>  links
    DhConvention              convention
    SE3Transform<T>           tool              # constant tool pose in the last DH frame (M6)
    std::array<T, N>          cosAlpha, sinAlpha   # precomputed at construction

template<typename T, std::size_t TaskDim, std::size_t N>     # TaskDim ∈ {3, 6}
class DhTaskJacobian : public JacobianProvider<T, TaskDim, N>:   # M8 seam, mirrors ChainTaskJacobian
    const DenavitHartenberg<T, N>& model
```

## Interface

```cpp
DenavitHartenberg(const std::array<DhLink<T>, N>& links, DhConvention convention,
                  const SE3Transform<T>& tool = SE3Transform<T>::Identity())
SE3Transform<T>   LinkTransform(std::size_t i, T q) const     # one Aᵢ; hot path
SE3Transform<T>   Forward(const JointVector& q) const         # tool pose ⁰Tₙ·tool; hot path
FrameChain<T, N>  Frames(const JointVector& q) const          # M30 frame chain (for M8)

# DhTaskJacobian: Jacobian(q) = rows of GeometricJacobian::Compute(model.Frames(q)) (TaskDim = 3 → linear rows),
# BiasAcceleration(q, q̇) = rows of GeometricJacobian::BiasAcceleration(model.Frames(q), q̇),
# ToolPose(q) = model.Forward(q)
```

## Algorithm (pseudocode)

```text
function LinkTransform(i, q):                   # OPTIMIZE_FOR_SPEED
    link = links[i]
    θ = link.theta + (link.type == Revolute  ? q : 0)
    d = link.d     + (link.type == Prismatic ? q : 0)
    (cθ, sθ) = (cos θ, sin θ);  (cα, sα) = (cosAlpha[i], sinAlpha[i])
    if convention == Standard:                  # Rotz(θ)·Transz(d)·Transx(a)·Rotx(α)
        R = [[ cθ, −sθ·cα,  sθ·sα ],
             [ sθ,  cθ·cα, −cθ·sα ],
             [  0,     sα,     cα ]]
        p = (a·cθ, a·sθ, d)
    else:                                       # Modified: Rotx(αᵢ₋₁)·Transx(aᵢ₋₁)·Rotz(θ)·Transz(d)
        R = [[    cθ,    −sθ,   0 ],
             [ sθ·cα,  cθ·cα, −sα ],
             [ sθ·sα,  cθ·sα,  cα ]]
        p = (a, −sα·d, cα·d)
    return { R, p }

function Forward(q):                            # OPTIMIZE_FOR_SPEED
    A = Identity
    for i in 0..N-1: A = A * LinkTransform(i, q[i])
    return A * tool

function Frames(q):
    A = Identity
    for i in 0..N-1:
        if convention == Standard:              # joint i acts about z of frame i−1 (the frame before its link)
            frames.jointOrigins[i] = A.p;  frames.jointAxes[i] = A.R · ẑ
            A = A * LinkTransform(i, q[i])
        else:                                   # Modified: joint i acts about z of frame i
            A = A * LinkTransform(i, q[i])
            frames.jointOrigins[i] = A.p;  frames.jointAxes[i] = A.R · ẑ
        frames.jointTypes[i] = links[i].type
    frames.tool = A * tool
    return frames
```

## Complexity & memory

- `LinkTransform`: `O(1)` — one `sin`/`cos` pair and a fixed 3×3 fill (`α` terms precomputed).
- `Forward` / `Frames`: `O(N)` SE(3) products; `Frames` stores `2N` vectors + one transform.
- Memory: the constant link table plus `2N` precomputed trig values; stack only.

## Numerical / embedded notes

- **Offsets are kept:** real DH tables carry constant `θ` offsets on revolute joints and constant `d`
  on prismatic ones; the joint variable is *added*, never substituted.
- **Conventions differ in the row layout, not just the factor order:** a modified row holds
  `(aᵢ₋₁, αᵢ₋₁)` of the previous link, and the last standard `(aₙ, αₙ)` moves into `tool`. The same
  arm described both ways yields identical `Forward` and `Frames`.
- The origin written into `FrameChain` is any point on the joint axis; for revolute columns
  `z × (p − o)` is unchanged by sliding `o` along `z`, so the modified-frame origin (which includes `dᵢ`)
  is valid.
- Both factored forms are exactly orthonormal — no re-orthonormalization needed.
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Header: `robotics/kinematics/DenavitHartenberg.hpp` (+ `DhTaskJacobian.hpp`) — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `LinkTransform`/`Forward`, and
  `extern template class DenavitHartenberg<float, 1>;` / `<float, 2>` / `<float, 6>` plus
  `extern template class DhTaskJacobian<float, 6, 6>;` under `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `robotics/kinematics/DenavitHartenberg.cpp` → the same instantiations.
- Test: `robotics/kinematics/test/TestDenavitHartenberg.cpp`
- Doc: `doc/kinematics/DenavitHartenberg.md` (per `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `robotics_add_coverage_sources`;
  `TestDenavitHartenberg.cpp` → the `_test` target.
- Depends on: M6 (`SE3Transform`), M30 (`FrameChain`), M8 (`GeometricJacobian`, `JacobianProvider`).
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
