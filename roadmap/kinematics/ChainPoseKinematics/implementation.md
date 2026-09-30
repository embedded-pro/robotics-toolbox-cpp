# Chain Pose Kinematics (Full-Pose FK + Frame Chain) — Implementation Pseudocode

> Roadmap ref: #M30 (Tier 2) · Target: `robotics/kinematics` · Namespace `kinematics` · Type: `float` (templated on `T`, instantiated for `float` only)

The shipped `ForwardKinematics` returns joint origins and the tool point only. Pose-level algorithms
(M8, M13, M17–M19) also need the tool **orientation** and every joint **axis** in the base frame. This
spec extends the same recursion over the existing link model (`RevoluteJointLink`, and
`GenericJointLink` once M1 lands) and produces a model-independent `FrameChain` — the single input
the geometric Jacobian (M8) consumes, whether the frames come from this link model, from DH (M7) or
from PoE (M15).

## Data structures

```
enum class JointType : uint8_t { Revolute, Prismatic }      # shared with M1 / M7

template<typename T, std::size_t N>                          # static_assert(std::is_floating_point_v<T>); instantiated for float
struct FrameChain:
    std::array<Vector3<T>, N>  jointOrigins                  # base frame
    std::array<Vector3<T>, N>  jointAxes                     # unit, base frame
    std::array<JointType, N>   jointTypes
    SE3Transform<T>            tool                          # tool pose in the base frame (M6)

template<typename T, std::size_t N>
class ChainPoseKinematics:
    SE3Transform<T> toolInLastLink                           # constant tool frame, injected
```

## Interface

```
explicit ChainPoseKinematics(const SE3Transform<T>& toolInLastLink)
FrameChain<T, N>  Compute(const LinkArray& links, const JointVector& q) const   # hot path
SE3Transform<T>   ToolPose(const LinkArray& links, const JointVector& q) const
```

## Algorithm (pseudocode)

```
function Compute(links, q):                     # OPTIMIZE_FOR_SPEED
    R = I;  o = links[0].parentToJoint
    for i in 0..N-1:
        if i > 0: o = o + R · links[i].parentToJoint
        frames.jointOrigins[i] = o
        frames.jointAxes[i]    = R · links[i].jointAxis          # axis is invariant under its own rotation
        frames.jointTypes[i]   = Revolute                        # (GenericJointLink: links[i].type)
        R = R · RotationAboutAxis(links[i].jointAxis, q[i])      # prismatic (M1): R unchanged, o += axis·q
    frames.tool = SE3Transform{ R, o } * toolInLastLink
    return frames
```

## Complexity & memory

- `Compute`: `O(N)` — one Rodrigues rotation and two 3×3 products per joint.
- Memory: `2N` vectors + one transform; stack only.

## Numerical / embedded notes

- Consistent with the shipped `ForwardKinematics`: `jointOrigins[i]` equals its `positions[i]`, and
  `tool.p` equals its last entry when `toolInLastLink = { I, toolOffset }` — assert this in a test.
- The base mounting offset is `links[0].parentToJoint`; the tool is a kinematic parameter, never
  derived from a center of mass.
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Header: `robotics/kinematics/ChainPoseKinematics.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Compute`, and
  `extern template class ChainPoseKinematics<float, 2>;` / `<float, 3>` / `<float, 6>` under
  `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `robotics/kinematics/ChainPoseKinematics.cpp` → the same three instantiations.
- Test: `robotics/kinematics/test/TestChainPoseKinematics.cpp`
- Doc: `doc/kinematics/ChainPoseKinematics.md` (or extend `ForwardKinematics.md`)
- Depends on: M6 (`SE3Transform`).
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
