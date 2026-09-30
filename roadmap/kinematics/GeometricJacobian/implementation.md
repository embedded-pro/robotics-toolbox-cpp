# Geometric Jacobian (6×N) and Jacobian Provider — Implementation Pseudocode

> Roadmap ref: #M8 (Tier 2) · Target: `robotics/kinematics` · Namespace `kinematics` · Type: `float` (templated on `T`, instantiated for `float` only)

The **geometric** Jacobian maps joint rates to the tool's linear velocity (of the tool point) and
angular velocity, both in the base frame, ordered `(v; ω)` (M6 convention). It is *not* the
Lynch–Park "space Jacobian" (whose linear part is the velocity of the point at the base origin);
M15 provides that one. The routine is model-agnostic: it consumes a `FrameChain` (M30), which the
link model (M30), DH (M7) and PoE (M15) all produce, so FK, IK, dynamics and controllers share a
single Jacobian implementation — replacing the private 3×N Jacobian inside the shipped IK.

## Data structures

```
template<typename T, std::size_t N>             # static_assert(std::is_floating_point_v<T>); instantiated for float
class GeometricJacobian:                        # stateless
    using Jacobian = math::Matrix<T, 6, N>

template<typename T, std::size_t TaskDim, std::size_t Dof>   # DI seam used by all task-space controllers
class JacobianProvider:
    virtual ~JacobianProvider() = default
    virtual math::Matrix<T, TaskDim, Dof> Jacobian(const JointVector& q) const = 0
    virtual math::Vector<T, TaskDim> BiasAcceleration(const JointVector& q, const JointVector& qDot) const = 0   # J̇·q̇
    virtual SE3Transform<T> ToolPose(const JointVector& q) const = 0

template<typename T, std::size_t TaskDim, std::size_t Dof>   # TaskDim ∈ {3, 6}
class ChainTaskJacobian : public JacobianProvider<T, TaskDim, Dof>:
    const LinkArray& links                      # link model (shipped RevoluteJointLink chain)
    ChainPoseKinematics<T, Dof> kinematics      # M30
```

## Interface

```
static Jacobian     Compute(const FrameChain<T, N>& frames)                                # hot path
static Vector6<T>   BiasAcceleration(const FrameChain<T, N>& frames, const JointVector& qDot)   # J̇·q̇
static JointVector  JointTorques(const Jacobian& J, const Vector6<T>& wrench)             # Jᵀ·w

# ChainTaskJacobian: TaskDim = 6 → all rows; TaskDim = 3 → the linear rows only (position tasks)
```

## Algorithm (pseudocode)

```
function Compute(frames):                       # OPTIMIZE_FOR_SPEED
    p = frames.tool.p
    for i in 0..N-1:
        z = frames.jointAxes[i];  o = frames.jointOrigins[i]
        if frames.jointTypes[i] == Revolute:  column i = ( CrossProduct(z, p − o) ; z )
        else:                                  column i = ( z ; 0 )
    return J

function BiasAcceleration(frames, q̇):         # O(N) recursion, q̈ = 0
    ω = 0;  vOrigin = 0;  oPrev = frames.jointOrigins[0]
    ṗ = Compute(frames).linearRows · q̇         # tool-point velocity (or recurse, see note)
    acc = 0 (6-vector)
    for i in 0..N-1:
        o = frames.jointOrigins[i];  z = frames.jointAxes[i]
        vOrigin = vOrigin + CrossProduct(ω, o − oPrev)        # velocity of joint origin i
        ż = CrossProduct(ω, z)                                 # axis carried by link i−1
        if Revolute:
            acc.linear  += q̇[i] · ( CrossProduct(ż, p − o) + CrossProduct(z, ṗ − vOrigin) )
            acc.angular += q̇[i] · ż
            ω = ω + z · q̇[i]
        else:  # Prismatic
            acc.linear  += q̇[i] · ż
            vOrigin      = vOrigin + z · q̇[i]
        oPrev = o
    return acc

function JointTorques(J, w):                    # statics: τ = Jᵀ·w, w = (f; n) at the tool point
    return Transpose(J) · w
```

## Complexity & memory

- `Compute`: `O(N)` — one cross product per column.
- `BiasAcceleration`: `O(N)` after the `O(6N)` tool-velocity product.
- `JointTorques`: `O(6N)`.
- Memory: one `6×N` matrix; stack only.

## Numerical / embedded notes

- **Ordering is normative:** rows `0–2` are linear velocity of the tool point, rows `3–5` angular
  velocity, both in the base frame; wrenches passed to `JointTorques` are `(f; n)` about the tool point.
- DH frames (M7) put joint `i`'s axis on `z` of frame `i−1` (standard) or frame `i` (modified); the DH
  spec is responsible for filling `FrameChain` correctly — this routine never looks at DH parameters.
- `BiasAcceleration` (`J̇q̇`) is required by operational-space control (M18) and by pose tracking;
  check it against a finite difference `(J(q + εq̇) − J(q))/ε · q̇`.
- Near a **singularity** columns become dependent; detect with M11 and never invert `J` directly.
- The provider interface is a virtual seam for testability (StrictMock). Hard real-time users can
  template the controllers on the concrete `ChainTaskJacobian` to avoid the virtual call.
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Header: `robotics/kinematics/GeometricJacobian.hpp` (+ `JacobianProvider.hpp`, `ChainTaskJacobian.hpp`) —
  `#pragma once` → `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on
  `Compute`/`BiasAcceleration`, and `extern template class GeometricJacobian<float, 2>;` / `<float, 3>` /
  `<float, 6>` under `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `robotics/kinematics/GeometricJacobian.cpp` → the same instantiations.
- Test: `robotics/kinematics/test/TestGeometricJacobian.cpp`
- Doc: `doc/kinematics/GeometricJacobian.md` (per `doc/TEMPLATE.md`)
- Depends on: M6 (`SE3Transform`), M30 (`FrameChain`, `ChainPoseKinematics`).
- After deployment, the shipped `InverseKinematics` should build its 3×N Jacobian from this routine.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
