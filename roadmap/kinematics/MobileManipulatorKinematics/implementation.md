# Mobile-Manipulator Kinematics (Differential-Drive Base) — Implementation Pseudocode

> Roadmap ref: #M24 (Tier 4) · Target: `robotics/kinematics` · Namespace `kinematics` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T>                            # static_assert(std::is_floating_point_v<T>); instantiated for float
struct BaseState:                               # planar pose of the axle midpoint in the world frame
    T x, y, phi

template<typename T, std::size_t ArmDof>
class MobileManipulatorKinematics:
    const JacobianProvider<T, 6, ArmDof>& arm   # M8: J and ToolPose in the arm-base frame
    SE3Transform<T>                       mount # arm base in the mobile-base frame (M6)
    T                                     wheelBase    # L, axle length
```

## Interface

```
MobileManipulatorKinematics(const JacobianProvider<T, 6, ArmDof>& arm, const SE3Transform<T>& mount, T wheelBase)
Matrix<T, 6, 2 + ArmDof>  Compute(const BaseState<T>& base, const JointVector& q) const   # hot path
SE3Transform<T>           ToolPose(const BaseState<T>& base, const JointVector& q) const
static Matrix<T, 3, 2>    BaseConstraint(T phi)                  # (ẋ, ẏ, φ̇) = S(φ)·(v, ω)
std::array<T, 2>          WheelSpeeds(T v, T omega) const        # (left, right) rim speeds
```

Columns of `Compute` act on `u = (v, ω, q̇)`: base forward speed, base yaw rate, arm joint rates.
Rows are the tool-point twist `(v; ω)` in the world frame (M6/M8 ordering).

## Algorithm (pseudocode)

```
function ToolPose(base, q):
    T_wb = { Rz(base.phi), (base.x, base.y, 0) }
    return T_wb * mount * arm.ToolPose(q)

function Compute(base, q):                       # OPTIMIZE_FOR_SPEED
    R_wa = Rz(base.phi) · mount.R                # arm-base orientation in the world
    J_arm = arm.Jacobian(q)                      # 6×ArmDof, arm-base frame
    r = ToolPose(base, q).p − (base.x, base.y, 0)                # lever arm, world frame
    column 0 (v) = ( (cos φ, sin φ, 0) ; 0 )
    column 1 (ω) = ( CrossProduct(ẑ, r) ; ẑ )
    columns 2.. = ( R_wa · J_arm.linear ; R_wa · J_arm.angular )  # blockdiag(R_wa, R_wa)·J_arm
    return J

function BaseConstraint(φ):                      # no side-slip: [−sin φ, cos φ, 0]·(ẋ, ẏ, φ̇) = 0
    return [[cos φ, 0], [sin φ, 0], [0, 1]]

function WheelSpeeds(v, ω):
    return (v − ω·L/2, v + ω·L/2)
```

## Complexity & memory

- `Compute`: one arm `Jacobian` + `ToolPose`, `2·ArmDof` 3×3 rotations, two fixed base columns.
- Memory: one `6 × (2 + ArmDof)` matrix; stack only.

## Numerical / embedded notes

- **Frames:** the arm Jacobian is expressed in the arm-base frame; it must be rotated by
  `R_wa = Rz(φ)·R_mount` before being stacked with the world-frame base columns (skipping it gives an
  error of order ‖J_arm·q̇‖ — 0.52 in the test configuration). No lever-arm correction is needed for the
  arm block: its linear rows already describe the tool point.
- **Nonholonomic base:** a differential drive has three planar coordinates but two controls; the base
  columns are already the constrained ones, so no lateral-velocity column exists to command.
- Base plus arm is redundant for a 6-DOF task — split the motion with M14 (e.g. arm for fine motion,
  base for reach); scale the base columns (`|r|` amplifies yaw) before resolving.
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Header: `robotics/kinematics/MobileManipulatorKinematics.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Compute`, and
  `extern template class MobileManipulatorKinematics<float, 3>;` / `<float, 6>` under
  `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `robotics/kinematics/MobileManipulatorKinematics.cpp` → the same instantiations.
- Test: `robotics/kinematics/test/TestMobileManipulatorKinematics.cpp`
- Doc: `doc/kinematics/MobileManipulatorKinematics.md` (per `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `robotics_add_coverage_sources`;
  `TestMobileManipulatorKinematics.cpp` → the `_test` target.
- Depends on: M6 (`SE3Transform`), M8 (`JacobianProvider`).
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
