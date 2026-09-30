# Generic (Revolute / Prismatic) Joint Link with Limits and Armature — Implementation Pseudocode

> Roadmap ref: #M1 (Tier 1) · Target: `robotics/dynamics` (+ changes in `kinematics`) · Namespace `dynamics` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```cpp
enum class JointType : uint8_t { Revolute, Prismatic }          # shared with M7 / M30

template<typename T>                                            # static_assert(std::is_floating_point_v<T>); instantiated for float
struct JointLimits:
    T positionMin = −∞, positionMax = +∞                         # rad or m
    T velocityMax = +∞                                           # rad/s or m/s
    T effortMax   = +∞                                           # N·m or N

template<typename T>
struct JointLink:                                               # evolves the shipped RevoluteJointLink
    T                         mass
    math::SquareMatrix<T, 3>  inertia         # at the CoM, link frame
    math::Vector<T, 3>        jointAxis       # unit, link frame (rotation axis or slide direction)
    math::Vector<T, 3>        parentToJoint   # joint origin at q = 0, parent frame
    math::Vector<T, 3>        jointToCoM      # link frame
    JointType                 type = JointType::Revolute       # new fields are trailing and defaulted,
    JointLimits<T>            limits = {}                       # so every existing aggregate
    T                         armature = 0                      # initializer keeps compiling

template<typename T> using RevoluteJointLink = JointLink<T>    # migration alias
```

`armature` is the reflected actuator inertia (`G²·I_rotor`, kg·m² for revolute, kg for prismatic).

## Interface

```text
(Matrix3 R, Vector3 offset) JointTransform(T q) const          # link frame relative to parent; hot path
Vector3 AngularAxis() const    # axis if Revolute else 0        # motion subspace S = (AngularAxis; LinearAxis)
Vector3 LinearAxis()  const    # axis if Prismatic else 0       # in the dynamics' (ω; v) ordering
```

## Algorithm (pseudocode)

```cpp
function JointTransform(q):                                    # OPTIMIZE_FOR_SPEED
    if type == Revolute:  return (RotationAboutAxis(jointAxis, q), parentToJoint)
    else:                 return (Identity, parentToJoint + jointAxis·q)

# Required changes in the shipped algorithms (per joint, link frame = parent rotated by R, offset r):
FK / ChainPoseKinematics (M30):  origin_i = origin_{i−1} + R_{i−1}·r_i ;  R_i = R_{i−1}·R(q_i)
                                  Jacobian column (M8): revolute (z × (p − o); z), prismatic (z; 0)
RNEA forward pass, prismatic i:  ω_i = Rᵀω_{i−1} = ω_{i−1};  ω̇_i = ω̇_{i−1}
                                  a_i = a_{i−1} + ω̇_{i−1}×r_i + ω_{i−1}×(ω_{i−1}×r_i) + 2·ω_i×(z·q̇_i) + z·q̈_i
RNEA backward pass:               child offset is r_{i+1} (= parentToJoint + z·q for a prismatic child)
                                  τ_i = z·n_i (revolute)   |   τ_i = z·f_i (prismatic)
                                  τ_i += armature_i·q̈_i
ABA (Featherstone, S = (0; z) for prismatic): U = I^A·S, D = SᵀU + armature, u = τ − Sᵀp^A,
                                  c_i = v_i × S·q̇_i, transforms use r_i(q)
CRBA (M28):                       S as above; M_ii += armature_i
Limits:                           consumed by IK clamping (M13), trajectory planners (M2/M3/M9/M33), TOPP (M27)
```

## Complexity & memory

- `JointTransform`: `O(1)`. The algorithm changes keep every pass `O(N)` (`O(N²)` for CRBA).
- Memory: one enum byte, four limit scalars and the armature per link.

## Numerical / embedded notes

- **Migration:** keep the aggregate order of the shipped struct and append defaulted fields; alias
  `RevoluteJointLink` so FK/IK/RNEA/ABA and their tests compile unchanged, then add the prismatic
  branches behind `type`.
- The prismatic `2ω×(z·q̇)` Coriolis term and the `z·f` projection are the two places a revolute-only
  implementation silently goes wrong — both are pinned by tests.
- Keep `jointAxis` unit-normalized (assert at construction); a non-unit axis rescales both the angle
  and the slide.
- Armature adds to the joint-space inertia only; it does not change the kinematics or gravity.
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Header: `robotics/dynamics/JointLink.hpp` (and `RevoluteJointLink.hpp` becomes the alias) —
  `#pragma once` → `#pragma GCC optimize("O3","fast-math")`, `extern template struct JointLink<float>;`
  under `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `robotics/dynamics/JointLink.cpp` → `template struct JointLink<float>;`
- Tests: `robotics/dynamics/test/TestJointLink.cpp` plus prismatic cases in the RNEA/ABA/FK tests.
- Doc: `doc/dynamics/JointLink.md` (per `doc/TEMPLATE.md`) and updates to the RNEA/ABA/FK docs.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
