# Parallel Manipulator Kinematics (Stewart–Gough / Delta) — Implementation Pseudocode

> Roadmap ref: #M23 (Tier 4) · Target: `robotics/kinematics` · Namespace `kinematics` · Type: `float` (templated on `T`, instantiated for `float` only)

Two mechanisms, two classes: the Stewart–Gough hexapod (six prismatic legs, 6-DOF platform) and the
Delta robot (three revolute actuated arms with parallelogram forearms, 3-DOF translation). Their
actuated variables differ (leg lengths vs arm angles), so they share no data model.

## Data structures

```cpp
template<typename T>                            # static_assert(std::is_floating_point_v<T>); instantiated for float
struct StewartGoughGeometry:
    std::array<Vector3<T>, 6> baseAnchors       # bᵢ, base frame
    std::array<Vector3<T>, 6> platformAnchors   # pᵢ, platform frame

template<typename T>
struct StewartForwardConfig:
    T tolerance;  T damping;  std::size_t maxIterations      # damping λ: 0 ⇒ plain Newton

template<typename T>
struct StewartForwardResult:
    SE3Transform<T> pose;  std::size_t iterations;  bool converged

template<typename T>
class StewartGoughKinematics:
    StewartGoughGeometry<T>  geometry
    StewartForwardConfig<T>  config

template<typename T>
struct DeltaGeometry:                           # base plane z = 0, effector below (z < 0)
    T baseRadius        # R: centre → shoulder axis (triangle side = 2√3·R)
    T effectorRadius    # r: effector centre → forearm attachment
    T upperArm          # rf
    T forearm           # re (parallelogram length)
    # arm i shoulder azimuth φᵢ = 0°, 120°, 240°; θᵢ = 0 ⇒ upper arm horizontal, outward; θ > 0 ⇒ down

template<typename T>
class DeltaKinematics:
    DeltaGeometry<T> geometry
    std::array<T, 3> cosPhi, sinPhi             # precomputed
```

## Interface

```cpp
StewartGoughKinematics(const StewartGoughGeometry<T>& geometry, const StewartForwardConfig<T>& config)
std::array<T, 6>         Inverse(const SE3Transform<T>& pose) const          # leg lengths; hot path
Matrix6<T>               LegJacobian(const SE3Transform<T>& pose) const      # l̇ = J·(v; ω)
StewartForwardResult<T>  Forward(const std::array<T, 6>& lengths, const SE3Transform<T>& guess) const

explicit DeltaKinematics(const DeltaGeometry<T>& geometry)
std::optional<Vector3<T>>        Inverse(const Vector3<T>& effector) const   # returns (θ₁, θ₂, θ₃); hot path
std::optional<Vector3<T>>        Forward(const Vector3<T>& angles) const     # effector position
```

## Algorithm (pseudocode)

```cpp
# ---- Stewart–Gough ----
function Inverse(pose):                          # OPTIMIZE_FOR_SPEED, closed form
    for i: lengths[i] = ‖pose.R·pᵢ + pose.p − bᵢ‖
    return lengths

function LegJacobian(pose):                      # row i = (nᵢᵀ, (R·pᵢ × nᵢ)ᵀ), nᵢ = unit leg vector
    for i: a = pose.R·pᵢ;  n = Normalize(a + pose.p − bᵢ);  row i = (n ; CrossProduct(a, n))
    # (v; ω): platform-origin velocity and angular velocity, base frame (M6 order); statics: wrench (f; n) = Jᵀ·legForces

function Forward(L, guess):                      # Newton on SE(3)
    x = guess
    for iter in 0..maxIterations-1:
        f = Inverse(x) − L
        if ‖f‖ < tolerance: return { x, iter, true }
        J = LegJacobian(x)
        δ = −Jᵀ·LuDecomposition(J·Jᵀ + λ²·I₆).Solve(f)          # λ = 0: δ = −J⁻¹f
        x.p = x.p + δ.linear
        x.R = SE3Transform::Exp((0; δ.angular), 1).R · x.R       # left (base-frame) retraction
    return { x, maxIterations, ‖Inverse(x) − L‖ < tolerance }

# ---- Delta ----
function ArmAngle(i, P):                         # circle (elbow) ∩ sphere (forearm) in arm i's plane
    x' = cosφᵢ·P.x + sinφᵢ·P.y;  y' = −sinφᵢ·P.x + cosφᵢ·P.y;  z = P.z
    X = x' + r − R
    a = 2·rf·X;  b = −2·rf·z;  c = X² + y'² + z² + rf² − re²     # a·cos θ + b·sin θ = c
    ρ = √(a² + b²);  if |c| > ρ: return nullopt                  # out of reach
    return atan2(b, a) − acos(c / ρ)             # elbow-out root (larger cos θ) for z < 0

function Inverse(P):                             # OPTIMIZE_FOR_SPEED
    return (ArmAngle(0, P), ArmAngle(1, P), ArmAngle(2, P))      # nullopt if any arm fails

function Forward(θ):                             # three-sphere intersection, closed form
    Cᵢ = Rz(φᵢ)·(R − r + rf·cos θᵢ, 0, −rf·sin θᵢ)               # elbow shifted by −effector radius
    ex = Normalize(C₂ − C₁);  d = ‖C₂ − C₁‖;  i = ex·(C₃ − C₁)
    ey = Normalize(C₃ − C₁ − i·ex);  j = ey·(C₃ − C₁);  ez = ex × ey
    x = d/2;  y = (i² + j² − 2·i·x) / (2j);  h² = re² − x² − y²  # equal radii re
    if h² < 0: return nullopt
    return the one of C₁ + x·ex + y·ey ± √h²·ez with the smaller z  # effector below the base
```

## Complexity & memory

- Stewart `Inverse` / `LegJacobian`: `O(6)`; `Forward`: `O(iters·6³)` (one 6×6 LU per step).
- Delta `Inverse`: 3 × (`sqrt`, `acos`, `atan2`); `Forward`: `O(1)`, one `sqrt`.
- Memory: geometry + one `6×6` matrix; stack only.

## Numerical / embedded notes

- Parallel robots invert the serial difficulty: Stewart IK is closed form, FK needs Newton and has up to
  40 assembly modes — `guess` selects one (seed from the previous cycle). The Delta's special
  geometry (parallelograms keep the effector parallel to the base) makes both directions closed form.
- `LegJacobian` is the inverse Jacobian of the platform; it is singular where the platform gains
  uncontrollable motion (the symmetric fixture at yaw 90°). `λ > 0` keeps the step finite there;
  the leg residual still converges but the pose along the singular direction is ill-determined.
- Delta branch choice: `atan2(b, a) ± acos(·)` are elbow-out / elbow-in; for `z < 0`, `b > 0` so the
  minus root has the larger `cos θ` (elbow out). The lower sphere intersection is the physical effector.
- Upstream `solvers::LuDecomposition` ([numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp))
  for the 6×6 solve.
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Headers: `robotics/kinematics/StewartGoughKinematics.hpp`, `robotics/kinematics/DeltaKinematics.hpp` —
  `#pragma once` → `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Inverse`, and
  `extern template class StewartGoughKinematics<float>;` / `extern template class DeltaKinematics<float>;`
  under `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `robotics/kinematics/StewartGoughKinematics.cpp`, `robotics/kinematics/DeltaKinematics.cpp`
  → `template class StewartGoughKinematics<float>;` / `template class DeltaKinematics<float>;`
- Tests: `robotics/kinematics/test/TestStewartGoughKinematics.cpp`, `robotics/kinematics/test/TestDeltaKinematics.cpp`
- Doc: `doc/kinematics/ParallelManipulatorKinematics.md` (per `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `robotics_add_coverage_sources`; both tests → the `_test` target.
- Depends on: M6 (`SE3Transform`).
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
