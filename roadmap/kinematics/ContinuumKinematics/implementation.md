# Continuum Kinematics (Constant Curvature) — Implementation Pseudocode

> Roadmap ref: #M26 (Tier 4) · Target: `robotics/kinematics` · Namespace `kinematics` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T>                            # static_assert(std::is_floating_point_v<T>); instantiated for float
struct ArcParameters:                           # configuration of one section
    T kappa      # curvature κ (1/m); sign folds into φ: (−κ, φ) ≡ (κ, φ + π)
    T phi        # bending-plane angle about the section's base z
    T length     # arc length s

template<typename T, std::size_t NumSections>
class ContinuumKinematics:
    std::array<ArcParameters<T>, NumSections>  sections
```

## Interface

```
explicit ContinuumKinematics(const std::array<ArcParameters<T>, NumSections>& sections)
static SE3Transform<T>  SectionTransform(const ArcParameters<T>& arc)     # one arc; hot path
SE3Transform<T>         Forward() const                                   # tip pose; hot path
static ArcParameters<T> InverseSection(const Vector3<T>& tip)             # single section, closed form
```

## Algorithm (pseudocode)

```
function SectionTransform(arc):                 # OPTIMIZE_FOR_SPEED, robot-independent map
    θ = arc.kappa · arc.length                  # total bend angle
    if |θ| < θ_series:                          # straight-ish: no 1/κ, no 0/0
        f1 = θ/2 − θ³/24;   f2 = 1 − θ²/6
    else:
        f1 = 2·sin²(θ/2) / θ;  f2 = sin θ / θ   # (1 − cos θ)/θ without cancellation
    p = arc.length · ( f1·(cos φ, sin φ, 0) + f2·ẑ )
    R = Rz(φ) · Ry(θ) · Rz(−φ)                  # tangent R·ẑ = (cos φ sin θ, sin φ sin θ, cos θ) = dp/ds
    return { R, p }

function Forward():                             # OPTIMIZE_FOR_SPEED
    A = Identity
    for i in 0..NumSections-1: A = A * SectionTransform(sections[i])   # M6 compose
    return A

function InverseSection(tip):                   # tip position in the section base frame, κs < 2π
    ρ = ‖(tip.x, tip.y)‖;  L = ‖tip‖
    if ρ < ε: return { 0, 0, tip.z }            # straight: φ undefined, returned as 0
    φ = atan2(tip.y, tip.x)
    θ = 2·atan2(ρ, tip.z)                       # tan(θ/2) = ρ / z
    κ = 2ρ / L²                                 # from L² = 2ρ/κ
    s = L · (θ/2) / sin(θ/2)                    # = θ/κ, finite as θ → 0
    return { κ, φ, s }
```

## Complexity & memory

- `SectionTransform`: `O(1)` — two `sin`/`cos` pairs and a 3×3 build.
- `Forward`: `O(NumSections)` SE(3) products; `InverseSection`: `O(1)`.
- Memory: the section array plus one accumulator; stack only.

## Numerical / embedded notes

- **Straight limit:** writing the arc as `s·f(θ)` removes the `1/κ` blow-up; below `θ_series ≈ 1e-2`
  (`float`) the series is accurate to `θ⁴/120 < 1e-9`. The result is independent of `φ` when straight.
- **Chord identity:** `‖p‖ = 2|sin(θ/2)|/|κ|` — the basis of `InverseSection` and of the tests.
- Keep the robot-independent map `(κ, φ, s) → SE(3)` separate from the robot-specific actuator map
  (tendon lengths / chamber pressures → `(κ, φ, s)`); only the latter changes between hardware.
- Constant curvature is a modelling assumption; gravity and tip loads bend real sections off-arc.
- The single-section inverse returns `κ ≥ 0`, `φ ∈ (−π, π]`; multi-section inverse needs iteration
  (M13-style damped least squares on `(κᵢ, φᵢ, sᵢ)`).
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Header: `robotics/kinematics/ContinuumKinematics.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `SectionTransform`/`Forward`, and
  `extern template class ContinuumKinematics<float, 1>;` / `<float, 2>` under `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `robotics/kinematics/ContinuumKinematics.cpp` → the same instantiations.
- Test: `robotics/kinematics/test/TestContinuumKinematics.cpp`
- Doc: `doc/kinematics/ContinuumKinematics.md` (per `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `robotics_add_coverage_sources`;
  `TestContinuumKinematics.cpp` → the `_test` target.
- Depends on: M6 (`SE3Transform`).
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
