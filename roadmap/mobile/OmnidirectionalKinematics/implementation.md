# Omnidirectional (Mecanum / Swedish-Wheel) Kinematics — Implementation Pseudocode

> Roadmap ref: #W3 (★☆☆☆☆) · Target: `robotics/mobile` · Namespace `mobile` · Type: `float` (templated on `T`, instantiated for `float` only)

Holonomic bases built from `N ≥ 3` Swedish wheels: three-wheel omni ("kiwi") platforms, four-wheel
mecanum carts, and any other layout. Each wheel has free rollers on its rim, so it drives in one
direction and slides freely in another. The body can then follow any twist `(vx, vy, ω)`.

## Data structures

```cpp
template<typename T>                            # static_assert(std::is_floating_point_v<T>); instantiated for float
struct SwedishWheel:
    math::Vector2<T> position                   # contact point in the body frame
    T driveAngle                                # β: direction the hub moves for a positive wheel speed
    T rollerAngle                               # γ: 0 for omniwheels, ±π/4 for mecanum (sign = roller handedness)
    T radius                                    # r

template<typename T, std::size_t NumWheels>     # static_assert(NumWheels >= 3)
class OmnidirectionalKinematics:
    math::Matrix<T, NumWheels, 3> jacobian      # H: wheel speeds = H · (vx, vy, ω)
    math::Matrix<T, 3, NumWheels> pseudoInverse # H⁺ = (HᵀH)⁻¹Hᵀ, fixed at construction
    T                             maxWheelSpeed
```

## Interface

```cpp
OmnidirectionalKinematics(const std::array<SwedishWheel<T>, NumWheels>& wheels, T maxWheelSpeed)
math::Vector<T, NumWheels> Inverse(const math::Vector3<T>& twist) const          # hot path
math::Vector3<T>           Forward(const math::Vector<T, NumWheels>& speeds) const   # least squares; hot path
math::Vector<T, NumWheels> Limit(const math::Vector<T, NumWheels>& speeds) const     # uniform scaling
T                          SlipResidual(const math::Vector<T, NumWheels>& speeds) const
const math::Matrix<T, NumWheels, 3>& WheelJacobian() const
```

## Algorithm (pseudocode)

```text
function Construct(wheels, ω_max):
    for each wheel i:                           # really_assert |cos γᵢ| > 1e-3 (rollers along the drive transmit nothing)
        a = βᵢ + γᵢ
        H[i] = ( cos a,  sin a,  xᵢ·sin a − yᵢ·cos a ) / (rᵢ·cos γᵢ)
    G = Hᵀ·H                                    # 3×3, SPD for a rank-3 layout
    L = CholeskyDecomposition<T, 3>::TryFactor(G)   # really_assert has_value: the layout must span all 3 DOF
    H⁺ = solve G·X = Hᵀ column by column with the factor (N triangular solve pairs, construction only)

function Inverse(V):   return H·V               # OPTIMIZE_FOR_SPEED
function Forward(u):   return H⁺·u              # OPTIMIZE_FOR_SPEED; exact for N = 3, least squares for N > 3

function Limit(u):
    m = maxᵢ |uᵢ|
    return m ≤ ω_max ? u : u·(ω_max / m)        # same body direction, lower speed

function SlipResidual(u):  return ‖u − H·Forward(u)‖
```

## Complexity & memory

- `Inverse` / `Forward`: `3N` multiply-adds each; construction `O(N)` plus one 3×3 Cholesky.
- Memory: `6N + 1` scalars (`H` and `H⁺`). No heap.

## Numerical / embedded notes

- **Row formula.** `uᵢ = (1/(rᵢ cos γᵢ))·[cos(βᵢ+γᵢ), sin(βᵢ+γᵢ), xᵢ sin(βᵢ+γᵢ) − yᵢ cos(βᵢ+γᵢ)]·(vx, vy, ω)`,
  from Lynch & Park eq. (13.5) with the twist reordered to linear-first. For a standard mecanum cart
  (wheels at `(±l_x, ±l_y)`, `β = 0`, `γ = −π/4, π/4, π/4, −π/4` for FL, FR, RL, RR, `k = l_x + l_y`) it
  reduces to `r·u = (vx − vy − kω, vx + vy + kω, vx + vy − kω, vx − vy + kω)`.
- **Slip detection.** With `N > 3` the wheel speeds overdetermine the twist. A non-zero `SlipResidual` on
  *measured* speeds flags a slipping or lifted wheel or a faulty encoder, which odometry (F1) should then
  down-weight.
- `Limit` scales every wheel by one factor, so the body moves in the commanded direction, more slowly.
- Roller handedness is easy to get wrong in the field: confirm each `γᵢ` sign by commanding pure `vy`.
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Header: `robotics/mobile/OmnidirectionalKinematics.hpp` — `#pragma once`, `OPTIMIZE_FOR_SPEED` on
  `Inverse`/`Forward`, and `extern template class OmnidirectionalKinematics<float, 3>;` / `<float, 4>` under
  `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `robotics/mobile/OmnidirectionalKinematics.cpp` → the same instantiations.
- Test: `robotics/mobile/test/TestOmnidirectionalKinematics.cpp`
- Doc: `doc/mobile/OmnidirectionalKinematics.md` (per `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources(robotics.mobile ...)`; `.cpp` → `robotics_add_coverage_sources`;
  test → `robotics.mobile_test`.
- Depends on: NT66 (twist convention); upstream `math::CholeskyDecomposition`.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
