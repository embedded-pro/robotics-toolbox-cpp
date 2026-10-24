# Skid-Steer Kinematics with Slip (ICR Model) — Implementation Pseudocode

> Roadmap ref: #W4 (★★☆☆☆) · Target: `robotics/mobile` · Namespace `mobile` · Type: `float` (templated on `T`, instantiated for `float` only)

Tracked vehicles and four-wheel skid-steered robots turn by skidding, so the ideal differential-drive
model (W1) over-predicts their turn rate. The instantaneous-centre-of-rotation (ICR) model keeps the
linear form but replaces `±B/2` with the identified positions of the left and right tread ICRs, plus a
longitudinal offset of the vehicle ICR that produces the lateral slip velocity. A small least-squares
estimator identifies the three numbers from logged motion.

## Data structures

```cpp
template<typename T>                            # static_assert(std::is_floating_point_v<T>); instantiated for float
struct IcrParameters:                           # body frame (FLU), metres
    T yLeft                                     # y of the left tread's ICR   (ideal:  B/2)
    T yRight                                    # y of the right tread's ICR  (ideal: −B/2)
    T xIcr                                      # x of the vehicle's ICR      (ideal:  0)

template<typename T>
class SkidSteerKinematics:
    IcrParameters<T> icr                        # really_assert yLeft > yRight
    T sprocketRadius                            # r: tread speed = r · sprocket speed
    T maxSprocketSpeed

template<typename T>
class IcrEstimator:                             # closed-form linear least squares over logged samples
    T sumOmega2, sumLeft, sumRight, sumLateral
    std::size_t samples
```

## Interface

```cpp
static IcrParameters<T> IdealIcr(T trackWidth)                                  # (B/2, −B/2, 0)

SkidSteerKinematics(const IcrParameters<T>& icr, T sprocketRadius, T maxSprocketSpeed)
math::Vector3<T> Forward(const WheelSpeeds<T>& sprockets) const    # (vx, vy, ω); hot path
WheelSpeeds<T>   Inverse(T vx, T omega) const                      # hot path; vy follows, it is not commanded
WheelSpeeds<T>   Limit(const WheelSpeeds<T>& sprockets) const      # W1 ScaleToLimit

explicit IcrEstimator(T sprocketRadius)
void Add(const WheelSpeeds<T>& sprockets, const math::Vector3<T>& measuredTwist)
std::optional<IcrParameters<T>> Estimate(T minExcitation) const    # nullopt while Σω² < minExcitation
void Reset()
std::size_t Samples() const
```

`WheelSpeeds` is W1's (left, right) pair, here the sprocket speeds in rad/s.

## Algorithm (pseudocode)

```text
# A body point (x, y) moves at (vx − ω·y, vy + ω·x). A tread at speed v_t slides on the ground at
# vx − ω·y − v_t, which is zero at its ICR y_t = (vx − v_t)/ω.

function Forward(s):                            # OPTIMIZE_FOR_SPEED
    vl = r·s.left;  vr = r·s.right;  D = yLeft − yRight
    ω  = (vr − vl) / D
    vx = (vr·yLeft − vl·yRight) / D
    vy = −xIcr·ω
    return (vx, vy, ω)

function Inverse(vx, ω):                        # OPTIMIZE_FOR_SPEED
    return { (vx − ω·yLeft)/r,  (vx − ω·yRight)/r }

function IcrEstimator::Add(s, (vx, vy, ω)):     # each sample: yLeft·ω = vx − vl, yRight·ω = vx − vr, xIcr·ω = −vy
    vl = r·s.left;  vr = r·s.right
    sumOmega2 += ω²;  sumLeft += ω·(vx − vl);  sumRight += ω·(vx − vr);  sumLateral += ω·vy
    samples += 1

function IcrEstimator::Estimate(minExcitation):
    if sumOmega2 < minExcitation: return nullopt   # straight driving cannot identify the ICR
    return { sumLeft/sumOmega2,  sumRight/sumOmega2,  −sumLateral/sumOmega2 }
```

## Complexity & memory

- `Forward` / `Inverse` / `Add`: `O(1)`. Kinematics: 5 scalars; estimator: 4 sums and a count. No heap.

## Numerical / embedded notes

- **Reduces to W1.** `IdealIcr(B)` gives `ω = r(s_r − s_l)/B`, `vx = r(s_l + s_r)/2`, `vy = 0`; the test
  checks it against `DifferentialDrive`.
- **Identification data.** The measured twist comes from an external reference: GNSS heading plus speed,
  a total station, motion capture, or scan matching (E5). Turning manoeuvres at both turn directions are
  required; `Estimate` returns `nullopt` until `Σω²` is large enough.
- The ICR moves with terrain, load and speed (Martínez et al. found it nearly constant per terrain at
  moderate speeds): identify per surface and `Reset` when the surface changes.
- `xIcr ≠ 0` makes `vy` non-zero in turns. Trackers that assume `vy = 0` (W5, W7) see it as a
  disturbance, and odometry (F1) should integrate the full twist.
- Float sums: for runs of more than about 10⁶ samples, accumulate in blocks or `Reset` periodically.
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Header: `robotics/mobile/SkidSteerKinematics.hpp` — `#pragma once`, `OPTIMIZE_FOR_SPEED` on
  `Forward`/`Inverse`, and `extern template class SkidSteerKinematics<float>;` and
  `extern template class IcrEstimator<float>;` under `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `robotics/mobile/SkidSteerKinematics.cpp` → the same instantiations.
- Test: `robotics/mobile/test/TestSkidSteerKinematics.cpp`
- Doc: `doc/mobile/SkidSteerKinematics.md` (per `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources(robotics.mobile ...)`; `.cpp` → `robotics_add_coverage_sources`;
  test → `robotics.mobile_test`.
- Depends on: W1 (`WheelSpeeds`, `ScaleToLimit`).
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
