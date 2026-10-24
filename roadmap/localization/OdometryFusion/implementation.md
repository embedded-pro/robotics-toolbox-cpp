# Wheel + Gyro Odometry Fusion (EKF on SE(2)) — Implementation Pseudocode

> Roadmap ref: #E1 (★★★☆☆) · Target: `robotics/localization` · Namespace `localization` · Type: `float` (templated on `T`, instantiated for `float` only)

Dead reckoning that trusts the gyro for rotation and the wheels for distance. It estimates the gyro
bias from the agreement between wheel and gyro yaw rates, and refuses that comparison when the wheels
slip. State `(x, y, θ, b_g)`: planar pose plus gyro bias. The mean is propagated with the exact SE(2)
arc (NT66, as in F1). Every measurement update goes through upstream `ResidualKalmanUpdate` (NT74), so
heading residuals are wrapped and slip shows up as a χ²-gated wheel update. Optional absolute heading
(compass, dual-antenna GNSS) and position (GNSS, scan matching E5) updates share the same path.

## Data structures

```cpp
template<typename T>                            # static_assert(std::is_floating_point_v<T>); instantiated for float
struct OdometryFusionNoise:
    T speedNoiseFloor                           # σ_v at standstill, m/s
    T speedNoiseGain                            # σ_v grows by this fraction of |v|
    T gyroNoise                                 # σ_g, rad/s per sample
    T biasRandomWalk                            # σ_b, rad/s/√s
    T wheelYawRateNoise                         # σ_w, rad/s (wheel-derived yaw rate)
    T slipGate                                  # χ²(1) gate for the wheel update, e.g. 6.63 (99 %)

template<typename T>
class OdometryFusion:
    math::Vector<T, 4>       state              # (x, y, θ, b_g), θ wrapped to (−π, π]
    math::SquareMatrix<T, 4> covariance
    OdometryFusionNoise<T>   noise
```

## Interface

```cpp
OdometryFusion(const math::SE2Transform<T>& pose, const math::SquareMatrix<T, 4>& covariance, const OdometryFusionNoise<T>& noise)
void Predict(T speed, T gyroRate, T dt)                                            # hot path
filters::KalmanUpdateResult<T> UpdateWheelYawRate(T wheelYawRate, T gyroRate)       # gated: status gated ⇒ slip
filters::KalmanUpdateResult<T> UpdateHeading(T heading, T sigma)
filters::KalmanUpdateResult<T> UpdatePosition(const math::Vector2<T>& position, const math::SquareMatrix<T, 2>& covariance)
math::SE2Transform<T> Pose() const;  T GyroBias() const
const math::Vector<T, 4>& State() const;  const math::SquareMatrix<T, 4>& Covariance() const
```

`speed` is the forward speed from a wheel model (W1 `Forward`, W4), `gyroRate` the raw z-gyro, and
`wheelYawRate` the wheel model's `ω`.

## Algorithm (pseudocode)

```text
function Predict(v, ω_g, dt):                   # OPTIMIZE_FOR_SPEED
    ω = ω_g − b;  θm = θ + ω·dt/2               # midpoint heading for the Jacobians
    pose = (Pose() * SE2Transform::Exp((v, 0, ω), dt))       # exact arc (NT66)
    F = I₄ with F[0,2] = −v·dt·sin θm,  F[0,3] =  v·dt²/2·sin θm,
                F[1,2] =  v·dt·cos θm,  F[1,3] = −v·dt²/2·cos θm,  F[2,3] = −dt
    G = [[dt·cos θm, −v·dt²/2·sin θm], [dt·sin θm, v·dt²/2·cos θm], [0, dt], [0, 0]]   # ∂f/∂(v, ω_g)
    σ_v = speedNoiseFloor + speedNoiseGain·|v|
    Q = G·diag(σ_v², σ_g²)·Gᵀ + diag(0, 0, 0, σ_b²·dt)
    P = CongruenceTransform(F, P) + Q;  state = (pose.x, pose.y, pose.Heading(), b)

function UpdateWheelYawRate(ω_w, ω_g):          # both rates measure the true ω; the difference observes b
    r = (ω_w − (ω_g − b));  H = [0, 0, 0, −1];  R = σ_w² + σ_g²
    return Apply(state, P, r, H, R, slipGate);  wrap θ          # NT74; gated ⇒ wheels slipping, nothing applied

function UpdateHeading(θ_m, σ):
    r = AngleDifference(θ_m, θ);  H = [0, 0, 1, 0];  return Apply(state, P, r, H, σ², ∞);  wrap θ

function UpdatePosition(p, Σ):
    r = p − (x, y);  H = [[1, 0, 0, 0], [0, 1, 0, 0]];  return Apply(state, P, r, H, Σ, ∞);  wrap θ
```

## Complexity & memory

- `Predict`: one `Exp`, one 4×4 congruence and a 4×2 noise map. Updates: NT74 with `M ≤ 2`.
- Memory: 20 scalars of state plus the noise block. No heap.

## Numerical / embedded notes

- **Why a bias state.** An uncorrected 0.02 rad/s gyro bias turns 5 s at standstill into a 0.1 rad heading
  error. With the wheel update the estimated bias reaches 0.019992 and the heading error stays below
  `4e-5` (test). Standstill, where the wheel yaw rate is exactly 0, is the strongest bias observation.
- **Slip awareness.** During slip (spinning on carpet, a stuck caster) the wheel yaw rate disagrees with
  the gyro far beyond `σ_w`. The gate rejects the update, and the returned status lets the caller flag
  slip or raise `speedNoiseGain` for the following predicts.
- **Linearisation.** The mean uses the exact arc; the Jacobians use the midpoint heading, whose difference
  from the exact arc's Jacobian is `O(v·ω²·dt³)`.
- `R = σ_w² + σ_g²` in the wheel update because the gyro sample enters the predicted rate.
- An accelerometer can later add a lateral-slip test (`a_y ≈ v·ω`). It is left out to keep the state at 4.
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Header: `robotics/localization/OdometryFusion.hpp` — `#pragma once`, `OPTIMIZE_FOR_SPEED` on `Predict`,
  and `extern template class OdometryFusion<float>;` under `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `robotics/localization/OdometryFusion.cpp` → `template class OdometryFusion<float>;`
- Test: `robotics/localization/test/TestOdometryFusion.cpp`
- Doc: `doc/localization/OdometryFusion.md` (per `doc/TEMPLATE.md`)
- CMake: new module `robotics/localization` (`robotics.localization`, test target
  `robotics.localization_test`), created by the first `localization` item deployed.
- Depends on: F1 (motion integration convention), NT66 (`SE2Transform`), NT54 (`AngleDifference`, `WrapToPi`),
  NT74 (`ResidualKalmanUpdate`); upstream `MatrixOperations::CongruenceTransform`.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
