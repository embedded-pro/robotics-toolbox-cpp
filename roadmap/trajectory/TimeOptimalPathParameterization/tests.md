# Time-Optimal Path Parameterization (TOPP-RA) — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```cpp
class PathGeometryMock : public PathGeometry<float, 2>          # MOCK_METHOD Position / FirstDerivative / SecondDerivative
class JointLimitsMock  : public JointLimits<float, 2>           # MOCK_METHOD Coefficients / Bounds
class InverseDynamicsModelMock : public dynamics::InverseDynamicsModel<float, 2>   # MOCK_METHOD ComputeInverseDynamics

class TestTopp : public ::testing::Test:
    StrictMock<PathGeometryMock> path
    StrictMock<JointLimitsMock>  limits
    TimeOptimalPathParameterization<float, 2, 64> topp{ path, limits }   # N = 63 stages, Δ = 1/63
    JointBounds<float, 2> unitTorque{ .torqueMin = (−1, −1), .torqueMax = (1, 1), .velocityMax = (100, 100) }

    void GivenStraightSingleJointPath(float L, JointBounds bounds):
        # q(s) = (L·s, 0), unit inertia, no gravity: a = (L, 0), b = 0, c = 0
        EXPECT_CALL(path, Position(_)).WillRepeatedly(Return((L·s, 0)))
        EXPECT_CALL(path, FirstDerivative(_)).WillRepeatedly(Return((L, 0)))
        EXPECT_CALL(path, SecondDerivative(_)).WillRepeatedly(Return((0, 0)))
        EXPECT_CALL(limits, Coefficients(_)).WillRepeatedly(Return({ (L, 0), (0, 0), (0, 0) }))
        EXPECT_CALL(limits, Bounds()).WillRepeatedly(ReturnRef(bounds))
# each case below is a TEST_F(TestTopp, <name>)
```

## Test cases (Arrange / Act / Assert)

```cpp
bang_bang_total_time_matches_closed_form:
    Arrange: GivenStraightSingleJointPath(L = 1, unitTorque)
    Act:     Parameterize()
    Assert:  true;  TotalTime() ≈ 2·sqrt(L) = 2  (tol 1e-3; exact value 2.000064 for Grid 64)

bang_bang_acceleration_saturates_torque:
    Arrange: as above, L = 4
    Assert:  |Sample(t).acceleration_0| ≈ 1 (= |τ|max / inertia) for t in the first and last 45 % of
             TotalTime(); positive before TotalTime()/2, negative after

velocity_limit_caps_path_speed:
    Arrange: GivenStraightSingleJointPath(L = 1, { torque ±100, velocityMax = (0.5, 0.5) })
    Assert:  every sampled |velocity_0| ≤ 0.5·(1 + tol);  Sample(TotalTime()/2).velocity_0 ≈ 0.5;
             TotalTime() ≈ 2 + 4Δ = 2.063492

profile_respects_torque_and_velocity_bounds:
    Arrange: q(s) = (s, 0.4s − 0.4s²) ⇒ q' = (1, 0.4 − 0.8s), q'' = (0, −0.8);
             coupled mock coefficients a = (1 + 0.5s, 0.4 − 0.8s), b = (0.3·cos 3s, −0.2 + 0.1s),
             c = (0.1, −0.05s); τ ∈ [(−1, −0.6), (1, 0.6)], q̇max = (1.5, 0.8)
    Act:     Parameterize(); dense Sample(t)  (s = position_0, ṡ = velocity_0, s̈ = acceleration_0)
    Assert:  true; TotalTime() ≈ 2.256496; |q̇_j| ≤ q̇max_j; a(s)·s̈ + b(s)·ṡ² + c(s) ∈ [τmin − 0.02, τmax + 0.02]
             (collocation: exact at gridpoints, O(Δ) between them — 0.018 measured)

starts_and_ends_at_rest_on_path_endpoints:
    Arrange: GivenStraightSingleJointPath(L = 1, unitTorque)
    Assert:  Sample(0) = { q(0), 0, · }  and  Sample(TotalTime()) = { q(1), ≈0, · }

sample_is_continuous_across_stages:
    Arrange: GivenStraightSingleJointPath(L = 1, unitTorque)
    Assert:  for every interior stage start time t_i: Sample(t_i − δ) ≈ Sample(t_i + δ) in position and
             velocity (δ = 1e-4·TotalTime())

infeasible_static_torque_reports_failure:
    Arrange: straight path with c = (2, 0) > τmax = 1 (gravity cannot be held)
    Assert:  Parameterize() == false  and  Sample(0) == nullopt

sample_before_parameterize_returns_nullopt:
    Assert: Sample(0) == nullopt

zero_length_path_takes_zero_time:
    Arrange: q'(s) = q''(s) = (0, 0), q(s) = (0.3, −0.2)
    Assert:  Parameterize() == true, TotalTime() == 0, Sample(0) = { (0.3, −0.2), 0, 0 }

inverse_dynamics_adapter_builds_coefficients:
    Arrange: StrictMock<InverseDynamicsModelMock> dyn; path mock q = (0.1, 0.2), q' = (1, 2), q'' = (3, 4)
             EXPECT_CALL(dyn, ComputeInverseDynamics(q, 0, 0))    → g       = (0.5, 0.25)
             EXPECT_CALL(dyn, ComputeInverseDynamics(q, 0, q'))   → g + Mq' = (1.5, 2.25)
             EXPECT_CALL(dyn, ComputeInverseDynamics(q, q', q'')) → g + Mq'' + Cq' = (4.5, 5.25)
    Act:     InverseDynamicsJointLimits<float, 2>{ path, dyn, unitTorque }.Coefficients(0.5)
    Assert:  a ≈ (1, 2), b ≈ (4, 5), c ≈ (0.5, 0.25)
```

## Reference vectors

Computed with a python3 implementation of exactly this pseudocode.

- Single joint, unit inertia, `|τ| ≤ 1`, path length `L`, rest-to-rest ⇒ bang-bang time `2·sqrt(L)`.
  Grid 64 (63 stages, odd ⇒ one coast stage at the midpoint): `L = 1 → 2.000064`, `L = 4 → 4.000128`
  (relative error `3.2e-5`); Grid 65 (even stage count): exact to `1e-15`; Grid 128: `7.8e-6`.
- Velocity-limited straight segment, `q̇max = 0.5`, `|τ| ≤ 100`, `L = 1`, Grid 64 ⇒ `ṡ = 0.5` on the
  interior, one full ramp stage at each end: `TotalTime = (N − 2)·Δ/0.5 + 2·(2Δ/0.5) = 2 + 4/63 = 2.063492`.
- Coupled case above ⇒ feasible, `TotalTime = 2.256496`; torque bound violation `1e-12` at gridpoints,
  `0.0178` between them (Grid 64); velocity bounds never violated.
- `c = 2 > τmax = 1` ⇒ every stage forces `u ≤ −1`, so `0 ∉ K_0` ⇒ infeasible.
- Adapter on a 2R planar arm: `τ(q'ṡ, q'u + q''ṡ²) − (a·u + b·ṡ² + c)` ≤ `2e-15` for arbitrary `(ṡ, u)`.

## Edge cases

- Zero-length path (`q'` ≡ 0) ⇒ `TotalTime() == 0`, trivial success.
- `q'_j(s) = 0` or `a_ij(s) = 0` at a gridpoint ⇒ no division, row becomes a pure `x` bound.
- Velocity-only vs torque-only limiting ⇒ the binding row changes per gridpoint; both cases above.
- `Grid = 2` (one stage) cannot leave and return to rest with one constant `u` ⇒ stalls; hence
  `static_assert(Grid >= 3)`. `Grid = 3` on the bang-bang case gives exactly `2·sqrt(L)`.
