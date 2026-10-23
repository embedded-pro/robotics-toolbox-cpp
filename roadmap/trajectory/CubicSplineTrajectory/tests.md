# Multi-Joint Cubic Spline Trajectory — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

The interpolation math (knot values, continuity, clamped/natural ends, reference second derivatives) is
tested upstream with `math::CubicSpline` (NT61). These cases cover only the trajectory mapping.

## Fixture

```cpp
class TestCubicSplineTrajectory : public ::testing::Test:
    using Spline = CubicSplineTrajectory<float, 3, 8>
    std::array<float, 8>              times{ 0, 1, 2.5, 3, 4.5 }        # K = 5 used
    std::array<math::Vector<float,3>, 8> positions{ (0, 0.5, 0), (1, 0.2, 0), (−0.5, 0.2, 0),
                                                    (0.3, −0.4, 0), (2, 0, 0) }
    Spline spline
    void SetUp(): ASSERT_TRUE(spline.Plan(times, positions, 5))      # default: clamped, zero end velocities
# each case below is a TEST_F(TestCubicSplineTrajectory, <name>)
```

## Test cases (Arrange / Act / Assert)

```cpp
default_plan_is_rest_to_rest:
    Assert: Sample(0).position ≈ q_0, Sample(4.5).position ≈ q_4;  Sample(0).velocity ≈ 0, Sample(4.5).velocity ≈ 0

state_maps_value_and_derivatives_to_position_velocity_acceleration:
    Assert: joint 0 knot accelerations Sample(t_k).acceleration_0 ≈ (5.665169, −5.330337, 5.991011,
            −0.737079, −1.898127) (the upstream second derivatives); Sample(0.5).velocity is the spline slope

start_time_and_duration_follow_the_knots:
    Arrange: times shifted by +2 s
    Assert:  StartTime() ≈ 2, Duration() ≈ 4.5, Sample(2).position ≈ q_0

two_knots_reduce_to_the_cubic_polynomial_profile:
    Arrange: times {0, 2}, q {0, 1}, Clamped startVelocity 0.5, endVelocity −0.5;
             motion_profiles::PolynomialProfile<float>{ { .q0 = 0, .qf = 1, .v0 = 0.5, .vf = −0.5 }, 2, Degree::Cubic }
    Assert:  position, velocity, acceleration equal at t ∈ {0.3, 1.0, 1.7}; knot accelerations (1, −2)

plan_rejects_invalid_knots:
    Assert: Plan(count = 1) == false;  Plan(count = 9) == false;  times {0, 1, 1, 2} == false
```

## Reference vectors

- Fixture joint 0, clamped zero ⇒ knot accelerations `(5.665169, −5.330337, 5.991011, −0.737079, −1.898127)`
  (python3; same values as the upstream spline test).
- Two knots, `v0 = 0.5, vf = −0.5, tf = 2` ⇒ `M = (2a2, 2a2 + 6a3·tf) = (1, −2)`.

## Edge cases

- `K = MaxPoints` (8) ⇒ accepted; `Sample` at `t = t_{K−1}` in range.
- `t` outside `[t_0, t_{K−1}]` ⇒ clamped by the spline (end state).
