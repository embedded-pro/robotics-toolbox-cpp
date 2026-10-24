# Planar Odometry — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```cpp
class TestPlanarOdometry : public ::testing::Test:
    PlanarOdometry<float> odometry                          # starts at Identity
    Vector3<float> arc{ 0.5, 0, 0.25 }                      # radius 2 m circle, period 8π s
# each case below is a TEST_F(TestPlanarOdometry, <name>)
```

## Test cases (Arrange / Act / Assert)

```cpp
straight_line_accumulates_distance:
    Act:    4 × Integrate((1, 0, 0), 0.25)
    Assert: Pose().Pose() ≈ (1, 0, 0)
quarter_arc_is_exact_in_one_step:
    Act:    Integrate(arc, 2π)
    Assert: Pose().Pose() ≈ (2, 2, π/2)
full_circle_closes_independent_of_step_count:
    Act:    7 steps of 8π/7, and separately 1000 steps of 8π/1000
    Assert: both end at (0, 0, 0)  (tol 1e-4 for 1000 steps)
increment_recovers_the_integrated_twist:
    Arrange: odometry starts at FromPose(1, 2, 0.3)
    Act:     Integrate((0.4, 0.1, −0.6), 0.5)
    Assert:  Increment(FromPose(1, 2, 0.3), Pose()) ≈ (0.2, 0.05, −0.3)
reset_replaces_the_pose:
    Act:    Integrate(arc, 1); Reset(FromPose(−1, 3, 2))
    Assert: Pose().Pose() ≈ (−1, 3, 2)
increment_deviation_grows_with_distance_and_rotation:
    Arrange: α = (0.05, 0.01, 0.05, 0.01)
    Assert:  IncrementDeviation(FromPose(0.5, 0, 0.1), α) ≈ (0.112250, 0.112250, 0.054772)
             IncrementDeviation(FromPose(−0.5, 0, 0), α) == IncrementDeviation(FromPose(0.5, 0, 0), α)
long_runs_keep_a_unit_rotation:
    Act:    10⁵ × Integrate(arc, 1e-3)
    Assert: cosine² + sine² ≈ 1 (tol 1e-6)
```

## Reference vectors

- Twist `(0.5, 0, 0.25)`: radius `v/ω = 2`; after `2π` s the robot is at `(2, 2, π/2)`; after `8π` s it is
  back at the origin.
- `FromPose(1, 2, 0.3)` integrated with `(0.4, 0.1, −0.6)` for 0.5 s ⇒ `Increment = (0.2, 0.05, −0.3)`.

## Edge cases

- `dt = 0` leaves the pose unchanged.
- `ω = 0` reduces to straight-line motion along the body heading.
