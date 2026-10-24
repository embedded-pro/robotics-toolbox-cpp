# Car-Like Kinematics — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```cpp
class TestCarLikeKinematics : public ::testing::Test:
    CarLikeKinematics<float> car{ { 2.5, 1.5, 0.6, 0.5 } }   # L, W, δ_max, δ̇_max
# each case below is a TEST_F(TestCarLikeKinematics, <name>)
```

## Test cases (Arrange / Act / Assert)

```cpp
forward_gives_the_rear_axle_twist:
    Assert: Forward(10, 0.1) ≈ (10, 0, 0.401339)
twist_at_offset_matches_the_se2_adjoint:
    Assert: TwistAt(10, 0.1, 1.2) ≈ (10, 0.481606, 0.401339)
            ≈ FromPose(−1.2, 0, 0).TransformTwist(Forward(10, 0.1))
steer_for_curvature_inverts_curvature_and_saturates:
    Assert: SteerForCurvature(0.1) ≈ 0.244979;  Curvature(0.244979) ≈ 0.1
            SteerForCurvature(0.5) == 0.6       # unclamped atan(1.25) = 0.896055
limit_steer_applies_angle_then_rate:
    Assert: LimitSteer(0.5, 0.1, 0.2) ≈ 0.2     # rate: 0.1 per 0.2 s
            LimitSteer(0.9, 0.55, 1) ≈ 0.6      # angle limit binds
            LimitSteer(−0.9, 0.55, 1) ≈ 0.05    # angle limit, then rate
ackermann_wheels_share_one_turning_centre:
    Assert: Ackermann(0.3) ≈ { 0.328616, 0.275851 }
            tan(left)·(R − 0.75) ≈ tan(right)·(R + 0.75) ≈ 2.5 with R = 2.5 / tan 0.3 = 8.081820
            Ackermann(−0.3) ≈ { −0.275851, −0.328616 };  Ackermann(0) == { 0, 0 }
min_turning_radius_uses_max_steer:
    Assert: MinTurningRadius() ≈ 3.654240
```

## Reference vectors

- `L = 2.5`: `tan 0.1 = 0.100335` ⇒ `ω = 10·0.100335/2.5 = 0.401339`.
- Ackermann at `δ = 0.3`: inner (left) `0.328616`, outer (right) `0.275851`.

## Edge cases

- `v < 0` reverses the yaw-rate sign for the same steer (`Forward(−10, 0.1)` ⇒ `ω = −0.401339`).
- `SteerForCurvature(0)` = 0; `Curvature(±δ_max)` = `±1/MinTurningRadius()`.
