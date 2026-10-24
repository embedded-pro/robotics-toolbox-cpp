# Odometry Fusion — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```cpp
class TestOdometryFusion : public ::testing::Test:
    OdometryFusionNoise<float> noise{ 0.01, 0.05, 0.01, 1e-4, 0.02, 6.63 }
    SquareMatrix<float, 4> P0{ diag(0.01, 0.01, 0.01, 0.0025) }
    OdometryFusion<float> fusion{ Identity, P0, noise }
# each case below is a TEST_F(TestOdometryFusion, <name>)
```

## Test cases (Arrange / Act / Assert)

```cpp
straight_drive_grows_the_covariance_along_and_across:
    Act:    10 × Predict(1, 0, 0.1)
    Assert: State() ≈ (1, 0, 0, 0);  diag(P) ≈ (0.010360, 0.020628, 0.012510, 0.0025);  P(1,2) ≈ 0.011255
arc_uses_the_exact_se2_exponential:
    Act:    20 × Predict(1, 0.5, 0.1)
    Assert: Pose().Pose() ≈ (1.682942, 0.919395, 1.0) == Exp((1, 0, 0.5), 2)
standstill_estimates_the_gyro_bias:
    Act:    500 × { Predict(0, 0.02, 0.01); UpdateWheelYawRate(0, 0.02) }
    Assert: GyroBias() ≈ 0.019992 (tol 1e-4);  |θ| < 1e-4;  P(3,3) ≈ 1.02e-6
            # after the first update the bias is already 0.016667
slip_is_gated_and_leaves_the_state_unchanged:
    Arrange: Predict(1, 0, 0.1)
    Act:     result = UpdateWheelYawRate(1.0, 0.0)
    Assert:  result.status == gated, nis ≈ 333.33;  State() and Covariance() unchanged
heading_update_wraps_the_residual:
    Arrange: fusion at FromPose(0, 0, 3.1)
    Act:     UpdateHeading(−3.1, 0.05)
    Assert:  status applied;  θ ≈ −3.116637   # K = 0.8 on the wrapped residual 0.083185
position_update_pulls_the_pose_and_shrinks_the_covariance:
    Act:    UpdatePosition((0.1, −0.05), diag(0.01, 0.01))
    Assert: State() ≈ (0.05, −0.025, 0, 0);  P(0,0) ≈ P(1,1) ≈ 0.005;  nis ≈ 0.625
```

## Reference vectors

- Gyro bias 0.02 rad/s for 5 s without wheel updates ⇒ heading 0.1 rad; with them ⇒ `4e-5` rad.
- Straight drive (v = 1, dt = 0.1, 10 steps): the cross-track variance doubles (`0.01 → 0.020628`) through
  the heading uncertainty.

## Edge cases

- `v = 0` and `ω_g − b = 0` ⇒ `F = I` except the bias column; covariance grows only by `Q`.
- A singular update (all-zero covariance and noise) returns `singular` and changes nothing.
