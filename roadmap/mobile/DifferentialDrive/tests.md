# Differential-Drive Kinematics — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```cpp
class TestDifferentialDrive : public ::testing::Test:
    DifferentialDrive<float> drive{ 0.05, 0.3, 20 }        # r = 5 cm, B = 30 cm, 20 rad/s
# each case below is a TEST_F(TestDifferentialDrive, <name>)
```

## Test cases (Arrange / Act / Assert)

```cpp
forward_maps_wheel_speeds_to_a_unicycle_twist:
    Assert: Forward({ 10, 14 }) ≈ (0.6, 0, 0.666667)
inverse_inverts_forward:
    Assert: Inverse(0.6, 0.666667) ≈ { 10, 14 }
spin_in_place_counter_rotates_the_wheels:
    Assert: Inverse(0, 2) ≈ { −6, 6 };  Forward({ −6, 6 }) ≈ (0, 0, 2)
limit_preserves_curvature:
    Assert: Inverse(1.2, 2) ≈ { 18, 30 };  Limit({ 18, 30 }) ≈ { 12, 20 }
            Achievable(1.2, 2) ≈ (0.8, 0, 1.333333)   # ω/v = 1.666667 before and after
limit_leaves_feasible_speeds_unchanged:
    Assert: Limit({ 10, −14 }) == { 10, −14 }
speed_envelope_corners:
    Assert: MaxLinearSpeed() ≈ 1;  MaxAngularSpeed() ≈ 6.666667
            Achievable(1, 0) ≈ (1, 0, 0);  Achievable(0, 6.666667) ≈ (0, 0, 6.666667)
```

## Reference vectors

- `r = 0.05, B = 0.3`: wheels `(10, 14)` rad/s ⇔ `(v, ω) = (0.6, 0.666667)`.
- `(1.2, 2)` needs `(18, 30)` rad/s; scaled by `20/30` it becomes `(12, 20)` ⇒ `(0.8, 1.333333)`.

## Edge cases

- Negative `v` (reversing) maps symmetrically; `Limit` uses `|·|`.
- `ScaleToLimit` with both wheels at zero returns zeros (no division).
