# Skid-Steer Kinematics — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```cpp
class TestSkidSteerKinematics : public ::testing::Test:
    IcrParameters<float> identified{ 0.30, −0.28, 0.05 }
    SkidSteerKinematics<float> vehicle{ identified, 0.1, 20 }       # r = 0.1
    IcrEstimator<float> estimator{ 0.1 }
# each case below is a TEST_F(TestSkidSteerKinematics, <name>)
```

## Test cases (Arrange / Act / Assert)

```cpp
ideal_icr_reduces_to_differential_drive:
    Arrange: SkidSteerKinematics ideal{ IdealIcr(0.4), 0.1, 20 };  DifferentialDrive drive{ 0.1, 0.4, 20 }
    Assert:  ideal.Forward({ 5, 8 }) ≈ drive.Forward({ 5, 8 }) ≈ (0.65, 0, 0.75)
forward_with_identified_icr_turns_slower_and_slips:
    Assert: vehicle.Forward({ 5, 8 }) ≈ (0.655172, −0.025862, 0.517241)
inverse_inverts_forward:
    Assert: vehicle.Inverse(0.655172, 0.517241) ≈ { 5, 8 }
estimator_recovers_the_icr_from_consistent_samples:
    Arrange: for s in { (5, 8), (8, 5), (−3, 4), (6, 6), (2, −2) }: estimator.Add(s, vehicle.Forward(s))
    Assert:  Samples() == 5;  Estimate(0.01) ≈ { 0.30, −0.28, 0.05 }   # Σω² = 2.467301
estimator_rejects_straight_driving:
    Arrange: estimator.Add({ 5, 5 }, vehicle.Forward({ 5, 5 }));  estimator.Add({ 8, 8 }, vehicle.Forward({ 8, 8 }))
    Assert:  Estimate(0.01) == nullopt;  after Reset(), Samples() == 0
```

## Reference vectors

- `(5, 8)` rad/s with `r = 0.1`: tread speeds `(0.5, 0.8)`, `D = 0.58` ⇒ `ω = 0.517241`,
  `vx = (0.8·0.30 + 0.5·0.28)/0.58 = 0.655172`, `vy = −0.05·ω = −0.025862`.
- The ideal model (`B = 0.4`) would predict `ω = 0.75` for the same treads: identified skid lowers the
  turn rate by 31 %.

## Edge cases

- Equal tread speeds ⇒ `ω = 0`, `vy = 0`, `vx = r·s` for any ICR.
- `Limit` is W1's `ScaleToLimit` (not re-tested here).
