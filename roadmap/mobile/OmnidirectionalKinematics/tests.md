# Omnidirectional Kinematics — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```cpp
class TestOmnidirectionalKinematics : public ::testing::Test:
    # mecanum: r = 0.05, l_x = 0.2, l_y = 0.15, order FL, FR, RL, RR, β = 0
    OmnidirectionalKinematics<float, 4> mecanum{ { { (0.2, 0.15), 0, −π/4, 0.05 }, { (0.2, −0.15), 0, π/4, 0.05 },
                                                   { (−0.2, 0.15), 0, π/4, 0.05 }, { (−0.2, −0.15), 0, −π/4, 0.05 } }, 20 }
    # kiwi: omniwheels (γ = 0) at radius 0.2, angles a = 90°, 210°, 330°, β = a + 90°, r = 0.05
    OmnidirectionalKinematics<float, 3> kiwi{ ..., 20 }
# each case below is a TEST_F(TestOmnidirectionalKinematics, <name>)
```

## Test cases (Arrange / Act / Assert)

```cpp
mecanum_jacobian_matches_the_closed_form:
    Assert: mecanum.WheelJacobian() ≈ [[20, −20, −7], [20, 20, 7], [20, 20, −7], [20, −20, 7]]
inverse_then_forward_recovers_the_twist:
    Assert: mecanum.Inverse((0.3, −0.2, 0.5)) ≈ (6.5, 5.5, −1.5, 13.5)
            mecanum.Forward((6.5, 5.5, −1.5, 13.5)) ≈ (0.3, −0.2, 0.5);  SlipResidual ≈ 0
forward_is_least_squares_and_flags_slip:
    Arrange: u = (10, 0, 0, 0)                  # one wheel spinning, three stalled
    Assert:  mecanum.Forward(u) ≈ (0.125, −0.125, −0.357143);  mecanum.SlipResidual(u) ≈ 5
kiwi_is_exactly_determined:
    Assert: kiwi.WheelJacobian() ≈ [[−20, 0, 4], [10, −17.320508, 4], [10, 17.320508, 4]]
            kiwi.Forward((5, 5, 5)) ≈ (0, 0, 1.25);  kiwi.SlipResidual((3, −1, 7)) ≈ 0
limit_scales_all_wheels_uniformly:
    Arrange: u = mecanum.Inverse((1, 0.5, 1)) = (3, 37, 23, 17)
    Assert:  mecanum.Limit(u) ≈ u·20/37;  mecanum.Forward(mecanum.Limit(u)) ≈ (0.540541, 0.270270, 0.540541)
```

## Reference vectors

- Mecanum `H⁺ = [[0.0125]·4; [−0.0125, 0.0125, 0.0125, −0.0125]; [−0.035714, 0.035714, −0.035714, 0.035714]]`.
- `u = (10, 0, 0, 0)` ⇒ residual `(2.5, 2.5, −2.5, −2.5)`, norm 5.

## Edge cases

- A degenerate layout (all wheels driving parallel, e.g. every `β = 0, γ = 0`) fails the construction
  assert (rank-deficient `HᵀH`).
- `Limit` of an all-zero vector returns zeros.
