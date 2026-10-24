# Artificial Potential Fields — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```cpp
class TestPotentialField : public ::testing::Test:
    PotentialFieldParameters<float> parameters{ 1.0, 1.0, 0.1, 1.0, 0.5, 1e-3, 0.05 }
        # k_a, d*, k_r, ρ₀, maxSpeed, stallSpeed, goalTolerance
    PotentialField<float> field{ parameters }
    Vector3 origin{ 0, 0, 0 }
# each case below is a TEST_F(TestPotentialField, <name>)
```

## Test cases (Arrange / Act / Assert)

```cpp
near_goal_the_pull_is_quadratic:
    Assert: Compute(origin, (0.4, 0.3, 0), {}) ⇒ velocity (0.4, 0.3, 0), attractive 0.5, moving
far_goal_the_pull_is_conic_and_clamped:
    Assert: Compute(origin, (3, 4, 0), {}) ⇒ velocity (0.3, 0.4, 0)   # |F_att| = k_a·d* = 1, clamped to 0.5
obstacle_inside_the_influence_distance_repels:
    Assert: Compute(origin, (3, 4, 0), { ((0, −1, 0), 0.5) }) ⇒ repulsive 0.4, velocity ≈ (0.416025, 0.277350, 0)
obstacle_outside_the_influence_distance_is_ignored:
    Assert: Compute(origin, (3, 4, 0), { ((0, −1, 0), 1.2) }) ⇒ repulsive 0, velocity (0.3, 0.4, 0)
goal_within_tolerance_is_reached:
    Assert: Compute(origin, (0.03, 0, 0), {}).status == reached
opposing_obstacle_creates_a_detected_local_minimum:
    Assert: Compute(origin, (3, 0, 0), { ((−1, 0, 0), 0.393003) }) ⇒ attractive ≈ repulsive ≈ 1, |velocity| < 1e-3, stalled
            with ρ = 0.3 instead ⇒ moving, velocity (−0.5, 0, 0)   # repulsion wins: 2.592593
```

## Reference vectors

- Local-minimum distance: the root of `0.1·(1/ρ − 1)/ρ² = 1`, `ρ* = 0.393003`.
- `ρ = 0.5`: `0.1·(2 − 1)/0.25 = 0.4`.

## Edge cases

- No obstacles and `d = 0` ⇒ zero velocity, `reached`.
- Several obstacles add vectorially; symmetric obstacles cancel laterally.
