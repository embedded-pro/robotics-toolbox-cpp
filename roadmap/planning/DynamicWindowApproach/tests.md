# Dynamic Window Approach — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```cpp
class TestDynamicWindowApproach : public ::testing::Test:
    DwaParameters<float> parameters{ 0, 1.0, 2.0, 1.0, 3.0, 0.25, 1.0, 0.2, 2.0, 1.0, 1.0, 0.5 }
        # v_min, v_max, ω_max, a, α, Δt, prediction, radius, cap, w_heading, w_clearance, w_speed
    DynamicWindowApproach<float, 5, 11> dwa{ parameters }
# each case below is a TEST_F(TestDynamicWindowApproach, <name>)
```

## Test cases (Arrange / Act / Assert)

```cpp
straight_clearance_stops_at_the_disc_contact:
    Assert: ArcClearance(1, 0, (2, 0.1), 0.2, 2) ≈ 1.826795      # 2 − √0.03
            ArcClearance(1, 0, (−1, 0), 0.2, 2) == 2              # behind the robot
arc_clearance_on_a_turn:
    Assert: ArcClearance(1, 1, (1, 1), 0.2, 2) ≈ 1.370461         # π/2 − acos 0.98
            ArcClearance(1, −1, (1, −1), 0.2, 2) ≈ 1.370461;  ArcClearance(1, 1, (3, 3), 0.2, 2) == 2
contact_and_rotation_in_place:
    Assert: ArcClearance(1, 0, (0.1, 0.05), 0.2, 2) == 0;  ArcClearance(0, 1, (0.5, 0), 0.2, 2) == 2
window_respects_acceleration_and_bounds:
    Assert: Window(0.5, 0) ≈ { 0.25, 0.75, −0.75, 0.75 };  Window(0.95, 1.9) ≈ { 0.7, 1.0, 1.15, 2.0 }
            Window(0.1, −1.9) ≈ { 0, 0.35, −2.0, −1.15 }
free_space_drives_fast_towards_the_goal:
    Assert: Compute(0.5, 0, (5, 0), {}) ⇒ { 0.75, 0, 2 }
goal_to_the_left_turns_left:
    Assert: Compute(0.5, 0, (0, 3), {}) ⇒ { 0.75, 0.75 }
obstacle_slightly_left_turns_right:
    Assert: Compute(0.5, 0, (5, 0), { (0.45, 0.05) }) ⇒ { 0.375, −0.75 }
            Compute(0.5, 0, (5, 0), { (0.45, −0.05) }) ⇒ { 0.375, 0.75 }   # mirror
too_fast_to_stop_returns_nullopt:
    Arrange: wall of 21 points x = 0.3, y ∈ [−1, 1]
    Assert:  Compute(0.75, 0, (5, 0), wall) == nullopt;  Compute(0, 0, (5, 0), wall) ⇒ { 0, 0 }
```

## Reference vectors

- Turn `v = ω = 1` (radius 1, centre `(0, 1)`), obstacle `(1, 1)`, `r = 0.2`: contact half-angle
  `acos 0.98 = 0.200335` ⇒ clearance `π/2 − 0.200335 = 1.370461`.
- Obstacle `(0.45, 0.05)`: 44 of 55 samples admissible; best score `2.052105` against `2.004153` for the
  runner-up `(0.25, −0.6)`.

## Edge cases

- Empty obstacle span ⇒ every sample has clearance `cap`.
- `NumV`/`NumW` grid endpoints coincide when the window collapses (`v` at `v_max` with `a·Δt` clipped):
  duplicate samples are harmless.
