# Pure Pursuit and Planar Path — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixtures

```cpp
class TestPlanarPath : public ::testing::Test:
    PlanarPath<float, 8> path                    # Assign({ (0, 0), (2, 0), (4, 2), (4, 5) }) in the constructor
class TestPurePursuit : public ::testing::Test:
    PlanarPath<float, 8> path                    # same waypoints
    PlanarPath<float, 8> uTurn                   # { (0, 0), (4, 0), (4, 1), (0, 1) }
    PurePursuitParameters<float> parameters{ 1.0, 3.0, 0.5, 0.1, 2 }   # L_min, L_max, gain, tolerance, window
# each case below is a TEST_F(<fixture>, <name>)
```

## Test cases — `TestPlanarPath`

```cpp
assign_computes_arc_lengths_and_rejects_bad_input:
    Assert: Length() ≈ 7.828427;  ArcLength(2) ≈ 4.828427
            Assign of 1 point, of 9 points, or with a repeated point ⇒ false and the path is unchanged
project_reports_the_signed_cross_track:
    Assert: Project((1, 0.5), 0, 3) ≈ { s 1, point (1, 0), heading 0, cross +0.5, segment 0 }
            Project((3.5, 0.5), 0, 3) ≈ { s 3.414214, point (3, 1), heading π/4, cross −0.707107, segment 1 }
point_at_clamps_to_the_path:
    Assert: PointAt(5.5) ≈ (4, 2.671573);  HeadingAt(5.5) ≈ π/2;  PointAt(−1) == (0, 0);  PointAt(99) == (4, 5)
```

## Test cases — `TestPurePursuit`

```cpp
goal_on_the_current_segment:
    Assert: Compute(FromPose(0.5, −0.2, 0), 1) ⇒ goal ≈ (1.986607, 0), κ ≈ 0.177778   # L = 1.5
goal_on_the_next_segment:
    Assert: Compute(FromPose(1.8, 0, 0), 1) ⇒ goal ≈ (2.955936, 0.955936), κ ≈ 0.849721
lookahead_is_speed_scheduled_and_clamped:
    Assert: |goal − p| ≈ 3 for speed 10, and ≈ 1 for speed 0, from FromPose(0.5, 0, 0)
falls_back_to_a_path_point_when_far_off_the_path:
    Assert: Compute(FromPose(1, −3, 0), 0) ⇒ goal ≈ (2, 0), κ ≈ 0.6
end_of_path_targets_the_last_point_and_reports_reached:
    Assert: Compute(FromPose(4, 4.5, π/2), 1) ⇒ goal (4, 5), κ ≈ 0, reached false
            Compute(FromPose(4, 4.95, π/2), 1) ⇒ reached true
search_window_keeps_progress_on_the_current_leg:
    Arrange: PurePursuit on uTurn, window 2
    Assert:  Compute(FromPose(1, 0.6, 0), 0) ⇒ goal ≈ (1.8, 0), κ ≈ −1.2
             # an unbounded window projects onto the return leg (distance 0.4 < 0.6): goal (0.083485, 1), κ +0.8
closed_loop_unicycle_reaches_the_goal:
    Arrange: parameters { 0.6, 2.0, 0.5, 0.1, 2 }, start FromPose(0, −0.5, 0.3), v = 0.8, dt = 0.02
    Act:     pose = pose * Exp((v, 0, v·κ), dt) until reached, at most 600 steps
    Assert:  reached within 600 steps (double reference: 479), final position within 0.1 of (4, 5)
```

## Reference vectors

- Path `(0,0) → (2,0) → (4,2) → (4,5)`: arc lengths `0, 2, 4.828427, 7.828427`.
- From `(0.5, −0.2, 0)`, `L = 1.5`: goal `x = 0.5 + √2.21 = 1.986607`, body `(1.486607, 0.2)`,
  `κ = 2·0.2/1.5² = 0.177778`.
- From `(1.8, 0, 0)`: `2t² + 0.4t − 2.21 = 0` ⇒ `t = 0.955936` on segment 1.

## Edge cases

- Robot exactly on the last waypoint ⇒ `b = 0`, `κ = 0`, `reached`.
- `Restart()` after reaching the goal sets `Progress()` back to 0.
