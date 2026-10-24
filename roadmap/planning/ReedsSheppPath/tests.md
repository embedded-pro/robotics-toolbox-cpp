# Reeds–Shepp Paths — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```cpp
class TestReedsSheppPath : public ::testing::Test:
    ReedsSheppPlanner<float> planner{ 1 }
    SE2Transform<float> start{ Identity }
    # ExpectReaches(path, start, goal) helper: path.Sample(start, path.Length()) ≈ goal (tol 1e-4)
# each case below is a TEST_F(TestReedsSheppPath, <name>)
```

## Test cases (Arrange / Act / Assert)

```cpp
straight_reverse_is_a_single_backward_segment:
    Assert: goal FromPose(−3, 0, 0) ⇒ Length ≈ 3, one straight segment of −3, the rest ≈ 0; reaches the goal
forward_path_when_it_is_shortest:
    Assert: goal FromPose(3, 0.5, 0.2) ⇒ L 0.169722, S 2.842168, L 0.030278 (all forward);  Length ≈ 3.042168
sideways_shift_uses_cusps:
    Assert: goal FromPose(0, 1, 0) ⇒ Length ≈ 2.636232, contains forward and reversing segments, reaches the goal
ccsc_word_with_a_reversed_quarter_turn:
    Assert: goal FromPose(−2, 1, π/2) ⇒ L −0.729728, S −0.236068, R −π/2, L 0.729728;  Length ≈ 3.266320
ccc_word_with_a_cusp:
    Assert: goal FromPose(−1, −1, 0.5) ⇒ L 0.166243, R −0.965586, L −0.631829;  Length ≈ 1.763658
three_point_turn_reverses_heading_in_place:
    Assert: goal FromPose(0, 0, π) ⇒ Length ≈ π, reaches the goal
never_longer_than_dubins_symmetric_and_reaching:
    Arrange: ReedsSheppPlanner{ 1.5 } and DubinsPlanner{ 1.5 }, start FromPose(1, 2, π/4);
             goals x, y ∈ { −3, −1, 0.5, 2 }, θ ∈ { −2.5, −π/2, 0, 1, π }
    Assert:  for all 80 goals: RS Length ≤ Dubins Length + 1e-5;  Shortest(goal, start).Length ≈ Shortest(start, goal).Length;
             Sample(start, Length) ≈ goal
```

## Reference vectors

- `(0,0,0) → (0,1,0)`: Dubins needs `7.283185`, Reeds–Shepp `2.636232` (RLRL with two cusps).
- `(0,0,0) → (−2,1,π/2)`: LSRL `3.266320`, just ahead of the CCC alternative at `3.266920`.
- `(0,0,0) → (0,0,π)`: `3.141593` (three π/3 arcs: forward, reverse, forward).

## Edge cases

- `start == goal` ⇒ `Length = 0`.
- Ties between symmetric words (e.g. the sideways shift) may pick either word; tests assert only the
  length and the endpoint there.
