# Dubins Paths — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```cpp
class TestDubinsPath : public ::testing::Test:
    DubinsPlanner<float> planner{ 1 }
    SE2Transform<float> start{ Identity }
    # ExpectPath(path, letters, lengths) and ExpectReaches(path, start, goal) helpers (tol 1e-4)
# each case below is a TEST_F(TestDubinsPath, <name>)
```

## Test cases (Arrange / Act / Assert)

```cpp
straight_ahead_is_a_single_segment:
    Assert: Shortest(start, FromPose(4, 0, 0)) ⇒ Length ≈ 4, both arcs ≈ 0, reaches the goal
csc_shortest_is_lsl:
    Assert: goal FromPose(4, 4, π/2) ⇒ L 0.785398, S 4.242641, L 0.785398;  Length ≈ 5.813437
csc_shortest_is_rsl:
    Assert: goal FromPose(3, −2, π/3) ⇒ R 1.379520, S 0.896575, L 2.426718;  Length ≈ 4.702813
ccc_shortest_is_rlr:
    Assert: goal FromPose(0.2, 0.6, −2.5) ⇒ R 0.752181, L 4.974858, R 0.439492;  Length ≈ 6.166531
turning_on_the_spot_needs_lrl:
    Assert: goal FromPose(0, 0, π/2) ⇒ L 0.424031, R 5.560451, L 0.424031;  Length ≈ 6.408513
single_arc_goal_survives_the_float_boundary:
    Assert: goal FromPose(1, 1, π/2) ⇒ Length ≈ π/2 and reaches the goal   # a strict p² ≥ 0 test gives 7.853982
mirroring_the_goal_swaps_left_and_right:
    Assert: goal FromPose(3, 2, −π/3) ⇒ L 1.379520, S 0.896575, R 2.426718
radius_scales_every_length:
    Assert: DubinsPlanner{ 2 }.Shortest(start, FromPose(8, 8, π/2)).Length() ≈ 11.626874
sample_walks_along_the_path:
    Arrange: path to FromPose(4, 4, π/2)
    Assert:  Sample(start, 0) ≈ start;  Sample(start, Length/2) ≈ (2.207107, 1.792893, π/4)
             Sample(start, Length) ≈ Sample(start, Length + 1) ≈ goal
every_shortest_path_reaches_its_goal:
    Arrange: DubinsPlanner{ 1.5 }, start FromPose(1, 2, π/4); goals x, y ∈ { −3, −1, 0.5, 2 }, θ ∈ { −2.5, −π/2, 0, 1, π }
    Assert:  Sample(start, Length) ≈ goal for all 80 goals
```

## Reference vectors

- `(0,0,0) → (4,4,π/2)`, `ρ = 1`: LSL with a 45° arc, `3√2` straight, 45° arc.
- `(0,0,0) → (0,0,π/2)`: LRL `0.424031 + 5.560451 + 0.424031 = 6.408513` (RLR 9.550106, LSL 9.268195).
- All six candidate words reach their goal to `3.4e-11` over 20 000 random double-precision cases; in
  `float` with `ε = 1e-5` the shortest length matches double to `1.1e-6`.

## Edge cases

- `start == goal` ⇒ `Length = 0` (every segment 0).
- `CarPath::Sample` with `s < 0` returns `start`.
