# RRT-Connect — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```cpp
class TestRrtConnect : public ::testing::Test:
    # 2-DOF configuration space [−π, π]², blocked region (wall): |q₀| ≤ 0.3 and q₁ ≤ 1.0 (open above q₁ = 1)
    IsFree(q) ⇒ not (|q₀| ≤ 0.3 and q₁ ≤ 1.0)
    XorShift32 generator(seed) (x ^= x << 13; x ^= x >> 17; x ^= x << 5);  Uniform() ⇒ next()/2³²
    Sample() ⇒ (−π + 2π·Uniform(), −π + 2π·Uniform())
    RrtConnect<float, 2, 256> planner{ { 0.3, 0.05, 500 } }
    # PathIsValid(path): first == start, last == goal, every edge free when checked every 0.05 rad
# each case below is a TEST_F(TestRrtConnect, <name>)
```

## Test cases (Arrange / Act / Assert)

```cpp
finds_a_valid_path_around_the_wall:
    Act:    Plan((−2, −2), (2, −2), Sample with seed 1, IsFree)
    Assert: found;  PathIsValid;  path length ≥ 7.496376 (geodesic over the wall top);  Iterations() ≤ 500
            # double reference: seed 1 ⇒ 78 iterations, 70 nodes, 40 waypoints, length 11.53
results_are_reproducible_for_a_seed:
    Assert: two runs with seed 7 return identical paths
shortcut_shortens_and_stays_valid:
    Act:    Plan as above (seed 1); Shortcut(100, Uniform with seed 99, IsFree)
    Assert: PathIsValid;  length decreases (double reference: 11.53 ⇒ 8.12, 4 waypoints);  length ≥ 7.496376
trivial_case_returns_the_straight_edge:
    Assert: Plan((−2, 2), (2, 2), …) == found, Path() == [(−2, 2), (2, 2)], Iterations() == 0
invalid_endpoints_are_reported:
    Assert: Plan((0, 0), (2, 2), …) == startInCollision;  Plan((2, 2), (0, 0), …) == goalInCollision
small_pool_is_exhausted:
    Arrange: RrtConnect<float, 2, 10>
    Assert:  Plan((−2, −2), (2, −2), Sample with seed 1, IsFree) == poolExhausted
```

## Reference vectors

- Seeds 1, 7 and 12345 (double): found after 78, 104 and 95 iterations; raw lengths 11.53, 9.83, 11.83;
  after 100 shortcut attempts 8.12, 7.84, 8.05 against the 7.496 lower bound.
- A base-2 Halton sampler consumed in alternation finds nothing in 2000 iterations (19 nodes): documents the
  sampler-independence rule (not a test).

## Edge cases

- `maxIterations = 0` with a blocked straight edge ⇒ `iterationLimit`.
- `Shortcut` on a two-waypoint path changes nothing.
