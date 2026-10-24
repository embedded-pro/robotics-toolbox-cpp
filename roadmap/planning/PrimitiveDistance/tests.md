# Primitive Distance Queries — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```cpp
class TestPrimitiveDistance : public ::testing::Test:
    Box<float> box{ { RotationAboutAxis(ẑ, π/6), (1, 0, 0) }, (0.5, 0.25, 0.1) }
# each case below is a TEST_F(TestPrimitiveDistance, <name>)
```

## Test cases (Arrange / Act / Assert)

```cpp
point_segment_projects_and_clamps:
    Assert: PointSegment((1, 1, 0), [(0,0,0), (2,0,0)]) ⇒ onSecond (1, 0, 0), distance 1
            PointSegment((−1, 1, 0), same) ⇒ onSecond (0, 0, 0), distance √2
segment_segment_matches_brute_force:
    Assert: [(0,0,0), (1,2,3)] vs [(2,0,1), (0,1,−1)] ⇒ s = 0.1, t = 0.6, points (0.1, 0.2, 0.3), (0.8, 0.6, −0.2),
            distance ≈ 0.948683                    # 801 × 801 sampling gives 0.948683
            skew [(0,0,0), (1,0,0)] vs [(0.5,−1,1), (0.5,1,1)] ⇒ (0.5, 0, 0), (0.5, 0, 1), distance 1
segment_segment_handles_parallel_and_degenerate_inputs:
    Assert: parallel [(0,0,0), (2,0,0)] vs [(1,1,0), (3,1,0)] ⇒ distance 1 (one valid pair, e.g. (1,0,0)–(1,1,0))
            endpoint case [(0,0,0), (1,0,0)] vs [(2,1,0), (3,2,0)] ⇒ s = 1, t = 0, distance √2
            point–point and point–segment (zero-length inputs) ⇒ √3 and 1
point_box_is_signed:
    Assert: PointBox((2, 1, 0), box) ⇒ onSecond (1.308013, 0.466506, 0), distance 0.873763
            PointBox((1.05, 0, 0.02), box) ⇒ onSecond (1.05, 0, 0.1), distance −0.08   # inside: nearest face z = +0.1
rounded_pairs_subtract_the_radii:
    Assert: SphereSphere(((0,0,0), 1), ((3,1,0), 0.5)) ⇒ distance √10 − 1.5 = 1.662278, onFirst (0.948683, 0.316228, 0)
            CapsuleCapsule(([(0,0,0),(1,0,0)], 0.2), ([(0.5,−1,1),(0.5,1,1)], 0.3)) ⇒ distance 0.5
            the same with the second axis at z = 0.4 ⇒ distance −0.1 (overlap)
            SphereBox(((2, 1, 0), 0.3), box) ⇒ distance 0.573763
sphere_inside_a_box_reports_depth:
    Assert: SphereBox(((1.05, 0, 0.02), 0.05), box) ⇒ distance −0.13
coincident_cores_still_return_a_normal:
    Assert: SphereSphere(((1,1,1), 0.2), ((1,1,1), 0.3)) ⇒ distance −0.5, points separated along +z
```

## Reference vectors

- Box `R = Rz(30°)`, centre `(1, 0, 0)`, half extents `(0.5, 0.25, 0.1)`; `(2, 1, 0)` is `0.873763` from its surface.
- The general segment pair agrees with brute-force sampling to `1e-7`.

## Edge cases

- Point exactly on a box face ⇒ distance 0 (inside branch with zero depth).
- Segment endpoint touching the other segment ⇒ distance 0, `s` or `t` at an end.
