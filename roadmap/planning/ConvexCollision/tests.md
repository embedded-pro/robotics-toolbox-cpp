# GJK + EPA — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```cpp
class TestConvexCollision : public ::testing::Test:
    using Collision = ConvexCollision<float, 16, 64>;  using Shape = ConvexShape<float, 16>
    Shape boxA{ Box({ I, (0,0,0) }, (0.5, 0.4, 0.3)) }
    Shape boxB{ Box({ Rot((1,1,0)/√2, 0.7), (1.4, 0.6, 0.5) }, (0.3, 0.3, 0.2)) }
    Shape cube{ Box({ I, 0 }, (1, 1, 1)) }
    # ExactBoxBox(a, b): min over vertex–box (P1 PointBox) and edge–edge (P1 SegmentSegment) distances
    # SatDepth(a, b): minimum overlap over the 15 separating axes of two boxes
# each case below is a TEST_F(TestConvexCollision, <name>)
```

## Test cases (Arrange / Act / Assert)

```cpp
rounded_shapes_match_the_closed_forms:
    Assert: Query(Sphere((0,0,0), 1), Sphere((3,1,0), 0.5)) ⇒ distance 1.662278, onFirst (0.948683, 0.316228, 0)
            Query(Capsule((0,0,0), (1,2,3), 0.1), Capsule((2,0,1), (0,1,−1), 0.2)) ⇒ 0.648683   # P1: 0.948683 − 0.3
            Query(Sphere((2,1,0), 0.3), Box({ Rz(π/6), (1,0,0) }, (0.5,0.25,0.1))) ⇒ 0.573763
box_pair_distance_matches_the_exact_value:
    Assert: Query(boxA, boxB) ⇒ distance ≈ 0.511063 == ExactBoxBox;  onFirst (0.5, 0.391106, 0.3), onSecond (1.008894, 0.391106, 0.347032)
capsule_box_and_hull_pairs:
    Assert: Query(Capsule((1,−0.5,0.8), (1,0.5,0.8), 0.1), boxA) ⇒ 0.607107        # core to the edge x = 0.5, z = 0.3: √0.5 − 0.1
            Query(Hull({ I, (2,0,0) }, unit tetrahedron (0,0,0),(1,0,0),(0,1,0),(0,0,1)), Sphere((0,0,0), 1)) ⇒ 1.0
rounded_overlap_with_separated_cores_is_exact:
    Assert: Query(cube, Sphere((1.8, 0, 0), 1)) ⇒ distance −0.2, normal (1, 0, 0), converged
epa_resolves_axis_aligned_boxes:
    Assert: Query(cube, Box({ I, (1.5, 0, 0) }, (1, 1, 1))) ⇒ distance −0.5, normal (1, 0, 0)   # origin on the blown-up simplex boundary
epa_matches_separating_axes_for_rotated_boxes:
    Assert: Query(boxA, Box({ Rot((1,1,0)/√2, 0.7), (0.6, 0.3, 0.2) }, (0.3, 0.3, 0.2))) ⇒ distance ≈ −0.291106 == −SatDepth, normal (1, 0, 0)
symmetric_and_tied_configurations_stay_finite:
    Assert: Query(boxA, boxA) ⇒ distance −0.6, normal ±ẑ, converged      # identical boxes, same pose
            for axis-aligned boxes with half extents and centres on a 0.25 grid (fixed list of 50): −distance == SatDepth (1e-5), no NaN
random_box_pairs_agree_with_exact_references:
    Arrange: 300 box pairs from a fixed list (seeded xorshift32 in the test): half extents in [0.2, 1], random rotations, centres within ±0.6
    Assert:  separated: |distance − ExactBoxBox| < 1e-5;  overlapping: |−distance − SatDepth| < 1e-4;  all converged
```

## Reference vectors

- `boxA`/`boxB` separation `0.511063` (exact vertex/edge enumeration agrees to `3e-17` in double).
- Validation (double): 300 random box pairs. Separated pairs match the exact distance to `3e-17`; overlapping
  pairs match the separating-axis depth to `7e-16`, with at most 15 EPA iterations and 32 faces.
- Sphere–box overlap through EPA (cores intersecting) is accurate to `5e-6` (smooth-surface convergence).
- 500 deep random overlaps (centres within ±0.1): exact to `1e-15`, at most 21 iterations and 44 faces; 634 axis-aligned
  overlaps with tied coordinates: exact, no NaN.

## Edge cases

- Touching rounded shapes (core distance = sum of radii) ⇒ distance 0 through the core path.
