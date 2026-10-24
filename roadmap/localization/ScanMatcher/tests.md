# Scan Matching — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```cpp
class TestScanMatcher : public ::testing::Test:
    # reference: 68 points on three walls, in scan order:
    #   x = 2 for y = −1.0 … 1.5 (26 points), y = 1.5 for x = −0.5 … 2.0 (26), y = −1 for x = −0.5 … 1.0 (16)
    std::array<Vector2<float>, 68> reference
    SE2Transform<float> truth{ FromPose(0.1, −0.05, 0.05) }               # current → reference
    # sameSpots:  truth.Inverse() applied to every reference point (exact correspondences exist)
    # halfSpots:  wall points at half-spacing offsets (y = −0.95 … 1.45, x = −0.45 … 1.95, x = −0.45 … 0.95;
    #             65 points) mapped by truth.Inverse()
    ScanMatcherParameters<float> parameters{ pointToLine, 0.5, 50, 1e-6, 1e-6, 3, 3 }
# each case below is a TEST_F(TestScanMatcher, <name>)
```

## Test cases (Arrange / Act / Assert)

```cpp
both_metrics_recover_the_transform_with_exact_correspondences:
    Act:    Match(reference, sameSpots, Identity) with each metric
    Assert: transform ≈ truth (1e-4);  correspondences == 68;  converged;  rms ≈ 0
            iterations: point-to-point 3, point-to-line 2
point_to_line_removes_the_sampling_bias:
    Act:    Match(reference, halfSpots, Identity) with each metric
    Assert: point-to-line: transform ≈ truth (1e-4), 65 correspondences
            point-to-point: max |error| of (x, y, θ) > 0.01            # double reference: 0.034
point_to_line_converges_from_an_offset_guess:
    Act:    Match(reference, sameSpots, FromPose(−0.05, 0.05, −0.05))
    Assert: transform ≈ truth (1e-4)
far_outlier_is_rejected:
    Arrange: halfSpots plus the point (0.6, 0.2)                       # 1.2 m from every wall
    Act:     Match(reference, that, Identity)
    Assert:  correspondences == 65 (of 66);  transform ≈ truth (1e-4)
too_few_correspondences_fail:
    Arrange: maxCorrespondenceDistance = 0.01
    Assert:  Match(reference, halfSpots, Identity) == nullopt
corridor_is_reported_as_degenerate:
    Arrange: reference and current both on the two walls y = ±1 (x = −1 … 1, 21 points each), current shifted by (0.1, 0)
    Assert:  point-to-line Match(...) == nullopt               # n_x = 0 for every pair: JᵀJ has a zero eigenvalue
```

## Reference vectors

- `truth = (0.1, −0.05, 0.05)`.
- Half-spacing point-to-point fixed point: `(0.065830, −0.062154, 0.043042)`, an error of 0.034.
- From the offset guess `(−0.05, 0.05, −0.05)` with `sameSpots`, point-to-point stalls at
  `(0.062305, −0.054129, 0.016504)` while point-to-line reaches `truth` in 2 iterations.

## Edge cases

- `current` larger than `MaxPoints` ⇒ `nullopt`.
- `maxIterations` reached without meeting the tolerances ⇒ `converged == false` with the last transform.
