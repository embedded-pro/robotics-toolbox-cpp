# Monte Carlo Localization — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```cpp
class TestMonteCarloLocalization : public ::testing::Test:
    # 4 m × 4 m room, 40 × 40 cells of 0.1 m, origin (0, 0); occupied cells set with SetLogOdds(c, 3.5):
    #   the border (x or y index 0 or 39), an inner wall x = 20 for y = 0..14, a shelf y = 28 for x = 25..39
    OccupancyGridMap<float, 40, 40> room
    LikelihoodField<float, 40, 40> field{ room, 0, 2.0 }
    MclParameters<float> parameters{ { 0.05, 0.01, 0.05, 0.01 }, 0.1, 0.9, 0.1, 4.0, 0.5, 1 }
    SE2Transform<float> truth{ FromPose(1.0, 1.5, 0.2) }
    # Scan(): 12 beams at bearings 0°, 30°, …, 330°, ray-cast in the test from the truth against `room` (5 mm steps, 4 m cap)
    # Lattice(i): the 27 poses truth ⊕ offsets dx, dy ∈ {−0.2, 0, 0.2}, dθ ∈ {−0.1, 0, 0.1}
# each case below is a TEST_F(TestMonteCarloLocalization, <name>)
```

## Test cases (Arrange / Act / Assert)

```cpp
likelihood_field_follows_the_chamfer_metric:
    Arrange: a 40 × 40 map with only cell (20, 20) occupied
    Assert:  distance at cells (20, 20) = 0, (25, 20) = 0.5, (24, 24) ≈ 0.565685, (23, 24) ≈ 0.524264 (Euclidean 0.5)
             Distance(world point outside the map) == 2.0
predict_without_noise_applies_the_increment:
    Arrange: MonteCarloLocalization<float, 4, 40, 40> with every particle at FromPose(1, 2, 0.3); normal() ⇒ 0
    Act:     Predict(FromPose(0.5, 0, 0.1), normal)
    Assert:  every particle ≈ (1.477668, 2.147760, 0.4)
predict_noise_scales_with_the_motion:
    Arrange: same, normal() ⇒ +1
    Act:     Predict(FromPose(0.5, 0, 0.1), normal)
    Assert:  every particle ≈ FromPose(1, 2, 0.3) * FromPose(0.5 + 0.112250, 0.112250, 0.1 + 0.054772)
             = (1.551732, 2.288168, 0.454772)
scan_update_favours_the_true_pose:
    Arrange: MonteCarloLocalization<float, 27, 40, 40>, Initialize(Lattice), resampleFraction = 0 (never resample)
    Act:     Update(Scan(), uniform ⇒ 0.5)
    Assert:  returns false;  MaximumWeight() == truth with weight ≈ 0.995028;  Estimate() ≈ (1.0, 1.5, 0.200381)
resampling_concentrates_on_the_true_pose:
    Arrange: as above with resampleFraction = 0.5
    Act:     Update(Scan(), uniform ⇒ 0.5)
    Assert:  returns true (ESS ≈ 1.01 < 13.5);  every particle == truth;  Estimate() ≈ (1.0, 1.5, 0.2)
max_range_beams_are_ignored:
    Arrange: Initialize(Lattice)
    Act:     Update(12 beams of range 4.0, uniform ⇒ 0.5)
    Assert:  all weights still 1/27
```

## Reference vectors

- `Scan()` ranges from `(1.0, 1.5, 0.2)`: `2.960, 2.005, 2.535, 2.450, 1.360, 0.950, 0.920, 1.205, 1.480, 1.430,
  1.515, 1.055` m at 0°…330°.
- Log-likelihood at the truth `−0.935538`; 0.2 m off in x, `−15.590720`.
- Noise for `Δ = (0.5, 0, 0.1)`: `σ_xy = 0.112250`, `σ_θ = 0.054772`.

## Edge cases

- A beam ending outside the map scores with `maxDistance` (does not crash, does not favour leaving the map).
- `beamStride = 3` uses beams 0, 3, 6, 9 only.
