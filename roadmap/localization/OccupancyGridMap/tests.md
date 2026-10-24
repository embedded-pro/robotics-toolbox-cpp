# Occupancy-Grid Mapping — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```cpp
class TestOccupancyGridMap : public ::testing::Test:
    OccupancyGridParameters<float> parameters{ 0.1, (0, 0), 0.85, −0.4, −2.0, 3.5, 4.0 }
    OccupancyGridMap<float, 10, 10> map{ parameters }          # 1 m × 1 m
    SE2Transform<float> sensor{ FromPose(0.05, 0.05, 0) }      # centre of cell (0, 0)
    # NonZeroCells() lists (x, y, logOdds) of every touched cell
# each case below is a TEST_F(TestOccupancyGridMap, <name>)
```

## Test cases (Arrange / Act / Assert)

```cpp
axis_beam_clears_the_path_and_marks_the_hit:
    Act:    Integrate(sensor, { (0.5, 0) })
    Assert: cells (0..4, 0) = −0.4;  cell (5, 0) = 0.85;  nothing else touched
diagonal_and_shallow_beams_follow_bresenham:
    Act:    Integrate(sensor, { (0.5√2, π/4) }) on one map, Integrate(sensor, { (hypot(0.7, 0.3), atan2(0.3, 0.7)) }) on another
    Assert: diagonal: (0,0), (1,1), (2,2), (3,3), (4,4) free, (5,5) occupied
            shallow: (0,0), (1,0), (2,1), (3,1), (4,2), (5,2), (6,3) free, (7,3) occupied
max_range_reading_only_clears:
    Arrange: parameters.maxRange = 0.5
    Act:     Integrate(sensor, { (0.5, 0) })
    Assert:  cells (0..5, 0) = −0.4;  no occupied cell
ray_leaving_the_map_stops_at_the_border:
    Act:    Integrate(sensor, { (2.0, 0) })
    Assert: cells (0..9, 0) = −0.4;  no occupied cell, no write outside the grid
repeated_hits_clamp_the_log_odds:
    Act:    10 × Integrate(sensor, { (0.5, 0) })
    Assert: LogOdds((5, 0)) == 3.5;  LogOdds((0, 0)) == −2.0
            Probability((5, 0)) ≈ 0.970688;  Probability((0, 0)) ≈ 0.119203
world_to_cell_respects_the_origin:
    Arrange: parameters.origin = (−0.5, −0.5)
    Assert:  WorldToCell((0, 0)) == (5, 5);  WorldToCell((−0.5, −0.5)) == (0, 0);  WorldToCell((0.449, −0.051)) == (9, 4)
            CellCenter((5, 5)) ≈ (0.05, 0.05)
```

## Reference vectors

- `l = 0.85` ⇔ `p = 0.700567`; `l = −0.4` ⇔ `p = 0.401312`; clamp `[−2, 3.5]` ⇔ `p ∈ [0.119203, 0.970688]`.
- Bresenham `(0,0) → (7,3)`: `(0,0) (1,0) (2,1) (3,1) (4,2) (5,2) (6,3) (7,3)`, the same cells in reverse.

## Edge cases

- A beam of range 0 marks only the sensor cell (occupied if `0 < maxRange`, which the caller should
  filter as an invalid reading).
- Negative-direction beams (bearing π) walk with `sx = −1` and stop at column 0.
