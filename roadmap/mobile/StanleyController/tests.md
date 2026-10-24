# Stanley Controller — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```cpp
class TestStanleyController : public ::testing::Test:
    PlanarPath<float, 4> road                                      # { (0, 0), (50, 0) }
    CarLikeKinematics<float> car{ { 2.5, 1.5, 0.6, 10 } }          # δ̇_max = 10 rad/s: rate limit inactive unless noted
    StanleyController<float, 4> stanley{ road, car, { 2.5, 1.0, 2 } }
# each case below is a TEST_F(TestStanleyController, <name>)
```

## Test cases (Arrange / Act / Assert)

```cpp
lateral_offset_steers_back_towards_the_path:
    Assert: Compute(FromPose(0, 0.5, 0), 5, 0, 1) ≈ −0.205395     # −atan(1.25 / 6)
heading_error_adds_to_the_correction:
    Assert: Compute(FromPose(0, 0, 0.2), 5, 0, 1) ≈ −0.404067
error_is_measured_at_the_front_axle:
    Assert: Error(FromPose(0, 0, 0.2)) ≈ { crossTrack 0.496673, headingError −0.2 }   # front axle (2.450166, 0.496673)
output_saturates_at_max_steer:
    Assert: Compute(FromPose(0, −3, 0), 5, 0, 1) == 0.6
steering_rate_is_limited:
    Arrange: car with δ̇_max = 0.5
    Assert:  Compute(FromPose(0, 0.5, 0), 5, 0, 0.1) ≈ −0.05
closed_loop_converges:
    Arrange: rear-axle pose FromPose(0, 1, 0), v = 5, dt = 0.01, δ_prev = 0
    Act:     400 × { δ = Compute(pose, v, δ_prev, dt); pose = pose * Exp(car.Forward(v, δ), dt); δ_prev = δ }
    Assert:  |Error(pose).crossTrack| < 1e-3     # double reference: 3.0e-4
```

## Reference vectors

- `k = 2.5`, `k_s = 1`, `v = 5`: offset `0.5` ⇒ `δ = −atan2(1.25, 6) = −0.205395`.
- Heading `0.2`: front axle at `(2.450166, 0.496673)` ⇒ `δ = −0.2 − atan2(1.241683, 6) = −0.404067`.

## Edge cases

- `v = 0` stays finite thanks to `k_s`: `Compute(FromPose(0, 0.5, 0), 0, 0, 1) = −atan(1.25) = −0.896055`
  before the ±0.6 clamp.
- Vehicle exactly on the path with zero heading error ⇒ steer 0.
