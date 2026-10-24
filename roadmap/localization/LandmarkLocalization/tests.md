# EKF Landmark Localization — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```cpp
class TestLandmarkLocalization : public ::testing::Test:
    std::array<Landmark<float>, 4> landmarks{ { (5, 0), 0 }, { (0, 5), 1 }, { (5, 5), 2 }, { (−3, 2), 3 } }
    LandmarkLocalizationNoise<float> noise{ { 0.05, 0.01, 0.05, 0.01 }, 0.1, 0.02, 9.21 }
    LandmarkLocalization<float, 8> filter{ FromPose(1, 1, 0.3), diag(0.05, 0.05, 0.01), noise }   # SetMap(landmarks)
    Vector3 truth{ 1.1, 0.9, 0.32 }
    # Measure(truth, k) = exact range and bearing of landmark k from the true pose
# each case below is a TEST_F(TestLandmarkLocalization, <name>)
```

## Test cases (Arrange / Act / Assert)

```cpp
predict_composes_the_increment_and_grows_the_covariance:
    Act:    Predict(FromPose(0.5, 0, 0.1))
    Assert: Pose().Pose() ≈ (1.477668, 1.147760, 0.4);  diag(P) ≈ (0.062818, 0.064882, 0.013)
            P(0,1) ≈ −0.000706;  P(1,2) ≈ 0.004777
reverse_motion_has_the_same_noise_as_forward:
    Arrange: filter at Identity with P = 0
    Act:     Predict(FromPose(−0.5, 0, 0))
    Assert:  Pose().Pose() ≈ (−0.5, 0, 0);  diag(P) ≈ (0.0125, 0.0125, 0.0025)
unlabelled_observation_is_associated_by_mahalanobis_distance:
    Act:    Update({ Measure(truth, 2), nullopt })
    Assert: landmark == 2;  status applied, nis ≈ 0.002138;  Pose().Pose() ≈ (1.001568, 0.996349, 0.295825)
            # NIS against landmarks 0, 1, 3: 119.6, 118.1, 371.8
labelled_observations_converge_towards_the_truth:
    Act:    Update({ Measure(truth, 2), 2 }), then { 0 }, { 1 }, { 3 }
    Assert: Pose().Pose() ≈ (1.092522, 0.904252, 0.318948);  diag(P) ≈ (0.002722, 0.002077, 0.000122)
outlier_is_gated:
    Act:    Update({ (0.5, 2.5), nullopt })
    Assert: status gated (best nis ≈ 219.48 against landmark 3);  pose and covariance unchanged
bearing_residual_is_wrapped_across_pi:
    Arrange: filter at FromPose(6, 4.9, 0), P = diag(0.05, 0.05, 0.01); landmark 1 at bearing +3.124928
    Act:     Update({ (6.0, −π + 0.01), 1 })
    Assert:  status applied, nis ≈ 0.060327;  Pose().Pose() ≈ (5.999620, 4.918856, −0.022620)
```

## Reference vectors

- From `(1, 1, 0.3)`: landmark 2 predicted at `(5.656854, 0.485398)`; measured from the truth at
  `(5.658622, 0.490393)`. The Jacobian is `[[−0.707107, −0.707107, 0], [0.125, −0.125, −1]]`.
- The wrapped bearing residual is `0.026665`; unwrapped it would be `−6.256520`.

## Edge cases

- Empty map ⇒ `Update` returns `{ singular, nullopt }`.
- An `id` that is not in the map ⇒ `{ singular, nullopt }`, state unchanged.
