# Dynamic Parameter Identification — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestDynamicParameterIdentification : public ::testing::Test:
    # planar 2R uniform rods; "true" links generate noiseless torques with RNEA
    ChainInertialRegressor<float, 2> regressor{ links, gravity }
    DynamicParameterIdentification<float, 2> identification{ regressor }
    # excitation: 200 samples of q = Σ Fourier terms, q̇, q̈ analytic
# each case below is a TEST_F(TestDynamicParameterIdentification, <name>)
```

## Test cases (Arrange / Act / Assert)

```
base_parameter_count_with_gravity_is_six:
    Arrange: joint axes horizontal (gravity acts)
    Assert:  Solve().rank == 6
base_parameter_count_without_gravity_is_four:
    Arrange: joint axes parallel to gravity
    Assert:  Solve().rank == 4
noiseless_data_reproduces_torques:
    Assert: Predict(result, q, q̇, q̈) ≈ τ on held-out samples (≈ 1e-3 relative)
identifiable_combinations_match_the_true_parameters:
    Assert: baseDirections·π̂ ≈ baseDirections·π_true
streaming_is_order_independent:
    Assert: shuffling the sample order gives the same result
no_samples_returns_nullopt:
    Assert: Solve(tol) == std::nullopt after Reset()
```

## Reference vectors

- Planar 2R: 6 base parameters with gravity in the plane of motion, 4 without (e.g. `ZZ1R, ZZ2,
  MX2, MY2` plus `MX1R, MY1R` when gravity acts) — rank checked numerically.

## Edge cases

- Non-exciting data (constant pose) ⇒ rank drops; residual still finite.
- Additive torque noise ⇒ bounded bias; RMS residual ≈ noise level.
