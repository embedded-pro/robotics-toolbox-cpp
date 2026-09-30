# Coriolis Matrix and Inertial Regressor — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestCoriolisMatrix : public ::testing::Test:
    # 3-link chain: skewed unit axes, full SPD inertia tensors, off-axis CoM, arbitrary offsets
    std::array<RevoluteJointLink<float>, 3> chain{ ... }
    CoriolisMatrix<float, 3> coriolis
    ModifiedRecursiveNewtonEuler<float, 3> modified
    RecursiveNewtonEuler<float, 3> rnea
    CompositeRigidBodyAlgorithm<float, 3> crba
# each case below is a TEST_F(TestCoriolisMatrix, <name>)   (regressor cases in TestChainInertialRegressor)
```

## Test cases (Arrange / Act / Assert)

```
reduces_to_rnea_when_reference_equals_actual:
    Assert: modified.InverseDynamics(q, q̇, q̇, q̈, g) ≈ rnea.InverseDynamics(q, q̇, q̈, g)
coriolis_times_velocity_equals_rnea_bias:
    Assert: C(q, q̇)·q̇ ≈ rnea.InverseDynamics(q, q̇, 0, 0)
matches_christoffel_symbols_of_the_mass_matrix:
    Arrange: Γ_ijk = ½(∂M_ij/∂q_k + ∂M_ik/∂q_j − ∂M_jk/∂q_i) by central differences of crba.Compute
    Assert:  C_ij ≈ Σ_k Γ_ijk q̇_k
mass_matrix_derivative_minus_twice_coriolis_is_skew:
    Assert: N = Ṁ − 2C (Ṁ by finite difference along q̇) satisfies N + Nᵀ ≈ 0
two_link_planar_closed_form:
    Assert: C = [[−h q̇2, −h(q̇1 + q̇2)], [h q̇1, 0]], h = m2 l1 lc2 sin q2 (uniform rods, lc2 = l2/2)
regressor_times_parameters_reproduces_dynamics:
    Arrange: π = ChainInertialRegressor::Parameters(chain)
    Assert:  Y(q, q̇, q̇r, q̈r)·π ≈ modified.InverseDynamics(q, q̇, q̇r, q̈r, g)
regressor_is_independent_of_parameters:
    Arrange: second chain with different masses/inertias but the same geometry
    Assert:  Y is identical for both chains (only π differs)
```

## Reference vectors

- Planar 2R with uniform rods: `C` as above; `C q̇ = [−h(2q̇1q̇2 + q̇2²), h q̇1²]`.

## Edge cases

- `q̇ = 0` ⇒ `C = 0`.
- Single link ⇒ `C = 0` for any `q̇` (no velocity coupling about a fixed axis).
