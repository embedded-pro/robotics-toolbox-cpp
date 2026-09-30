# Chain Dynamics Model — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```cpp
class TestChainDynamicsModel : public ::testing::Test:
    # 2-link uniform rods about ŷ, gravity (0, 0, −9.81), q measured from horizontal
    ChainDynamicsModel<float, 2> model{ MakeRodChain(1.2, 0.6, 0.7, 0.45), (0, 0, −9.81) }
# each case below is a TEST_F(TestChainDynamicsModel, <name>)
```

## Test cases (Arrange / Act / Assert)

```text
gravity_terms_match_closed_form:
    Assert: g(q) = −g·[m1l1/2·c1 + m2(l1c1 + l2/2·c12), m2l2/2·c12]
coriolis_terms_match_closed_form:
    Assert: C q̇ = [−h(2q̇1q̇2 + q̇2²), h q̇1²],  h = m2 l1 l2/2 · sin q2
mass_matrix_matches_crba:
    Assert: ComputeMassMatrix(q) == CompositeRigidBodyAlgorithm::Compute(links, q)
inverse_dynamics_equals_manipulator_equation:
    Assert: ComputeInverseDynamics(q, q̇, q̈) ≈ M q̈ + C q̇ + g
euler_lagrange_forward_dynamics_matches_articulated_body_algorithm:
    Assert: EulerLagrangeSolver::ForwardDynamics(model, q, q̇, τ) ≈ ABA(links, q, q̇, τ, gravity)
set_links_updates_every_term:
    Arrange: double the last link mass via SetLinks
    Assert:  g(q) changes by the closed-form payload contribution
```

## Reference vectors

- `m1=1.2, l1=0.6, m2=0.7, l2=0.45`, `q=(0.4,−0.9)`, `q̇=(1.1,−0.6)`, `q̈=(0.8,−1.7)` ⇒
  `τ = (−8.2064, −1.4410)` (analytic, matches RNEA).

## Edge cases

- Zero gravity ⇒ `g(q) = 0` for every `q`.
- `q̇ = 0` ⇒ `C q̇ = 0`.
