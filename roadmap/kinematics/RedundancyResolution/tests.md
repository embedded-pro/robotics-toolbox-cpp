# Redundancy Resolution — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

Projector algebra (idempotence, symmetry, right inverse, minimum norm, damping bound) is tested upstream
with `solvers::DampedPseudoInverse` (NT62). These cases cover the binding to a real redundant arm.

## Fixture

```cpp
class TestRedundancyResolution : public ::testing::Test:
    # 7R arm (iiwa-like), standard DH (a, α, d): (0,−π/2,0.34) (0,π/2,0) (0,π/2,0.4) (0,−π/2,0)
    #                                           (0,−π/2,0.4) (0,π/2,0) (0,0,0.126)
    DenavitHartenberg<float, 7>  arm{ iiwaTable, Standard }
    DhTaskJacobian<float, 6, 7>  pose{ arm }
    DhTaskJacobian<float, 3, 7>  position{ arm }
    JointVector q   { 0.1, 0.5, −0.3, −1.2, 0.4, 0.8, −0.2 }
    TaskVector  xDot{ 0.1, −0.05, 0.02, 0.1, 0.2, −0.1 }
    JointVector qDot0{ 1, −1, 0.5, 0.3, −0.2, 0.7, −0.4 }
# each case below is a TEST_F(TestRedundancyResolution, <name>)
```

## Test cases (Arrange / Act / Assert)

```cpp
undamped_primary_task_is_exact:
    Arrange: RedundancyResolution<float, 6, 7>{ pose, λ = 0 }
    Assert:  pose.Jacobian(q)·Resolve(q, xDot, qDot0) ≈ xDot  (tol 1e-5)
damped_resolution_matches_reference:
    Arrange: λ = 0.01
    Assert:  Resolve(q, xDot, qDot0) ≈ (−0.093934, 0.325324, −0.092001, 0.551001, 0.088089, 0.448837, 0.140035)
null_space_motion_is_task_invisible_despite_damping:
    Arrange: λ = 0.01
    Assert:  J·NullSpaceProjector(q)·qDot0 ≈ 0  (tol 1e-5)
secondary_objective_reduces_cost_without_moving_the_tool:
    Arrange: H(q) = ½Σ(qᵢ/2.9)²; qDot0 = −∇H; xDot = 0; λ = 0
    Act:     q̇ = Resolve(q, 0, qDot0)
    Assert:  ∇H·q̇ ≈ −0.0051809 (< 0); ‖J·q̇‖ ≈ 0
position_task_has_four_dimensional_null_space:
    Arrange: RedundancyResolution<float, 3, 7>{ position, λ = 0 }
    Assert:  trace NullSpaceProjector(q) ≈ 4; J_pos·P ≈ 0
rank_loss_grows_the_null_space:
    Arrange: stretched q_s = (0.1, 0, −0.3, 0, 0.4, 0, −0.2) (σ = 2, 1.950561, 0.498565, 0.311084, 0, 0)
    Assert:  trace P ≈ 3; J(q_s)·P ≈ 0; Resolve finite with λ = 0.01
```

## Reference vectors

- 7R fixture at `q`: `σ(J) = (1.851950, 1.718255, 1.317197, 0.446306, 0.267222, 0.178251)`.
- `λ = 0`: `Resolve(q, xDot, qDot0) = (−0.094193, 0.326285, −0.092053, 0.552967, 0.088112, 0.449943, 0.140364)`.
- `λ = 0.01`: `Resolve(q, xDot, qDot0) = (−0.093934, 0.325324, −0.092001, 0.551001, 0.088089, 0.448837, 0.140035)`,
  `‖Jq̇ − ẋ‖ = 4.44e-4`, `‖J·P·q̇₀‖` at rounding level (python3/numpy, standard DH).
- Position rows only: `σ = (0.839288, 0.805624, 0.257020)`, `trace P = 4`.

## Edge cases

- `Dof = TaskDim` rejected at compile time (`static_assert(Dof > TaskDim)`); use M13 instead.
- Conflicting or over-scaled `q̇₀` ⇒ primary task unaffected (projector exact); only the secondary objective suffers.
