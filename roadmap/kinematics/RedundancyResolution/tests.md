# Redundancy Resolution — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

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
damped_primary_task_error_is_bounded:
    Arrange: λ = 0.01
    Assert:  ‖J·Resolve(q, xDot, qDot0) − xDot‖ ≤ λ²/σ_min²·‖xDot‖ = 8.5e-4  (observed 4.4e-4)
null_space_motion_is_task_invisible_despite_damping:
    Arrange: λ = 0.01
    Assert:  J·NullSpaceProjector(q)·qDot0 ≈ 0  (tol 1e-5)
projector_is_an_orthogonal_projector:
    Assert: P·P ≈ P, Pᵀ ≈ P (tol 1e-5), trace P ≈ 1 (= Dof − rank)
undamped_pseudo_inverse_is_a_right_inverse:
    Assert: J·DampedPseudoInverse(q)|_{λ=0} ≈ I₆  (tol 1e-5)
primary_term_is_minimum_norm:
    Assert: P·Resolve(q, xDot, 0) ≈ 0  (no null-space component)
secondary_objective_reduces_cost_without_moving_the_tool:
    Arrange: H(q) = ½Σ(qᵢ/2.9)²; qDot0 = −∇H; xDot = 0; λ = 0
    Act:     q̇ = Resolve(q, 0, qDot0)
    Assert:  ∇H·q̇ ≈ −0.0051809 (< 0); ‖J·q̇‖ ≈ 0
position_task_has_four_dimensional_null_space:
    Arrange: RedundancyResolution<float, 3, 7>{ position, λ = 0 }
    Assert:  trace NullSpaceProjector(q) ≈ 4; J_pos·P ≈ 0
rank_loss_grows_the_null_space:
    Arrange: stretched q_s = (0.1, 0, −0.3, 0, 0.4, 0, −0.2) (σ = 2, 1.950561, 0.498565, 0.311084, 0, 0)
    Assert:  trace P ≈ 3; P·P ≈ P; J(q_s)·P ≈ 0; Resolve finite with λ = 0.01
zero_inputs_give_zero_motion:
    Assert: Resolve(q, 0, 0) == 0
```

## Reference vectors

- 7R fixture at `q`: `σ(J) = (1.851950, 1.718255, 1.317197, 0.446306, 0.267222, 0.178251)`.
- `λ = 0`: `q̇ = (−0.094193, 0.326285, −0.092053, 0.552967, 0.088112, 0.449943, 0.140364)`.
- `λ = 0.01`: `q̇ = (−0.093934, 0.325324, −0.092001, 0.551001, 0.088089, 0.448837, 0.140035)`,
  `‖Jq̇ − ẋ‖ = 4.44e-4`, `‖J·P·q̇₀‖` at rounding level. The old damped projector `I − J⁺_λJ` would give
  `‖J·P_λ·q̇₀‖ = 2.85e-4` and `‖P_λ² − P_λ‖ = 1.98e-3` — do not use it.
- Position rows only: `σ = (0.839288, 0.805624, 0.257020)`, `trace P = 4`.

## Edge cases

- `Dof = TaskDim` rejected at compile time (`static_assert(Dof > TaskDim)`); use M13 instead.
- Conflicting or over-scaled `q̇₀` ⇒ primary task unaffected (projector exact); only the secondary objective suffers.
- Exact singularity with `λ = 0` ⇒ zero gain on lost directions (no blow-up), task partially met.
