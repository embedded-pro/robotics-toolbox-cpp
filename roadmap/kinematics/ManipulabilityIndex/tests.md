# Manipulability Index — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestManipulabilityIndex : public ::testing::Test:
    # 2-link unit planar arm (z-axis joints, link lengths 1), link model M30
    std::array<RevoluteJointLink<float>, 2>  planar{ ... }
    ChainTaskJacobian<float, 3, 2>           position{ planar, SE3Transform{ I, (1, 0, 0) } }
    ChainTaskJacobian<float, 6, 2>           full{ planar, SE3Transform{ I, (1, 0, 0) } }
    ManipulabilityIndex<float, 3, 2>         w{ position }
    StrictMock<MockJacobianProvider<float, 2, 3>>  planarTask       # 3-link planar arm, xy rows
    ManipulabilityIndex<float, 2, 3>         wRedundant{ planarTask }
# each case below is a TEST_F(TestManipulabilityIndex, <name>)
```

## Test cases (Arrange / Act / Assert)

```
position_manipulability_is_abs_sin_elbow:
    Assert: w.Compute({0, q₂}) ≈ |sin q₂| for q₂ ∈ {0.3, 1.2, 2.5}  (Dof < Rows ⇒ Jᵀ·J branch)
right_angle_elbow_is_most_dexterous:
    Assert: w.Compute({0, π/2}) ≈ 1 and > w.Compute({0, π/2 ± 0.1})
stretched_and_folded_arm_are_singular:
    Assert: w.Compute({0, 0}) ≈ 0 and w.Compute({0, π}) ≈ 0
redundant_task_uses_j_jt_branch:
    Arrange: EXPECT_CALL(planarTask, Jacobian) returns [[−1, −1, 0], [0, −1, −1]]   # q = (0, π/2, π/2)
    Assert:  wRedundant.Compute(q) ≈ √3 = 1.732051;  EllipsoidAxes ≈ (1.732051, 1)
six_row_task_measures_units_separately:
    Arrange: ManipulabilityIndex<float, 6, 2>{ full }
    Assert:  Compute({0, 1.2}) ≈ 0.932039 (= |sin 1.2|);  ComputeRotational({0, 1.2}) ≈ 0 (parallel axes)
ellipsoid_axes_are_sorted_singular_values:
    Assert: w.EllipsoidAxes({0, π/2}) ≈ (1.618034, 0.618034); product ≈ w.Compute({0, π/2})
condition_number_grows_near_singularity:
    Assert: ConditionNumber({0, π/2}) ≈ 2.618034; ({0, 0.1}) ≈ 49.963; ({0, 0.01}) ≈ 499.996 (rel tol 1e-3)
exactly_singular_condition_number_is_infinite:
    Assert: ConditionNumber({0, 0}) == +∞
near_singular_flag_trips:
    Assert: NearSingular({0, 0.01}, 0.05) == true;  NearSingular({0, π/2}, 0.05) == false
```

## Reference vectors

- 2-link unit arm, position rows: `w(q₂) = |sin q₂|` — `0.295520` (0.3), `0.932039` (1.2), `1` (π/2),
  `0.598472` (2.5), `0` at `{0, π}`. The full 6-row `√det(J·Jᵀ)` is `0` for every `q` (rank ≤ 2).
- `q = (0, π/2)`: `σ = (φ, 1/φ) = (1.618034, 0.618034)`, `κ = φ² = 2.618034`.
- 3-link unit planar arm, `q = (0, π/2, π/2)`: `J_xy = [[−1,−1,0],[0,−1,−1]]`, `J_xy·J_xyᵀ = [[2,1],[1,2]]`
  ⇒ `w = √3`, `σ = (√3, 1)`.

## Edge cases

- `Rows = Dof` ⇒ `w = |det B|`.
- Planar arm with `TaskDim = 3`, `Dof ≥ 3` ⇒ zero `z` row, `w ≡ 0` (documented, use `TaskDim = 2`).
- Tiny `w` below `float` epsilon ⇒ `NearSingular` still deterministic; `Determinant` clamped at 0.
