# Cable Tension Distribution — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap. No mocks needed: pure geometry + closed form.

## Fixture

```
class TestCableTensionDistribution : public ::testing::Test:     # planar point mass, 3 cables, WrenchDim = 2
    std::array<Vector3<float>,3> base     = { (0, 2, 0), (−√3, −1, 0), (√3, −1, 0) }   # 120° apart, radius 2
    std::array<Vector3<float>,3> platform = { 0, 0, 0 }
    float tMin = 10, tMax = 200                                   # tm = 105
    CableTensionDistribution<float,3,2> distributor{ base, platform, tMin, tMax }
    SE3Transform<float> pose = Identity                           # point mass at the origin
    # u = (0, 1), (−√3/2, −1/2), (√3/2, −1/2);  A·Aᵀ = 1.5·I;  A·(1,1,1) = 0

class TestCableTensionDistributionSquare : public ::testing::Test:   # 2 cables, WrenchDim = 2
    base = { (−1, 1, 0), (1, 1, 0) }, platform = { 0, 0 }, tMin = 10, tMax = 200
    CableTensionDistribution<float,2,2> distributor{ base, platform, tMin, tMax }

class TestCableTensionDistributionSpatial : public ::testing::Test:  # 8 cables, WrenchDim = 6
    base[0] = (2, 2, 2), platform[0] = (0.2, 0, 0); remaining anchors: cube / box corners
    CableTensionDistribution<float,8,6> distributor{ base, platform, tMin, tMax }
# each case below is a TEST_F(<fixture>, <name>); default fixture TestCableTensionDistribution
```

## Test cases (Arrange / Act / Assert)

```
structure_matrix_from_geometry:
    Act:     A = ComputeStructureMatrix(pose)
    Assert:  columns == (0, 1), (−0.866025, −0.5), (0.866025, −0.5)

spatial_structure_matrix_includes_moment_arm (TestCableTensionDistributionSpatial):
    Arrange: pose = { Rz(π/2), p = 0 }  ⇒  R·b₀ = (0, 0.2, 0)
    Assert:  column 0 == (u₀; (R·b₀) × u₀) = (0.59655, 0.53689, 0.59655, 0.11931, 0, −0.11931)

zero_wrench_gives_centred_pretension:
    Arrange: w = 0
    Assert:  t == (I − A⁺A)·tm·1 = (105, 105, 105)       (tm·1 already lies in null(A) here)

redundant_case_centres_tensions:
    Arrange: w = (0, 60)
    Assert:  t == tm·1 + A⁺·w = (145, 85, 85)            (min ‖t − tm‖, all within bounds)

wrench_is_reproduced:
    Arrange: w = (30, −40)
    Assert:  A·t ≈ w and tMin ≤ t ≤ tMax;  t ≈ (78.333, 101.013, 135.654)

upper_bound_fixes_worst_cable:
    Arrange: w = (0, 150)          (plain closed form: t₀ = 205 > tMax)
    Assert:  t == (200, 50, 50), A·t ≈ w

lower_bound_fixes_slack_cable:
    Arrange: w = (0, −150)         (plain closed form: t₀ = 5 < tMin)
    Assert:  t == (10, 160, 160), A·t ≈ w

infeasible_wrench_returns_nullopt:
    Arrange: w = (0, 300)          (t₀ fixed at 200 ⇒ remaining pair needs −100 ⇒ < WrenchDim free cables)
    Assert:  Distribute(w, pose) == nullopt

square_case_unique_solution (TestCableTensionDistributionSquare):
    Arrange: w = (0, 100)
    Assert:  t == A⁻¹·w = (70.7107, 70.7107)             (tm cancels)

collinear_cables_return_nullopt (TestCableTensionDistributionSquare):
    Arrange: pose.p = (0, 1, 0)    (both cables horizontal and opposite ⇒ rank(A) = 1)
    Assert:  Distribute(w, pose) == nullopt
```

## Reference vectors

- Planar fixture: `A⁺ = (2/3)·Aᵀ`, so `t = 105 + (2/3)·Aᵀw` while unclamped; `w = (0, W)` ⇒
  `t = 105 + (2W/3)·(1, −½, −½)` — symmetric tensions on cables 1 and 2.
- Upper-bound pass: fix `t₀ = 200` ⇒ `w' = (0, −50)` ⇒ square pair gives `t₁ = t₂ = 50`.
- Square fixture: `u = (∓1, 1)/√2`, vertical `w = (0, W)` ⇒ `t₀ = t₁ = W/√2`.

## Edge cases

- Platform anchor coincident with a base anchor (zero-length cable): `nullopt`.
- Wrench exactly on the workspace boundary: one tension equals `tMin` or `tMax`, still feasible.
- Near-parallel cables (`Af·Afᵀ` ill-conditioned): `TrySolveSystem` pivot threshold ⇒ `nullopt`, never `inf`.
