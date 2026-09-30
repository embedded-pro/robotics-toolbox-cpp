# Product of Exponentials — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestProductOfExponentials : public ::testing::Test:
    # 2-link unit planar arm: revolute about ẑ through (0,0,0) and through (1,0,0); screws (v; ω)
    std::array<Vector6<float>, 2> screws{ ((0,0,0); ẑ), ((0,−1,0); ẑ) }
    ProductOfExponentials<float, 2> poe{ screws, SE3Transform{ I, (2, 0, 0) } }
    # UR5 (Lynch–Park Ex. 4.5, lengths W1=0.109 W2=0.082 L1=0.425 L2=0.392 H1=0.089 H2=0.095), (v; ω):
    #   ((0,0,0);ẑ) ((−H1,0,0);ŷ) ((−H1,0,L1);ŷ) ((−H1,0,L1+L2);ŷ) ((−W1,L1+L2,0);−ẑ) ((H2−H1,0,L1+L2);ŷ)
    #   M = { [[−1,0,0],[0,0,1],[0,1,0]], (L1+L2, W1+W2, H1−H2) }
    ProductOfExponentials<float, 6> ur5{ ... }
# each case below is a TEST_F(TestProductOfExponentials, <name>)
```

## Test cases (Arrange / Act / Assert)

```
zero_config_returns_home:
    Assert: poe.Compute({0, 0}) ≈ home (tip (2, 0, 0)); ur5.Compute(0).p ≈ (0.817, 0.191, −0.006)
single_joint_rotation:
    Assert: poe.Compute({π/2, 0}).p ≈ (0, 2, 0)
elbow_rotation:
    Assert: poe.Compute({0, π/2}).p ≈ (1, 1, 0)
agrees_with_denavit_hartenberg:
    Assert: poe.Compute(q) ≈ DH 2-link unit arm Forward(q) for q ∈ {(0.4,−0.7), (1.3,2.1)}
space_jacobian_linear_part_is_base_origin_velocity:
    Arrange: q = (0.4, −0.7)
    Assert:  SpaceJacobian(q) = [[0, 0.389418], [0, −0.921061], [0,0], [0,0], [0,0], [1,1]]
space_to_geometric_matches_frame_chain_jacobian:
    Arrange: ur5, q = (0.3,−0.5,0.8,−0.2,0.6,1.1)
    Assert:  SpaceToGeometric(SpaceJacobian(q), Compute(q).p) ≈ GeometricJacobian::Compute(Frames(q))
geometric_jacobian_matches_finite_difference:
    Arrange: ur5, q as above, q̇ = (0.2,−0.4,0.3,0.5,−0.1,0.7), ε = 1e-3
    Assert:  J_geo·q̇ ≈ (Δp; Log(ΔR)) / ε  (tol 1e-3)
frames_place_joint_origins_on_the_axes:
    Assert: poe.Frames({0.4, −0.7}).jointOrigins ≈ {(0,0,0), (0.921061, 0.389418, 0)}, axes = ẑ
prismatic_screw_translates:
    Arrange: ProductOfExponentials<float, 1>{ { ((1,0,0); 0) }, Identity }
    Assert:  Compute({0.3}).p ≈ (0.3, 0, 0); Frames type Prismatic, axis x̂
```

## Reference vectors

- Planar arm: `q = (π/2, 0)` ⇒ `(0, 2, 0)`; `q = (0, π/2)` ⇒ `(1, 1, 0)`; `q = (0.4, −0.7)` ⇒
  `(1.876397, 0.093898, 0)`, `J_geo = [[−0.093898, 0.295520], [1.876397, 0.955336], 0, 0, 0, [1, 1]]`.
- UR5 at the test `q`: tool `p = (0.696820, 0.400489, 0.077764)`;
  `J_space·q̇ = (−0.170343, −0.064298, 0.868026; 0.096307, 1.053237, 0.260041)`,
  `J_geo·q̇ = (−0.192582, 0.109415, 0.172680; 0.096307, 1.053237, 0.260041)`.

## Edge cases

- `q = 0` ⇒ `Exp` returns identity, `Compute = M`.
- Prismatic screw (`ω = 0`) ⇒ pure translation branch of `Exp`.
- Pitched screw (`ω·v ≠ 0`) ⇒ rejected at construction (`really_assert`).
