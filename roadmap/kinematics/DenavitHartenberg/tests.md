# Denavit-Hartenberg Parameters — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestDenavitHartenberg : public ::testing::Test:
    # 2-link unit planar arm, both revolute (a, α, d, θ, type)
    DenavitHartenberg<float, 2> standard{ { {1,0,0,0,Revolute}, {1,0,0,0,Revolute} }, Standard }
    DenavitHartenberg<float, 2> modified{ { {0,0,0,0,Revolute}, {1,0,0,0,Revolute} }, Modified,
                                          SE3Transform{ I, (1, 0, 0) } }      # a₂ moves into the tool
# each case below is a TEST_F(TestDenavitHartenberg, <name>)
```

## Test cases (Arrange / Act / Assert)

```
zero_config_is_stretched_along_x:
    Assert: standard.Forward({0, 0}).p ≈ (2, 0, 0), R ≈ I
planar_elbow_ninety_deg:
    Assert: standard.Forward({0, π/2}).p ≈ (1, 1, 0)
single_link_revolute_rotates:
    Arrange: 1-link standard arm {1,0,0,0,Revolute}
    Assert:  Forward({π/2}).p ≈ (0, 1, 0)
theta_offset_is_added_to_the_joint_variable:
    Arrange: standard table with link 1 θ = π/2
    Assert:  Forward({0, 0}).p ≈ (0, 2, 0);  Forward({0, π/2}).p ≈ (−1, 1, 0)
prismatic_variable_adds_to_d:
    Arrange: 1-link {0,0,0.2,0,Prismatic}
    Assert:  Forward({0.5}).p ≈ (0, 0, 0.7)
link_twist_maps_z_to_minus_y:
    Arrange: link {0, π/2, 0, 0, Revolute}, both conventions
    Assert:  LinkTransform(0, 0).R · ẑ ≈ (0, −1, 0)
modified_link_transform_matches_elementary_product:
    Arrange: link {0.3, 0.7, 0.2, 0, Revolute}, q = 1.1
    Assert:  LinkTransform ≈ Rotx(0.7)·Transx(0.3)·Rotz(1.1)·Transz(0.2)
             ⇒ p ≈ (0.3, −0.128844, 0.152968)
standard_and_modified_give_the_same_pose:
    Assert: standard.Forward(q) ≈ modified.Forward(q) for q ∈ {(0,0), (0,π/2), (0.4,−0.7), (1.3,2.1)}
standard_and_modified_give_the_same_frame_chain:
    Assert: Frames(q).jointOrigins, jointAxes, tool agree; origins = {(0,0,0), (cos q₁, sin q₁, 0)}, axes = ẑ
frame_chain_jacobian_matches_finite_difference:
    Arrange: PUMA-560 standard table (below), q = (0.3,−0.4,0.5,0.6,−0.7,0.8), q̇ = (0.3,−0.2,0.5,0.1,−0.4,0.7)
    Assert:  GeometricJacobian::Compute(Frames(q))·q̇ ≈ (Δp; Log(ΔR)) / ε   (tol 1e-3, ε = 1e-3)
```

## Reference vectors

- 2-link unit arm: `q = (0, π/2)` ⇒ tip `(1, 1, 0)`; `q = (0.4, −0.7)` ⇒ `(1.876397, 0.093898, 0)`;
  `q = (1.3, 2.1)` ⇒ `(−0.699299, 0.708017, 0)` — identical for both conventions.
- `θ₁` offset `π/2`: `q = (0,0)` ⇒ `(0, 2, 0)`; `q = (0, π/2)` ⇒ `(−1, 1, 0)`.
- `α = π/2` ⇒ `R·ẑ = −ŷ` (both conventions).
- PUMA-560 standard `(a, α, d)`: `(0, π/2, 0)`, `(0.4318, 0, 0)`, `(0.0203, −π/2, 0.15005)`,
  `(0, π/2, 0.4318)`, `(0, −π/2, 0)`, `(0, 0, 0)`. At the test `q`: tool `p ≈ (0.402407, −0.032586,
  0.263519)`, `J·q̇ ≈ (−0.146069, 0.072514, −0.086416; −0.005657, 0.296324, 0.946824)`; forward-difference
  truncation error at `ε = 1e-3` is `≈ 1e-4`.

## Edge cases

- `α = ±π/2` ⇒ frame axis swaps; sign of `sα` checked by `link_twist_maps_z_to_minus_y`.
- Zero-length links (`a = d = 0`) ⇒ coincident origins.
- Mixed revolute/prismatic chain ⇒ `Frames` reports the per-joint type.
- `θ = 2π` ⇒ same pose as `θ = 0`.
