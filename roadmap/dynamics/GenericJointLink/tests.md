# Generic Joint Link — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestJointLink : public ::testing::Test:
    Vector3 x = {1, 0, 0}, z = {0, 0, 1}
    JointLink<float> MakeRevolute(axis, parentToJoint), MakePrismatic(axis, parentToJoint)
# each case below is a TEST_F(TestJointLink, <name>)   (algorithm cases live in the RNEA/ABA/FK tests)
```

## Test cases (Arrange / Act / Assert)

```
existing_aggregate_initializers_default_to_revolute:
    Arrange: RevoluteJointLink<float>{ m, I, axis, offset, com }
    Assert:  type == Revolute, armature == 0, limits unbounded
revolute_transform_rotates_about_axis:
    Assert: JointTransform(π/2) about z maps x → y, offset == parentToJoint
prismatic_transform_slides_along_axis:
    Assert: JointTransform(0.3) along x == (I, parentToJoint + (0.3, 0, 0))
motion_subspace_selects_one_channel:
    Assert: revolute → (axis; 0), prismatic → (0; axis)
vertical_slider_needs_weight_plus_inertial_force:           # RNEA test file
    Arrange: prismatic along z, mass m, gravity (0, 0, −g), q̈ = a
    Assert:  τ = m·(g + a)
prismatic_coriolis_term_on_rotating_base:                    # RNEA test file
    Arrange: revolute base about z spinning at ω, prismatic slider along x at q = r, q̇ = v
    Assert:  base torque = 2·m·r·v·ω (Coriolis) + inertial terms per closed form
mixed_chain_round_trips_through_aba:                         # ABA test file
    Assert: ABA(RNEA(q̈)) ≈ q̈ for a revolute–prismatic–revolute chain under gravity
armature_adds_to_diagonal_inertia:
    Assert: single rod: τ = (m l²/3 + armature)·q̈ in RNEA; ABA inverts it; CRBA M = m l²/3 + armature
```

## Reference vectors

- Slider: `m = 2`, `a = 1`: `τ = 2·(9.81 + 1) = 21.62 N`.
- Rotating slider (point mass at `r` on an arm spinning at `ω`, extending at `v`): Coriolis torque
  `2 m r v ω` about the base axis.

## Edge cases

- Negative `q` retracts the slider / reverses the rotation.
- Full `2π` revolute wrap returns to identity.
