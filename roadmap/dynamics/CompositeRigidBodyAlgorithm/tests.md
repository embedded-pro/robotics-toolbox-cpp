# Composite Rigid Body Algorithm — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```cpp
class TestCompositeRigidBodyAlgorithm : public ::testing::Test:
    CompositeRigidBodyAlgorithm<float, 2> crba2
    CompositeRigidBodyAlgorithm<float, 3> crba3
    # uniform rods: inertia diag(0, ml²/12, ml²/12), CoM at (l/2, 0, 0)
# each case below is a TEST_F(TestCompositeRigidBodyAlgorithm, <name>)
```

## Test cases (Arrange / Act / Assert)

```text
single_rod_about_end_is_ml2_over_3:
    Assert: M = m·l²/3 for a z-axis rod
two_link_planar_matches_closed_form:
    Arrange: z-axis rods m1, l1, m2, l2; q2 = −0.5
    Assert:  M11 = m1l1²/3 + m2(l1² + l2²/3 + l1l2 cos q2), M12 = m2(l2²/3 + l1l2 cos q2 / 2), M22 = m2l2²/3
matrix_is_independent_of_first_joint_angle:
    Assert: M(q1 = 0.3, q2) ≈ M(q1 = −1.2, q2) for the planar arm
columns_match_recursive_newton_euler:
    Arrange: 3-link chain with skewed axes, full inertia tensors, off-axis CoM
    Assert:  M·e_j ≈ RNEA(q, 0, e_j, gravity = 0) for every unit vector e_j
matrix_is_symmetric_positive_definite:
    Assert: M == Mᵀ and all leading principal minors > 0 on the 3-link chain
kinetic_energy_matches_link_sum:
    Assert: ½ q̇ᵀ M q̇ ≈ Σ ½(m‖v_c‖² + ωᵀ I_c ω) using link velocities from forward propagation
```

## Reference vectors

- Unit rods, `q2 = −0.5`: `M11 = 5/3 + cos(0.5)`, `M12 = 1/3 + cos(0.5)/2`, `M22 = 1/3`.

## Edge cases

- One link ⇒ `1×1` matrix, no ancestor loop.
- Point-mass link (zero `I_c`) ⇒ still SPD while the mass is off the joint axis.
