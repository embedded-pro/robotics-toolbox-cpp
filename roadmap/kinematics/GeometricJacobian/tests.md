# Geometric Jacobian — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestGeometricJacobian : public ::testing::Test:
    # 2-link unit planar arm, z-axis joints, joint 2 at (1,0,0), tool at (1,0,0) of link 2
    std::array<RevoluteJointLink<float>, 2> planar{ ... }
    ChainPoseKinematics<float, 2> kinematics{ SE3Transform{ I, (1, 0, 0) } }
# each case below is a TEST_F(TestGeometricJacobian, <name>)
```

## Test cases (Arrange / Act / Assert)

```
stretched_planar_arm_columns:
    Act:    J = GeometricJacobian::Compute(kinematics.Compute(planar, (0, 0)))
    Assert: linear rows [[0,0],[2,1],[0,0]], angular rows [[0,0],[0,0],[1,1]]
linear_rows_match_finite_difference_of_tool_position:
    Arrange: q = (0.4, −0.7), q̇ = (0.3, −0.5)
    Assert:  J_linear·q̇ ≈ (p(q + εq̇) − p(q)) / ε
angular_rows_match_finite_difference_of_tool_rotation:
    Assert: J_angular·q̇ ≈ Log(R(q + εq̇)·R(q)ᵀ).angular / ε   (spatial 3-link arm, mixed axes)
bias_acceleration_matches_finite_difference:
    Assert: BiasAcceleration(frames, q̇) ≈ (J(q + εq̇) − J(q)) / ε · q̇
prismatic_column_is_pure_translation:
    Arrange: FrameChain with a prismatic joint along ŷ
    Assert:  its column = (ŷ; 0)
wrench_along_link_line_needs_no_torque:
    Assert: JointTorques(J(0,0), (x̂; 0)) ≈ (0, 0)
link_model_and_dh_frames_give_the_same_jacobian:
    Arrange: same 2-link arm described by DH (M7) and by links (M30)
    Assert:  Compute(dhFrames) ≈ Compute(linkFrames)
task_jacobian_position_rows_match_six_dof_rows:
    Assert: ChainTaskJacobian<float, 3, 2>::Jacobian(q) == first three rows of the 6-row version
```

## Reference vectors

- 2-link unit arm at `q = (0,0)`: linear `[[0,0],[2,1],[0,0]]`, angular `[[0,0],[0,0],[1,1]]`.
- Tool force `x̂` on the stretched arm acts along the links ⇒ joint torques `(0, 0)`.

## Edge cases

- Stretched arm ⇒ columns linearly dependent (singular).
- `q̇ = 0` ⇒ `BiasAcceleration = 0`.
- Single joint ⇒ `6×1`.
