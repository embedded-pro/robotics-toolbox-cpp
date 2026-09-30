# Chain Pose Kinematics — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```cpp
class TestChainPoseKinematics : public ::testing::Test:
    # 3-link arm: z-axis base joint raised 0.2, then two y-axis joints (0.5 up, 0.4 along x)
    std::array<RevoluteJointLink<float>, 3> arm{ ... }
    ChainPoseKinematics<float, 3> kinematics{ SE3Transform{ I, (0.3, 0, 0) } }
# each case below is a TEST_F(TestChainPoseKinematics, <name>)
```

## Test cases (Arrange / Act / Assert)

```text
origins_and_tool_match_shipped_forward_kinematics:
    Assert: jointOrigins[i] and tool.p equal ForwardKinematics{ (0.3,0,0) }.Compute positions
axes_are_expressed_in_the_base_frame:
    Arrange: q = (π/2, 0, 0)
    Assert:  jointAxes[1] ≈ (−1, 0, 0)  (y rotated by +90° about z)
tool_orientation_composes_joint_rotations:
    Arrange: q = (π/2, π/2, 0)
    Assert:  tool.R ≈ Rz(π/2)·Ry(π/2)
tool_frame_rotation_is_applied_last:
    Arrange: toolInLastLink = { Rx(π/2), 0 }
    Assert:  tool.R ≈ Rz(q1)·Ry(q2)·Ry(q3)·Rx(π/2)
base_offset_moves_every_origin:
    Assert: jointOrigins[0] == links[0].parentToJoint for all q
```

## Reference vectors

- `Rz(+90°)·ŷ = −x̂`; `Rz(90°)Ry(90°)·x̂ = (0, 0, −1)`.

## Edge cases

- Zero-length links (coincident origins) — valid, origins repeat.
- Non-unit axis — asserted at construction of the link (M1), not here.
