# Mobile-Manipulator Kinematics — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```cpp
class TestMobileManipulatorKinematics : public ::testing::Test:
    # 3-link arm (M30 link model): z-axis joint at (0,0,0.1); y-axis joint +(0,0,0.2); y-axis joint +(0.3,0,0)
    std::array<RevoluteJointLink<float>, 3>  links{ ... }
    ChainTaskJacobian<float, 6, 3>           arm{ links, SE3Transform{ I, (0.25, 0, 0) } }
    MobileManipulatorKinematics<float, 3>    mm{ arm, SE3Transform{ Rz(π/2), (0.1, 0, 0.3) }, 0.3 }
    BaseState<float> base{ 1, 2, π/6 };  JointVector q{ 0.4, −0.6, 0.9 }
    u = (v, ω, q̇) = (0.3, −0.5, 0.2, −0.1, 0.4)
# each case below is a TEST_F(TestMobileManipulatorKinematics, <name>)
```

## Test cases (Arrange / Act / Assert)

```cpp
tool_pose_composes_base_mount_and_arm:
    Assert: ToolPose(base, q) = T_wb·mount·arm.ToolPose(q); p ≈ (0.698536, 2.343297, 0.695513)
twist_matches_finite_difference:
    Arrange: integrate base (x += εv cos φ, y += εv sin φ, φ += εω) and q += εq̇, ε = 1e-3
    Assert:  Compute(base, q)·u ≈ (Δp; Log(ΔR)) / ε  (tol 1e-3);
             Compute·u ≈ (0.403993, 0.199541, −0.046890; −0.180886, −0.239333, −0.3)
arm_block_is_rotated_into_the_world:
    Arrange: u = (0, 0, q̇)
    Assert:  columns 2..4 = blockdiag(R_wa, R_wa)·arm.Jacobian(q), R_wa = Rz(π/6)·Rz(π/2)
forward_drive_moves_tool_along_heading:
    Arrange: base φ = 0, u = (1, 0, 0, 0, 0)
    Assert:  tool twist = ((1, 0, 0); 0)
yaw_moves_tool_by_lever_arm:
    Arrange: base φ = 0, u = (0, 1, 0, 0, 0)
    Assert:  linear = ẑ × r, ‖linear‖ = ‖r_xy‖ = 0.456874; angular = ẑ
base_constraint_blocks_sideslip:
    Assert: (−sin φ, cos φ, 0)·BaseConstraint(φ)·(v, ω) = 0 for φ ∈ {0, π/6, π}
wheel_speeds_from_unicycle_command:
    Assert: WheelSpeeds(1, 2) = (0.7, 1.3)  (L = 0.3)
```

## Reference vectors

- Fixture: tool world `p = (0.698536, 2.343297, 0.695513)`, `r = (−0.301464, 0.343297, 0.695513)`.
- `J·u` agrees with the finite difference to `3.6e-8` (ε = 1e-6) / `3.6e-5` (ε = 1e-3) in double;
  stacking the unrotated arm block instead gives an error of `0.516`.

## Edge cases

- Tool above the axle midpoint (`r_xy = 0`) ⇒ yaw adds no tool translation.
- Heading wrap `φ = ±π` ⇒ columns continuous.
- Redundancy split (base vs arm) is M14's job, not tested here.
