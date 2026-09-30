# Operational-Space Control — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```cpp
class TestOperationalSpaceControl : public ::testing::Test:          # Dof = 2, TaskDim = 2
    StrictMock<dynamics::MockEulerLagrangeDynamics<float,2>>  model
    StrictMock<kinematics::MockJacobianProvider<float,2,2>>   jacobian
    SquareMatrix<float,2> Kp = diag(100, 100)
    SquareMatrix<float,2> Kd = diag( 20,  20)
    OperationalSpaceControl<float,2,2> controller{ model, jacobian, Kp, Kd, 0 }     # σ = 0
    # ToolPose mocked as { I, (x, y, 0) }; desiredPose = { I, (xd, yd, 0) }

class TestOperationalSpaceControlRedundant : public ::testing::Test: # Dof = 3, TaskDim = 2
    StrictMock<dynamics::MockEulerLagrangeDynamics<float,3>>  model     # M = [[3,.5,.2],[.5,2,.3],[.2,.3,1]], g = (4,−2,1.5)
    StrictMock<kinematics::MockJacobianProvider<float,2,3>>   jacobian  # J = [[1,.5,.2],[0,1,.7]]
    OperationalSpaceControl<float,3,2> controller{ model, jacobian, Kp, Kd, 0 }
# each case below is a TEST_F(<fixture>, <name>); default fixture TestOperationalSpaceControl
```

## Test cases (Arrange / Act / Assert)

```text
unit_inertia_gives_task_pd:
    Arrange: M = I, J = I, J̇q̇ = 0, C q̇ = 0, g = 0; eX, eXDot, xdDdot given
    Act:     τ = ComputeTorque(q, qDot, desiredPose, xdDot, xdDdot, 0)
    Assert:  τ == xdDdot + Kd·eXDot + Kp·eX

task_inertia_scales_command:
    Arrange: M = diag(2, 3), J = I (Λ = diag(2, 3)), J̇q̇ = 0, model terms 0
    Assert:  τ == diag(2, 3)·aX

bias_acceleration_subtracted:
    Arrange: M = I, J = I, all errors 0, xdDdot = 0, J̇q̇ = b
    Assert:  τ == −b

coriolis_and_gravity_added_in_joint_space:
    Arrange: M = I, J = I, aX = 0, J̇q̇ = 0, C q̇ = c, g != 0
    Assert:  τ == c + g

jacobian_transpose_realises_wrench:
    Arrange: M = I, J = [[1,0],[1,2]], errors 0, xdDdot = aX, model terms 0
    Assert:  τ == Jᵀ·(J Jᵀ)⁻¹·aX   (= J⁻¹·aX)

redundant_at_rest_holds_gravity_exactly (TestOperationalSpaceControlRedundant):
    Arrange: q̇ = 0 (C q̇ = 0, J̇q̇ = 0), ToolPose = desiredPose, xdDot = xdDdot = 0, tauSecondary = 0
    Assert:  τ == g = (4, −2, 1.5)        (null-space gravity compensated)

secondary_torque_invisible_to_task (TestOperationalSpaceControlRedundant):
    Arrange: same state; τ₀ = (1, −2, 0.5); Δτ = τ(τ₀) − τ(0)
    Assert:  Δτ == N·τ₀ = (0.38863, −1.32780, 1.06224) != 0 and J·M⁻¹·Δτ ≈ 0

singular_jacobian_uses_damping:
    Arrange: local controller with σ = 0.1; M = I, J = [[1,1],[1,1]] (rank 1), aX != 0
    Assert:  τ finite (no NaN / inf)

dependencies_queried_once:
    Arrange: any state
    Assert:  ComputeMassMatrix, ComputeCoriolisTerms, ComputeGravityTerms, Jacobian, BiasAcceleration,
             ToolPose each called exactly once (StrictMock)
```

## Reference vectors

- `M = I`, `J = I`: `Λ = I`, controller collapses to task-space PD `τ = xdDdot + Kd·eXDot + Kp·eX + C q̇ + g`.
- Redundant fixture: the classic `J̄ᵀg` form would return `JᵀJ̄ᵀg = (3.3365, 0.2670, −0.3136) ≠ g`;
  the law returns `g` exactly. `J·M⁻¹·N·τ₀ ≈ 0` (verified ≈ 1e-16 in double).

## Edge cases

- Redundant arm (`Dof > TaskDim`): non-trivial null space; secondary objective realised without
  disturbing the task, and self-motion does not sag under gravity.
- Singularity: σ > 0 keeps torque bounded; task tracking becomes approximate.
- Model mismatch: task decoupling degrades gracefully, stays stable.
