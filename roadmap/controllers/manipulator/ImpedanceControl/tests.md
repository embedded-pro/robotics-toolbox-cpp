# Impedance Control — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```cpp
class TestImpedanceControl : public ::testing::Test:
    # both injected dependencies mocked; Dof = 2, TaskDim = 2 (planar xy point):
    StrictMock<dynamics::MockEulerLagrangeDynamics<float,2>>   model
    StrictMock<kinematics::MockJacobianProvider<float,2,2>>    jacobian
    SquareMatrix<float,2> K  = diag(500, 300)
    SquareMatrix<float,2> D  = diag( 40,  30)
    SquareMatrix<float,2> Md = diag(  2, 1.5)
    ImpedanceControl<float,2,2> controller{ model, jacobian, K, D, Md }
    # ToolPose mocked as { I, (x, y, 0) }; desiredPose = { I, (xd, yd, 0) }

class TestImpedanceControlPose : public ::testing::Test:     # TaskDim = 6 branch of TaskError
    StrictMock<dynamics::MockEulerLagrangeDynamics<float,6>>   model
    StrictMock<kinematics::MockJacobianProvider<float,6,6>>    jacobian
    ImpedanceControl<float,6,6> controller{ model, jacobian, K6 = diag(500,500,500,20,20,20), D6, Md6 = I }
# each case below is a TEST_F(<fixture>, <name>); default fixture TestImpedanceControl
```

## Test cases (Arrange / Act / Assert)

```cpp
# ---- (a) ComputeTorque: stiffness/damping, no force sensing ----
compliant_law_with_identity_jacobian:
    Arrange: J = I; e = xd − x != 0; qDot, xdDot != 0; g != 0
    Act:     τ = ComputeTorque(q, qDot, desiredPose, xdDot)
    Assert:  τ == K·e + D·(xdDot − qDot) + g

jacobian_transpose_maps_wrench:
    Arrange: J = [[1,0],[0,2]], e != 0, qDot = 0, xdDot = 0, g = 0
    Assert:  τ == Jᵀ·(K·e)

compliant_law_queries_only_gravity:
    Arrange: any state
    Assert:  Jacobian, ToolPose, ComputeGravityTerms each once; ComputeMassMatrix,
             ComputeCoriolisTerms, BiasAcceleration never called (StrictMock)

static_deflection_equals_compliance:
    Arrange: simulated 2-DOF Cartesian plant M q̈ + g = τ + fExt with M = diag(1, 2), g = (0, 19.62),
             J = I and x = q via Invoke; constant fExt = (3, −4); xd constant; semi-implicit Euler,
             dt = 1 ms, 5 s from x = xd
    Assert:  x − xd → K⁻¹·fExt = (0.006, −0.013333), q̇ → 0      (tool yields along fExt)

pose_error_uses_rotation_vector (TestImpedanceControlPose):
    Arrange: J = I; ToolPose = { Rz(0.2), 0 }, desiredPose = Identity; qDot = 0, xdDot = 0, g = 0
    Assert:  τ == K6·(0, 0, 0, 0, 0, −0.2)       (PoseError, never a difference of orientations)

# ---- (b) ComputeTorqueWithInertiaShaping ----
inertia_shaping_with_unit_model:
    Arrange: M = I, J = I, J̇q̇ = 0, C q̇ = 0, g = 0; e, eDot, xdDdot, fExt != 0
    Assert:  τ == aX − fExt,  aX = xdDdot + Md⁻¹·(D·eDot + K·e + fExt)

inertia_shaping_uses_task_inertia_bias_and_model:
    Arrange: M = diag(2, 3), J = I (Λ = diag(2, 3)), J̇q̇ = b, C q̇ = c, g != 0
    Assert:  τ == Λ·(aX − b) − fExt + c + g; each model / provider method called once

matched_inertia_reduces_to_compliant_law:
    Arrange: local controller with Md = Λ = diag(2, 3) (M = diag(2, 3), J = I); J̇q̇ = b, C q̇ = c; fExt != 0
    Assert:  ComputeTorqueWithInertiaShaping == ComputeTorque + Λ·(xdDdot − b) + c, independent of fExt

rendered_impedance_matches_target:
    Arrange: plant of static_deflection (M = diag(1, 2) ≠ Md, C = 0, J̇q̇ = 0), step fExt, xd constant;
             integrate the plant and the target  Md·x̃̈ + D·x̃̇ + K·x̃ = fExt  side by side (same integrator)
    Assert:  x − xd tracks x̃(t) within integration tolerance for the whole run
```

## Reference vectors

- (a) equilibrium: `K = diag(500, 300)`, `fExt = (3, −4)` ⇒ `x − xd = (0.006, −0.013333)`.
- (b) `M = I`, `J = I`: `Λ = I`, `τ = aX − fExt + C q̇ + g`.
- `Rz(0.2)` current vs identity desired ⇒ `e = (0, 0, 0, 0, 0, −0.2)`.

## Edge cases

- Near-singular `J`: (a) torque stays finite (`Jᵀ` only); (b) needs the damped `Λ` (see implementation).
- Very high `K`: approaches rigid position control; watch contact instability vs sample rate.
- Sensor bias `β` in (b): static `x̃ = K⁻¹·(fExt + (I − Md·Λ⁻¹)·β)` — no effect only when `Md = Λ`.
