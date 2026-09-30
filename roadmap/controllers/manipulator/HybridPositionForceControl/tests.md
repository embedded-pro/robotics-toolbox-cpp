# Hybrid Position/Force Control — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestHybridPositionForceControl : public ::testing::Test:
    # both dependencies mocked; planar TaskDim = 2, Dof = 2:
    StrictMock<dynamics::MockEulerLagrangeDynamics<float,2>>  model
    StrictMock<kinematics::MockJacobianProvider<float,2,2>>   jacobian
    SquareMatrix<float,2> Rc = I                    # constraint frame = base frame
    SquareMatrix<float,2> S  = diag(1, 0)           # constraint axis 0 = motion, axis 1 = force
    Gains gains{ Kp = diag(100,100), Kd = diag(20,20), Kf = diag(2,2), Ki = diag(5,5),
                 Kdf = diag(3,3), integralLimit = (1, 1) }
    float dt = 0.001
    HybridPositionForceControl<float,2,2> controller{ model, jacobian, Rc, S, gains, dt }
    # ToolPose mocked as { I, (x, y, 0) }
# each case below is a TEST_F(TestHybridPositionForceControl, <name>)
```

## Test cases (Arrange / Act / Assert)

```
motion_axis_does_position_pd:
    Arrange: J = I, model 0; only axis-0 position/velocity error, fd = fMeasured = 0
    Assert:  τ[0] == Kp[0]·eX[0] + Kd[0]·eXDot[0];  τ[1] == 0

force_axis_does_damped_force_pi:
    Arrange: J = I, model 0; axis-1 force error eF, qDot[1] = v, position error 0, one step
    Assert:  τ[1] == fd[1] + Kf[1]·eF[1] + Ki[1]·eF[1]·dt − Kdf[1]·v

selection_masks_cross_terms:
    Arrange: position error only on axis 1, force error only on axis 0, fd = 0, qDot = 0
    Assert:  τ == 0          (each axis responds to exactly one loop)

constraint_rotation_maps_axes:
    Arrange: local controller with Rc = [[0,−1],[1,0]] (constraint x = base y, constraint y = base −x);
             J = I, model 0, qDot = 0; eX = (0.02, 0.1), fd = (−5, 0), fMeasured = (−3, 0) (base frame)
    Assert:  τ == (−9.01, 10)     (base-y error → motion loop; base-x error lies on the force axis
                                   and is ignored; force loop acts along base −x)

force_integral_accumulates:
    Arrange: constant eF[1], N calls with N·dt·|eF[1]| < integralLimit[1]
    Assert:  integral contribution == Ki[1]·eF[1]·(N·dt)

force_integral_clamped:
    Arrange: constant eF[1] = 10, 1000 calls (unclamped ∫ = 10 > integralLimit = 1)
    Assert:  τ[1] == fd[1] + Kf[1]·10 + Ki[1]·1      (anti-windup)

reset_clears_force_integral:
    Arrange: accumulate integral, Reset()
    Assert:  next force term == fd[1] + Kf[1]·eF[1] + Ki[1]·eF[1]·dt   (no history)

jacobian_transpose_maps_wrench:
    Arrange: J = [[1,0],[0,2]], known F
    Assert:  τ == Jᵀ·F

gravity_and_coriolis_added:
    Arrange: model g, C q̇ nonzero; errors 0, fd = fMeasured = 0, qDot = 0
    Assert:  τ == C q̇ + g

dependencies_queried_once:
    Arrange: any state
    Assert:  Jacobian, ToolPose, ComputeCoriolisTerms, ComputeGravityTerms each once;
             ComputeMassMatrix, BiasAcceleration never called (StrictMock)
```

## Reference vectors

- `Rc = I`, `S = diag(1,0)`, `J = I`, model 0:
  `F = [Kp·eX[0] + Kd·eXDot[0], fd[1] + Kf·eF[1] + Ki·∫eF[1] − Kdf·ẋ[1]]`, `τ = JᵀF` — hand-checkable.
- Rotated case: `Rcᵀ·eX = (0.1, −0.02)`, `Rcᵀ·fd = (0, 5)`, `eF = (0, 2)`, `∫eF = (0, 0.002)` ⇒
  constraint wrench `(10, 9.01)` ⇒ base `F = Rc·(10, 9.01) = (−9.01, 10)`.
- Contact equilibrium on a stiff surface: at rest the applied force equals the force-axis wrench,
  and the PI drives `fMeasured → fd` ⇒ steady `eF = 0`.

## Edge cases

- Contact loss (`fMeasured → 0`): the integral saturates at `±integralLimit` (bounded) — `Reset()` on re-contact.
- `S = I` (all motion) reduces to Cartesian PD + `g` + `Cq̇`; `S = 0` (all force) to damped force PI.
- Near-singular `J`: `Jᵀ` stays finite, torque bounded (no inversion).
