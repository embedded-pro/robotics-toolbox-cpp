# Computed-Torque Control — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestComputedTorqueControl : public ::testing::Test:
    StrictMock<dynamics::MockInverseDynamicsModel<float,2>> model
    SquareMatrix<float,2> Kp = diag(100, 100)
    SquareMatrix<float,2> Kd = diag( 20,  20)
    ComputedTorqueControl<float,2> controller{ model, Kp, Kd }
# each case below is a TEST_F(TestComputedTorqueControl, <name>)
# single-step cases set EXPECT_CALL(model, ComputeInverseDynamics(q, qDot, aqExpected)).Times(1)
```

## Test cases (Arrange / Act / Assert)

```
pure_feedforward_passes_desired_acceleration:
    Arrange: q = qd, qDot = qdDot, qdDdot = a; model returns r
    Act:     τ = ComputeTorque(q, qDot, qd, qdDot, qdDdot)
    Assert:  called once with (q, qDot, a); τ == r      (model output returned unchanged)

position_error_enters_command:
    Arrange: e = qd − q != 0, eDot = 0, qdDdot = 0
    Assert:  called once with aq == Kp · e

velocity_error_enters_command:
    Arrange: eDot = qdDot − qDot != 0, e = 0, qdDdot = 0
    Assert:  called once with aq == Kd · eDot

full_command_superposition:
    Arrange: e, eDot, qdDdot all nonzero
    Assert:  called once with aq == qdDdot + Kd·eDot + Kp·e, and with the MEASURED q, qDot

decoupled_error_dynamics:
    Arrange: 2-link plant M(q)q̈ + C(q,q̇)q̇ + g(q) = τ; mock Invoke returns M(q)·aq + C(q,q̇)q̇ + g(q)
             of the same plant (exact model); run K steps from e(0) != 0
    Assert:  e(t) matches the solution of ë + Kd·ė + Kp·e = 0 (same integrator), ||e|| -> 0
```

## Reference vectors

- With the exact model, closed-loop error obeys `ë + Kd·ė + Kp·e = 0` — for `Kp = 100`, `Kd = 20`
  critically damped: `e(t) = (e₀ + (ė₀ + 10e₀)t)·e^{−10t}` per joint.
- `e = [0.1, −0.2]`, `ė = 0`, `q̈d = 0` ⇒ expected `aq = [10, −20]` passed to the model.

## Edge cases

- Only `ComputeInverseDynamics` exists on the mock — no mass-matrix query is possible (StrictMock
  fails on any unexpected call), pinning the single `O(n)` call.
- Model mismatch (plant `M` scaled 1.2 vs the mocked model): bounded tracking error, closed loop stays stable.
- Large `qdDdot`: torque stays within the documented actuator model.
