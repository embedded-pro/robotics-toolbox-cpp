# Generalized-Momentum Observer — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestMomentumObserver : public ::testing::Test:
    # 2-link uniform rods about ŷ, gravity on; plant simulated with ABA at dt = 1 ms
    ChainDynamicsModel<float, 2> model{ ... }
    CoriolisMatrix<float, 2>     coriolis
    MomentumObserver<float, 2>   observer{ model, coriolis, links, gain = (50, 50) }
# each case below is a TEST_F(TestMomentumObserver, <name>)
```

## Test cases (Arrange / Act / Assert)

```
free_motion_keeps_residual_near_zero:
    Arrange: gravity-compensated motion with commanded torque, no external torque, 1 s
    Assert:  max |r| stays below 1e-2 N·m
step_external_torque_is_recovered_with_first_order_response:
    Arrange: τ_ext = (0, 0.8) N·m applied to the plant from t = 0.2 s
    Assert:  r_2(0.2 + 1/K) ≈ 0.8·(1 − e^{−1}) and r_2 → 0.8
collision_flag_uses_per_joint_thresholds:
    Assert: Collision((0.5, 0.5)) is false before and true after the step settles
reset_restarts_from_current_momentum:
    Arrange: run with non-zero velocity, Reset(q, q̇)
    Assert:  residual == 0 immediately after
uses_coriolis_transpose_not_the_rnea_vector:
    Arrange: fast motion where Cᵀq̇ ≠ C q̇
    Assert:  residual stays near zero (a C q̇ implementation drifts — documents why M31 is required)
```

## Reference vectors

- First-order response: `r(t) = τ_ext (1 − e^{−K_O (t − t0)})`.

## Edge cases

- `gain = 0` ⇒ residual identically zero.
- `dt` large relative to `1/K_O` ⇒ document instability; the test uses `K_O·dt = 0.05`.
