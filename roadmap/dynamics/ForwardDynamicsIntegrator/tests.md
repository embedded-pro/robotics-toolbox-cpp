# Forward-Dynamics Integrator — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestForwardDynamicsIntegrator : public ::testing::Test:
    # single uniform rod about ŷ (pendulum) and a 2-link rod chain, gravity (0, 0, −9.81)
    ForwardDynamicsIntegrator<float, 1, IntegrationMethod::RungeKutta4> pendulum{ rod, gravity }
    ForwardDynamicsIntegrator<float, 2, IntegrationMethod::SemiImplicitEuler> doubleRod{ rods, gravity }
# each case below is a TEST_F(TestForwardDynamicsIntegrator, <name>)
```

## Test cases (Arrange / Act / Assert)

```
small_oscillation_period_matches_physical_pendulum:
    Arrange: rod length L hanging at q = π/2, released from π/2 + 0.05 rad, dt = 1 ms
    Assert:  measured period ≈ 2π·sqrt(2L/(3g)) within 0.5 %
rk4_energy_drift_is_small:
    Assert: |E(10 s) − E(0)| / |E(0)| < 1e-4 for the pendulum at dt = 1 ms
semi_implicit_euler_energy_stays_bounded:
    Assert: energy error over 10 s never exceeds its maximum over the first second by more than 2×
step_acceleration_equals_articulated_body_algorithm:
    Assert: Step(x, τ, dt).qDDot == ABA(links, x.q, x.q̇, τ, gravity)
constant_torque_on_free_link_gives_quadratic_angle:
    Arrange: z-axis rod (gravity ⟂ motion), τ constant
    Assert:  q(t) ≈ ½·(τ / (mL²/3))·t²
```

## Reference vectors

- Uniform rod pendulum: `ω₀ = sqrt(3g / (2L))`; `L = 1 m`, `g = 9.81` ⇒ period `1.638 s`.

## Edge cases

- `dt = 0` ⇒ state unchanged.
- Zero gravity, zero torque ⇒ constant velocity.
