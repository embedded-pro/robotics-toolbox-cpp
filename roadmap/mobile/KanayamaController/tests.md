# Kanayama Controller — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```cpp
class TestKanayamaController : public ::testing::Test:
    KanayamaController<float> controller{ { 1, 4, 4 } }    # kx, ky, kθ (critically damped)
    float vr{ 0.5 }
# each case below is a TEST_F(TestKanayamaController, <name>)
```

## Test cases (Arrange / Act / Assert)

```cpp
zero_error_returns_the_feedforward:
    Assert: Compute(Identity, Identity, 0.5, 0.1) ≈ { 0.5, 0.1 }
longitudinal_error_changes_speed:
    Assert: Compute(Identity, FromPose(0.2, 0, 0), 0.5, 0) ≈ { 0.7, 0 }
lateral_error_turns_towards_the_reference:
    Assert: Compute(Identity, FromPose(0, 0.1, 0), 0.5, 0) ≈ { 0.5, 0.2 }
heading_error_enters_through_sine_and_cosine:
    Assert: Compute(Identity, FromPose(0, 0, 0.3), 0.5, 0) ≈ { 0.477668, 0.591040 }
error_is_expressed_in_the_robot_frame:
    Assert: TrackingError(FromPose(1, 1, π/2), FromPose(1, 2, π/2)) ≈ (1, 0, 0)
            Compute(FromPose(1, 1, π/2), FromPose(1, 2, π/2), 0.5, 0) ≈ { 1.5, 0 }
critically_damped_gains:
    Assert: CriticallyDamped(1, 4) == { 1, 4, 4 }
closed_loop_converges_with_a_non_increasing_lyapunov_function:
    Arrange: pose = FromPose(−0.3, 0.5, −0.4), reference starts at Identity moving at (0.5, 0, 0), dt = 0.01
    Act:     2000 steps: u = Compute(pose, ref, 0.5, 0); pose = pose * Exp((u.v, 0, u.ω), dt); ref = ref * Exp((0.5, 0, 0), dt)
    Assert:  ‖TrackingError‖ < 1e-4 at t = 20 s;  V(k+1) ≤ V(k) + 1e-6 at every step   # V(0) = 0.379470
```

## Reference vectors

- Gains `(1, 4, 4)`, `v_r = 0.5`: `eθ = 0.3` ⇒ `v = 0.5 cos 0.3 = 0.477668`, `ω = 0.5·4·sin 0.3 = 0.591040`.
- Closed loop from `(−0.3, 0.5, −0.4)`: error `≈ (6e-6, −1.4e-4, 2.5e-4)` at 10 s and `‖e‖ ≈ 3e-8` at 20 s (double).

## Edge cases

- `eθ = π` (reference behind and reversed): `v = −v_r + kx·ex`, bounded output.
- `v_r = 0`: `ey` has no effect on `ω` (documented limitation).
