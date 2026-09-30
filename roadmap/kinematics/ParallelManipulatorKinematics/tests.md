# Parallel Manipulator Kinematics — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture — Stewart–Gough

```cpp
class TestStewartGoughKinematics : public ::testing::Test:
    # leg k = 0..5, j = k/2, s = (k even ? −1 : +1)
    # bₖ = 1.0·(cos β, sin β, 0), β = 120°·j + s·15°;  pₖ = 0.6·(cos γ, sin γ, 0), γ = 120°·j + s·45°
    StewartGoughKinematics<float> hexapod{ geometry, { tol = 1e-5, λ = 0, 50 } }
    SE3Transform<float> home{ I, (0, 0, 0.8) }
    SE3Transform<float> tilted{ Rz(0.1)·Rx(0.05), (0.05, −0.03, 0.85) }
# each case below is a TEST_F(TestStewartGoughKinematics, <name>)
```

## Test cases — Stewart–Gough

```text
home_legs_are_equal:
    Assert: Inverse(home) ≈ 0.980189 for all six legs
raising_the_platform_lengthens_every_leg:
    Assert: Inverse({ I, (0, 0, 0.9) }) ≈ 1.063376 for all six legs
inverse_of_a_general_pose:
    Assert: Inverse(tilted) ≈ (0.955703, 1.037816, 1.049122, 1.083232, 0.981789, 1.042429)
leg_jacobian_matches_finite_difference:
    Arrange: twist (v; ω) = (0.1, −0.2, 0.05; 0.3, −0.1, 0.2), pose perturbed as p + εv, Exp(εω)·R
    Assert:  LegJacobian(tilted)·twist ≈ (Inverse(perturbed) − Inverse(tilted)) / ε   (ε = 1e-3, tol 1e-3)
forward_recovers_the_pose:
    Act:    r = Forward(Inverse(tilted), { Rz(0.15), (0, 0, 0.8) })
    Assert: r.converged; r.iterations ≤ 6; r.pose ≈ tilted (tol 1e-4)
assembly_mode_is_selected_by_the_guess:
    Arrange: L = Inverse(home)
    Assert:  Forward(L, { I, (0,0,0.7) }).pose.p ≈ (0,0,0.8);  Forward(L, { I, (0,0,−0.7) }).pose.p ≈ (0,0,−0.8)
singular_pose_step_stays_finite:
    Arrange: target { Rz(90°), (0,0,0.8) } (det LegJacobian = 0); λ = 1e-3; guess yaw 80°
    Assert:  pose finite; ‖Inverse(r.pose) − L‖ < 1e-4
```

## Fixture — Delta

```cpp
class TestDeltaKinematics : public ::testing::Test:
    DeltaKinematics<float> delta{ { R = 0.1, r = 0.03, rf = 0.2, re = 0.45 } }
# each case below is a TEST_F(TestDeltaKinematics, <name>)
```

## Test cases — Delta

```text
zero_angles_put_the_effector_on_the_axis:
    Assert: Forward((0,0,0)) ≈ (0, 0, −0.36)   (√(re² − (R − r + rf)²) = √(0.2025 − 0.0729))
inverse_of_known_points:
    Assert: Inverse((0.05, −0.03, −0.40)) ≈ (0.042468, 0.371178, 0.205749)
            Inverse((−0.08, 0.06, −0.32)) ≈ (0.195443, −0.538250, −0.084938)
inverse_forward_round_trip:
    Assert: Forward(Inverse(P)) ≈ P for P ∈ {(0.05,−0.03,−0.40), (−0.08,0.06,−0.32), (0.12,0.10,−0.45)} (tol 1e-5)
forward_inverse_round_trip:
    Assert: Inverse(Forward(θ)) ≈ θ over the grid θᵢ ∈ {−0.6, 0, 0.6, 1.2} (all 64 assemblable; tol 1e-4)
forearm_constraint_holds:
    Assert: ‖elbowᵢ − (P + Rz(φᵢ)·(r,0,0))‖ ≈ re for each arm at Inverse(P)
unreachable_point_has_no_solution:
    Assert: Inverse((0, 0, −0.9)) == nullopt
```

## Reference vectors

- Hexapod: `L₀ = √(1 + 0.36 − 1.2·cos 30° + 0.64) = 0.980189`; singular at yaw `90°`
  (`det J` from `−0.8417` at 0° through `0`), legs there `(1.183216, 1.612452) × 3`.
- Newton from the stated guess converges in 4 iterations (double); with `λ = 1e-3` near the
  singularity the leg residual reaches `7e-7` while the yaw is only fixed to `≈ 0.1°`.
- Delta: home `(0, 0, −0.36)`; `θ → Forward → Inverse` reproduces `θ` to `2e-15` (double) over 1000
  random `θ ∈ [−0.6, 1.2]³`; the minus root was the elbow-out root in all 46 265 sampled arm solves.

## Edge cases

- Stewart: wrong assembly-mode guess converges to a different valid pose (documented).
- Stewart: non-converging lengths ⇒ `converged == false`, best-effort pose, finite.
- Delta: `h² < 0` in `Forward` ⇒ nullopt (angles not assemblable).
- Delta: target on the reach boundary ⇒ `|c| = ρ`, single (tangent) solution; clamp `acos` argument.
