# Pose Inverse Kinematics — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestPoseInverseKinematics : public ::testing::Test:
    DenavitHartenberg<float, 6>              puma{ pumaTable, Standard }     # PUMA-560, see M7 tests
    DhTaskJacobian<float, 6, 6>              arm{ puma }
    StrictMock<MockJacobianProvider<float, 6, 6>>  mock                      # scripted J and ToolPose
    PoseIkConfig<float, 6> config{ λ₀ = 0.05, w₀ = 0, ρ = 0.3, tol = 1e-4, maxStep = ∞, 100, ∓∞ }
    JointVector qTrue{ 0.3, −0.4, 0.5, 0.6, −0.7, 0.8 }
# each case below is a TEST_F(TestPoseInverseKinematics, <name>)
```

## Test cases (Arrange / Act / Assert)

```
round_trip_reachable_pose:
    Arrange: target = puma.Forward(qTrue); q0 = qTrue + 0.1
    Act:     r = PoseInverseKinematics{ arm, config }.Solve(target, q0)
    Assert:  r.converged; r.iterations ≤ 10; ‖W·PoseError(target, puma.Forward(r.q))‖ < 1e-4
damping_slows_but_does_not_bias:
    Arrange: λ₀ = 0.2, tol = 1e-5, maxIterations = 200
    Assert:  r.converged; ‖r.q − qTrue‖∞ < 1e-3; r.iterations > the λ₀ = 0.05 count
zero_error_is_immediate:
    Assert: Solve(puma.Forward(q0), q0) → iterations == 0, converged
final_error_is_measured_after_the_last_update:
    Arrange: maxIterations = 1, λ₀ = 0, ρ = 1; target = { Rz(0.1), (0.1, 0, 0) }
             mock.ToolPose → Identity (1st call), { Rz(0.08), (0.08, 0, 0) } (2nd call); mock.Jacobian → I₆
    Assert:  r.q ≈ q0 + (0.1,0,0,0,0,0.1); r.finalError ≈ 0.028284 (not 0.141421); converged == false
rotation_rows_are_weighted_in_the_step:
    Arrange: maxIterations = 1, λ₀ = 0.1, ρ = 0.1, target = { Rz(0.1), (0.1, 0, 0) };
             mock.ToolPose → Identity (twice); mock.Jacobian → I₆
    Assert:  r.q − q0 ≈ (0.0990099, 0, 0, 0, 0, 0.05)  [= wᵢ²/(wᵢ² + λ²)·eᵢ];  r.finalError ≈ 0.100499
adaptive_damping_vanishes_away_from_singularity:
    Arrange: w₀ = 0.5, λ₀ = 0.1, ρ = 1, maxIterations = 1; mock.Jacobian → I₆ (w = 1)
    Assert:  Δq == e exactly (λ = 0)
adaptive_damping_engages_near_singularity:
    Arrange: as above, mock.Jacobian → diag(1,1,1,1,1,0.1) (w = 0.1 ⇒ λ² = 0.0096), e = 0.01·𝟙
    Assert:  Δq ≈ (0.0099049 ×5, 0.0510204)
step_is_clamped:
    Arrange: maxStep = 0.1, mock J = I₆, e = (0.3, 0.4, 0, 0, 0, 0)
    Assert:  Δq ≈ (0.06, 0.08, 0, 0, 0, 0)
joint_limits_are_enforced:
    Arrange: upperLimits[0] = q0[0] + 0.05, mock J = I₆, e = (0.3, 0, 0, 0, 0, 0)
    Assert:  r.q[0] == upperLimits[0]
damping_survives_wrist_singularity:
    Arrange: q0 = (0.3, −0.4, 0.5, 0.6, 0, 0.8) (θ₅ = 0: wrist axes aligned, σ_min ≈ 2e-9);
             target = puma.Forward((0.3, −0.4, 0.5, 0.9, 0.3, 0.5)); w₀ = 1e-3, maxStep = 0.2
    Assert:  r.converged; every r.q component finite
unreachable_target_fails_gracefully:
    Arrange: target.p = (3, 0, 0)  (beyond reach ≈ 0.9 m)
    Assert:  !r.converged; r.finalError > tol; q finite
```

## Reference vectors

- PUMA-560 at `qTrue`: tool `p = (0.402407, −0.032586, 0.263519)`; from `qTrue + 0.1` (double):
  `λ = 0.05` reaches `1e-4` in 3 iterations; `λ = 0` / `0.05` / `0.2` reach `‖W e‖ < 1e-12` in 4 / 12 /
  64 iterations, all to `qTrue` within `1e-11` (damping is unbiased).
- Weighted manipulability `√det(W J Jᵀ W)` at `qTrue` with `ρ = 0.3` is `1.09e-3` — pick `w₀` per arm
  on that scale. Wrist-singular case above converges in 8 iterations (double).
- Stale-error scenario on the real arm: initial `‖W e‖ = 0.136739`, after one step `0.011815` — the
  result must report the latter.
- `+190°` about `ẑ` ⇒ rotation vector `−2.967060·ẑ` (short way, via M6).

## Edge cases

- Antipodal orientation (180°) ⇒ `PoseError` axis ambiguous (either sign valid, M6).
- Target at the reach boundary ⇒ slow but bounded convergence (adaptive damping engages).
- `Dof = 7` ⇒ same code; the minimum-norm step drifts in the null space — use M14 to steer it.
