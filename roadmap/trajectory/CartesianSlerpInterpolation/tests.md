# Cartesian Path + Orientation (SLERP) Interpolation — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```cpp
class TestCartesianSlerp : public ::testing::Test:
    using Law = PolynomialTrajectory<float>
    Law linearLaw{ { .q0 = 0, .qf = 1, .v0 = 1, .vf = 1 }, 1.0f, Degree::Cubic }   # s(t) = t, ṡ = 1
    kinematics::SE3Transform<float> start{ .R = I,        .p = {0,0,0} }
    kinematics::SE3Transform<float> goal { .R = Rz(90°),  .p = {1,0,0} }
    CartesianSlerpInterpolation<float, Law> path{ start, goal, linearLaw }
# each case below is a TEST_F(TestCartesianSlerp, <name>)
```

## Test cases (Arrange / Act / Assert)

```text
endpoints_match_start_and_goal:
    Assert: Sample(0).pose ≈ start  and  Sample(tf).pose ≈ goal   (R and p element-wise)

position_is_straight_line:
    Assert:  Sample(tf/2).pose.p ≈ (0.5, 0, 0)

rotation_stays_orthonormal:
    Assert: Sample(tf/2).pose.R · Sample(tf/2).pose.Rᵀ ≈ I, det ≈ +1

slerp_midpoint_is_half_angle:
    Assert:  Sample(tf/2).pose.R ≈ Rz(45°)

shortest_path_takes_short_arc:
    Arrange: goal.R = Rz(270°) (= Rz(−90°))
    Assert:  Sample(tf/2).pose.R ≈ Rz(−45°)  and  twist ω ≈ (0, 0, −π/2)

near_parallel_orientation_is_finite:
    Arrange: goal.R = Rz(0.01°)
    Assert:  no NaN/inf in pose or twist; pose.R ≈ I; ω_z ≈ 0.01°·π/180 (ṡ = 1)

constant_angular_rate_for_linear_time_law:
    Assert:  angle(start.R, Sample(t).pose.R) ≈ t·90° for t ∈ {0.25, 0.5, 0.75}

twist_is_sdot_times_path_tangent:
    Arrange: quintic law { .q0 = 0, .qf = 1 }, tf = 2  ⇒ ṡ(1) = 0.9375
    Assert:  Sample(1).twist ≈ (0.9375, 0, 0; 0, 0, 0.9375·π/2 = 1.472622)
             i.e. v = ṡ·(p1 − p0), |ω| = ṡ·θ with θ = 90°, axis ẑ; Sample(0).twist ≈ 0

sample_clamps_outside_domain:
    Assert: Sample(-1).pose ≈ start  and  Sample(tf+1).pose ≈ goal
```

## Reference vectors

- SLERP of `I → Rz(90°)` at `s = 0.5` ⇒ `Rz(45°)` (quaternion `(0.923880, 0, 0, 0.382683)`).
- Straight-line position `p(s) = p0 + s(p1 − p0)`; midpoint of `(0,0,0)→(1,0,0)` is `(0.5,0,0)`.
- Twist `(v; ω) = ṡ·(p1 − p0; φ·k)`, `φ·k` = rotation vector of `R1·R0ᵀ`; for `Rz(90°)` and `ṡ = 1.3`,
  `ω = (0, 0, 2.042035)` (matches finite differences of `R(s(t))`).
- Rest-to-rest quintic `0 → 1`, `tf = 2`: `ṡ(tf/2) = (15/8)/tf = 0.9375`.

## Edge cases

- Identical start and goal orientation ⇒ constant rotation, `ω = 0`, no division by zero.
- Antipodal quaternions (`dot ≈ −1`, from the sign ambiguity of `FromRotationMatrix`) ⇒ sign fix,
  identical rotation, `ω = 0`.
- 180° rotation ⇒ either arc is shortest; `|ω| = ṡ·π`, pose and twist stay consistent.
- Zero-length Cartesian move with pure rotation ⇒ position constant, `v = 0`, orientation still slerps.
