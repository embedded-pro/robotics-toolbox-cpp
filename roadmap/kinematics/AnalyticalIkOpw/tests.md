# Analytical IK (OPW) — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestAnalyticalIkOpw : public ::testing::Test:
    # KUKA KR6 R700 sixx
    OpwParameters<float> kr6{ a1 = 0.025, a2 = −0.035, b = 0, c1 = 0.400, c2 = 0.315, c3 = 0.365, c4 = 0.080,
                              offsets = (0, −π/2, 0, 0, 0, 0), signs = (−1, 1, 1, −1, 1, −1) }
    AnalyticalIkOpw<float> ik{ kr6 }
    JointVector qTrue{ 0.3, −1.0, 1.2, 0.5, 0.6, 0.7 }
# each case below is a TEST_F(TestAnalyticalIkOpw, <name>)
```

## Test cases (Arrange / Act / Assert)

```
forward_matches_elementary_chain:
    Assert: ik.Forward(q) ≈ Rz(θ1)·Trans(a1,b,c1)·Ry(θ2)·Trans(0,0,c2)·Ry(θ3)·Trans(a2,0,c3)
                          ·Rz(θ4)·Ry(θ5)·Rz(θ6)·Trans(0,0,c4)  (θ = signs⊙q − offsets), for qTrue and q = 0
home_flange_pose:
    Assert: ik.Forward(0) ≈ { [[0,0,1],[0,1,0],[−1,0,0]], (0.785, 0, 0.435) }
every_solution_reproduces_target:
    Arrange: target = ik.Forward(qTrue)
    Act:     sols = ik.Solve(target)
    Assert:  all 8 slots valid; Forward(sols[k].q) ≈ target (position and R, tol 1e-4)
finds_the_known_configuration:
    Assert: slot 0 ≈ qTrue (mod 2π)
wrist_flip_pairs:
    Assert: slot j+4 = slot j with q4 + π, −θ5, q6 − π (in model angles), for j = 0..3
random_poses_round_trip:
    Arrange: 200 deterministic pseudo-random q ∈ [−π, π]⁶ (fixed LCG seed, std::array)
    Assert:  every valid slot reproduces Forward(q) (tol 1e-3); some slot ≈ q (mod 2π, tol 1e-3)
general_offsets_signs_and_lateral_offset:
    Arrange: a1=0.10 a2=−0.05 b=0.07 c1=0.50 c2=0.60 c3=0.55 c4=0.10,
             offsets=(0.1,−0.2,0.3,0,0.4,−0.5), signs=(1,−1,1,1,−1,1)
    Assert:  as random_poses_round_trip (exercises b ≠ 0 and every offset/sign path)
wrist_singularity_keeps_the_sum:
    Arrange: q = (0.3, −1.0, 1.2, 0.5, 0, 0.7); target = Forward(q)
    Assert:  slot 0 = (0.3, −1.0, 1.2, 0, 0, 1.2), wristSingular; slot 4 empty; Forward(slot 0) ≈ target
stretched_arm_elbows_coincide:
    Arrange: q = (0.3, −0.3, −ψ3 = 0.095598, 0.5, 0.6, 0.7)
    Assert:  slots 0 and 1 agree to 1e-3; both reproduce the target (tol 1e-3)
unreachable_target_returns_no_solution:
    Arrange: flange at (1.5, 0, 0.4)
    Assert:  all 8 slots empty
```

## Reference vectors

- `Forward(qTrue)`: `p = (0.582764, −0.202939, 0.574882)`, `R = [[−0.707382, −0.375709, 0.598710],
  [−0.689753, 0.551987, −0.468563], [−0.154437, −0.744415, −0.649612]]`.
- The 8 solutions for that target (joint space, wrapped):

  | slot | q1 | q2 | q3 | q4 | q5 | q6 |
  |---|---|---|---|---|---|---|
  | 0 | 0.3 | −1.0 | 1.2 | 0.5 | 0.6 | 0.7 |
  | 1 | 0.3 | 0.1977 | −1.0088 | 0.2742 | 1.5525 | 1.1184 |
  | 2 | −2.8416 | 3.0760 | 0.9021 | −2.8675 | 1.5770 | 1.1253 |
  | 3 | −2.8416 | −2.3360 | −0.7109 | −2.7791 | 0.8687 | 0.8834 |
  | 4 | 0.3 | −1.0 | 1.2 | −2.6416 | −0.6 | −2.4416 |
  | 5 | 0.3 | 0.1977 | −1.0088 | −2.8674 | −1.5525 | −2.0232 |
  | 6 | −2.8416 | 3.0760 | 0.9021 | 0.2741 | −1.5770 | −2.0163 |
  | 7 | −2.8416 | −2.3360 | −0.7109 | 0.3625 | −0.8687 | −2.2582 |

- Wrist-singular target `p = (0.609771, −0.188624, 0.610958)`: 7 slots valid (slot 4 empty).
- Verified in double over 2000 random poses each for KR6 R700, ABB IRB2400 (a1=0.100, a2=−0.135,
  c1=0.615, c2=0.705, c3=0.755, c4=0.085, offset₃ = −π/2), Fanuc R2000iB (0.720, −0.225, 0.600, 1.075,
  1.280, 0.235, offset₃ = −π/2) and the `b ≠ 0` set: max pose error `2.3e-12`, the true `q` always found,
  8 or 4 solutions per pose. A `float32`-emulated solver (every operation rounded) over 200 random
  KR6 poses: worst pose error `4.3e-5`, worst recovery of the true `q` `2.7e-4` rad; at `qTrue` `3.7e-7`.

## Edge cases

- `|acos argument| > 1` ⇒ that arm branch (and its wrist flip) empty.
- Wrist centre inside the `b`-cylinder ⇒ every slot empty.
- Shoulder singularity (`Cx = Cy = 0`) ⇒ `θ1 = 0` returned, solutions still valid.
- `θ5 = ±π` ⇒ singular branch with `θ4 − θ6` fixed.
- Angle wrap ⇒ compare modulo `2π`.
