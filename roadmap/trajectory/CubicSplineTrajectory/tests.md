# Multi-Joint Cubic Spline Trajectory — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```
class TestCubicSplineTrajectory : public ::testing::Test:
    using Spline = CubicSplineTrajectory<float, 3, 8>
    std::array<float, 8>              times{ 0, 1, 2.5, 3, 4.5 }        # K = 5 used
    std::array<math::Vector<float,3>, 8> positions{ (0, 0.5, 0), (1, 0.2, 0), (−0.5, 0.2, 0),
                                                    (0.3, −0.4, 0), (2, 0, 0) }
    Spline spline
    void SetUp(): ASSERT_TRUE(spline.Plan(times, positions, 5))      # clamped, zero end velocities
# each case below is a TEST_F(TestCubicSplineTrajectory, <name>)
```

## Test cases (Arrange / Act / Assert)

```
passes_through_every_knot:
    Assert: Sample(t_k).position ≈ q_k for k = 0..4, every joint

velocity_and_acceleration_continuous_at_interior_knots:
    Assert: for k = 1..3: Sample(t_k − δ) ≈ Sample(t_k + δ) in velocity and acceleration
            (δ = 1e-4, tol 1e-2 ≥ |jerk|·2δ), every joint

knot_accelerations_match_reference:
    Assert: joint 0: Sample(t_k).acceleration ≈ (5.665169, −5.330337, 5.991011, −0.737079, −1.898127);
            joint 2 (all knots 0) ≡ 0

clamped_end_velocities_are_honoured:
    Arrange: Plan(..., { Clamped, startVelocity = (0.4, 0, 0), endVelocity = (−0.7, 0, 0) })
    Assert:  Sample(0).velocity_0 ≈ 0.4, Sample(4.5).velocity_0 ≈ −0.7;
             Sample(0).acceleration_0 ≈ 4.296629, Sample(4.5).acceleration_0 ≈ −3.637453
             (fixture default: both end velocities ≈ 0)

natural_ends_have_zero_acceleration:
    Arrange: Plan(..., { Natural })
    Assert:  Sample(0).acceleration ≈ 0, Sample(4.5).acceleration ≈ 0;
             joint 0 end velocities ≈ 1.680287 and 0.783154

collinear_knots_reproduce_straight_line:
    Arrange: times {0, 0.5, 1, 1.5, 2}, q_k = 1 + 0.8·t_k (all joints), Clamped with v = 0.8
    Assert:  Sample(0.7) ≈ { 1.56, 0.8, 0 };  acceleration ≈ 0 at every sampled t

two_knots_reduce_to_cubic_polynomial:
    Arrange: times {0, 2}, q {0, 1}, Clamped startVelocity 0.5, endVelocity −0.5;
             PolynomialTrajectory<float>{ { .q0 = 0, .qf = 1, .v0 = 0.5, .vf = −0.5 }, 2, Degree::Cubic }
    Assert:  position, velocity, acceleration equal at t ∈ {0.3, 1.0, 1.7}; knot accelerations (1, −2)

three_knot_reference_values:
    Arrange: times {0, 1, 2}, q {0, 1, 0}, clamped zero
    Assert:  Sample(0.5) ≈ { 0.5, 1.5, 0 };  knot accelerations (6, −6, 6);  Sample(1).velocity ≈ 0

sample_clamps_outside_domain:
    Assert: Sample(−1) == Sample(0)  and  Sample(5.5) == Sample(4.5)

plan_rejects_invalid_knots:
    Assert: Plan(count = 1) == false;  Plan(count = 9) == false;  times {0, 1, 1, 2} == false
```

## Reference vectors

Computed with python3 (tridiagonal system + Thomas solve, checked against the defining conditions).

- Fixture joint 0 (`t = 0, 1, 2.5, 3, 4.5`; `q = 0, 1, −0.5, 0.3, 2`):
  - clamped zero ⇒ `M = (5.665169, −5.330337, 5.991011, −0.737079, −1.898127)`;
  - clamped `(0.4, −0.7)` ⇒ `M = (4.296629, −4.993258, 5.779775, −0.058427, −3.637453)`;
  - natural ⇒ `M = (0, −4.081720, 5.605735, −1.400717, 0)`, end velocities `1.680287`, `0.783154`.
- Fixture joint 1 (`q = 0.5, 0.2, 0.2, −0.4, 0`), clamped zero ⇒
  `M = (−1.666292, 1.532584, −2.797753, 3.384270, −2.225468)`.
- Three knots `(0,0), (1,1), (2,0)`, clamped zero ⇒ `M = (6, −6, 6)`, `q(0.5) = 0.5`, `v(0.5) = 1.5`
  (first interval is the rest-to-rest cubic `3τ² − 2τ³`); natural ⇒ `M = (0, −3, 0)`, `q(0.5) = 0.6875`.
- Two knots, `v0 = 0.5, vf = −0.5, tf = 2` ⇒ `M = (2a2, 2a2 + 6a3·tf) = (1, −2)`; rest-to-rest
  `q0=0, qf=1, tf=2` ⇒ `q(1) = 0.5`, `v(1) = 0.75` (same as PolynomialTrajectory).
- Collinear knots with matching clamped velocity ⇒ right-hand side ≡ 0 ⇒ `M ≡ 0`.

## Edge cases

- `K = 2`, Natural ⇒ `M = 0`: straight line at constant velocity `(q1 − q0)/h` (end velocities free).
- Unequal spacing (fixture) — the formulation never assumes uniform `h`.
- Near-coincident knot times (`h ≤ ε`) ⇒ rejected by `Plan`, no division blow-up.
- `K = MaxPoints` (8) ⇒ accepted; no out-of-bounds access in the binary search at `t = t_{K−1}`.
