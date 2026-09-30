# Continuum Kinematics — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```cpp
class TestContinuumKinematics : public ::testing::Test:
    ArcParameters<float> quarter{ π/2, 0, 1 }         # θ = π/2, radius 2/π
    ArcParameters<float> general{ 2.0, 0.7, 1.2 }     # θ = 2.4
    ContinuumKinematics<float, 2> twoSections{ { general, general } }
# each case below is a TEST_F(TestContinuumKinematics, <name>)
```

## Test cases (Arrange / Act / Assert)

```cpp
straight_section_is_pure_translation:
    Assert: SectionTransform({0, 0, 1}) ≈ { I, (0, 0, 1) }
quarter_circle_bend:
    Assert: SectionTransform(quarter).p ≈ (2/π)·(1, 0, 1) = (0.636620, 0, 0.636620); R ≈ Ry(π/2)
bending_plane_angle_rotates_tip:
    Assert: SectionTransform({π/2, π/2, 1}).p ≈ (0, 0.636620, 0.636620)
tip_distance_equals_chord:
    Assert: ‖SectionTransform(arc).p‖ ≈ 2·|sin(κs/2)|/|κ| for quarter (0.900316), general (0.932039),
            and {−1.5, 0.3, 0.8} (0.752857)
tip_tangent_matches_bend:
    Assert: SectionTransform(general).R·ẑ ≈ (cos 0.7·sin 2.4, sin 0.7·sin 2.4, cos 2.4) = (0.516623, 0.435145, −0.737394)
forward_composes_sections:
    Assert: twoSections.Forward() ≈ SectionTransform(general) * SectionTransform(general); p ≈ (0.348960, 0.293925, −0.498082)
inverse_recovers_arc_parameters:
    Assert: InverseSection(SectionTransform(a).p) ≈ a for a ∈ {quarter, general};
            {−1.5, 0.3, 0.8} ⇒ (1.5, 0.3 − π, 0.8)  (sign folded into φ)
near_zero_curvature_uses_series:
    Assert: SectionTransform({1e-9, 0.4, 1}) ≈ { I, (0, 0, 1) }; SectionTransform({1e-5, 0.4, 1}).p ≈ (4.605e-6, 1.947e-6, 1), finite
```

## Reference vectors

- Straight unit section ⇒ `(0, 0, 1)`, `R = I`.
- `κ = π/2, s = 1, φ = 0` ⇒ tip `(0.636620, 0, 0.636620)`, chord `2√2/π = 0.900316`.
- `κ = 2, φ = 0.7, s = 1.2` ⇒ tip `(0.664416, 0.559630, 0.337732)`, chord `sin 1.2 = 0.932039`.
- All closed-form tips agree with a 4000-step numerical integration of the tangent `R(s)·ẑ` to `1e-8`.

## Edge cases

- Full loop `θ = 2π` ⇒ tip at the base `(0, 0, 0)`; `InverseSection` is limited to `κs < 2π`.
- Negative `κ` ⇒ same arc as `(|κ|, φ + π)`.
- Straight tip in `InverseSection` ⇒ `κ = 0`, `φ = 0`, `s = z`.
- Multi-section inverse ⇒ iterative (M13), not here.
