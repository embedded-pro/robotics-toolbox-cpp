# Self-Collision Checking — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```cpp
class TestSelfCollision : public ::testing::Test:
    SelfCollision<float, 4> model{ { 0.1, 0.08, 0.06, 0.05 } }
    # Planar(q): FrameChain<float, 4> of a planar arm with z-axis joints and link lengths (0.4, 0.8, 0.7, 0.5):
    #   jointOrigins[0] = 0, jointOrigins[k+1] = jointOrigins[k] + L_k·(cos θ_k, sin θ_k, 0), θ_k = q₀ + … + q_k;
    #   tool.translation = end of link 3
# each case below is a TEST_F(TestSelfCollision, <name>)
```

## Test cases (Arrange / Act / Assert)

```cpp
straight_arm_is_clear:
    Assert: Check(Planar(0, 0, 0, 0), 0) ⇒ minimumDistance ≈ 0.57, pair (1, 3), colliding false, checkedPairs == 3
folded_arm_collides_between_links_0_and_2:
    Assert: Check(Planar(0, 2.4, 2.4, 0.5), 0) ⇒ minimumDistance ≈ −0.018095, pair (0, 2), colliding true
allowing_a_pair_reports_the_next_closest:
    Arrange: Allow(0, 2)
    Assert:  Check(Planar(0, 2.4, 2.4, 0.5), 0) ⇒ minimumDistance ≈ 0.052945, pair (0, 3), colliding false, checkedPairs == 2
margin_turns_proximity_into_a_collision:
    Assert: Check(Planar(0, 2.6, 2.2, 1.0), 0) ⇒ 0.088331, not colliding;  Check(…, 0.1) ⇒ colliding
links_1_and_3_can_also_collide:
    Assert: Check(Planar(0.5, −1.0, −2.5, −2.0), 0) ⇒ minimumDistance ≈ −0.13, pair (1, 3), colliding true
adjacent_pairs_are_allowed_by_default:
    Assert: IsAllowed(0, 1) and IsAllowed(2, 3);  not IsAllowed(0, 2);  after Disallow(0, 1), checkedPairs == 4
```

## Reference vectors

- `Planar(0, 2.4, 2.4, 0.5)` joint origins: `(0,0,0), (0.4,0,0), (−0.189915, 0.540371, 0), (−0.128666, −0.156945, 0)`,
  tool `(0.148521, −0.573078, 0)`.
- Straight arm: links 1 and 3 are 0.7 apart along the axis; minus radii `0.08 + 0.05` ⇒ 0.57.

## Edge cases

- Every pair allowed ⇒ `minimumDistance = +∞`, `checkedPairs = 0`, not colliding.
- Zero-length link (coincident joint origins) ⇒ a sphere capsule; P1's degenerate branch handles it.
