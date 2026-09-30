# Inverse Dynamics with an External Tool Wrench — Unit Test Plan (Pseudocode)

> GoogleTest · `TEST_F` (`float`) · `StrictMock` only · no heap.

## Fixture

```cpp
class TestRecursiveNewtonEuler : public ::testing::Test:     # extends the shipped fixture
    RecursiveNewtonEuler<float, 3> rnea3
    std::array<RevoluteJointLink<float>, 3> chain{ ... }     # skewed axes, tool offset (0.3, 0.05, 0)
# each case below is a TEST_F(TestRecursiveNewtonEuler, <name>)
```

## Test cases (Arrange / Act / Assert)

```text
zero_wrench_matches_shipped_overload:
    Assert: InverseDynamics(..., ToolWrench{0, 0, tool}) == InverseDynamics(...)
wrench_enters_as_minus_jacobian_transpose:
    Arrange: w = ((1, −2, 0.5); (0.1, 0, −0.2)), J from GeometricJacobian (M8) at the same tool point
    Assert:  τ(w) − τ(0) ≈ −Jᵀ·w
horizontal_link_holding_a_payload:
    Arrange: single y-axis rod length l, payload force (0, 0, −m_p·g) at the tip, static
    Assert:  τ = −(m l/2 + m_p l)·g   (both moments add)
pure_moment_projects_on_the_axis:
    Arrange: single z-axis link, w.moment = (0, 0, 2)
    Assert:  τ = −2
```

## Reference vectors

- Rod `m = 1, l = 1`, payload `0.5 kg`: `τ = −(0.5 + 0.5)·9.81 = −9.81 N·m`.

## Edge cases

- Tool offset zero ⇒ the force acts at the last joint origin (no moment arm from the offset).
