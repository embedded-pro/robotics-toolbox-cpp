---
description: "Robotics Toolbox testing (float-only): TEST_F on float, StrictMock only, anonymous-namespace fixtures, no plain TEST(), no redundant cases, Arrange-Act-Assert. Canonical: AGENTS.md."
applyTo: "**/test/**"
---

# Robotics Toolbox Testing Guidelines

## File Structure

- Test files: `robotics/{domain}/test/Test{ComponentName}.cpp`
- CMake: tests added via `add_subdirectory(test)` with standard test target patterns

## Framework

- GoogleTest for assertions — **`TEST_F` on `float`** (no `TYPED_TEST`, no multi-type)
- GoogleMock (`testing::StrictMock<>`) only when needed
- No heap allocation in tests — same rules as production code
- **NEVER use plain `TEST()` macro** — cppcheck reports `syntaxError`

## Fixture Test Pattern (float)

```cpp
#include "robotics/kinematics/ForwardKinematics.hpp"
#include "numerical/math/Tolerance.hpp"
#include <gtest/gtest.h>

namespace
{
    class TestForwardKinematics
        : public ::testing::Test
    {
    protected:
        kinematics::ForwardKinematics<float, 2> fk{ math::Vector<float, 3>{ 0.8f, 0.0f, 0.0f } };
    };
}

TEST_F(TestForwardKinematics, elbow_at_ninety_degrees_places_tool_above_elbow)
{
    // Arrange, Act, Assert — EXPECT_NEAR(actual, expected, math::Tolerance<float>())
}
```

## Rules

- Fixture class and type aliases go inside anonymous `namespace {}`
- `TEST_F` macros go **outside** the anonymous namespace
- Include `<gtest/gtest.h>` (not `<gmock/gmock.h>`) unless gmock matchers are needed
- Use `testing::StrictMock<MockType>` for strict mock expectations
- **ONLY `StrictMock`**: Never use `testing::NiceMock<>` or bare mock instantiation — `NiceMock` silences unexpected-call warnings, masking test gaps; `StrictMock` enforces all interactions explicitly
- Test `float` only (single type) — no multi-type tests
- **No redundant tests** — implement exactly the spec's enumerated cases; no overlapping/extra cases
- Test numerical accuracy against known reference values (not just "doesn't crash")
- Pick the properties to assert from [`TESTING.md`](../../TESTING.md) — the per-family metric reference (accuracy, frequency/transient response, stability, boundaries, convergence, statistical consistency, conditioning)
- Test genuine edge cases (zero input, extreme values) without duplicating coverage
- One behavior per test — keep tests focused
- Use descriptive test names that explain the scenario
- Allman brace style and PascalCase naming apply to test code too

## TDD Approach

- **Clarify requirements first**: Before writing any code, define and document all use cases, inputs, outputs, and edge cases as test cases
- **Write tests before implementation**: Tests define the expected behavior; implementation exists only to satisfy the tests
- **Red-Green-Refactor cycle**: Write a failing test, make it pass with minimal code, then refactor while keeping tests green

## Coverage for Template Code

When `EMIL_ENABLE_COVERAGE` is set, template code needs explicit instantiation in a `.cpp` file that is compiled with coverage flags. Add to the header (guarded):

```cpp
#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
extern template class ForwardKinematics<float, 3>;
#endif
```

And in the matching `.cpp` file:

```cpp
#include "robotics/kinematics/ForwardKinematics.hpp"

namespace kinematics
{
    template class ForwardKinematics<float, 3>;
}
```
