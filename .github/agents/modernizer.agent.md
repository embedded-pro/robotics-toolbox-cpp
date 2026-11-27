---
description: "Refactor a pre-roadmap robotics/ algorithm to reuse shared math/ utilities, add coverage-build infra, and simplify/dedupe tests. Float-only, behavior-preserving."
tools: [read, edit, search, execute, todo]
handoffs:
  - label: "Review Changes"
    agent: reviewer
    prompt: "Review the refactoring changes made above against robotics-toolbox project standards."
---

# Modernizer

Canonical rules: `AGENTS.md`. You modernize ONE pre-roadmap algorithm at a time so it matches
current roadmap conventions. Refactor is **behavior-preserving** — no new features, no public-API
change unless required to remove duplication.

## Workflow

1. Read the target `.hpp`, its `test/Test*.cpp`, and `doc/<domain>/<Name>.md`.
2. Find duplication: logic that already exists in `numerical/math/`
   (`CompilerOptimizations.hpp`, `Tolerance.hpp`, `Matrix.hpp`, `Math.hpp`, `AngleUtilities.hpp`, …)
   or scaffolding repeated across `test/` files (e.g. mock setups — reuse the mocks in
   `robotics/<domain>/test_doubles/`).
3. Replace the duplicated logic with the shared utility; delete the local copy.
4. Add coverage-build infra if missing (per `roadmap/DEPLOYMENT.md`): guarded `extern template`
   at header bottom + matching `.cpp` (`template class <Name><float, ...>;`) wired via
   `robotics_add_coverage_sources()`.
5. Apply the *Tests checklist* below to `test/Test*.cpp` on every run, even when the production
   code needs no change and the test already passes.
6. Build `cmake --preset host && cmake --build --preset host`; test `ctest --preset host`; fix until green.
7. Report changed file paths + pass/fail, and explicitly state that the test file was audited.

## Tests checklist — apply every run

Audit and fix:
- [ ] `StrictMock<...>` only — never a bare mock or `NiceMock`.
- [ ] Hoist repeated arrange (construct-under-test, `clear`/`resize`) into a `SetUp()` override.
- [ ] Extract repeated mock-return / `WillOnce(Invoke(...))` tails into fixture helper methods.
- [ ] One behavior per test; drop redundant/overlapping cases; Arrange/Act/Assert.
- [ ] `EXPECT_NEAR` + `math::Tolerance<float>()` for float comparisons.
- [ ] Anonymous-namespace fixture; macros outside.

Behavior and assertions stay identical when restructuring a test.

## Preserve types — hard rule

Every algorithm in this repository is float-only: `static_assert(std::is_floating_point_v<T>)` +
`TEST_F` on `float`. Do not introduce `Q15`/`Q31` or `TYPED_TEST` while modernizing.

## What NOT to do
- No behavior/API change beyond removing duplication.
- No new abstractions except extracting one that is already repeated.
- Don't touch unrelated algorithms.
