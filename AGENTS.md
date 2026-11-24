# robotics-toolbox-cpp — Agent Rules

<!-- shared:begin 0.4.0 sha256:9aee198041110762 -->
Canonical rules for Claude, Copilot and sub-agents. Sections marked *(shared)* come from embedded-pro/embedded-agent-kit:
edit them there, not here. *(repo)* sections belong to this repository. *Exceptions* overrides a shared rule.

## Behaviour *(shared)*

- **Be as terse as possible.** No preamble, summary or narration. Report file paths and pass/fail.
- Everything you write — code, docs, agent files, PR text — uses the fewest words that stay unambiguous.
- Prose (docs, comments, agent files, commit and PR text) follows ASD-STE100 Simplified Technical English:
  one word, one meaning; active voice; one instruction per sentence; sentences ≤ 20 words (procedures) or ≤ 25 (descriptions).
- Ambiguous request: ask. Clear request: act.
- Do only what was asked. No unrelated refactors, no extra features.
- Batch reads, don't re-read, prefer targeted edits.
- Never change a shared section. Add a deviation to *Exceptions* only when the user approves it.

## Workflow *(shared)*

1. Multi-file or architectural change: update the design document first. Get approval before you write code.
2. Read *Domain* and *Build*. Read each `.github/instructions/*.instructions.md` whose `applyTo` matches a file you change.
3. Find the nearest existing pattern. Follow it.
4. Per behaviour: write a failing test, write the minimum code that passes it, refactor with all tests green.
5. Add each new file to its `CMakeLists.txt`.
6. Update the documentation for each behavioural change.
7. Run the *Build* commands until build and tests pass. Never disable, skip or weaken a test to get green.
8. Non-trivial change: run the `reviewer` agent, or ask the user to run it. Fix each CRITICAL and WARNING finding.
9. Report changed files, build pass/fail, test pass/fail.

## Memory *(shared)*

Scope: production code. Tests and host tools may use the heap.

- Forbidden: `new`/`delete`/`malloc`/`free`, `make_unique`/`make_shared`,
  `std::vector`/`string`/`deque`/`list`/`map`/`set`, recursion.
- Use: `infra::BoundedVector<T>::WithMaxSize<N>`, `infra::BoundedString::WithStorage<N>`,
  `infra::BoundedDeque`, `infra::BoundedList`/`IntrusiveList`, `std::array`, `std::optional`, `std::variant`.

## Interfaces & errors *(shared)*

- No virtual destructors: each adds a deleting destructor and thunks to flash.
  Interface = pure virtual methods + `protected: ~I() = default;` (non-virtual).
- Public `virtual ~Base() = default;` only on a base destroyed through a base pointer:
  `delete`/`std::unique_ptr<Base>`, `infra::PolymorphicVariant<Base, ...>`, `infra::OptionalForPolymorphicObjects<Base, ...>`.
  Put it on that base, not on every interface.
- Not needed for `std::variant`, `std::optional<Concrete>`, `infra::SharedPtr`: they destroy the concrete type.
- No exceptions. `std::optional`, `bool` or status enum; `really_assert` for preconditions. Never swallow an error.
- Constructor injection; no global state.

## Optimisation *(shared)*

- No `#pragma GCC optimize`, no `optimize` attribute: GCC won't inline across differing options.
  The consumer sets flags per translation unit.
- `OPTIMIZE_FOR_SPEED` only on hot functions defined in a header.
- Hot paths: no heap, no virtual dispatch, no blocking.

## Style *(shared)*

- `.clang-format` is authoritative. Allman, 4 spaces, `{}` init.
- PascalCase types/methods; camelCase members and enum values; lowercase namespaces.
  No `I` prefix; implementations end in `Impl`.
- `#pragma once` only.
- Fixed-width integers. `const`/`constexpr`-correct. Functions ≤ 30 lines (hard limit 50).
- Comments: non-obvious *why* only, one line. No `TODO`/`FIXME`, commented-out code, or restating the code.
  Fix or delete a comment the change makes stale.

## Testing *(shared)*

- `TEST_F` for fixtures, `TYPED_TEST` for types, plain `TEST()` for stateless checks.
- `StrictMock` only. `EXPECT_NEAR` with explicit tolerance. Fixture in an anonymous namespace, macros outside.
- Heap allowed in tests.

## Documentation *(shared)*

- Update docs before or with any behavioural change.
- One fact, one place: link, never restate.
- Visuals: Mermaid or ASCII.
<!-- shared:end -->

## Domain *(repo)*

- Robotics algorithms for embedded targets: manipulator kinematics, dynamics, trajectories and control.
  `ROADMAP.md` lists the planned families (mobile, aerial, legged, marine).
- Source root: `robotics/`. Namespaces: `kinematics`, `dynamics`, `trajectory`, `controllers`.
  Planned: `mobile`, `aerial`, `legged`, `marine`, `grasping`, `planning`, `localization`, `calibration`, `safety`.
- `math`, `solvers` and other shared primitives come from numerical-toolbox-cpp through `FetchContent`, included as
  `numerical/<domain>/…`. A generic algorithm goes there, not here.
- `simulator/` is a host-only Qt tool. *Memory* does not apply to it.
- Float only: `template<typename T>` with `static_assert(std::is_floating_point_v<T>)`. Instantiate and test `float`.
  No `Q15`/`Q31`. Use `std::numbers::pi_v<float>`, never a literal.
- `OPTIMIZE_FOR_SPEED` on `Compute`, `Filter`, `Calculate`, `Solve`, `Update` and `Step`. Embedded consumers build
  with `-O2`/`-O3 -ffast-math -fno-finite-math-only`; check finiteness with `math::IsFinite`.
- Hold the concrete type and pass `I&`. Never own an object through an interface pointer.
- 6-vectors put the linear part first: twists `(v; ω)`, wrenches `(f; n)`, Jacobian linear rows first.
  Featherstone `(ω; v)` order stays inside the recursive dynamics algorithms.
- Orientation error: SE(3) logarithm (`PoseError`), never angle subtraction.
- Tool frame and base mounting offset are explicit kinematic parameters, never derived from a center of mass.
- Controllers get dynamics and Jacobians through small interfaces (`EulerLagrangeDynamics`, `InverseDynamicsModel`,
  `JacobianProvider`). Hard real-time code may bind the concrete types.
- Tests: `TEST_F` on `float`, no `TYPED_TEST`. `EXPECT_NEAR` with `math::Tolerance<float>()`. Exactly the cases of
  the spec. Spatial fixtures use non-parallel axes and gravity across the joint axes. Properties to assert:
  `TESTING.md`. Coverage ≥ 90 % (SonarQube gate).
- CMake: `robotics_add_header_library(<target>)`, `robotics_add_coverage_sources(<target> <Name>.cpp)`,
  `${ROBOTICS_VISIBILITY}`. Coverage `.cpp`: `template class <Name><float, ...>;`; header bottom:
  `extern template` inside `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD`.
- Docs: design-first `doc/<domain>/<Name>.md` per `doc/TEMPLATE.md`: math, behaviour, complexity, pitfalls.
  No code, class names or usage. Add new algorithms to `doc/<domain>/README.md`.
- Deploy a roadmap spec: `roadmap/DEPLOYMENT.md` (`algo-implementer` skill). Write tests: `unit-tester` skill.
  Bring an old algorithm up to date: `modernizer` agent.

## Build *(repo)*

```bash
cmake --preset host && cmake --build --preset host
ctest --preset host
```

`host` builds the Qt simulator. Without Qt6, use the `host-single-Debug` configure, build and test presets.

## Exceptions *(repo)*

- No plain `TEST()`: MegaLinter's cppcheck reports a syntax error on it. Use `TEST_F`.
