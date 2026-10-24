# RRT-Connect in Joint Space with a Fixed Node Pool — Implementation Pseudocode

> Roadmap ref: #P6 (★★★★☆) · Target: `robotics/planning` · Namespace `planning` · Type: `float` (templated on `T`, instantiated for `float` only)

Bidirectional rapidly-exploring random trees (Kuffner & LaValle, 2000) for collision-free joint-space
paths. One tree grows from the start and one from the goal. Each iteration extends one tree one step
towards a random sample, then greedily connects the other tree to the new node, and the trees swap roles.
Nodes live in compile-time pools. The sampler, the configuration validity test (typically P4 plus
environment checks with P1/P5) and the shortcutting randomness are all injected, so a seeded run is exactly
reproducible.

## Data structures

```cpp
template<typename T>                            # static_assert(std::is_floating_point_v<T>); instantiated for float
struct RrtConnectParameters:
    T stepSize                                  # maximum joint-space extension per step (rad)
    T collisionResolution                       # edge validity: checked every ≤ this distance
    std::size_t maxIterations

enum class PlanStatus : std::uint8_t { found, startInCollision, goalInCollision, poolExhausted, iterationLimit }

template<typename T, std::size_t Dof, std::size_t MaxNodes>     # static_assert(MaxNodes ≤ 65535)
class RrtConnect:
    using JointVector = math::Vector<T, Dof>
    struct Tree: std::array<JointVector, MaxNodes> configuration;  std::array<std::uint16_t, MaxNodes> parent;  std::size_t size
    Tree trees[2]                               # [0] rooted at start, [1] at goal
    std::array<JointVector, 2·MaxNodes> path;  std::size_t pathLength
    std::size_t iterations
    RrtConnectParameters<T> parameters
```

## Interface

```cpp
explicit RrtConnect(const RrtConnectParameters<T>& parameters)
template<class Sample, class IsFree>
PlanStatus Plan(const JointVector& start, const JointVector& goal, Sample&& sample, IsFree&& isFree)   # sample() ⇒ JointVector
template<class Uniform, class IsFree>
void Shortcut(std::size_t attempts, Uniform&& uniform, IsFree&& isFree)                              # uniform() ⇒ [0, 1)
std::span<const JointVector> Path() const;  std::size_t Iterations() const;  std::size_t NodeCount() const
```

## Algorithm (pseudocode)

```text
function EdgeFree(a, b):                         # n = ⌈‖b − a‖ / resolution⌉ interior checks plus b
    return isFree(a + (b − a)·k/n) for k = 1 … n

function Extend(tree, x):                        # one step towards x
    i = argmin_j ‖tree.q_j − x‖                  # linear scan (bounded pool)
    d = x − tree.q_i;  L = ‖d‖;  q_new = (L ≤ stepSize) ? x : tree.q_i + d·stepSize/L
    if not EdgeFree(tree.q_i, q_new): return trapped
    if tree.size == MaxNodes: return full
    append q_new with parent i;  return (L ≤ stepSize) ? reached : advanced

function Connect(tree, x):  repeat Extend(tree, x) while it returns advanced

function Plan(start, goal, sample, isFree):
    if not isFree(start): return startInCollision;  if not isFree(goal): return goalInCollision
    if EdgeFree(start, goal): path = [start, goal]; return found                 # trivial case, no sampling
    reset trees;  A = tree 0 (start), B = tree 1 (goal)
    for iterations = 1 … maxIterations:
        status = Extend(A, sample())
        if status == full: return poolExhausted
        if status != trapped:
            status = Connect(B, A.newest)
            if status == full: return poolExhausted
            if status == reached: path = branch(A to its root) joined with branch(B to its root),
                                         oriented start → goal;  return found
        swap(A, B)
    return iterationLimit

function Shortcut(attempts, uniform, isFree):    # random shortcutting: drop interior waypoints a straight edge can bypass
    repeat attempts times:
        i = ⌊uniform·len⌋;  j = ⌊uniform·len⌋;  order so i < j;  if j − i < 2: continue
        if EdgeFree(path[i], path[j]): remove path[i+1 … j−1]
```

## Complexity & memory

- Each `Extend` is `O(n)` for the nearest-neighbour scan over `n` nodes plus `O(L/resolution)` validity checks.
  The validity checks (forward kinematics plus P4 and environment queries) usually dominate.
- Memory: two trees of `MaxNodes·(Dof·sizeof(T) + 2)` bytes plus a `2·MaxNodes·Dof` path buffer. Dof 6 with
  1000 nodes is about 100 KB in `float`. No heap.

## Numerical / embedded notes

- **Sampler independence (normative).** The trees alternate every iteration, so the sampler must not
  correlate consecutive draws. A base-2 Halton sequence consumed in order hands every odd sample to one half
  of the space. In the validation it fed each tree samples from the far side of the wall, and the trees
  stalled at 19 nodes after 2000 iterations. A seeded PRNG (xorshift32 in the tests), or a low-discrepancy
  sequence drawn separately per tree, avoids this.
- **Resolution.** Edge checks are discrete: choose `collisionResolution` below the thinnest obstacle as
  seen in joint space, scaled by the farthest link's lever arm.
- Paths from RRT-Connect are feasible but long and jagged (11.5 against a 7.5 lower bound in the test).
  `Shortcut` brings them close to optimal (8.1) at the cost of extra validity checks; time-parameterize the
  result with M27 or F2.
- The metric is unweighted Euclidean in joint space. Scale joints by their lever arm if one joint dominates
  Cartesian motion.
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Header: `robotics/planning/RrtConnect.hpp` — `#pragma once`, and `extern template class RrtConnect<float, 6, 1024>;`
  under `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD` (the template members are instantiated by the tests).
- Coverage: `robotics/planning/RrtConnect.cpp` → `template class RrtConnect<float, 6, 1024>;`
- Test: `robotics/planning/test/TestRrtConnect.cpp`
- Doc: `doc/planning/RrtConnect.md` (per `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources(robotics.planning ...)`; `.cpp` → `robotics_add_coverage_sources`;
  test → `robotics.planning_test`.
- Depends on: nothing at compile time; P4 (and P1/P5) supply `isFree` in applications.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
