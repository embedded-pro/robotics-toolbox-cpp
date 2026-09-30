# Time-Optimal Path Parameterization (TOPP-RA) — Implementation Pseudocode

> Roadmap ref: #M27 (Tier 5) · Target: `robotics/trajectory` · Namespace `trajectory` · Type: `float` (templated on `T`, instantiated for `float` only)

Reachability-analysis TOPP (H. Pham, Q.-C. Pham, IEEE T-RO 34(3), 2018). Planning-time only; the result
is streamed with `Sample`.

## Data structures

`JointTrajectoryState<T, Dof>` comes from `robotics/trajectory/TrajectoryTypes.hpp` (canonical
definition: PolynomialTrajectory spec → Data structures).

```cpp
template<typename T, std::size_t Dof> using JointVector = math::Vector<T, Dof>

template<typename T, std::size_t Dof>        # static_assert(std::is_floating_point_v<T>); instantiated for float
class PathGeometry:                          # DI, virtual ~PathGeometry() = default — fixed path q(s), s ∈ [0, 1]
    virtual JointVector Position(T s) const = 0           # q(s)
    virtual JointVector FirstDerivative(T s) const = 0    # q'(s)
    virtual JointVector SecondDerivative(T s) const = 0   # q''(s)

template<typename T, std::size_t Dof>
struct PathCoefficients:                     # joint torque along the path: τ = a·s̈ + b·ṡ² + c
    JointVector a, b, c

template<typename T, std::size_t Dof>
struct JointBounds:
    JointVector torqueMin, torqueMax         # τmin < τmax
    JointVector velocityMax                  # q̇max > 0

template<typename T, std::size_t Dof>
class JointLimits:                           # DI, virtual ~JointLimits() = default
    virtual PathCoefficients<T, Dof> Coefficients(T s) const = 0
    virtual const JointBounds<T, Dof>& Bounds() const = 0

template<typename T, std::size_t Dof>        # production JointLimits, built on M29
class InverseDynamicsJointLimits : public JointLimits<T, Dof>:
    const PathGeometry<T, Dof>&                    path
    const dynamics::InverseDynamicsModel<T, Dof>&  dynamics      # e.g. ChainDynamicsModel<T, Dof>
    JointBounds<T, Dof>                            bounds

template<typename T>
struct StageRow:                             # one linear constraint α·x + β·u ≤ γ  (x = ṡ², u = s̈)
    T alpha, beta, gamma

template<typename T, std::size_t Dof, std::size_t Grid>   # static_assert(Grid >= 3); gridpoints s_0..s_N, N = Grid − 1 stages
class TimeOptimalPathParameterization:
    const PathGeometry<T, Dof>&  path
    const JointLimits<T, Dof>&   limits
    array<PathCoefficients<T, Dof>, Grid> coefficients     # a_i, b_i, c_i at s_i
    array<T, Grid> xMax                      # velocity bound on x at s_i
    array<T, Grid> lo, hi                    # controllable sets K_i = [lo_i, hi_i]
    array<T, Grid> sDot                      # sqrt(x_i) of the chosen profile
    array<T, Grid> u                         # constant s̈ on stage i (entry N unused)
    array<T, Grid> stageStartTime            # time at s_i; entry N = TotalTime
    bool parameterized
```

## Interface

```cpp
TimeOptimalPathParameterization(const PathGeometry<T, Dof>& path, const JointLimits<T, Dof>& limits)
bool  Parameterize()                                         # false if infeasible
std::optional<JointTrajectoryState<T, Dof>> Sample(T t) const  # nullopt until Parameterize() succeeds
T     TotalTime() const

InverseDynamicsJointLimits(const PathGeometry<T, Dof>&, const dynamics::InverseDynamicsModel<T, Dof>&,
                           const JointBounds<T, Dof>&)
```

## Algorithm (pseudocode)

```cpp
# Along the path q̇ = q'·ṡ, q̈ = q'·u + q''·x, so every joint torque is affine in (x, u):
#   τ(s) = a(s)·u + b(s)·x + c(s),   a = M q',  b = M q'' + C(q,q') q',  c = g(q)
# Grid s_i = i/N, Δ_i = s_{i+1} − s_i (uniform 1/N). Stage i holds u_i constant on [s_i, s_{i+1}]:
#   x_{i+1} = x_i + 2Δ_i·u_i

function InverseDynamicsJointLimits::Coefficients(s):   # three O(n) RNEA passes (M29 includes gravity)
    q = path.Position(s);  q1 = path.FirstDerivative(s);  q2 = path.SecondDerivative(s)
    c = dynamics.ComputeInverseDynamics(q, 0, 0)          # RNEA(q, 0, 0, g)    = g(q)
    a = dynamics.ComputeInverseDynamics(q, 0, q1) − c     # RNEA(q, 0, q', 0)   = M q'
    b = dynamics.ComputeInverseDynamics(q, q1, q2) − c    # RNEA(q, q', q'', 0) = M q'' + C(q,q') q'
    return { a, b, c }                                    # C(q, q'ṡ) q'ṡ = ṡ²·C(q,q') q'

function stageRows(i, targetLo, targetHi):   # m = 2·Dof + 4 rows, bounded array
    for j in 0..Dof−1:
        {  b_ij,  a_ij, τmax_j − c_ij }       # τ_j ≤ τmax_j
        { −b_ij, −a_ij, c_ij − τmin_j }       # τ_j ≥ τmin_j
    { 1, 0, xMax_i }, { −1, 0, 0 }            # 0 ≤ x ≤ xMax_i
    { 1, 2Δ_i, targetHi }, { −1, −2Δ_i, −targetLo }   # x + 2Δ_i·u ∈ [targetLo, targetHi]

function tighten(K, α, γ):                   # apply α·x ≤ γ to K = [lo, hi]
    if α > ε:        K.hi = min(K.hi, γ/α)
    else if α < −ε:  K.lo = max(K.lo, γ/α)
    else if γ < −ε:  K.empty = true           # row violated for every x

function projectOntoX(rows):                 # min / max x of the 2-D polygon {(x,u) : rows}
    K = [0, xCeiling]
    for r in rows with |r.β| ≤ ε: tighten(K, r.α, r.γ)
    for p in rows with p.β > ε:               # u ≤ (γ_p − α_p·x)/β_p
        for n in rows with n.β < −ε:          # u ≥ (γ_n − α_n·x)/β_n
            tighten(K, p.β·n.α − n.β·p.α, p.β·n.γ − n.β·p.γ)   # eliminate u (Fourier–Motzkin)
    return (K.empty or K.lo > K.hi + ε) ? empty : K

function maxControl(rows, x):                # 1-D LP: largest u satisfying every row at this x
    return min over rows with r.β > ε of (r.γ − r.α·x)/r.β      # finite: transition row has β = 2Δ_i

function Parameterize():
    parameterized = false
    for i in 0..N:
        q1 = path.FirstDerivative(s_i);  coefficients[i] = limits.Coefficients(s_i)
        xMax[i] = min over j with |q1_j| > ε of (q̇max_j / q1_j)²     # |q'_j·ṡ| ≤ q̇max_j
                  (xCeiling when every q1_j ≈ 0)
    if every q1_j(s_i) ≈ 0:                   # zero-length path
        stageStartTime, sDot, u = 0;  parameterized = true;  return true
    lo[N] = hi[N] = 0                         # K_N = [0, 0]: rest at the end
    for i = N−1 down to 0:                    # backward pass: controllable sets
        K = projectOntoX(stageRows(i, lo[i+1], hi[i+1]))
        if K empty: return false
        lo[i], hi[i] = K.lo, K.hi
    if lo[0] > ε: return false                # 0 ∉ K_0: cannot start from rest
    x = 0;  sDot[0] = 0;  stageStartTime[0] = 0
    for i = 0 .. N−1:                         # forward pass: greedy, stays inside K_{i+1}
        xNext = clamp(x + 2Δ_i·maxControl(stageRows(i, lo[i+1], hi[i+1]), x), lo[i+1], hi[i+1])
        u[i] = (xNext − x) / (2Δ_i)          # re-derived after the clamp ⇒ stage ends exactly at s_{i+1}
        sDot[i+1] = sqrt(xNext)
        if sDot[i] + sDot[i+1] ≤ ε: return false          # stalled at rest
        stageStartTime[i+1] = stageStartTime[i] + 2Δ_i / (sDot[i] + sDot[i+1])
        x = xNext
    parameterized = true;  return true

function Sample(t):                           # OPTIMIZE_FOR_SPEED
    if !parameterized: return nullopt
    t = clamp(t, 0, stageStartTime[N])
    i = binary search: last stage i ∈ [0, N−1] with stageStartTime[i] ≤ t
    τ = t − stageStartTime[i]
    s = s_i + sDot[i]·τ + u[i]·τ²/2;   sd = sDot[i] + u[i]·τ;   sdd = u[i]
    q1 = path.FirstDerivative(s)
    return { path.Position(s), q1·sd, q1·sdd + path.SecondDerivative(s)·sd² }
```

`projectOntoX` is the exact projection of the stage polygon onto `x`: every `(p, n)` pair is a candidate
vertex (intersection of an upper and a lower `u`-bound line), so `lo`/`hi` are the two 2-variable LP
optima `min x` / `max x` without an iterative solver.

## Complexity & memory

- `Parameterize`: `O(Grid·Dof²)` — per stage one projection over `m = 2·Dof + 4` rows (`O(m²)` pairs) and
  one `O(m)` 1-D LP, plus one `Coefficients` call (three `O(Dof)` RNEA passes). No iteration, deterministic.
- `Sample`: `O(log Grid)` binary search + three path evaluations.
- Memory: `(3·Dof + 6)·Grid` scalars in bounded `std::array`s (≈ 3 KB for `<float, 2, 64>`); the rows of one
  stage live on the stack. No heap — place the planner in static storage.

## Numerical / embedded notes

- **Planning-time only**, never in an ISR. For a hard real-time loop, pre-sample the result into a
  joint-space `CubicSplineTrajectory` (M33) instead of calling the virtual path geometry per tick.
- Stage time `Δt_i = 2Δ_i / (sqrt(x_i) + sqrt(x_{i+1}))` is exact for constant `u_i` and finite at the rest
  endpoints; never integrate `ds / sqrt(x)` (singular at `x = 0`).
- Zero-inertia points (`a_ij = 0`) and `q'_j = 0` need no special case: the row becomes a pure `x` bound or
  the joint drops out of `xMax` — no division by zero. `xCeiling` (e.g. `1 / math::Tolerance<T>()`) keeps
  every stage polygon bounded when no joint limits the speed.
- Constraints are collocated at gridpoints: between them `τ` may exceed a bound by `O(Δ)` — refine `Grid`
  or shrink the bounds by a margin.
- `ε`: relative tolerance on the row scale (`math::Tolerance<T>()`·max|coefficient|).
- The affine form holds for `M q̈ + C q̇ + g` (+ armature, Coulomb friction with fixed sign along the path);
  **viscous friction is linear in `ṡ`, not `ṡ²`** — use a friction-free model or a torque margin.
- Infeasible iff some `K_i` is empty or `0 ∉ K_0`; both surface as `Parameterize() == false`.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `robotics/trajectory/TimeOptimalPathParameterization.hpp` (interfaces `PathGeometry`,
  `JointLimits`, the `InverseDynamicsJointLimits` adapter and the planner) — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Sample`, and
  `extern template class TimeOptimalPathParameterization<float, 2, 64>;` /
  `extern template class InverseDynamicsJointLimits<float, 2>;` under `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD`.
- Uses `robotics/trajectory/TrajectoryTypes.hpp` and `robotics/dynamics/InverseDynamicsModel.hpp` (M29);
  `robotics.trajectory` links `robotics.dynamics`.
- Coverage: `robotics/trajectory/TimeOptimalPathParameterization.cpp` →
  `template class TimeOptimalPathParameterization<float, 2, 64>;` and
  `template class InverseDynamicsJointLimits<float, 2>;`
- Test: `robotics/trajectory/test/TestTimeOptimalPathParameterization.cpp`
- Doc: `doc/trajectory/TimeOptimalPathParameterization.md` (per `doc/TEMPLATE.md`) + row in `doc/trajectory/README.md`.
- CMake: `.hpp` → `target_sources(robotics.trajectory …)`;
  `robotics_add_coverage_sources(robotics.trajectory TimeOptimalPathParameterization.cpp)`;
  `TestTimeOptimalPathParameterization.cpp` → `robotics.trajectory_test`.
- New module (first trajectory item only): `robotics/trajectory/CMakeLists.txt` with
  `robotics_add_header_library(robotics.trajectory)`, `TrajectoryTypes.hpp` in `target_sources`, a
  `test/` subdir, `add_subdirectory(trajectory)` in `robotics/CMakeLists.txt`, and a `doc/trajectory/` folder.
- Depends on: M29 (`InverseDynamicsModel`, `ChainDynamicsModel`).
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
