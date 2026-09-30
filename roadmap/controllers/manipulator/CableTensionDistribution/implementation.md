# Cable Tension Distribution — Implementation Pseudocode

> Roadmap ref: #M25 (Tier 4) · Target: `robotics/controllers/manipulator` · Namespace `controllers` · Type: `float` (templated on `T`, instantiated for `float` only)

Closed-form force distribution for cable-driven parallel robots (Pott, Bruckmann, Mikelsons 2009) with
Pott's improved iteration (2014). The structure matrix comes from the **cable geometry** — never from a
serial-arm Jacobian — and no QP solver is needed.

## Data structures

```cpp
template<typename T, std::size_t NumCables, std::size_t WrenchDim>   # static_assert(std::is_floating_point_v<T>); instantiated for float
class CableTensionDistribution:            # static_assert(WrenchDim == 6 || WrenchDim <= 3); NumCables ≥ WrenchDim
    # WrenchDim = 6: spatial platform, wrench (f; n) about the platform origin, world frame (M6 order)
    # WrenchDim ≤ 3: point-mass platform, force rows only (2 = planar xy); platform anchors ignored
    std::array<math::Vector<T,3>, NumCables> baseAnchors       # aᵢ, world frame
    std::array<math::Vector<T,3>, NumCables> platformAnchors   # bᵢ, platform frame
    T tMin        # minimum tension (> 0: cables never go slack)
    T tMax        # maximum tension (winch / cable limit)

using StructureMatrix = math::Matrix<T, WrenchDim, NumCables>
using TensionVector   = math::Vector<T, NumCables>
using WrenchVector    = math::Vector<T, WrenchDim>              # wrench the cables must apply to the platform
```

## Interface

```cpp
CableTensionDistribution(const AnchorArray& baseAnchors, const AnchorArray& platformAnchors, T tMin, T tMax)

std::optional<StructureMatrix> ComputeStructureMatrix(const kinematics::SE3Transform<T>& platformPose) const

# nullopt when the wrench is not reachable with tMin ≤ t ≤ tMax (or the geometry is degenerate):
std::optional<TensionVector> Distribute(const WrenchVector& wrench,
                                        const kinematics::SE3Transform<T>& platformPose) const   # hot path
```

## Algorithm (pseudocode)

```cpp
function ComputeStructureMatrix(pose):                  # wrench balance  A·t = w
    for i in 0..NumCables-1:
        r = pose.R·bᵢ                                   # platform anchor offset, world frame
        l = aᵢ − (pose.p + r)                           # platform anchor → base anchor
        if ‖l‖ ≤ ε: return nullopt                      # zero-length cable: direction undefined
        u = l / ‖l‖                                     # unit direction in which cable i pulls the platform
        column i = (u ; CrossProduct(r, u))             # WrenchDim = 6
                 = first WrenchDim entries of u         # WrenchDim ≤ 3 (point mass)
    return A

function Distribute(w, pose):                           # OPTIMIZE_FOR_SPEED
    A = ComputeStructureMatrix(pose);  if not A: return nullopt
    tm = (tMin + tMax) / 2
    fixed = {false…};  free = NumCables;  wFree = w;  t = 0
    while free ≥ WrenchDim:                             # ≤ NumCables − WrenchDim + 1 passes
        Af  = A with the columns of fixed cables zeroed
        tmF = tm on free cables, 0 on fixed ones
        # closed form  t = tm + A⁺(w − A·tm),  A⁺ = Aᵀ(AAᵀ)⁻¹  — min ‖t − tm‖ subject to A·t = w
        y = solvers::TrySolveSystem(Af·Afᵀ, wFree − Af·tmF)        # WrenchDim×WrenchDim
        if not y: return nullopt                        # free cables do not span the wrench space
        t[i] = tm + (Afᵀ·y)[i]  for every free i
        k = argmax over free i of violation(t[i])       # violation = max(tMin − t, t − tMax, 0); ties → lowest i
        if violation(t[k]) == 0: return t               # all tensions within bounds
        t[k] = clamp(t[k], tMin, tMax);  fixed[k] = true;  free −= 1
        wFree = wFree − A[:, k]·t[k]                    # fixed cable's contribution moves to the wrench side
    return nullopt                                      # fewer than WrenchDim free cables remain
```

## Complexity & memory

- Time: per pass `O(WrenchDim²·NumCables)` for `Af·Afᵀ` plus an `O(WrenchDim³)` solve; at most
  `NumCables − WrenchDim + 1` passes — deterministic, bounded, no iterative optimiser.
- Memory: one `WrenchDim×NumCables` matrix, one `WrenchDim×WrenchDim` system, a `NumCables` flag array;
  stack only, no heap.

## Numerical / embedded notes

- **Cables pull only** (`t ≥ tMin > 0`) — the defining constraint; `tMin > 0` keeps cables taut
  (no backlash / loss of control), `tMax` is sized to the winch.
- **Centring.** With redundancy (`NumCables > WrenchDim`) the closed form is the tension closest to
  `tm·1` in the 2-norm, keeping cables away from slack and overload (Pott 2009). With
  `NumCables = WrenchDim` it reduces to the unique `A⁻¹w` (`tm` cancels).
- **Improved closed form (Pott 2014).** Fixing the worst-violating cable at its bound and re-solving the
  reduced system enlarges the usable workspace over the plain closed form. It can still return `nullopt`
  close to the wrench-feasible workspace boundary where an exact LP/QP would succeed — treat `nullopt` as a
  workspace-boundary event (no exceptions).
- `TrySolveSystem` (upstream, pivot-threshold Gaussian elimination) reports rank deficiency — near-parallel
  cables or a degenerate pose — as `nullopt`; `Af·Afᵀ` is SPD otherwise, so `math::CholeskyDecomposition`
  (`numerical/math/CholeskyDecomposition.hpp`) is an equivalent choice.
- The wrench is `(f; n)` about the platform origin `pose.p` in the world frame; `w` is what the cables
  must supply (e.g. `−(m·g_vec)` plus the dynamic wrench from the platform controller).
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `robotics/controllers/manipulator/CableTensionDistribution.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Distribute`, and
  `extern template class CableTensionDistribution<float, 3, 2>;` / `<float, 2, 2>` / `<float, 8, 6>`
  under `#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `robotics/controllers/manipulator/CableTensionDistribution.cpp` →
  `template class CableTensionDistribution<float, 3, 2>;` / `<float, 2, 2>` / `<float, 8, 6>`
- Test: `robotics/controllers/manipulator/test/TestCableTensionDistribution.cpp`
- Doc: `doc/controllers/manipulator/CableTensionDistribution.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `robotics_add_coverage_sources`;
  `TestCableTensionDistribution.cpp` → the `_test` target.
- New module: create `robotics/controllers/manipulator/CMakeLists.txt` via `robotics_add_header_library(...)`,
  add a `test/` subdir, register it in `robotics/CMakeLists.txt`, and add a
  `doc/controllers/manipulator/` folder.
- Depends on: M6 (`SE3Transform`); `solvers::TrySolveSystem` from
  [numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp).
- Generic pattern: see `roadmap/DEPLOYMENT.md`.
