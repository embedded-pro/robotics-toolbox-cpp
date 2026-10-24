# Reeds–Shepp Paths — Overview

## What it is
The shortest way for a car that can drive both forwards and backwards, with a limited turning radius,
to get from one position and heading to another. The answer is a short sequence of turns and straight
pieces with up to two changes of driving direction — exactly how a driver parks.

## Why it matters (embedded)
Automated parking, warehouse tuggers, forklifts and agricultural vehicles constantly reverse. Reeds–Shepp
paths connect poses in sampling-based and lattice planners for such vehicles, and their length is a
good, smooth distance for search heuristics. Everything is closed-form, so the cost per query is fixed
and small.

## How it works (intuition)
Reeds and Shepp showed that only 48 patterns of arcs and straights can be optimal. Their lengths come from
eight small geometric formulas; the rest follow by symmetry, because driving a path backwards, mirroring
it left-to-right or traversing it in reverse order maps one valid pattern onto another. The planner
moves the goal into the start's frame, evaluates every formula with its symmetric variants, and keeps
the shortest. Negative piece lengths mean the car reverses along that piece.

## Key parameters
- **turning radius `ρ`** — the vehicle's minimum radius (W2's `MinTurningRadius`).
- **start and goal poses**.

## Reference
J. A. Reeds, L. A. Shepp, "Optimal paths for a car that goes both forwards and backwards," *Pacific J.
Mathematics*, 145(2), 1990; P. Souères, J.-P. Laumond, "Shortest paths synthesis for a car-like robot,"
*IEEE T-AC*, 41(5), 1996.

## See also
`DubinsPlanner` and `CarPath` (W8), `CarLikeKinematics` (W2), `math::SE2Transform` (NT66).
