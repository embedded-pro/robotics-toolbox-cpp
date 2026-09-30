# Multi-Joint Cubic Spline Trajectory — Overview

## What it is
A joint-space trajectory through a list of **via points** reached at given times. Between consecutive
knots each joint follows a cubic polynomial; the pieces are chosen so position, velocity **and**
acceleration are continuous everywhere (C²). The ends are either **clamped** (prescribed velocities,
zero by default ⇒ rest-to-rest) or **natural** (zero end acceleration).

## Why it matters (embedded)
Teach-in programs, planner outputs and resampled time-optimal profiles all arrive as a list of
waypoints. A single cubic spline turns them into one smooth reference without stopping at every point,
and without the ringing of high-order polynomials. Planning is one linear-time pass per joint on fixed
bounded storage; each servo tick is a binary search plus a cubic evaluation — cheap and deterministic.

## How it works (intuition)
Unknown are the accelerations at the knots. Once they are known, each segment is fixed: it must hit
both knot positions and match the two knot accelerations. Requiring the velocities of neighbouring
segments to agree at every interior knot gives one linear equation per knot that couples only the knot
and its two neighbours — a **tridiagonal** system. The end conditions add the first and last equations.
That system is strongly diagonally dominant, so the Thomas algorithm (a two-sweep Gaussian elimination)
solves it in linear time with no pivoting; the elimination factors depend only on the knot times and are
shared by all joints.

## Key parameters
- **Knot times `t_k`** — strictly increasing; they set the speed between via points (stretching all of
  them by `λ` divides velocities by `λ` and accelerations by `λ²`).
- **Knot positions `q_k`** — one joint vector per via point.
- **End condition** — clamped (start/end velocity, default zero) or natural (zero end acceleration).
- **`MaxPoints`** — compile-time capacity of the bounded storage.

## Reference
L. Biagiotti, C. Melchiorri, *Trajectory Planning for Automatic Machines and Robots* (2008), Ch. 4
(cubic splines: continuity conditions, clamped/natural ends, tridiagonal solution).

## See also
`PolynomialTrajectory` (the two-knot special case), `TimeOptimalPathParameterization` (its output can be
resampled into a spline), `TrapezoidalProfile` / `SCurveProfile` (single-segment, limit-driven moves).
