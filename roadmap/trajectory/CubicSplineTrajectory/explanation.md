# Multi-Joint Cubic Spline Trajectory — Overview

## What it is
A joint-space trajectory through a list of **via points** reached at given times. Between consecutive
knots each joint follows a cubic polynomial; position, velocity **and** acceleration are continuous
everywhere (C²). The ends are either **clamped** (prescribed velocities, zero by default ⇒ rest-to-rest)
or **natural** (zero end acceleration).

## Why it matters (embedded)
Teach-in programs, planner outputs and resampled time-optimal profiles all arrive as a list of
waypoints. A single cubic spline turns them into one smooth reference without stopping at every point,
and without the ringing of high-order polynomials. Planning is one linear-time pass on fixed bounded
storage; each servo tick is a binary search plus a cubic evaluation — cheap and deterministic.

## How it works (intuition)
The generic part — fitting a smooth vector-valued curve through knots by solving one tridiagonal system
for the knot curvatures — is the shared cubic-spline interpolant of the numerical toolbox. This item uses
time as the abscissa and the joint vector as the value, so the curve's slope and curvature are the joint
velocity and acceleration, and it defaults the end slopes to zero so a move starts and ends at rest.

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
Upstream `CubicSpline` (NT61, the interpolant) and `PolynomialProfile` (NT55, the two-knot special case);
`TimeOptimalPathParameterization` (M27, its output can be resampled into a spline); upstream
`TrapezoidalProfile` / `SCurveProfile` (NT56/NT60, single-segment, limit-driven moves).
