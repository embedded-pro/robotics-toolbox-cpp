# Occupancy-Grid Mapping — Overview

## What it is
A map of the robot's surroundings as a grid of small square cells, each holding how likely it is to be
occupied. Every laser or sonar beam updates the cells it passes through (probably free) and the cell where
it hit something (probably occupied).

## Why it matters (embedded)
Occupancy grids are the standard map for indoor robots: they are what localization matches scans
against, what planners search, and what a robot builds when it explores. The update is pure integer
stepping and additions, so mapping keeps up with the sensor on a small processor. The fixed grid size
makes memory use known in advance.

## How it works (intuition)
Each cell stores the log of the odds that it is occupied, so combining evidence is just adding. A beam is
traced from the sensor's cell to the cell where it ended with Bresenham's line algorithm. Every cell on
the way gets a small negative amount (the beam passed through, so it is likely free), and the end cell
gets a positive amount (something reflected it). Beams that reached the sensor's maximum range found
nothing, so they only clear. Values are clamped, so a cell that has been seen as free many times can
still become occupied when a door closes.

## Key parameters
- **resolution** and **origin** — cell size and where the grid sits in the world.
- **occupied / free increments** — how much one beam changes a cell.
- **clamp limits** and **maximum range**.

## Reference
A. Elfes, "Using occupancy grids for mobile robot perception and navigation," *Computer*, 22(6), 1989;
S. Thrun, W. Burgard, D. Fox, *Probabilistic Robotics* (2005), Ch. 9; J. E. Bresenham, "Algorithm for
computer control of a digital plotter," *IBM Systems Journal*, 4(1), 1965.

## See also
`MonteCarloLocalization` (E4, uses the map), `math::SE2Transform` (NT66).
