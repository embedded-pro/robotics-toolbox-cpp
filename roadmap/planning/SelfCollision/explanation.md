# Self-Collision Checking — Overview

## What it is
A test of whether a robot arm is about to hit itself. Each link is wrapped in a capsule (a cylinder with
rounded ends) between its joints, and the distance between every pair of links that could touch is
computed. The answer is the smallest gap, which pair it belongs to, and whether it is below a safety
margin.

## Why it matters (embedded)
Arms with many joints can fold into themselves, under teleoperation, during redundancy resolution, or along
sampled plans. Checking this in every control cycle protects the hardware. Distances rather than yes/no
answers also let controllers slow down or steer away before contact.

## How it works (intuition)
The link capsules come straight from the arm's joint positions, which forward kinematics already computes.
Neighbouring links always touch at their shared joint, so they are skipped; so are pairs that can never meet
or that the coarse model would always flag. The remaining pairs are measured with the exact
capsule–capsule distance, and the minimum over them is reported along with the offending pair.

## Key parameters
- **link radii** — capsule thickness per link.
- **allowed pairs** — pairs never checked (adjacent links by default).
- **margin** — the clearance treated as a collision.

## Reference
C. Ericson, *Real-Time Collision Detection* (2004), Ch. 4–5; S. Quinlan, "Real-time modification of
collision-free paths," PhD thesis, Stanford University (1994), Ch. 3.

## See also
`PrimitiveDistance` (P1), `ChainPoseKinematics` (M30, the frame chain), `RrtConnect` (P6), `PotentialField` (P2).
