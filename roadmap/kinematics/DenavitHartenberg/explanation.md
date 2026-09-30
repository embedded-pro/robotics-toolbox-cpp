# Denavit-Hartenberg Parameters — Overview

## What it is
A minimal four-number recipe — `(a, α, d, θ)` — that describes where each link of a robot arm sits
relative to the previous one. Each row generates one rigid transform; multiplying them down the chain
gives the forward kinematics, and reading each joint's axis off the intermediate frames gives the input
the Jacobian needs.

## Why it matters (embedded)
DH is the *lingua franca* of industrial robotics: nearly every arm's datasheet ships a DH table.
Encoding a manipulator as `N×4` constants (plus a joint-type flag) is the most compact possible model,
and the per-link transform is one sine/cosine pair plus a fixed fill — cheap enough for a servo loop on
a microcontroller. Emitting the shared frame chain lets DH-described arms reuse the library's single
Jacobian, inverse-kinematics and control code.

## How it works (intuition)
The convention forces every joint axis onto a frame's `z`-axis and every common normal onto an
`x`-axis. That discipline collapses the six numbers of a general rigid transform down to four: two
describe the joint (`d` slides along `z`, `θ` rotates about `z`) and two describe the link (`a` slides
along `x`, `α` twists about `x`). The joint variable is *added* to the constant `θ` (revolute) or `d`
(prismatic), because real tables carry offsets. The standard (distal) convention attaches frame `i` at
the far end of link `i`, so joint `i` turns about the `z`-axis of the previous frame; the modified
(proximal, Craig) convention attaches it at the near end, so joint `i` turns about its own frame's
`z`-axis and each row carries the previous link's `a` and `α`. Both describe the same arm.

## Key parameters
- **a (link length), α (link twist)** — fixed geometry of the link.
- **d (link offset), θ (joint angle)** — constant offsets; the joint variable adds to one of them.
- **convention** — standard (distal) or modified (proximal); rows differ between the two.
- **tool transform** — constant pose of the tool in the last DH frame.

## Reference
J. Denavit, R. S. Hartenberg, "A Kinematic Notation for Lower-Pair Mechanisms Based on Matrices,"
*ASME J. Applied Mechanics*, 1955; J. J. Craig, *Introduction to Robotics: Mechanics and Control*,
4th ed., Ch. 3 (modified convention).

## See also
`SE3Transform` (M6, the per-link transform), `ChainPoseKinematics` (M30, the `FrameChain` it emits),
`GeometricJacobian` (M8, consumes the frame chain), `ProductOfExponentials` (M15, the screw-theory
alternative).
