# Product of Exponentials — Overview

## What it is
A forward-kinematics formula built from screw theory: `T = e^{[S₁]q₁}···e^{[Sₙ]qₙ}·M`. Each joint is a
**screw axis** `Sᵢ` written once in the base frame; moving the joint by `qᵢ` applies the exponential of
that screw, and `M` is the tool's pose when every joint is at zero.

## Why it matters (embedded)
PoE describes an arm with one home pose and one screw per joint — all in the *base* frame — instead of
a chain of intermediate DH frames. There is nothing to line up between links, so it is less
error-prone to author from a CAD model, and the same screws give the Jacobian and the frame chain in
the same pass. With an SE(3) type already in the library it is the most direct FK to write.

## How it works (intuition)
A screw axis packages "rotate about this line while sliding along it" into one 6-vector: the unit
direction `ω` of the line and the velocity `v = −ω × a` that a point at the base origin would have when
the body spins about the line through `a`. Its exponential is the finite rigid motion produced by
riding that screw for `qᵢ`. Forward kinematics starts at the home pose and multiplies in each joint's
motion. Carrying each screw through the motions of the joints before it gives the "space Jacobian",
whose linear part is the velocity of the point at the base origin; shifting that reference point to
the tool gives the geometric Jacobian used everywhere else in the library.

## Key parameters
- **screw axes `Sᵢ = (vᵢ; ωᵢ)`** — one per joint, base frame, at `q = 0`.
- **home configuration `M`** — the tool pose at `q = 0`.

## Reference
K. M. Lynch, F. C. Park, *Modern Robotics* (2017), Ch. 4–5 (PoE and the space Jacobian; the book
orders screws `(ω; v)`, this library `(v; ω)`); R. W. Brockett, "Robotic Manipulators and the Product
of Exponentials Formula," 1984.

## See also
`SE3Transform` (M6, `Exp` and `Adjoint`), `ChainPoseKinematics` (M30, `FrameChain`),
`GeometricJacobian` (M8), `DenavitHartenberg` (M7, the frame-based alternative), `MatrixExponential`
([numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp), test cross-check).
