# Forward Kinematics

## Overview & Motivation

Forward Kinematics computes the 3D Cartesian positions of all joints of a serial kinematic chain, plus the tool (end-effector) point, given the joint angles. For a chain of $n$ revolute joints it produces $n + 1$ position vectors — the origin of every joint followed by the tool point — in $O(n)$ time.

This is the geometric foundation for visualization, collision detection, workspace analysis and Jacobian-based inverse kinematics. It answers the question: "given these joint angles, where is each joint, and where is the tool, in the base frame?"

## Mathematical Theory

### Chain Description

Each link $i$ carries:

- $\hat{z}_i$ — the unit joint axis, expressed in the link frame;
- $r_i$ — the position of joint $i$'s origin expressed in the frame of the preceding link (for the first joint, in the base frame — this is the base mounting offset).

The tool point is described separately by a constant offset $t$ expressed in the frame of the last link. The tool point is a purely kinematic parameter: it is unrelated to where the last link's center of mass happens to be.

### Orientation Composition

The orientation of link $i$ relative to the base is the ordered product

$$^{0}R_{i} = \prod_{k=0}^{i} R_k(q_k) = R_0(q_0)\,R_1(q_1)\cdots R_i(q_i)$$

where each factor is the rotation by angle $q_k$ about $\hat{z}_k$, obtained from Rodrigues' formula

$$R(\hat{u}, \theta) = \cos\theta\,\mathbf{I}_3 + (1 - \cos\theta)\,\hat{u}\hat{u}^T + \sin\theta\,[\hat{u}]_\times$$

The product is taken left to right because every axis is expressed in its own link frame: $^{0}R_{i}$ maps vectors written in link $i$ coordinates into base coordinates. Rotations are right-handed: a positive angle about $\hat{y}$ carries $\hat{x}$ towards $-\hat{z}$.

### Position Recursion

$$p_0 = r_0, \qquad p_{i+1} = p_i + {}^{0}R_{i}\, r_{i+1} \quad (0 \le i < n - 1), \qquad p_{\text{tool}} = p_{n-1} + {}^{0}R_{n-1}\, t$$

where $p_i$ is the base-frame position of joint $i$. The tool point is stored as the last element of the result.

## Complexity Analysis

| Case | Time   | Space  | Notes                                                                               |
|------|--------|--------|-------------------------------------------------------------------------------------|
| All  | $O(n)$ | $O(n)$ | One Rodrigues rotation, one 3×3 product and one 3×3 matrix–vector product per joint |

## Step-by-Step Walkthrough

Consider a 2-link arm with $y$-axis joints, joint 2 located $1$ along $\hat{x}$ of link 1, a tool offset $t = [0.8, 0, 0]^T$, no base offset, and $q = [\pi/4, -\pi/6]$.

1. $p_0 = [0, 0, 0]^T$.
2. $^{0}R_{0} = R_y(\pi/4)$, so $p_1 = R_y(\pi/4)\,[1, 0, 0]^T = [0.707, 0, -0.707]^T$ (a positive rotation about $\hat{y}$ tips $\hat{x}$ downwards).
3. $^{0}R_{1} = R_y(\pi/4)\,R_y(-\pi/6) = R_y(\pi/12)$, so $p_{\text{tool}} = p_1 + R_y(\pi/12)\,[0.8, 0, 0]^T = [0.707 + 0.773,\ 0,\ -0.707 - 0.207]^T = [1.480, 0, -0.914]^T$.

## Pitfalls & Edge Cases

- **Base offset**: the first joint's offset is a real mounting offset; ignoring it shifts every result by a constant vector and makes inverse kinematics converge to the wrong joint angles.
- **Tool point**: the tool offset must be supplied explicitly. Approximating it from the center of mass (e.g. "twice the center-of-mass distance") is only correct for uniform links and is wrong for typical robot links whose motors sit near the joints.
- **Composition order**: multiplying the joint rotations in the wrong order produces correct results for parallel axes (planar arms) but wrong results as soon as the axes differ.
- **Non-unit axes**: Rodrigues' formula assumes a unit axis; a non-unit axis silently scales the rotation.
- **Zero-length links**: consecutive joints collapse onto the same point, which is valid but makes the corresponding Jacobian columns dependent.
- **Floating-point only**: the trigonometric evaluation is unsuitable for fixed-point types.

## Variants & Generalizations

- **Planar vs. spatial chains**: in purely planar manipulators all axes are parallel and the rotations reduce to 2D trigonometry, but the algorithmic structure (one pass accumulating transforms) is the same.
- **Full pose**: returning $^{0}R_{n-1}$ together with $p_{\text{tool}}$ gives the complete tool pose needed for orientation control.
- **Prismatic joints**: $R_k$ stays constant while the joint offset becomes a function of the joint displacement; the same forward sweep applies.
- **Different parameterizations**: Denavit–Hartenberg, modified DH, or product-of-exponentials formulations all recursively compose rigid transforms along the chain.
- **Multiple end-effectors**: for branched chains the routine runs per branch, reusing shared prefixes.

## Applications

- **Visualization**: rendering joint frames and links for debugging controllers, planners and estimators.
- **Collision detection & workspace analysis**: computing link positions to test against environment geometry and to sample reachable workspaces.
- **Control & planning**: providing the tool position and intermediate joint positions to inverse kinematics solvers, trajectory planners and constraint checkers.
- **Dynamics algorithms**: the same orientation recursion appears in the forward passes of the recursive dynamics algorithms.

## Connections to Other Algorithms

- [Inverse Kinematics](InverseKinematics.md): evaluates forward kinematics every iteration and builds its Jacobian from the joint positions.
- [Articulated Body Algorithm](../dynamics/ArticulatedBodyAlgorithm.md): its first pass performs the same orientation recursion to propagate velocities.
- [Recursive Newton-Euler](../dynamics/RecursiveNewtonEuler.md): its forward pass also propagates rotations along the chain.
- The axis–angle rotation comes from the shared geometry primitives of [numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp).

## References & Further Reading

- Craig, J.J. (2005). *Introduction to Robotics: Mechanics and Control*. 3rd ed. Chapters 2–3.
- Siciliano, B. et al. (2009). *Robotics: Modelling, Planning and Control*. Chapter 2.
- Lynch, K.M. and Park, F.C. (2017). *Modern Robotics*. Chapter 4.
