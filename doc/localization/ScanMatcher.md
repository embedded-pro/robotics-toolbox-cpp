# 2-D Scan Matching (Point-to-Point and Point-to-Line ICP)

## Overview & Motivation

A laser scanner sees the same walls from slightly different poses as the robot moves. Aligning two consecutive scans therefore measures the motion between them, often far more accurately than wheel odometry, and aligning a scan with a map localises the robot. The standard tool is **iterative closest point** (ICP).

ICP needs no features. Each iteration pairs every point of the current scan with the nearest point of the reference scan, solves for the transform that best aligns the pairs, applies it, and repeats until the correction becomes negligible. Two error metrics are common:
- **Point-to-point** minimises the distances between paired points, with a closed-form solution.
- **Point-to-line** minimises only the distance across the local reference wall. It converges in fewer iterations and has no bias when the two scans sample the walls at different spots.

## Mathematical Theory

### Association

With the current estimate $T$, each current point is moved to $y_i = T\,c_i$ and paired with its nearest reference point $r_j$. Pairs farther apart than a gate $d_{\max}$ are outliers and get zero weight.

For point-to-line, each moved point also takes the normal $n_i$ of the nearest reference segment between consecutive scan points. Segments longer than the gate are gaps between walls, not walls, and are skipped. Without this, the junction between two walls in scan order would produce a bogus diagonal segment.

### Point-to-Point Step

The increment is the weighted rigid alignment of the moved points onto their pairs:

$$\Delta T = \arg\min_{R,\,t} \sum_i w_i \left\| r_{j(i)} - (R\,y_i + t) \right\|^2$$

This is solved in closed form by the 2-D Kabsch algorithm.

### Point-to-Line Step

The increment minimises the distances along the normals:

$$\Delta T = \arg\min_{\Delta} \sum_i w_i \left( n_i^T (\Delta\,y_i - q_i) \right)^2$$

Here $q_i$ is a point on the paired segment. A few Levenberg–Marquardt steps solve it with left perturbations $\Delta \leftarrow \exp(\delta)\,\Delta$, using the Jacobian row

$$J_i = w_i \left( n_{i,x},\;\; n_{i,y},\;\; n_{i,y}\,y'_{i,x} - n_{i,x}\,y'_{i,y} \right)$$

### Degeneracy

In a corridor whose walls are all parallel, motion along the walls changes no normal distance. $J^T J$ then has a zero eigenvalue. The matcher computes its eigenvalues and refuses the match when the smallest is below $10^{-6}$ of the largest. The caller then falls back to odometry along that direction.

### Convergence

The estimate is updated as $T \leftarrow \Delta T\,T$ until the increment's translation and rotation both fall below their tolerances.

## Complexity Analysis

| Operation           | Time     | Space  | Notes                                                    |
|---------------------|----------|--------|----------------------------------------------------------|
| Association         | $O(N M)$ | $O(N)$ | Brute-force nearest point and nearest segment            |
| Point-to-point step | $O(N)$   | $O(1)$ | One weighted 2-D alignment                               |
| Point-to-line step  | $O(N)$   | $O(N)$ | A few Levenberg–Marquardt steps on a $3 \times 3$ system |
| Work arrays         | —        | $O(N)$ | Moved points, pairs, normals and weights on the stack    |

For $N$ current and $M$ reference points. For full 360-beam scans, projective (scan-order) association or a bounded k-d tree replaces the brute-force search without changing the interface.

## Step-by-Step Walkthrough

The reference scan has 68 points on three walls of a room: $x = 2$, $y = 1.5$ and $y = -1$, spaced $0.1$ m apart. The current scan is taken after the robot moved by $(0.1, -0.05, 0.05)$:

1. **Exact correspondences**: when the current scan samples the same wall spots, both metrics recover the motion to $10^{-4}$. Point-to-point needs 3 iterations, point-to-line 2.
2. **Sampling bias**: when the current scan samples the walls halfway between the reference points, point-to-point settles about $3$ cm off, pulled towards the reference samples. Point-to-line still recovers the motion exactly.
3. **Offset guess**: starting from $(-0.05, 0.05, -0.05)$ instead of the identity, point-to-line still converges to the true motion.
4. **Outlier**: an extra point $1.2$ m from every wall, such as a person in the room, is rejected by the $0.5$ m gate. 65 of the 66 points are paired, and the result is unchanged.
5. **Tight gate**: with a $1$ cm gate, no pair survives, and the match fails.
6. **Corridor**: two parallel walls leave the motion along them unobservable, and the match is refused.

## Pitfalls & Edge Cases

- **Initial guess**: ICP converges only locally. Seed it with odometry, especially when turning.
- **Corners and scan ends**: duplicate corner points and the jumps between walls in scan order can produce wrong normals. Gating long segments avoids them.
- **Degenerate geometry**: corridors and open spaces with one wall leave directions unobservable. Check the refusal and fall back to odometry.
- **Dynamic objects**: people and doors produce outliers. Keep the gate tight once the estimate is close.
- **Point density**: brute-force association scales with the product of the scan sizes. Downsample or use projective association for large scans.

## Variants & Generalizations

- **Point-to-plane ICP**: the 3-D counterpart for depth cameras and 3-D lidar.
- **Generalised ICP**: plane-to-plane covariances on both scans.
- **Normal distributions transform (NDT)**: matching against a grid of local Gaussians.
- **Correlative scan matching**: exhaustive search over a pose grid for robustness to large initial errors.

## Applications

- Laser odometry for indoor robots and warehouse vehicles.
- Map-based localization by matching scans against a known map.
- Loop closure and pose-graph SLAM front ends.

## Connections to Other Algorithms

- [Odometry Fusion](OdometryFusion.md): seeds the initial guess and can absorb the match as a position update.
- [EKF Landmark Localization](LandmarkLocalization.md): the feature-based alternative when the environment has known landmarks.
- The point-set alignment, Levenberg–Marquardt solver, eigenvalue solver and SE(2) transform come from [numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp).

## References & Further Reading

- Besl, P.J. and McKay, N.D. (1992). "A method for registration of 3-D shapes." *IEEE Transactions on Pattern Analysis and Machine Intelligence*, 14(2), 239–256.
- Censi, A. (2008). "An ICP variant using a point-to-line metric." *IEEE International Conference on Robotics and Automation*, 19–25.
- Lu, F. and Milios, E. (1997). "Robot pose estimation in unknown environments by matching 2D range scans." *Journal of Intelligent and Robotic Systems*, 18(3), 249–275.
