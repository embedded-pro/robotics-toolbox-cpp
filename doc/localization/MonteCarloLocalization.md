# Monte Carlo Localization (Likelihood Field)

## Overview & Motivation

A robot with a map still has to find out where it is on that map. At power-up it may be anywhere. After a collision or being carried, its estimate may be wrong. In a building of repeated corridors, several places may look the same until the robot drives far enough to tell them apart. A Gaussian filter, with its single mode, cannot represent any of these situations.

**Monte Carlo localization** (MCL) represents the belief over poses with a set of particles, each a candidate pose with a weight. Odometry moves every particle with its own noise sample. Each laser scan reweights the particles by how well the scan fits the map from that pose. Resampling then concentrates the particles on the poses that explain the data.
Started from particles spread over the whole map, MCL localizes globally; started near a known pose, it tracks.

The **likelihood field** sensor model makes the scan evaluation cheap. Instead of ray-casting each beam through the map, it looks up how far each beam's end point lies from the nearest obstacle. These distances are precomputed once per map.

## Mathematical Theory

### Likelihood Field

From the occupancy grid, every cell gets the distance to its nearest occupied cell, computed by a two-pass 8-neighbour chamfer transform. The forward pass runs over rows and columns in ascending order and relaxes each cell from its west, south and two southern diagonal neighbours. The backward pass runs in descending order and relaxes from the east, north and two northern diagonal neighbours:

$$d(c) \leftarrow \min\big(d(c),\; d(c') + w(c, c')\big), \qquad w = 1 \text{ (axial)}, \;\; \sqrt{2} \text{ (diagonal)}$$

Distances are scaled by the cell size and capped at $d_{\max}$. Points outside the map score $d_{\max}$. The chamfer metric overestimates the Euclidean distance by at most about 8 %, well inside the sensor model's spread.

### Motion Model

For an odometry increment $\Delta = (d_x, d_y, d_\theta)$ in the body frame, the increment noise model gives the deviations $(\sigma_{xy}, \sigma_{xy}, \sigma_\theta)$. These grow with the distance and rotation travelled. Each particle draws its own noisy increment from a caller-supplied standard normal sampler and composes it:

$$x^{(i)} \leftarrow x^{(i)} \oplus \big(d_x + \sigma_{xy}\,n_1,\;\; d_y + \sigma_{xy}\,n_2,\;\; d_\theta + \sigma_\theta\,n_3\big)$$

### Sensor Model

For a particle, the sensor pose is the particle pose composed with the sensor mount. Each used beam's end point $e_k$ is scored against the likelihood field:

$$\ell = \sum_k \log\left(z_{\text{hit}}\, e^{-d(e_k)^2 / 2\sigma_{\text{hit}}^2} + \frac{z_{\text{rand}}}{r_{\max}}\right)$$

The random-measurement floor $z_{\text{rand}}/r_{\max}$ keeps one unexplained beam, such as a person or glass, from eliminating a particle. Beams at or beyond the maximum range carry no end point and are skipped. A beam stride uses every $k$-th beam to save computation.

### Filtering and Estimate

The log-likelihoods update the particle weights in the log domain. The particles are resampled systematically, from a caller-supplied uniform number, when the effective sample size falls below a fraction of the particle count. The pose estimate is the weighted mean, with the heading averaged as a unit vector so it stays correct across $\pm\pi$:

$$\bar\theta = \operatorname{atan2}\Big(\sum_i w_i \sin\theta_i,\; \sum_i w_i \cos\theta_i\Big)$$

## Complexity Analysis

| Operation        | Time         | Space    | Notes                                               |
|------------------|--------------|----------|-----------------------------------------------------|
| Likelihood field | $O(W H)$     | $O(W H)$ | Two raster passes, once per map                     |
| Predict          | $O(N)$       | $O(1)$   | Three normal draws and one composition per particle |
| Update           | $O(N B / s)$ | $O(N)$   | One lookup, exponential and logarithm per used beam |
| Particle store   | —            | $O(N)$   | Two particle buffers, weights and scratch           |

For $N$ particles, $B$ beams per scan, stride $s$ and a $W \times H$ grid. 500 particles with 60 beams cost about 30 000 field lookups per scan.

## Step-by-Step Walkthrough

A $4 \times 4$ m room at $10$ cm resolution has walls on its border, an inner wall and a shelf. The robot stands at $(1.0, 1.5)$ heading $0.2$ rad and takes a 12-beam scan every $30°$:

1. **Likelihood field**: around a single occupied cell, cells 5 apart along an axis are $0.5$ m away, and 4 apart diagonally $0.566$ m. The cell at $(3, 4)$ cells is $0.524$ m away by the chamfer metric, against $0.5$ m Euclidean.
2. **Noise-free prediction**: particles at $(1, 2, 0.3)$ moved by $0.5$ m and $0.1$ rad all reach $(1.478, 2.148, 0.4)$.
3. **Noisy prediction**: with every normal draw at $+1$, the increment grows by $\sigma_{xy} = 0.112$ m and $\sigma_\theta = 0.055$ rad, and the particles reach $(1.552, 2.288, 0.455)$.
4. **Scan update**: over 27 candidate poses offset by $\pm 0.2$ m and $\pm 0.1$ rad around the truth, the true pose takes $99.5$ % of the weight. The estimate is $(1.0, 1.5, 0.2004)$.
5. **Resampling**: with the effective sample size at about $1$, far below half of 27, resampling places every particle on the true pose.
6. **Maximum-range scan**: a scan where every beam hits nothing within range leaves the weights uniform.

## Pitfalls & Edge Cases

- **Particle deprivation**: with too few particles, or too little motion noise, the filter can lose the true pose. Size the particle set for the area, and keep the noise realistic.
- **Kidnapping**: once every particle is wrong, MCL cannot recover by itself. Inject uniformly drawn particles when the average likelihood drops (augmented MCL).
- **Overconfident sensor model**: a small $\sigma_{\text{hit}}$ or no random floor collapses the weights in one scan. Keep the floor and use a beam stride for correlated beams.
- **Map quality**: unmapped furniture and people violate the static-world assumption. The random floor absorbs some of it.
- **Determinism**: all randomness comes from the caller's samplers, so a seeded generator makes runs reproducible.

## Variants & Generalizations

- **KLD-sampling MCL**: adapts the particle count to the spread of the belief.
- **Augmented MCL**: recovers from kidnapping by injecting random particles when likelihoods drop.
- **Beam and ray-casting models**: model occlusions explicitly at a higher cost per beam.
- **Scan-matching refinement**: polishes the best particle with ICP for precise tracking.

## Applications

- Indoor service and warehouse robots localizing in a prebuilt map.
- Global localization at start-up, and recovery after a robot is moved.
- Localization in symmetric environments where several hypotheses must be kept.

## Connections to Other Algorithms

- [Occupancy-Grid Mapping](OccupancyGridMap.md): builds the map the likelihood field is computed from.
- [Planar Odometry](../mobile/PlanarOdometry.md): the increment convention and the motion noise model.
- [EKF Landmark Localization](LandmarkLocalization.md): the Gaussian alternative for tracking with known landmarks.
- [2-D Scan Matching (ICP)](ScanMatcher.md): refines a tracked pose against a reference scan.
- The particle filter and SE(2) transform come from [numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp).

## References & Further Reading

- Dellaert, F., Fox, D., Burgard, W. and Thrun, S. (1999). "Monte Carlo localization for mobile robots." *IEEE International Conference on Robotics and Automation*, 1322–1328.
- Thrun, S., Burgard, W. and Fox, D. (2005). *Probabilistic Robotics*. MIT Press. Chapters 6 and 8.
- Borgefors, G. (1986). "Distance transformations in digital images." *Computer Vision, Graphics, and Image Processing*, 34(3), 344–371.
