# Localization & Mapping

Maps, state estimation and localization for mobile robots moving in the plane.

## Algorithms

| Algorithm                                            | Description                                                                                                                                                              |
|------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| [Occupancy-Grid Mapping](OccupancyGridMap.md)        | Bounded grid of clamped log-odds cells updated from range scans along integer Bresenham rays, with max-range clearing — $O(\sum \rho / r)$                               |
| [Odometry Fusion](OdometryFusion.md)                 | Wheel and gyroscope EKF on SE(2) with gyro-bias estimation, χ²-gated slip detection and optional heading and position updates — $O(1)$                                   |
| [EKF Landmark Localization](LandmarkLocalization.md) | Range–bearing EKF against a known landmark map, with body-frame odometry noise, Mahalanobis association, χ² gating and wrapped bearings — $O(L)$                         |
| [2-D Scan Matching (ICP)](ScanMatcher.md)            | Point-to-point and point-to-line ICP between laser scans with gated nearest-neighbour association, gap-aware segment normals and corridor degeneracy detection — $O(NM)$ |
