# Robotics Algorithms Library

## Overview

This library provides robot-manipulator algorithms — kinematics, rigid-body dynamics, trajectory
generation, and manipulator control — designed for robust and efficient real-time use on
resource-constrained embedded systems (no heap, deterministic, float-only). It builds on the shared
numerical primitives (linear algebra, solvers, controllers) provided by
[numerical-toolbox-cpp](https://github.com/embedded-pro/numerical-toolbox-cpp), which it consumes
via CMake `FetchContent`.

## Getting Started

The library is a set of CMake targets. Consume it from your own project with `FetchContent`:

```cmake
include(FetchContent)
FetchContent_Declare(
    robotics_toolbox
    GIT_REPOSITORY https://github.com/embedded-pro/robotics-toolbox-cpp.git
    GIT_TAG        main
)
FetchContent_MakeAvailable(robotics_toolbox)

target_link_libraries(my_app PRIVATE robotics.kinematics robotics.dynamics)
```

The numerical-toolbox dependency is fetched automatically. Includes are namespaced by domain, e.g.
`#include "robotics/kinematics/ForwardKinematics.hpp"` and — for shared primitives —
`#include "numerical/math/Matrix.hpp"`.

## Documentation

| Category                             | Description                                                                                    |
|--------------------------------------|-----------------------------------------------------------------------------------------------|
| [Kinematics](doc/kinematics/README.md) | Forward Kinematics, Inverse Kinematics (Damped Least Squares)                                |
| [Dynamics](doc/dynamics/README.md)     | Euler-Lagrange, Newton-Euler, Recursive Newton-Euler (RNEA), Articulated Body Algorithm (ABA) |

Each category page lists its algorithms with a brief description and links to the detailed
documentation.

### Booklet

The entire documentation set is also published as a single book — read it online as a
[GitHub Pages site](https://embedded-pro.github.io/robotics-toolbox-cpp/) or download the latest
PDF from the [Releases page](../../releases/latest). Both are generated automatically from `doc/`
(cover, Summary/table of contents, one chapter per category, consolidated references, back cover).

Build it locally with [Pandoc](https://pandoc.org) + XeLaTeX installed:

```bash
python scripts/build-booklet.py --format all   # writes build/booklet/{RoboticsToolbox.pdf,index.html}
```

## Simulator

The `simulator/` directory contains an interactive Qt-based GUI application (Robot Arm) for
visualizing and experimenting with the library's algorithms. It is a desktop tool for development
and exploration, separate from the core embedded-targeted library.

### Building the Simulator

The simulator requires Qt6 and is disabled by default. Enable it with:

```bash
cmake --preset host  # host preset enables it automatically
# or manually:
cmake -DROBOTICS_TOOLBOX_BUILD_SIMULATOR=ON ...
```

Prerequisites: `qt6-base-dev` and `libgl1-mesa-dev` (Ubuntu/Debian).

## Roadmap

Planned algorithms and components are tracked in [ROADMAP.md](ROADMAP.md) — a prioritized backlog of
manipulator kinematics, dynamics, trajectory, and control components ordered by implementation
difficulty. Design-level pseudocode specifications live under [roadmap/](roadmap/README.md).

## Testing

Every algorithm is validated against the **mathematical invariants of its family** — not golden
output. Tests are `TEST_F` on `float`, no heap, one behaviour per test, asserted against independent
reference values. The per-family metric strategy is documented in **[TESTING.md](TESTING.md)**;
framework rules live in
[.github/instructions/testing.instructions.md](.github/instructions/testing.instructions.md).

Build & test locally:

```bash
cmake --preset host && cmake --build --preset host
ctest --preset host
```

## Contributing

Contributions, issues, and feature requests are welcome. Please check the contributing guidelines
before submitting pull requests.

## License

See [LICENSE](LICENSE).
