robotics_add_header_library(robotics.trajectory)

target_include_directories(robotics.trajectory ${ROBOTICS_VISIBILITY}
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(robotics.trajectory ${ROBOTICS_VISIBILITY}
    infra.util
    numerical.math
    numerical.motion_profiles
    robotics.dynamics
)

target_sources(robotics.trajectory PRIVATE
    CartesianSlerpInterpolation.hpp
    CubicSplineTrajectory.hpp
    TimeOptimalPathParameterization.hpp
    TrajectoryTypes.hpp
)

robotics_add_coverage_sources(robotics.trajectory
    CartesianSlerpInterpolation.cpp
    CubicSplineTrajectory.cpp
    TimeOptimalPathParameterization.cpp
)

add_subdirectory(test)
add_subdirectory(test_doubles)
