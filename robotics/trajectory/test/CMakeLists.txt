add_executable(robotics.trajectory_test)
emil_build_for(robotics.trajectory_test BOOL ROBOTICS_TOOLBOX_BUILD_TESTS)
emil_add_test(robotics.trajectory_test)

target_link_libraries(robotics.trajectory_test PUBLIC
    gmock_main
    numerical.motion_profiles
    robotics.dynamics_test_doubles
    robotics.trajectory
    robotics.trajectory_test_doubles
)

target_sources(robotics.trajectory_test PRIVATE
    TestCartesianSlerpInterpolation.cpp
    TestCubicSplineTrajectory.cpp
    TestTimeOptimalPathParameterization.cpp
)
