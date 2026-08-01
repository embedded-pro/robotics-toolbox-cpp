add_executable(robotics.dynamics_test)
emil_build_for(robotics.dynamics_test BOOL ROBOTICS_TOOLBOX_BUILD_TESTS)
emil_add_test(robotics.dynamics_test)

target_link_libraries(robotics.dynamics_test PUBLIC
    gmock_main
    robotics.dynamics
)

target_sources(robotics.dynamics_test PRIVATE
    TestArticulatedBodyAlgorithm.cpp
    TestEulerLagrangeSolver.cpp
    TestNewtonEulerSolver.cpp
    TestRecursiveNewtonEuler.cpp
)
