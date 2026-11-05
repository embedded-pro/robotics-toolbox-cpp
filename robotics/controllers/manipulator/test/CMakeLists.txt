add_executable(robotics.controllers.manipulator_test)
emil_build_for(robotics.controllers.manipulator_test BOOL ROBOTICS_TOOLBOX_BUILD_TESTS)
emil_add_test(robotics.controllers.manipulator_test)

target_link_libraries(robotics.controllers.manipulator_test PUBLIC
    gmock_main
    robotics.controllers.manipulator
    robotics.dynamics_test_doubles
)

target_sources(robotics.controllers.manipulator_test PRIVATE
    TestComputedTorqueControl.cpp
    TestPdGravityCompensation.cpp
)
