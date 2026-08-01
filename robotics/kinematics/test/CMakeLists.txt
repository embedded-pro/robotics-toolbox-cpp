add_executable(robotics.kinematics_test)
emil_build_for(robotics.kinematics_test BOOL ROBOTICS_TOOLBOX_BUILD_TESTS)
emil_add_test(robotics.kinematics_test)

target_link_libraries(robotics.kinematics_test PUBLIC
    gmock_main
    robotics.kinematics
)

target_sources(robotics.kinematics_test PRIVATE
    TestForwardKinematics.cpp
    TestInverseKinematics.cpp
)
