add_executable(robotics.simulator.dynamics.robotarm.application_test)
emil_build_for(robotics.simulator.dynamics.robotarm.application_test HOST All PREREQUISITE_BOOL ROBOTICS_TOOLBOX_BUILD_SIMULATOR)
emil_add_test(robotics.simulator.dynamics.robotarm.application_test)

target_link_libraries(robotics.simulator.dynamics.robotarm.application_test PUBLIC
    gmock_main
    robotics.simulator.dynamics.robotarm.application
)

target_sources(robotics.simulator.dynamics.robotarm.application_test PRIVATE
    TestRobotArmForm.cpp
)
