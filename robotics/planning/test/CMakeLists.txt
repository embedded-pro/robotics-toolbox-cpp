add_executable(robotics.planning_test)
emil_build_for(robotics.planning_test BOOL ROBOTICS_TOOLBOX_BUILD_TESTS)
emil_add_test(robotics.planning_test)

target_link_libraries(robotics.planning_test PUBLIC
    gmock_main
    robotics.planning
)

target_sources(robotics.planning_test PRIVATE
    TestGridSearch.cpp
    TestPotentialField.cpp
    TestPrimitiveDistance.cpp
    TestSelfCollision.cpp
)
