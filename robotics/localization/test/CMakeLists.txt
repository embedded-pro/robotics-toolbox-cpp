add_executable(robotics.localization_test)
emil_build_for(robotics.localization_test BOOL ROBOTICS_TOOLBOX_BUILD_TESTS)
emil_add_test(robotics.localization_test)

target_link_libraries(robotics.localization_test PUBLIC
    gmock_main
    robotics.localization
)

target_sources(robotics.localization_test PRIVATE
    TestOccupancyGridMap.cpp
    TestOdometryFusion.cpp
)
