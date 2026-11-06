add_library(robotics.trajectory_test_doubles INTERFACE)
emil_build_for(robotics.trajectory_test_doubles BOOL ROBOTICS_TOOLBOX_BUILD_TESTS)

target_link_libraries(robotics.trajectory_test_doubles INTERFACE
    gmock
    robotics.trajectory
)

target_sources(robotics.trajectory_test_doubles PRIVATE
    MockPathConstraints.hpp
    MockPathGeometry.hpp
)
