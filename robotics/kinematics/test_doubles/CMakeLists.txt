add_library(robotics.kinematics_test_doubles INTERFACE)
emil_build_for(robotics.kinematics_test_doubles BOOL ROBOTICS_TOOLBOX_BUILD_TESTS)

target_link_libraries(robotics.kinematics_test_doubles INTERFACE
    gmock
    robotics.kinematics
)

target_sources(robotics.kinematics_test_doubles PRIVATE
    MockJacobianProvider.hpp
)
