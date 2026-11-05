add_library(robotics.dynamics_test_doubles INTERFACE)
emil_build_for(robotics.dynamics_test_doubles BOOL ROBOTICS_TOOLBOX_BUILD_TESTS)

target_link_libraries(robotics.dynamics_test_doubles INTERFACE
    gmock
    robotics.dynamics
)

target_sources(robotics.dynamics_test_doubles PRIVATE
    MockEulerLagrangeDynamics.hpp
    MockInertialRegressor.hpp
    MockInverseDynamicsModel.hpp
)
