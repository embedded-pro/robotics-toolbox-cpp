robotics_add_header_library(robotics.kinematics)

target_include_directories(robotics.kinematics ${ROBOTICS_VISIBILITY}
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(robotics.kinematics ${ROBOTICS_VISIBILITY}
    robotics.dynamics
    numerical.math
    numerical.optimization
    numerical.solver
)

target_sources(robotics.kinematics PRIVATE
    ChainPoseKinematics.hpp
    ChainTaskJacobian.hpp
    ContinuumKinematics.hpp
    DenavitHartenberg.hpp
    DhTaskJacobian.hpp
    ForwardKinematics.hpp
    GeometricJacobian.hpp
    InverseKinematics.hpp
    JacobianProvider.hpp
    ManipulabilityIndex.hpp
    PoseInverseKinematics.hpp
    ProductOfExponentials.hpp
    RedundancyResolution.hpp
)

robotics_add_coverage_sources(robotics.kinematics
    ChainPoseKinematics.cpp
    ContinuumKinematics.cpp
    DenavitHartenberg.cpp
    ForwardKinematics.cpp
    GeometricJacobian.cpp
    InverseKinematics.cpp
    ManipulabilityIndex.cpp
    PoseInverseKinematics.cpp
    ProductOfExponentials.cpp
    RedundancyResolution.cpp
)

add_subdirectory(test)
add_subdirectory(test_doubles)
