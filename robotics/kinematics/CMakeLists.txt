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
    AnalyticalIkOpw.hpp
    ChainPoseKinematics.hpp
    ChainTaskJacobian.hpp
    ContinuumKinematics.hpp
    DeltaKinematics.hpp
    DenavitHartenberg.hpp
    DhTaskJacobian.hpp
    ForwardKinematics.hpp
    GeometricJacobian.hpp
    InverseKinematics.hpp
    JacobianProvider.hpp
    ManipulabilityIndex.hpp
    MobileManipulatorKinematics.hpp
    PoseInverseKinematics.hpp
    ProductOfExponentials.hpp
    RedundancyResolution.hpp
    StewartGoughKinematics.hpp
)

robotics_add_coverage_sources(robotics.kinematics
    AnalyticalIkOpw.cpp
    ChainPoseKinematics.cpp
    ContinuumKinematics.cpp
    DeltaKinematics.cpp
    DenavitHartenberg.cpp
    ForwardKinematics.cpp
    GeometricJacobian.cpp
    InverseKinematics.cpp
    ManipulabilityIndex.cpp
    MobileManipulatorKinematics.cpp
    PoseInverseKinematics.cpp
    ProductOfExponentials.cpp
    RedundancyResolution.cpp
    StewartGoughKinematics.cpp
)

add_subdirectory(test)
add_subdirectory(test_doubles)
