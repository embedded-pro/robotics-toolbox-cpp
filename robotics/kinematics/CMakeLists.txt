robotics_add_header_library(robotics.kinematics)

target_include_directories(robotics.kinematics ${ROBOTICS_VISIBILITY}
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(robotics.kinematics ${ROBOTICS_VISIBILITY}
    robotics.dynamics
    numerical.math
    numerical.solver
)

target_sources(robotics.kinematics PRIVATE
    ForwardKinematics.hpp
    InverseKinematics.hpp
)

robotics_add_coverage_sources(robotics.kinematics
    ForwardKinematics.cpp
    InverseKinematics.cpp
)

add_subdirectory(test)
