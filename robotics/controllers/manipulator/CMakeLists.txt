robotics_add_header_library(robotics.controllers.manipulator)

target_include_directories(robotics.controllers.manipulator ${ROBOTICS_VISIBILITY}
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(robotics.controllers.manipulator ${ROBOTICS_VISIBILITY}
    numerical.math
    numerical.solver
    robotics.dynamics
    robotics.kinematics
)

target_sources(robotics.controllers.manipulator PRIVATE
    ComputedTorqueControl.hpp
    ImpedanceControl.hpp
    PdGravityCompensation.hpp
    SlotineLiAdaptiveControl.hpp
    TaskError.hpp
)

robotics_add_coverage_sources(robotics.controllers.manipulator
    ComputedTorqueControl.cpp
    ImpedanceControl.cpp
    PdGravityCompensation.cpp
    SlotineLiAdaptiveControl.cpp
)

add_subdirectory(test)
