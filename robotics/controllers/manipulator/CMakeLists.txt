robotics_add_header_library(robotics.controllers.manipulator)

target_include_directories(robotics.controllers.manipulator ${ROBOTICS_VISIBILITY}
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(robotics.controllers.manipulator ${ROBOTICS_VISIBILITY}
    numerical.math
    robotics.dynamics
)

target_sources(robotics.controllers.manipulator PRIVATE
    PdGravityCompensation.hpp
)

robotics_add_coverage_sources(robotics.controllers.manipulator
    PdGravityCompensation.cpp
)

add_subdirectory(test)
