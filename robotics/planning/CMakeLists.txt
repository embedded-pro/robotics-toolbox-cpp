robotics_add_header_library(robotics.planning)

target_include_directories(robotics.planning ${ROBOTICS_VISIBILITY}
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(robotics.planning ${ROBOTICS_VISIBILITY}
    infra.util
    numerical.math
    robotics.kinematics
)

target_sources(robotics.planning PRIVATE
    GridSearch.hpp
    PotentialField.hpp
    PrimitiveDistance.hpp
    SelfCollision.hpp
)

robotics_add_coverage_sources(robotics.planning
    GridSearch.cpp
    PotentialField.cpp
    PrimitiveDistance.cpp
    SelfCollision.cpp
)

add_subdirectory(test)
