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
    ConvexCollision.hpp
    GridSearch.hpp
    PotentialField.hpp
    PrimitiveDistance.hpp
    RrtConnect.hpp
    SelfCollision.hpp
)

robotics_add_coverage_sources(robotics.planning
    ConvexCollision.cpp
    GridSearch.cpp
    PotentialField.cpp
    PrimitiveDistance.cpp
    RrtConnect.cpp
    SelfCollision.cpp
)

add_subdirectory(test)
