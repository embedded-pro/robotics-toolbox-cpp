robotics_add_header_library(robotics.localization)

target_include_directories(robotics.localization ${ROBOTICS_VISIBILITY}
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(robotics.localization ${ROBOTICS_VISIBILITY}
    infra.util
    numerical.filters.active
    numerical.math
    robotics.mobile
)

target_sources(robotics.localization PRIVATE
    LandmarkLocalization.hpp
    OccupancyGridMap.hpp
    OdometryFusion.hpp
)

robotics_add_coverage_sources(robotics.localization
    LandmarkLocalization.cpp
    OccupancyGridMap.cpp
    OdometryFusion.cpp
)

add_subdirectory(test)
