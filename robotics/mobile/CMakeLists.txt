robotics_add_header_library(robotics.mobile)

target_include_directories(robotics.mobile ${ROBOTICS_VISIBILITY}
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(robotics.mobile ${ROBOTICS_VISIBILITY}
    numerical.math
)

target_sources(robotics.mobile PRIVATE
    PlanarOdometry.hpp
)

robotics_add_coverage_sources(robotics.mobile
    PlanarOdometry.cpp
)

add_subdirectory(test)
