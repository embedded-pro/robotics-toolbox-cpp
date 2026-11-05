robotics_add_header_library(robotics.mobile)

target_include_directories(robotics.mobile ${ROBOTICS_VISIBILITY}
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(robotics.mobile ${ROBOTICS_VISIBILITY}
    infra.util
    numerical.math
)

target_sources(robotics.mobile PRIVATE
    CarLikeKinematics.hpp
    DifferentialDrive.hpp
    OmnidirectionalKinematics.hpp
    PlanarOdometry.hpp
    PlanarPath.hpp
    PurePursuit.hpp
    SkidSteerKinematics.hpp
)

robotics_add_coverage_sources(robotics.mobile
    CarLikeKinematics.cpp
    DifferentialDrive.cpp
    OmnidirectionalKinematics.cpp
    PlanarOdometry.cpp
    PlanarPath.cpp
    PurePursuit.cpp
    SkidSteerKinematics.cpp
)

add_subdirectory(test)
