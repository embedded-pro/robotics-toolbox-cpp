add_library(robotics.simulator.shell INTERFACE)

target_include_directories(robotics.simulator.shell INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_sources(robotics.simulator.shell INTERFACE
    AppRunner.hpp
)

target_link_libraries(robotics.simulator.shell INTERFACE
    ui.backend.qt
)
