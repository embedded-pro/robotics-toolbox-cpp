add_library(robotics.simulator.dynamics.robotarm.view STATIC)

target_include_directories(robotics.simulator.dynamics.robotarm.view PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(robotics.simulator.dynamics.robotarm.view PUBLIC
    robotics.simulator.dynamics.robotarm.application
    ui.core
    ui.theme
    ui.scene
    ui.backend.qt
    Qt6::Widgets
)

set_target_properties(robotics.simulator.dynamics.robotarm.view PROPERTIES
    AUTOMOC ON
)

target_sources(robotics.simulator.dynamics.robotarm.view PRIVATE
    RobotArmSceneView.cpp
    RobotArmSceneView.hpp
    RobotArmMainWindow.cpp
    RobotArmMainWindow.hpp
)

if(MSVC)
    target_compile_options(robotics.simulator.dynamics.robotarm.view PRIVATE /W4 /wd4244 /wd4100)
else()
    target_compile_options(robotics.simulator.dynamics.robotarm.view PRIVATE
        -Wno-error
    )
endif()

if (ROBOTICS_TOOLBOX_BUILD_TESTS)
    add_subdirectory(test)
endif()
