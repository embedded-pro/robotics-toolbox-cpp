add_library(robotics.simulator.dynamics.robotarm.application STATIC)

target_include_directories(robotics.simulator.dynamics.robotarm.application PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(robotics.simulator.dynamics.robotarm.application PUBLIC
    robotics.dynamics
    robotics.kinematics
)

target_sources(robotics.simulator.dynamics.robotarm.application PRIVATE
    RobotArmSimulator.cpp
    RobotArmSimulator.hpp
)

if(MSVC)
    target_compile_options(robotics.simulator.dynamics.robotarm.application PRIVATE /W4 /wd4244 /wd4100)
else()
    target_compile_options(robotics.simulator.dynamics.robotarm.application PRIVATE
        -Wno-error
    )
endif()
