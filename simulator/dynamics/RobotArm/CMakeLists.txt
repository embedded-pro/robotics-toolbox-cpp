find_package(Qt6 REQUIRED COMPONENTS Widgets)

add_subdirectory(application)
add_subdirectory(view)

add_executable(robotics.simulator.dynamics.robot_arm)
emil_build_for(robotics.simulator.dynamics.robot_arm HOST All PREREQUISITE_BOOL ROBOTICS_TOOLBOX_BUILD_SIMULATOR)

target_link_libraries(robotics.simulator.dynamics.robot_arm PRIVATE
    robotics.simulator.dynamics.robotarm.application
    robotics.simulator.dynamics.robotarm.view
)

target_sources(robotics.simulator.dynamics.robot_arm PRIVATE
    Main.cpp
)

if(MSVC)
    target_compile_options(robotics.simulator.dynamics.robot_arm PRIVATE /W4 /wd4244 /wd4100)
else()
    target_compile_options(robotics.simulator.dynamics.robot_arm PRIVATE
        -Wno-error
    )
endif()
