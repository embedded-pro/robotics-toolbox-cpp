add_executable(robotics.simulator.dynamics.robotarm.view_test)
emil_build_for(robotics.simulator.dynamics.robotarm.view_test BOOL ROBOTICS_TOOLBOX_BUILD_TESTS)
emil_add_test(robotics.simulator.dynamics.robotarm.view_test)

# The scene view is a ui::PaintedView, so it is asserted through RecordingCanvas with no display
# and no Qt painting; the view target's Qt link is incidental to the window beside it.
target_link_libraries(robotics.simulator.dynamics.robotarm.view_test PUBLIC
    gmock_main
    robotics.simulator.dynamics.robotarm.view
    ui.backend.recording
)

target_sources(robotics.simulator.dynamics.robotarm.view_test PRIVATE
    TestRobotArmSceneView.cpp
)

if(MSVC)
    target_compile_options(robotics.simulator.dynamics.robotarm.view_test PRIVATE /W4 /wd4244 /wd4100)
else()
    target_compile_options(robotics.simulator.dynamics.robotarm.view_test PRIVATE -Wno-error)
endif()
