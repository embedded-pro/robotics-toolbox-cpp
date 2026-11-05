add_executable(robotics.mobile_test)
emil_build_for(robotics.mobile_test BOOL ROBOTICS_TOOLBOX_BUILD_TESTS)
emil_add_test(robotics.mobile_test)

target_link_libraries(robotics.mobile_test PUBLIC
    gmock_main
    robotics.mobile
)

target_sources(robotics.mobile_test PRIVATE
    TestCarLikeKinematics.cpp
    TestDifferentialDrive.cpp
    TestKanayamaController.cpp
    TestOmnidirectionalKinematics.cpp
    TestPlanarOdometry.cpp
    TestPlanarPath.cpp
    TestPurePursuit.cpp
    TestSkidSteerKinematics.cpp
)
