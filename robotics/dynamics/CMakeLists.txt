robotics_add_header_library(robotics.dynamics)

target_include_directories(robotics.dynamics ${ROBOTICS_VISIBILITY}
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(robotics.dynamics ${ROBOTICS_VISIBILITY}
    infra.util
    numerical.math
    numerical.solver
)

target_sources(robotics.dynamics PRIVATE
    ArticulatedBodyAlgorithm.hpp
    ChainDynamicsModel.hpp
    CompositeRigidBodyAlgorithm.hpp
    EulerLagrangeDynamics.hpp
    EulerLagrangeSolver.hpp
    ForwardDynamicsIntegrator.hpp
    InverseDynamicsModel.hpp
    JointLink.hpp
    NewtonEulerBody.hpp
    NewtonEulerSolver.hpp
    RecursiveNewtonEuler.hpp
    RevoluteJointLink.hpp
    SpatialInertia.hpp
)

robotics_add_coverage_sources(robotics.dynamics
    ArticulatedBodyAlgorithm.cpp
    ChainDynamicsModel.cpp
    CompositeRigidBodyAlgorithm.cpp
    EulerLagrangeSolver.cpp
    ForwardDynamicsIntegrator.cpp
    JointLink.cpp
    NewtonEulerSolver.cpp
    RecursiveNewtonEuler.cpp
)

add_subdirectory(test)
add_subdirectory(test_doubles)
