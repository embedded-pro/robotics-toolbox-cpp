#pragma once

#include <array>
#include <cstddef>
#include <vector>

namespace simulator::dynamics
{
    // Split out of RobotArmSimulator.hpp: these are plain float aggregates, but sharing a header
    // with the templated simulator meant every view that named them also pulled in
    // robotics/{dynamics,kinematics} and the numerical headers behind those.
    struct RobotArmConfig
    {
        int dof = 2;
        std::vector<float> linkLengths = { 0.5f, 0.4f, 0.3f };
        std::vector<float> linkMasses = { 1.0f, 0.8f, 0.6f };
        float damping = 0.05f;
        float dt = 1.0f / 120.0f;
    };

    struct RobotArmState
    {
        std::vector<std::array<float, 3>> jointPositions;
        std::vector<float> q;
        std::vector<float> qDot;
        std::vector<float> qDDot;
        std::vector<float> inverseDynamicsTorques;
        std::vector<std::array<float, 3>> endEffectorTrail;
        float time = 0.0f;
    };
}
