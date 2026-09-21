#pragma once

#include "robotics/dynamics/ArticulatedBodyAlgorithm.hpp"
#include "robotics/dynamics/RecursiveNewtonEuler.hpp"
#include "robotics/dynamics/RevoluteJointLink.hpp"
#include "robotics/kinematics/ForwardKinematics.hpp"
#include "simulator/dynamics/RobotArm/application/RobotArmTypes.hpp"
#include <array>
#include <cstddef>
#include <vector>

namespace simulator::dynamics
{
    class RobotArmSimulator
    {
    public:
        void Configure(const RobotArmConfig& config);
        void SetTorques(const std::vector<float>& torques);
        void SetInitialPositions(const std::vector<float>& positions);
        void Step();
        void Reset();

        const RobotArmState& GetState() const;
        const RobotArmConfig& GetConfig() const;

    private:
        void StepDof2();
        void StepDof3();
        void UpdateForwardKinematics();
        void UpdateTrail();

        std::array<::dynamics::RevoluteJointLink<float>, 2> BuildLinks2() const;
        std::array<::dynamics::RevoluteJointLink<float>, 3> BuildLinks3() const;

        static constexpr float gravity = 9.81f;
        static constexpr std::size_t maxTrailSize = 500;

        ::dynamics::ArticulatedBodyAlgorithm<float, 2> aba2;
        ::dynamics::ArticulatedBodyAlgorithm<float, 3> aba3;
        ::dynamics::RecursiveNewtonEuler<float, 2> rnea2;
        ::dynamics::RecursiveNewtonEuler<float, 3> rnea3;
        ::kinematics::ForwardKinematics<float, 2> fk2;
        ::kinematics::ForwardKinematics<float, 3> fk3;

        RobotArmConfig config;
        RobotArmState state;
        std::vector<float> torques;
    };
}
