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
        template<std::size_t N>
        using LinkArray = std::array<::dynamics::RevoluteJointLink<float>, N>;

        template<std::size_t N>
        void StepChain(const LinkArray<N>& links, const ::dynamics::ArticulatedBodyAlgorithm<float, N>& aba,
            const ::dynamics::RecursiveNewtonEuler<float, N>& rnea);

        template<std::size_t N>
        void UpdateJointPositions(const LinkArray<N>& links);

        void UpdateForwardKinematics();
        void UpdateTrail();

        LinkArray<2> BuildLinks2() const;
        LinkArray<3> BuildLinks3() const;

        static constexpr float gravity = 9.81f;
        static constexpr std::size_t maxTrailSize = 500;

        ::dynamics::ArticulatedBodyAlgorithm<float, 2> aba2;
        ::dynamics::ArticulatedBodyAlgorithm<float, 3> aba3;
        ::dynamics::RecursiveNewtonEuler<float, 2> rnea2;
        ::dynamics::RecursiveNewtonEuler<float, 3> rnea3;

        RobotArmConfig config;
        RobotArmState state;
        std::vector<float> torques;
    };
}
