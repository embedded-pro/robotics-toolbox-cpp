#include "numerical/math/Tolerance.hpp"
#include "simulator/dynamics/RobotArm/application/RobotArmSimulator.hpp"
#include <cmath>
#include <gtest/gtest.h>
#include <numbers>

namespace
{
    using simulator::dynamics::RobotArmConfig;
    using simulator::dynamics::RobotArmSimulator;

    constexpr float gravity{ 9.81f };

    RobotArmConfig PlanarArm()
    {
        RobotArmConfig config;
        config.dof = 2;
        config.linkLengths = { 0.5f, 0.4f };
        config.linkMasses = { 1.0f, 0.8f };
        config.damping = 0.0f;
        return config;
    }

    RobotArmConfig SpatialArm()
    {
        RobotArmConfig config;
        config.dof = 3;
        config.linkLengths = { 0.5f, 0.4f, 0.3f };
        config.linkMasses = { 1.0f, 0.8f, 0.6f };
        config.damping = 0.0f;
        return config;
    }

    class TestRobotArmSimulator
        : public ::testing::Test
    {
    protected:
        RobotArmSimulator simulator;
    };
}

TEST_F(TestRobotArmSimulator, planar_inverse_dynamics_readout_reproduces_applied_torque)
{
    simulator.Configure(PlanarArm());
    simulator.SetInitialPositions({ 0.3f, -0.4f });
    simulator.SetTorques({ 1.5f, -0.5f });

    simulator.Step();
    simulator.Step();

    EXPECT_NEAR(simulator.GetState().inverseDynamicsTorques[0], 1.5f, math::Tolerance<float>());
    EXPECT_NEAR(simulator.GetState().inverseDynamicsTorques[1], -0.5f, math::Tolerance<float>());
}

TEST_F(TestRobotArmSimulator, spatial_inverse_dynamics_readout_reproduces_applied_torque)
{
    simulator.Configure(SpatialArm());
    simulator.SetInitialPositions({ 0.2f, 0.3f, -0.4f });
    simulator.SetTorques({ 0.7f, 1.5f, -0.5f });

    simulator.Step();
    simulator.Step();

    EXPECT_NEAR(simulator.GetState().inverseDynamicsTorques[0], 0.7f, math::Tolerance<float>());
    EXPECT_NEAR(simulator.GetState().inverseDynamicsTorques[1], 1.5f, math::Tolerance<float>());
    EXPECT_NEAR(simulator.GetState().inverseDynamicsTorques[2], -0.5f, math::Tolerance<float>());
}

TEST_F(TestRobotArmSimulator, hanging_arm_without_torque_stays_at_rest)
{
    const float hanging{ std::numbers::pi_v<float> / 2.0f };
    simulator.Configure(PlanarArm());
    simulator.SetInitialPositions({ hanging, 0.0f });

    for (int step = 0; step < 120; ++step)
        simulator.Step();

    EXPECT_NEAR(simulator.GetState().q[0], hanging, math::Tolerance<float>());
    EXPECT_NEAR(simulator.GetState().q[1], 0.0f, math::Tolerance<float>());
}

TEST_F(TestRobotArmSimulator, horizontal_release_matches_analytic_uniform_rod_acceleration)
{
    const float m1{ 1.0f };
    const float m2{ 0.8f };
    const float l1{ 0.5f };
    const float l2{ 0.4f };
    const float m11{ m1 * l1 * l1 / 3.0f + m2 * (l1 * l1 + l2 * l2 / 3.0f + l1 * l2) };
    const float m12{ m2 * (l2 * l2 / 3.0f + l1 * l2 / 2.0f) };
    const float m22{ m2 * l2 * l2 / 3.0f };
    const float g1{ gravity * (m1 * l1 / 2.0f + m2 * l1 + m2 * l2 / 2.0f) };
    const float g2{ gravity * m2 * l2 / 2.0f };
    const float determinant{ m11 * m22 - m12 * m12 };
    simulator.Configure(PlanarArm());

    simulator.Step();

    EXPECT_NEAR(simulator.GetState().qDDot[0], (m22 * g1 - m12 * g2) / determinant, 1e-2f);
    EXPECT_NEAR(simulator.GetState().qDDot[1], (m11 * g2 - m12 * g1) / determinant, 1e-2f);
}

TEST_F(TestRobotArmSimulator, tool_point_is_at_the_end_of_the_last_link)
{
    simulator.Configure(SpatialArm());

    const auto& tool = simulator.GetState().jointPositions.back();

    EXPECT_NEAR(tool[0], 0.7f, math::Tolerance<float>());
    EXPECT_NEAR(tool[1], 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(tool[2], 0.5f, math::Tolerance<float>());
}
