#include "numerical/math/Geometry3D.hpp"
#include "numerical/math/Tolerance.hpp"
#include "robotics/dynamics/ArticulatedBodyAlgorithm.hpp"
#include "robotics/dynamics/CompositeRigidBodyAlgorithm.hpp"
#include "robotics/dynamics/ForwardDynamicsIntegrator.hpp"
#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>
#include <numbers>

namespace
{
    using Vector3 = math::Vector<float, 3>;
    using Matrix3 = math::SquareMatrix<float, 3>;
    using Link = dynamics::RevoluteJointLink<float>;
    using Method = dynamics::IntegrationMethod;

    constexpr float g{ 9.81f };
    constexpr float halfPi{ std::numbers::pi_v<float> / 2.0f };
    const Vector3 yAxis{ 0.0f, 1.0f, 0.0f };
    const Vector3 zAxis{ 0.0f, 0.0f, 1.0f };

    Link MakeRod(const Vector3& axis, const Vector3& parentToJoint)
    {
        return Link{ 1.0f, Matrix3{ { 0.0f, 0.0f, 0.0f }, { 0.0f, 1.0f / 12.0f, 0.0f }, { 0.0f, 0.0f, 1.0f / 12.0f } }, axis, parentToJoint, Vector3{ 0.5f, 0.0f, 0.0f } };
    }

    class TestForwardDynamicsIntegrator
        : public ::testing::Test
    {
    protected:
        float PendulumEnergy(const dynamics::ChainState<float, 1>& state) const
        {
            return 0.5f * (1.0f / 3.0f) * state.qDot[0] * state.qDot[0] - g * 0.5f * std::sin(state.q[0]);
        }

        float DoubleRodEnergy(const dynamics::ChainState<float, 2>& state) const
        {
            const Matrix3 R0{ math::RotationAboutAxis(rods[0].jointAxis, state.q[0]) };
            const Matrix3 R1{ R0 * math::RotationAboutAxis(rods[1].jointAxis, state.q[1]) };
            const Vector3 com0{ R0 * rods[0].jointToCoM };
            const Vector3 com1{ R0 * rods[1].parentToJoint + R1 * rods[1].jointToCoM };
            const auto M{ crba.Compute(rods, state.q) };
            return 0.5f * (state.qDot.Transpose() * (M * state.qDot)).at(0, 0) + g * (com0[2] + com1[2]);
        }

        Vector3 gravity{ 0.0f, 0.0f, -g };
        std::array<Link, 1> rod{ MakeRod(yAxis, Vector3{}) };
        std::array<Link, 2> rods{ MakeRod(yAxis, Vector3{}), MakeRod(yAxis, Vector3{ 1.0f, 0.0f, 0.0f }) };
        dynamics::CompositeRigidBodyAlgorithm<float, 2> crba;
        dynamics::ForwardDynamicsIntegrator<float, 1, Method::RungeKutta4> pendulum{ rod, gravity, 1e-3f };
        dynamics::ForwardDynamicsIntegrator<float, 2, Method::SemiImplicitEuler> doubleRod{ rods, gravity, 1e-3f };
    };
}

TEST_F(TestForwardDynamicsIntegrator, small_oscillation_period_matches_physical_pendulum)
{
    dynamics::ChainState<float, 1> state{ math::Vector<float, 1>{ halfPi + 0.05f } };
    std::array<float, 2> upwardCrossings{};
    std::size_t crossings{ 0 };

    for (std::size_t step = 1; step <= 5000 && crossings < upwardCrossings.size(); ++step)
    {
        const float before{ state.q[0] - halfPi };
        state = pendulum.Step(state, math::Vector<float, 1>{});
        const float after{ state.q[0] - halfPi };
        if (before < 0.0f && after >= 0.0f)
            upwardCrossings[crossings++] = 1e-3f * (static_cast<float>(step) - after / (after - before));
    }

    const float expected{ 2.0f * std::numbers::pi_v<float> * std::sqrt(2.0f / (3.0f * g)) };
    ASSERT_EQ(crossings, upwardCrossings.size());
    EXPECT_NEAR(upwardCrossings[1] - upwardCrossings[0], expected, 0.005f * expected);
}

TEST_F(TestForwardDynamicsIntegrator, rk4_energy_drift_is_small)
{
    dynamics::ChainState<float, 1> state{ math::Vector<float, 1>{ halfPi + 0.05f } };
    const float initial{ PendulumEnergy(state) };

    for (std::size_t step = 0; step < 10000; ++step)
        state = pendulum.Step(state, math::Vector<float, 1>{});

    EXPECT_LT(std::abs(PendulumEnergy(state) - initial) / std::abs(initial), 1e-4f);
}

TEST_F(TestForwardDynamicsIntegrator, semi_implicit_euler_energy_stays_bounded)
{
    dynamics::ChainState<float, 2> state{ math::Vector<float, 2>{ halfPi + 0.3f, 0.2f } };
    const float initial{ DoubleRodEnergy(state) };
    float firstSecondError{ 0.0f };
    float overallError{ 0.0f };

    for (std::size_t step = 1; step <= 10000; ++step)
    {
        state = doubleRod.Step(state, math::Vector<float, 2>{});
        const float error{ std::abs(DoubleRodEnergy(state) - initial) };
        overallError = std::max(overallError, error);
        if (step <= 1000)
            firstSecondError = overallError;
    }

    EXPECT_LE(overallError, 2.0f * firstSecondError);
}

TEST_F(TestForwardDynamicsIntegrator, step_acceleration_equals_articulated_body_algorithm)
{
    const dynamics::ChainState<float, 2> state{ math::Vector<float, 2>{ 0.4f, -0.9f }, math::Vector<float, 2>{ 1.2f, -0.5f } };
    const math::Vector<float, 2> tau{ 0.7f, -0.3f };
    const dynamics::ArticulatedBodyAlgorithm<float, 2> aba;

    const auto next{ doubleRod.Step(state, tau) };
    const auto expected{ aba.ForwardDynamics(rods, state.q, state.qDot, tau, gravity) };

    EXPECT_NEAR(next.qDDot[0], expected[0], math::Tolerance<float>());
    EXPECT_NEAR(next.qDDot[1], expected[1], math::Tolerance<float>());
}

TEST_F(TestForwardDynamicsIntegrator, constant_torque_on_free_link_gives_quadratic_angle)
{
    const std::array<Link, 1> freeRod{ MakeRod(zAxis, Vector3{}) };
    dynamics::ForwardDynamicsIntegrator<float, 1, Method::RungeKutta4> integrator{ freeRod, gravity, 1e-3f };
    const float torque{ 0.5f };
    dynamics::ChainState<float, 1> state{};

    for (std::size_t step = 0; step < 1000; ++step)
        state = integrator.Step(state, math::Vector<float, 1>{ torque });

    EXPECT_NEAR(state.q[0], 0.5f * (torque / (1.0f / 3.0f)) * 1.0f, math::Tolerance<float>());
}
