#include "robotics/dynamics/ArticulatedBodyAlgorithm.hpp"
#include "robotics/dynamics/ChainDynamicsModel.hpp"
#include "robotics/dynamics/CoriolisMatrix.hpp"
#include "robotics/dynamics/MomentumObserver.hpp"
#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>

namespace
{
    using Vector2 = math::Vector<float, 2>;
    using Vector3 = math::Vector<float, 3>;
    using Link = dynamics::JointLink<float>;

    constexpr float dt{ 1e-3f };
    const Vector3 gravity{ 0.0f, 0.0f, -9.81f };

    Link MakeRod(float mass, float length, const Vector3& parentToJoint)
    {
        const float inertia{ mass * length * length / 12.0f };
        return Link{ mass, math::SquareMatrix<float, 3>{ { 0.0f, 0.0f, 0.0f }, { 0.0f, inertia, 0.0f }, { 0.0f, 0.0f, inertia } }, Vector3{ 0.0f, 1.0f, 0.0f }, parentToJoint,
            Vector3{ length / 2.0f, 0.0f, 0.0f } };
    }

    class TestMomentumObserver
        : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            observer.Reset(q, qDot);
        }

        Vector2 Command(float time, float amplitude) const
        {
            return model.ComputeGravityTerms(q) + Vector2{ amplitude * std::sin(3.0f * time), -amplitude * std::cos(4.0f * time) };
        }

        void Step(const Vector2& tau, const Vector2& external)
        {
            const auto acceleration{ aba.ForwardDynamics(links, q, qDot, Vector2{ tau + external }, gravity) };
            qDot = qDot + acceleration * dt;
            q = q + qDot * dt;
            observer.Update(q, qDot, tau, dt);
        }

        float MaxResidualOverFreeMotion(std::size_t steps, float amplitude)
        {
            float worst{ 0.0f };
            for (std::size_t k = 0; k < steps; ++k)
            {
                Step(Command(static_cast<float>(k) * dt, amplitude), Vector2{});
                worst = std::max({ worst, std::abs(observer.Residual()[0]), std::abs(observer.Residual()[1]) });
            }
            return worst;
        }

        std::array<Link, 2> links{ MakeRod(2.0f, 1.0f, Vector3{}), MakeRod(1.5f, 0.8f, Vector3{ 1.0f, 0.0f, 0.0f }) };
        dynamics::ChainDynamicsModel<float, 2> model{ links, gravity };
        dynamics::CoriolisMatrix<float, 2> coriolis;
        dynamics::ArticulatedBodyAlgorithm<float, 2> aba;
        dynamics::MomentumObserver<float, 2> observer{ model, coriolis, links, Vector2{ 50.0f, 50.0f } };
        Vector2 q{ 0.4f, -0.9f };
        Vector2 qDot{};
        Vector2 threshold{ 0.5f, 0.5f };
    };
}

TEST_F(TestMomentumObserver, free_motion_keeps_residual_near_zero)
{
    EXPECT_LT(MaxResidualOverFreeMotion(1000, 1.0f), 1e-2f);
}

TEST_F(TestMomentumObserver, step_external_torque_is_recovered_with_first_order_response)
{
    const Vector2 external{ 0.0f, 0.8f };
    for (std::size_t k = 0; k < 200; ++k)
        Step(Command(static_cast<float>(k) * dt, 0.0f), Vector2{});

    for (std::size_t k = 0; k < 20; ++k)
        Step(Command(0.0f, 0.0f), external);
    const float afterOneTimeConstant{ observer.Residual()[1] };
    for (std::size_t k = 0; k < 300; ++k)
        Step(Command(0.0f, 0.0f), external);

    EXPECT_NEAR(afterOneTimeConstant, 0.8f * (1.0f - std::exp(-1.0f)), 0.02f);
    EXPECT_NEAR(observer.Residual()[1], 0.8f, 0.01f);
    EXPECT_NEAR(observer.Residual()[0], 0.0f, 0.01f);
}

TEST_F(TestMomentumObserver, collision_flag_uses_per_joint_thresholds)
{
    for (std::size_t k = 0; k < 200; ++k)
        Step(Command(static_cast<float>(k) * dt, 0.0f), Vector2{});
    const bool before{ observer.Collision(threshold) };

    for (std::size_t k = 0; k < 300; ++k)
        Step(Command(0.0f, 0.0f), Vector2{ 0.0f, 0.8f });

    EXPECT_FALSE(before);
    EXPECT_TRUE(observer.Collision(threshold));
}

TEST_F(TestMomentumObserver, reset_restarts_from_current_momentum)
{
    for (std::size_t k = 0; k < 300; ++k)
        Step(Command(static_cast<float>(k) * dt, 1.0f), Vector2{ 0.3f, -0.2f });
    ASSERT_GT(std::abs(observer.Residual()[0]), 0.1f);

    observer.Reset(q, qDot);

    EXPECT_EQ(observer.Residual()[0], 0.0f);
    EXPECT_EQ(observer.Residual()[1], 0.0f);
}

TEST_F(TestMomentumObserver, uses_coriolis_transpose_not_the_rnea_vector)
{
    const Vector2 fastVelocity{ 3.0f, -4.0f };
    const auto C{ coriolis.Compute(links, q, fastVelocity) };
    const Vector2 transposed{ C.Transpose() * fastVelocity };
    const Vector2 direct{ C * fastVelocity };
    const float drift{ std::abs(transposed[0] - direct[0]) + std::abs(transposed[1] - direct[1]) };
    ASSERT_GT(drift, 1.0f);

    qDot = fastVelocity;
    observer.Reset(q, qDot);

    EXPECT_LT(MaxResidualOverFreeMotion(1000, 0.0f), 0.05f * drift);
}
