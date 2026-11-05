#include "numerical/math/Tolerance.hpp"
#include "robotics/controllers/manipulator/PdGravityCompensation.hpp"
#include "robotics/dynamics/EulerLagrangeSolver.hpp"
#include "robotics/dynamics/test_doubles/MockEulerLagrangeDynamics.hpp"
#include <cmath>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace
{
    using Vector2 = math::Vector<float, 2>;
    using Matrix2 = math::SquareMatrix<float, 2>;

    class VerticalTwoLinkArm
        : public dynamics::EulerLagrangeDynamics<float, 2>
    {
    public:
        MassMatrix ComputeMassMatrix(const StateVector& q) const override
        {
            const float c2{ std::cos(q[1]) };
            const float m11{ inertia + inertia + mass * lc * lc + mass * (length * length + lc * lc + 2.0f * length * lc * c2) };
            const float m12{ inertia + mass * (lc * lc + length * lc * c2) };
            return MassMatrix{ { m11, m12 }, { m12, inertia + mass * lc * lc } };
        }

        StateVector ComputeCoriolisTerms(const StateVector& q, const StateVector& qDot) const override
        {
            const float h{ mass * length * lc * std::sin(q[1]) };
            return StateVector{ -h * (2.0f * qDot[0] * qDot[1] + qDot[1] * qDot[1]), h * qDot[0] * qDot[0] };
        }

        StateVector ComputeGravityTerms(const StateVector& q) const override
        {
            const float distal{ mass * lc * gravity * std::cos(q[0] + q[1]) };
            return StateVector{ (mass * lc + mass * length) * gravity * std::cos(q[0]) + distal, distal };
        }

    private:
        static constexpr float mass{ 1.0f };
        static constexpr float length{ 1.0f };
        static constexpr float lc{ 0.5f };
        static constexpr float inertia{ mass * length * length / 12.0f };
        static constexpr float gravity{ 9.81f };
    };

    class TestPdGravityCompensation
        : public ::testing::Test
    {
    protected:
        void ExpectGravity(const Vector2& g)
        {
            EXPECT_CALL(model, ComputeGravityTerms(testing::_)).WillOnce(testing::Return(g));
        }

        void ExpectNear(const Vector2& actual, const Vector2& expected) const
        {
            EXPECT_NEAR(actual[0], expected[0], math::Tolerance<float>());
            EXPECT_NEAR(actual[1], expected[1], math::Tolerance<float>());
        }

        testing::StrictMock<dynamics::MockEulerLagrangeDynamics<float, 2>> model;
        Matrix2 kp{ { 100.0f, 0.0f }, { 0.0f, 100.0f } };
        Matrix2 kd{ { 20.0f, 0.0f }, { 0.0f, 20.0f } };
        controllers::PdGravityCompensation<float, 2> controller{ model, kp, kd };
        Vector2 q{ 0.3f, -0.6f };
        Vector2 qDesired{ 0.5f, -0.2f };
        Vector2 qDot{ 0.4f, -1.0f };
    };
}

TEST_F(TestPdGravityCompensation, gravity_only_at_matched_setpoint)
{
    const Vector2 g{ 0.0f, 4.905f };
    ExpectGravity(g);

    ExpectNear(controller.ComputeTorque(q, Vector2{}, q), g);
}

TEST_F(TestPdGravityCompensation, proportional_term_on_position_error)
{
    ExpectGravity(Vector2{});

    ExpectNear(controller.ComputeTorque(q, Vector2{}, qDesired), Vector2{ 20.0f, 40.0f });
}

TEST_F(TestPdGravityCompensation, derivative_term_damps_velocity)
{
    ExpectGravity(Vector2{});

    ExpectNear(controller.ComputeTorque(q, qDot, q), Vector2{ -8.0f, 20.0f });
}

TEST_F(TestPdGravityCompensation, superposition_of_all_terms)
{
    ExpectGravity(Vector2{ 1.5f, -2.5f });

    ExpectNear(controller.ComputeTorque(q, qDot, qDesired), Vector2{ 20.0f - 8.0f + 1.5f, 40.0f + 20.0f - 2.5f });
}

TEST_F(TestPdGravityCompensation, diagonal_gains_decouple_joints)
{
    ExpectGravity(Vector2{});

    const auto tau{ controller.ComputeTorque(q, Vector2{}, Vector2{ q[0] + 0.1f, q[1] }) };

    EXPECT_NEAR(tau[0], 10.0f, math::Tolerance<float>());
    EXPECT_NEAR(tau[1], 0.0f, math::Tolerance<float>());
}

TEST_F(TestPdGravityCompensation, zero_error_zero_velocity_holds_gravity)
{
    const float mgL{ 1.0f * 9.81f * 0.5f };
    const Vector2 g{ mgL * std::sin(qDesired[0]), mgL * std::sin(qDesired[1]) };
    ExpectGravity(g);

    ExpectNear(controller.ComputeTorque(qDesired, Vector2{}, qDesired), g);
}

TEST_F(TestPdGravityCompensation, only_gravity_method_queried)
{
    EXPECT_CALL(model, ComputeGravityTerms(testing::Truly([this](const Vector2& argument)
                           {
                               return argument[0] == q[0] && argument[1] == q[1];
                           })))
        .Times(1)
        .WillOnce(testing::Return(Vector2{}));

    controller.ComputeTorque(q, qDot, qDesired);
}

TEST_F(TestPdGravityCompensation, regulates_to_setpoint_closed_loop)
{
    const VerticalTwoLinkArm plant;
    const dynamics::EulerLagrangeSolver<float, 2> solver;
    const float dt{ 1e-3f };
    EXPECT_CALL(model, ComputeGravityTerms(testing::_)).WillRepeatedly([&plant](const Vector2& state)
        {
            return plant.ComputeGravityTerms(state);
        });

    Vector2 position{ -0.4f, 0.8f };
    Vector2 velocity{};
    for (std::size_t step = 0; step < 6000; ++step)
    {
        const auto tau{ controller.ComputeTorque(position, velocity, qDesired) };
        velocity = velocity + solver.ForwardDynamics(plant, position, velocity, tau) * dt;
        position = position + velocity * dt;
    }

    ExpectNear(position, qDesired);
    ExpectNear(velocity, Vector2{});
}
