#include "numerical/math/Tolerance.hpp"
#include "robotics/controllers/manipulator/ComputedTorqueControl.hpp"
#include "robotics/dynamics/ChainDynamicsModel.hpp"
#include "robotics/dynamics/EulerLagrangeSolver.hpp"
#include "robotics/dynamics/test_doubles/MockInverseDynamicsModel.hpp"
#include <cmath>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace
{
    using Vector2 = math::Vector<float, 2>;
    using Vector3 = math::Vector<float, 3>;
    using Matrix2 = math::SquareMatrix<float, 2>;
    using Link = dynamics::JointLink<float>;

    MATCHER_P(VectorNear, expected, "")
    {
        return std::abs(arg[0] - expected[0]) < math::Tolerance<float>() && std::abs(arg[1] - expected[1]) < math::Tolerance<float>();
    }

    Link MakeRod(float mass, float length, const Vector3& parentToJoint)
    {
        const float inertia{ mass * length * length / 12.0f };
        return Link{ mass, math::SquareMatrix<float, 3>{ { 0.0f, 0.0f, 0.0f }, { 0.0f, inertia, 0.0f }, { 0.0f, 0.0f, inertia } }, Vector3{ 0.0f, 1.0f, 0.0f }, parentToJoint,
            Vector3{ length / 2.0f, 0.0f, 0.0f } };
    }

    class TestComputedTorqueControl
        : public ::testing::Test
    {
    protected:
        void ExpectCommand(const Vector2& acceleration, const Vector2& torque)
        {
            EXPECT_CALL(model, ComputeInverseDynamics(VectorNear(q), VectorNear(qDot), VectorNear(acceleration))).Times(1).WillOnce(testing::Return(torque));
        }

        testing::StrictMock<dynamics::MockInverseDynamicsModel<float, 2>> model;
        Matrix2 kp{ { 100.0f, 0.0f }, { 0.0f, 100.0f } };
        Matrix2 kd{ { 20.0f, 0.0f }, { 0.0f, 20.0f } };
        controllers::ComputedTorqueControl<float, 2> controller{ model, kp, kd };
        Vector2 q{ 0.3f, -0.6f };
        Vector2 qDot{ 0.4f, -1.0f };
    };
}

TEST_F(TestComputedTorqueControl, pure_feedforward_passes_desired_acceleration)
{
    const Vector2 desiredAcceleration{ 1.5f, -0.7f };
    const Vector2 modelTorque{ 3.25f, -1.75f };
    ExpectCommand(desiredAcceleration, modelTorque);

    const auto tau{ controller.ComputeTorque(q, qDot, q, qDot, desiredAcceleration) };

    EXPECT_EQ(tau[0], modelTorque[0]);
    EXPECT_EQ(tau[1], modelTorque[1]);
}

TEST_F(TestComputedTorqueControl, position_error_enters_command)
{
    ExpectCommand(Vector2{ 10.0f, -20.0f }, Vector2{});

    controller.ComputeTorque(q, qDot, Vector2{ q[0] + 0.1f, q[1] - 0.2f }, qDot, Vector2{});
}

TEST_F(TestComputedTorqueControl, velocity_error_enters_command)
{
    ExpectCommand(Vector2{ 6.0f, 4.0f }, Vector2{});

    controller.ComputeTorque(q, qDot, q, Vector2{ qDot[0] + 0.3f, qDot[1] + 0.2f }, Vector2{});
}

TEST_F(TestComputedTorqueControl, full_command_superposition)
{
    const Vector2 desired{ q[0] + 0.1f, q[1] - 0.2f };
    const Vector2 desiredVelocity{ qDot[0] + 0.3f, qDot[1] + 0.2f };
    const Vector2 desiredAcceleration{ 1.5f, -0.7f };
    ExpectCommand(Vector2{ 1.5f + 6.0f + 10.0f, -0.7f + 4.0f - 20.0f }, Vector2{});

    controller.ComputeTorque(q, qDot, desired, desiredVelocity, desiredAcceleration);
}

TEST_F(TestComputedTorqueControl, decoupled_error_dynamics)
{
    const std::array<Link, 2> links{ MakeRod(1.2f, 0.6f, Vector3{}), MakeRod(0.7f, 0.45f, Vector3{ 0.6f, 0.0f, 0.0f }) };
    const dynamics::ChainDynamicsModel<float, 2> plant{ links, Vector3{ 0.0f, 0.0f, -9.81f } };
    const dynamics::EulerLagrangeSolver<float, 2> solver;
    EXPECT_CALL(model, ComputeInverseDynamics(testing::_, testing::_, testing::_)).WillRepeatedly([&plant](const Vector2& position, const Vector2& velocity, const Vector2& acceleration)
        {
            return plant.ComputeInverseDynamics(position, velocity, acceleration);
        });
    const Vector2 setpoint{ 0.5f, -0.2f };
    const float dt{ 1e-3f };

    Vector2 position{ 0.2f, 0.3f };
    Vector2 velocity{};
    Vector2 error{ setpoint - position };
    Vector2 errorRate{};
    for (std::size_t step = 0; step < 1000; ++step)
    {
        const auto tau{ controller.ComputeTorque(position, velocity, setpoint, Vector2{}, Vector2{}) };
        velocity = velocity + solver.ForwardDynamics(plant, position, velocity, tau) * dt;
        position = position + velocity * dt;
        errorRate = errorRate + Vector2{ kd * errorRate + kp * error } * (-dt);
        error = error + errorRate * dt;

        EXPECT_NEAR(setpoint[0] - position[0], error[0], math::Tolerance<float>());
        EXPECT_NEAR(setpoint[1] - position[1], error[1], math::Tolerance<float>());
    }

    EXPECT_NEAR(setpoint[0] - position[0], 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(setpoint[1] - position[1], 0.0f, math::Tolerance<float>());
}
