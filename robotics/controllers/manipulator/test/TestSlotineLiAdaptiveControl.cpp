#include "robotics/controllers/manipulator/SlotineLiAdaptiveControl.hpp"
#include "robotics/dynamics/test_doubles/MockInertialRegressor.hpp"
#include <cmath>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace
{
    using ::testing::_;
    using ::testing::Invoke;
    using ::testing::Return;
    using Vector2 = math::Vector<float, 2>;
    using Vector3 = math::Vector<float, 3>;
    using Matrix2 = math::SquareMatrix<float, 2>;
    using Matrix3 = math::SquareMatrix<float, 3>;
    using Regressor = math::Matrix<float, 2, 3>;
    using Controller = controllers::SlotineLiAdaptiveControl<float, 2, 3>;

    MATCHER_P(VectorNear, expected, "")
    {
        for (std::size_t i = 0; i < 2; ++i)
            if (std::abs(arg[i] - expected[i]) > 1e-5f)
                return false;
        return true;
    }

    Regressor PlanarArmRegressor(const Vector2& q, const Vector2& qDot, const Vector2& qrDot, const Vector2& qrDdot)
    {
        const float c2{ std::cos(q[1]) };
        const float s2{ std::sin(q[1]) };
        return Regressor{ { qrDdot[0], c2 * (2.0f * qrDdot[0] + qrDdot[1]) - s2 * (qDot[1] * qrDot[0] + (qDot[0] + qDot[1]) * qrDot[1]), qrDdot[1] },
            { 0.0f, c2 * qrDdot[0] + s2 * qDot[0] * qrDot[0], qrDdot[0] + qrDdot[1] } };
    }

    class TestSlotineLiAdaptiveControl
        : public ::testing::Test
    {
    protected:
        void ExpectNear(const Vector2& actual, const Vector2& expected, float tolerance = 1e-5f) const
        {
            for (std::size_t i = 0; i < 2; ++i)
                EXPECT_NEAR(actual[i], expected[i], tolerance);
        }

        void ExpectNear(const Vector3& actual, const Vector3& expected) const
        {
            for (std::size_t i = 0; i < 3; ++i)
                EXPECT_NEAR(actual[i], expected[i], 1e-6f);
        }

        Vector3 OneStep(const Vector2& sliding, const Vector3& start) const
        {
            return Vector3{ start - Vector3{ gamma * Vector3{ y.Transpose() * sliding } } * dt };
        }

        ::testing::StrictMock<dynamics::MockInertialRegressor<float, 2, 3>> regressor;
        Matrix2 lambda{ { 5.0f, 0.0f }, { 0.0f, 5.0f } };
        Matrix2 kd{ { 20.0f, 0.0f }, { 0.0f, 20.0f } };
        Matrix3 gamma{ Matrix3::Identity() };
        Vector3 initialEstimate{};
        float dt{ 0.001f };
        Controller controller{ regressor, lambda, kd, gamma, initialEstimate, dt };
        Regressor y{ { 1.0f, 0.5f, -0.2f }, { 0.3f, -0.4f, 0.8f } };
        Vector2 q{ 0.4f, -0.3f };
        Vector2 qDot{ 0.2f, 0.1f };
        Vector2 qd{ 0.35f, -0.2f };
        Vector2 qdDot{ 0.1f, 0.3f };
        Vector2 qdDdot{ 0.5f, -0.6f };
        Vector2 sliding{ Vector2{ qDot - qdDot } + Vector2{ lambda * Vector2{ q - qd } } };
    };
}

TEST_F(TestSlotineLiAdaptiveControl, zero_error_uses_feedforward_only)
{
    const Vector3 estimate{ 2.0f, 0.5f, 0.7f };
    controller.Reset(estimate);
    EXPECT_CALL(regressor, Compute(_, _, _, _)).WillOnce(Return(y));

    const auto tau{ controller.ComputeTorque(qd, qdDot, qd, qdDot, qdDdot) };

    ExpectNear(tau, Vector2{ y * estimate });
}

TEST_F(TestSlotineLiAdaptiveControl, sliding_variable_damping)
{
    EXPECT_CALL(regressor, Compute(_, _, _, _)).WillOnce(Return(Regressor{}));

    const auto tau{ controller.ComputeTorque(qd, qDot, qd, qdDot, qdDdot) };

    ExpectNear(tau, Vector2{ kd * Vector2{ qDot - qdDot } } * -1.0f);
}

TEST_F(TestSlotineLiAdaptiveControl, sliding_variable_definition)
{
    EXPECT_CALL(regressor, Compute(_, _, _, _)).WillOnce(Return(Regressor{}));

    const auto tau{ controller.ComputeTorque(q, qDot, qd, qdDot, qdDdot) };

    ExpectNear(Vector2{ tau * (-1.0f / 20.0f) }, Vector2{ 0.1f + 5.0f * 0.05f, -0.2f + 5.0f * -0.1f });
}

TEST_F(TestSlotineLiAdaptiveControl, reference_motion_uses_lambda)
{
    const Vector2 qrDot{ Vector2{ qdDot - Vector2{ lambda * Vector2{ q - qd } } } };
    const Vector2 qrDdot{ Vector2{ qdDdot - Vector2{ lambda * Vector2{ qDot - qdDot } } } };
    EXPECT_CALL(regressor, Compute(VectorNear(q), VectorNear(qDot), VectorNear(qrDot), VectorNear(qrDdot))).WillOnce(Return(y));

    static_cast<void>(controller.ComputeTorque(q, qDot, qd, qdDot, qdDdot));

    ExpectNear(qrDot, Vector2{ -0.15f, 0.8f });
    ExpectNear(qrDdot, Vector2{ 0.0f, 0.4f });
}

TEST_F(TestSlotineLiAdaptiveControl, parameter_estimate_adapts)
{
    EXPECT_CALL(regressor, Compute(_, _, _, _)).WillOnce(Return(y));

    static_cast<void>(controller.ComputeTorque(q, qDot, qd, qdDot, qdDdot));

    ExpectNear(controller.ParameterEstimate(), OneStep(sliding, initialEstimate));
}

TEST_F(TestSlotineLiAdaptiveControl, no_adaptation_on_zero_sliding)
{
    EXPECT_CALL(regressor, Compute(_, _, _, _)).WillOnce(Return(y));

    static_cast<void>(controller.ComputeTorque(qd, qdDot, qd, qdDot, qdDdot));

    ExpectNear(controller.ParameterEstimate(), initialEstimate);
}

TEST_F(TestSlotineLiAdaptiveControl, estimate_persists_across_calls)
{
    EXPECT_CALL(regressor, Compute(_, _, _, _)).Times(2).WillRepeatedly(Return(y));

    static_cast<void>(controller.ComputeTorque(q, qDot, qd, qdDot, qdDdot));
    static_cast<void>(controller.ComputeTorque(q, qDot, qd, qdDot, qdDdot));

    ExpectNear(controller.ParameterEstimate(), OneStep(sliding, OneStep(sliding, initialEstimate)));
}

TEST_F(TestSlotineLiAdaptiveControl, convergence_tracks_trajectory)
{
    const Vector3 truth{ 3.0f, 0.5f, 0.6f };
    EXPECT_CALL(regressor, Compute(_, _, _, _)).WillRepeatedly(Invoke(PlanarArmRegressor));
    Vector2 position{ 0.2f, -0.1f };
    Vector2 velocity{};
    float error{ 0.0f };

    for (std::size_t k = 0; k < 10000; ++k)
    {
        const float time{ static_cast<float>(k) * dt };
        const Vector2 desired{ 0.5f * std::sin(time), 0.3f * std::cos(2.0f * time) };
        const Vector2 desiredRate{ 0.5f * std::cos(time), -0.6f * std::sin(2.0f * time) };
        const Vector2 desiredAcceleration{ -0.5f * std::sin(time), -1.2f * std::cos(2.0f * time) };
        const auto tau{ controller.ComputeTorque(position, velocity, desired, desiredRate, desiredAcceleration) };

        const float c2{ std::cos(position[1]) };
        const float s2{ std::sin(position[1]) };
        const Matrix2 mass{ { truth[0] + 2.0f * truth[1] * c2, truth[2] + truth[1] * c2 }, { truth[2] + truth[1] * c2, truth[2] } };
        const Matrix2 coriolis{ { -truth[1] * s2 * velocity[1], -truth[1] * s2 * (velocity[0] + velocity[1]) }, { truth[1] * s2 * velocity[0], 0.0f } };
        const Vector2 net{ tau - coriolis * velocity };
        const float determinant{ mass.at(0, 0) * mass.at(1, 1) - mass.at(0, 1) * mass.at(1, 0) };
        const Vector2 acceleration{ (mass.at(1, 1) * net[0] - mass.at(0, 1) * net[1]) / determinant, (mass.at(0, 0) * net[1] - mass.at(1, 0) * net[0]) / determinant };
        velocity = velocity + acceleration * dt;
        position = position + velocity * dt;
        error = std::hypot(position[0] - 0.5f * std::sin(time + dt), position[1] - 0.3f * std::cos(2.0f * (time + dt)));
    }

    EXPECT_LT(error, 1e-2f);
    EXPECT_GT(std::abs(controller.ParameterEstimate()[0] - truth[0]), 0.1f);
}

TEST_F(TestSlotineLiAdaptiveControl, reset_restores_initial_estimate)
{
    EXPECT_CALL(regressor, Compute(_, _, _, _)).WillOnce(Return(y));
    static_cast<void>(controller.ComputeTorque(q, qDot, qd, qdDot, qdDdot));

    controller.Reset(initialEstimate);

    ExpectNear(controller.ParameterEstimate(), initialEstimate);
}

TEST_F(TestSlotineLiAdaptiveControl, regressor_queried_once_per_call)
{
    EXPECT_CALL(regressor, Compute(_, _, _, _)).WillOnce(Return(y));

    static_cast<void>(controller.ComputeTorque(q, qDot, qd, qdDot, qdDdot));
}
