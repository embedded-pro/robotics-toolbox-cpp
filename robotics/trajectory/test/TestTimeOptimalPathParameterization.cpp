#include "robotics/dynamics/test_doubles/MockInverseDynamicsModel.hpp"
#include "robotics/trajectory/TimeOptimalPathParameterization.hpp"
#include "robotics/trajectory/test_doubles/MockPathConstraints.hpp"
#include "robotics/trajectory/test_doubles/MockPathGeometry.hpp"
#include <cmath>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace
{
    using Vector2 = math::Vector<float, 2>;
    using Coefficients = trajectory::PathCoefficients<float, 2>;
    using Bounds = trajectory::JointBounds<float, 2>;
    using Topp = trajectory::TimeOptimalPathParameterization<float, 2, 64>;

    MATCHER_P(VectorNear, expected, "")
    {
        return std::abs(arg[0] - expected[0]) < 1e-6f && std::abs(arg[1] - expected[1]) < 1e-6f;
    }

    Vector2 CurvedPosition(float s)
    {
        return Vector2{ s, 0.4f * s - 0.4f * s * s };
    }

    Vector2 CurvedFirstDerivative(float s)
    {
        return Vector2{ 1.0f, 0.4f - 0.8f * s };
    }

    Coefficients CoupledCoefficients(float s)
    {
        return Coefficients{ Vector2{ 1.0f + 0.5f * s, 0.4f - 0.8f * s }, Vector2{ 0.3f * std::cos(3.0f * s), -0.2f + 0.1f * s }, Vector2{ 0.1f, -0.05f * s } };
    }

    class TestTimeOptimalPathParameterization
        : public ::testing::Test
    {
    protected:
        void GivenStraightSingleJointPath(float length, const Bounds& jointBounds, const Vector2& holdingTorque = Vector2{})
        {
            bounds = jointBounds;
            EXPECT_CALL(path, Position(testing::_)).WillRepeatedly(testing::Invoke([length](float s)
                {
                    return Vector2{ length * s, 0.0f };
                }));
            EXPECT_CALL(path, FirstDerivative(testing::_)).WillRepeatedly(testing::Return(Vector2{ length, 0.0f }));
            EXPECT_CALL(path, SecondDerivative(testing::_)).WillRepeatedly(testing::Return(Vector2{}));
            EXPECT_CALL(constraints, Coefficients(testing::_)).WillRepeatedly(testing::Return(Coefficients{ Vector2{ length, 0.0f }, Vector2{}, holdingTorque }));
            EXPECT_CALL(constraints, Bounds()).WillRepeatedly(testing::ReturnRef(bounds));
        }

        float TimeAt(float s) const
        {
            float low{ 0.0f };
            float high{ topp.TotalTime() };
            for (std::size_t k = 0; k < 40; ++k)
            {
                const float middle{ (low + high) / 2.0f };
                if (topp.Sample(middle)->position[0] < s)
                    low = middle;
                else
                    high = middle;
            }
            return (low + high) / 2.0f;
        }

        static void ExpectNear(const Vector2& actual, const Vector2& expected, float tolerance)
        {
            EXPECT_NEAR(actual[0], expected[0], tolerance);
            EXPECT_NEAR(actual[1], expected[1], tolerance);
        }

        testing::StrictMock<trajectory::MockPathGeometry<float, 2>> path;
        testing::StrictMock<trajectory::MockPathConstraints<float, 2>> constraints;
        Topp topp{ path, constraints };
        Bounds unitTorque{ Vector2{ -1.0f, -1.0f }, Vector2{ 1.0f, 1.0f }, Vector2{ 100.0f, 100.0f } };
        Bounds bounds{};
    };
}

TEST_F(TestTimeOptimalPathParameterization, bang_bang_total_time_matches_closed_form)
{
    GivenStraightSingleJointPath(1.0f, unitTorque);

    ASSERT_TRUE(topp.Parameterize());
    EXPECT_NEAR(topp.TotalTime(), 2.0f, 1e-3f);
}

TEST_F(TestTimeOptimalPathParameterization, bang_bang_acceleration_saturates_torque)
{
    GivenStraightSingleJointPath(4.0f, unitTorque);

    ASSERT_TRUE(topp.Parameterize());
    const float total{ topp.TotalTime() };
    for (std::size_t k = 0; k <= 45; ++k)
    {
        const float t{ total * 0.01f * static_cast<float>(k) };
        EXPECT_NEAR(topp.Sample(t)->acceleration[0], 1.0f, 1e-3f);
        EXPECT_NEAR(topp.Sample(total - t)->acceleration[0], -1.0f, 1e-3f);
    }
}

TEST_F(TestTimeOptimalPathParameterization, velocity_limit_caps_path_speed)
{
    GivenStraightSingleJointPath(1.0f, Bounds{ Vector2{ -100.0f, -100.0f }, Vector2{ 100.0f, 100.0f }, Vector2{ 0.5f, 0.5f } });

    ASSERT_TRUE(topp.Parameterize());
    const float total{ topp.TotalTime() };
    for (std::size_t k = 0; k <= 200; ++k)
        EXPECT_LE(std::abs(topp.Sample(total * 0.005f * static_cast<float>(k))->velocity[0]), 0.5f * (1.0f + 1e-4f));
    EXPECT_NEAR(topp.Sample(total / 2.0f)->velocity[0], 0.5f, 1e-4f);
    EXPECT_NEAR(total, 2.063492f, 1e-4f);
}

TEST_F(TestTimeOptimalPathParameterization, profile_respects_torque_and_velocity_bounds)
{
    bounds = Bounds{ Vector2{ -1.0f, -0.6f }, Vector2{ 1.0f, 0.6f }, Vector2{ 1.5f, 0.8f } };
    EXPECT_CALL(path, Position(testing::_)).WillRepeatedly(testing::Invoke(CurvedPosition));
    EXPECT_CALL(path, FirstDerivative(testing::_)).WillRepeatedly(testing::Invoke(CurvedFirstDerivative));
    EXPECT_CALL(path, SecondDerivative(testing::_)).WillRepeatedly(testing::Return(Vector2{ 0.0f, -0.8f }));
    EXPECT_CALL(constraints, Coefficients(testing::_)).WillRepeatedly(testing::Invoke(CoupledCoefficients));
    EXPECT_CALL(constraints, Bounds()).WillRepeatedly(testing::ReturnRef(bounds));

    ASSERT_TRUE(topp.Parameterize());
    EXPECT_NEAR(topp.TotalTime(), 2.256496f, 1e-4f);
    for (std::size_t k = 0; k <= 1000; ++k)
    {
        const auto state{ *topp.Sample(topp.TotalTime() * 0.001f * static_cast<float>(k)) };
        const float speed{ state.velocity[0] };
        const auto coefficients{ CoupledCoefficients(state.position[0]) };
        for (std::size_t j = 0; j < 2; ++j)
        {
            const float torque{ coefficients.a[j] * state.acceleration[0] + coefficients.b[j] * speed * speed + coefficients.c[j] };
            EXPECT_GE(torque, bounds.torqueMin[j] - 0.02f);
            EXPECT_LE(torque, bounds.torqueMax[j] + 0.02f);
            EXPECT_LE(std::abs(state.velocity[j]), bounds.velocityMax[j] + 1e-4f);
        }
    }
}

TEST_F(TestTimeOptimalPathParameterization, starts_and_ends_at_rest_on_path_endpoints)
{
    GivenStraightSingleJointPath(1.0f, unitTorque);

    ASSERT_TRUE(topp.Parameterize());
    const auto start{ *topp.Sample(0.0f) };
    const auto end{ *topp.Sample(topp.TotalTime()) };
    ExpectNear(start.position, Vector2{}, 1e-6f);
    ExpectNear(start.velocity, Vector2{}, 1e-6f);
    ExpectNear(end.position, Vector2{ 1.0f, 0.0f }, 1e-5f);
    ExpectNear(end.velocity, Vector2{}, 1e-4f);
}

TEST_F(TestTimeOptimalPathParameterization, sample_is_continuous_across_stages)
{
    GivenStraightSingleJointPath(1.0f, unitTorque);

    ASSERT_TRUE(topp.Parameterize());
    const float delta{ 1e-4f * topp.TotalTime() };
    for (std::size_t i = 1; i < 63; ++i)
    {
        const float t{ TimeAt(static_cast<float>(i) / 63.0f) };
        const auto before{ *topp.Sample(t - delta) };
        const auto after{ *topp.Sample(t + delta) };
        ExpectNear(before.position, after.position, 1e-3f);
        ExpectNear(before.velocity, after.velocity, 1e-3f);
    }
}

TEST_F(TestTimeOptimalPathParameterization, infeasible_static_torque_reports_failure)
{
    GivenStraightSingleJointPath(1.0f, unitTorque, Vector2{ 2.0f, 0.0f });

    EXPECT_FALSE(topp.Parameterize());
    EXPECT_FALSE(topp.Sample(0.0f).has_value());
}

TEST_F(TestTimeOptimalPathParameterization, stationary_joint_overload_empties_controllable_set)
{
    GivenStraightSingleJointPath(1.0f, unitTorque, Vector2{ 0.0f, 2.0f });

    EXPECT_FALSE(topp.Parameterize());
    EXPECT_FALSE(topp.Sample(0.0f).has_value());
}

TEST_F(TestTimeOptimalPathParameterization, zero_velocity_bound_stalls_at_rest)
{
    GivenStraightSingleJointPath(1.0f, Bounds{ unitTorque.torqueMin, unitTorque.torqueMax, Vector2{ 0.0f, 100.0f } });

    EXPECT_FALSE(topp.Parameterize());
    EXPECT_FALSE(topp.Sample(0.0f).has_value());
}

TEST_F(TestTimeOptimalPathParameterization, sample_before_parameterize_returns_nullopt)
{
    EXPECT_FALSE(topp.Sample(0.0f).has_value());
}

TEST_F(TestTimeOptimalPathParameterization, zero_length_path_takes_zero_time)
{
    bounds = unitTorque;
    EXPECT_CALL(path, Position(testing::_)).WillRepeatedly(testing::Return(Vector2{ 0.3f, -0.2f }));
    EXPECT_CALL(path, FirstDerivative(testing::_)).WillRepeatedly(testing::Return(Vector2{}));
    EXPECT_CALL(path, SecondDerivative(testing::_)).WillRepeatedly(testing::Return(Vector2{}));
    EXPECT_CALL(constraints, Coefficients(testing::_)).WillRepeatedly(testing::Return(Coefficients{}));
    EXPECT_CALL(constraints, Bounds()).WillRepeatedly(testing::ReturnRef(bounds));

    ASSERT_TRUE(topp.Parameterize());
    EXPECT_EQ(topp.TotalTime(), 0.0f);
    const auto state{ *topp.Sample(0.0f) };
    ExpectNear(state.position, Vector2{ 0.3f, -0.2f }, 1e-6f);
    ExpectNear(state.velocity, Vector2{}, 1e-6f);
    ExpectNear(state.acceleration, Vector2{}, 1e-6f);
}

TEST_F(TestTimeOptimalPathParameterization, inverse_dynamics_adapter_builds_coefficients)
{
    testing::StrictMock<dynamics::MockInverseDynamicsModel<float, 2>> model;
    const Vector2 q{ 0.1f, 0.2f };
    const Vector2 firstDerivative{ 1.0f, 2.0f };
    const Vector2 secondDerivative{ 3.0f, 4.0f };
    EXPECT_CALL(path, Position(0.5f)).WillOnce(testing::Return(q));
    EXPECT_CALL(path, FirstDerivative(0.5f)).WillOnce(testing::Return(firstDerivative));
    EXPECT_CALL(path, SecondDerivative(0.5f)).WillOnce(testing::Return(secondDerivative));
    EXPECT_CALL(model, ComputeInverseDynamics(VectorNear(q), VectorNear(Vector2{}), VectorNear(Vector2{}))).WillOnce(testing::Return(Vector2{ 0.5f, 0.25f }));
    EXPECT_CALL(model, ComputeInverseDynamics(VectorNear(q), VectorNear(Vector2{}), VectorNear(firstDerivative))).WillOnce(testing::Return(Vector2{ 1.5f, 2.25f }));
    EXPECT_CALL(model, ComputeInverseDynamics(VectorNear(q), VectorNear(firstDerivative), VectorNear(secondDerivative))).WillOnce(testing::Return(Vector2{ 4.5f, 5.25f }));

    const trajectory::InverseDynamicsPathConstraints<float, 2> adapter{ path, model, unitTorque };
    const auto coefficients{ adapter.Coefficients(0.5f) };
    ExpectNear(coefficients.a, Vector2{ 1.0f, 2.0f }, 1e-6f);
    ExpectNear(coefficients.b, Vector2{ 4.0f, 5.0f }, 1e-6f);
    ExpectNear(coefficients.c, Vector2{ 0.5f, 0.25f }, 1e-6f);
    ExpectNear(adapter.Bounds().torqueMax, unitTorque.torqueMax, 0.0f);
}
