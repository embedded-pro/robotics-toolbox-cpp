#include "robotics/kinematics/DeltaKinematics.hpp"
#include <array>
#include <cmath>
#include <gtest/gtest.h>
#include <numbers>

namespace
{
    using Vector3 = math::Vector3<float>;
    using Delta = kinematics::DeltaKinematics<float>;

    constexpr float pi{ std::numbers::pi_v<float> };

    class TestDeltaKinematics
        : public ::testing::Test
    {
    protected:
        static void ExpectNear(const Vector3& actual, const Vector3& expected, float tolerance)
        {
            for (std::size_t i = 0; i < 3; ++i)
                EXPECT_NEAR(actual[i], expected[i], tolerance);
        }

        kinematics::DeltaGeometry<float> geometry{ 0.1f, 0.03f, 0.2f, 0.45f };
        Delta delta{ geometry };
        std::array<Vector3, 3> points{ Vector3{ 0.05f, -0.03f, -0.40f }, Vector3{ -0.08f, 0.06f, -0.32f }, Vector3{ 0.12f, 0.10f, -0.45f } };
    };
}

TEST_F(TestDeltaKinematics, zero_angles_put_the_effector_on_the_axis)
{
    const auto effector{ delta.Forward(Vector3{}) };

    ASSERT_TRUE(effector.has_value());
    ExpectNear(*effector, Vector3{ 0.0f, 0.0f, -0.36f }, 1e-5f);
}

TEST_F(TestDeltaKinematics, inverse_of_known_points)
{
    const auto first{ delta.Inverse(points[0]) };
    const auto second{ delta.Inverse(points[1]) };

    ASSERT_TRUE(first.has_value());
    ASSERT_TRUE(second.has_value());
    ExpectNear(*first, Vector3{ 0.042468f, 0.371178f, 0.205749f }, 1e-5f);
    ExpectNear(*second, Vector3{ 0.195443f, -0.538250f, -0.084938f }, 1e-5f);
}

TEST_F(TestDeltaKinematics, inverse_forward_round_trip)
{
    for (const auto& point : points)
    {
        const auto angles{ delta.Inverse(point) };
        ASSERT_TRUE(angles.has_value());
        const auto effector{ delta.Forward(*angles) };
        ASSERT_TRUE(effector.has_value());
        ExpectNear(*effector, point, 1e-5f);
    }
}

TEST_F(TestDeltaKinematics, forward_inverse_round_trip)
{
    constexpr std::array<float, 4> grid{ -0.6f, 0.0f, 0.6f, 1.2f };
    for (float first : grid)
        for (float second : grid)
            for (float third : grid)
            {
                const Vector3 angles{ first, second, third };
                const auto effector{ delta.Forward(angles) };
                ASSERT_TRUE(effector.has_value());
                const auto recovered{ delta.Inverse(*effector) };
                ASSERT_TRUE(recovered.has_value());
                ExpectNear(*recovered, angles, 1e-4f);
            }
}

TEST_F(TestDeltaKinematics, forearm_constraint_holds)
{
    const auto angles{ delta.Inverse(points[0]) };
    ASSERT_TRUE(angles.has_value());

    for (std::size_t arm = 0; arm < 3; ++arm)
    {
        const float phi{ 2.0f * pi / 3.0f * static_cast<float>(arm) };
        const float elbowRadial{ geometry.baseRadius + geometry.upperArm * std::cos((*angles)[arm]) };
        const Vector3 elbow{ std::cos(phi) * elbowRadial, std::sin(phi) * elbowRadial, -geometry.upperArm * std::sin((*angles)[arm]) };
        const Vector3 attachment{ points[0][0] + std::cos(phi) * geometry.effectorRadius, points[0][1] + std::sin(phi) * geometry.effectorRadius, points[0][2] };
        EXPECT_NEAR(std::hypot(elbow[0] - attachment[0], elbow[1] - attachment[1], elbow[2] - attachment[2]), geometry.forearm, 1e-5f);
    }
}

TEST_F(TestDeltaKinematics, unreachable_point_has_no_solution)
{
    EXPECT_FALSE(delta.Inverse(Vector3{ 0.0f, 0.0f, -0.9f }).has_value());

    const Delta shortForearms{ kinematics::DeltaGeometry<float>{ 0.1f, 0.03f, 0.2f, 0.1f } };
    EXPECT_FALSE(shortForearms.Forward(Vector3{}).has_value());
}
