#include "numerical/math/SE2Transform.hpp"
#include "robotics/mobile/CarLikeKinematics.hpp"
#include <cmath>
#include <cstddef>
#include <gtest/gtest.h>

namespace
{
    using Vector3 = math::Vector3<float>;

    class TestCarLikeKinematics
        : public ::testing::Test
    {
    protected:
        static void ExpectNear(const Vector3& actual, const Vector3& expected)
        {
            for (std::size_t i = 0; i < 3; ++i)
                EXPECT_NEAR(actual[i], expected[i], 1e-5f);
        }

        mobile::CarLikeKinematics<float> car{ { 2.5f, 1.5f, 0.6f, 0.5f } };
    };
}

TEST_F(TestCarLikeKinematics, forward_gives_the_rear_axle_twist)
{
    ExpectNear(car.Forward(10.0f, 0.1f), Vector3{ 10.0f, 0.0f, 0.401339f });
}

TEST_F(TestCarLikeKinematics, twist_at_offset_matches_the_se2_adjoint)
{
    const auto twist{ car.TwistAt(10.0f, 0.1f, 1.2f) };

    ExpectNear(twist, Vector3{ 10.0f, 0.481606f, 0.401339f });
    ExpectNear(twist, math::SE2Transform<float>::FromPose(-1.2f, 0.0f, 0.0f).TransformTwist(car.Forward(10.0f, 0.1f)));
}

TEST_F(TestCarLikeKinematics, steer_for_curvature_inverts_curvature_and_saturates)
{
    EXPECT_NEAR(car.SteerForCurvature(0.1f), 0.244979f, 1e-5f);
    EXPECT_NEAR(car.Curvature(0.244979f), 0.1f, 1e-5f);
    EXPECT_EQ(car.SteerForCurvature(0.5f), 0.6f);
}

TEST_F(TestCarLikeKinematics, limit_steer_applies_angle_then_rate)
{
    EXPECT_NEAR(car.LimitSteer(0.5f, 0.1f, 0.2f), 0.2f, 1e-6f);
    EXPECT_NEAR(car.LimitSteer(0.9f, 0.55f, 1.0f), 0.6f, 1e-6f);
    EXPECT_NEAR(car.LimitSteer(-0.9f, 0.55f, 1.0f), 0.05f, 1e-6f);
}

TEST_F(TestCarLikeKinematics, ackermann_wheels_share_one_turning_centre)
{
    const auto angles{ car.Ackermann(0.3f) };
    const float radius{ 2.5f / std::tan(0.3f) };

    EXPECT_NEAR(angles.left, 0.328616f, 1e-5f);
    EXPECT_NEAR(angles.right, 0.275851f, 1e-5f);
    EXPECT_NEAR(std::tan(angles.left) * (radius - 0.75f), 2.5f, 1e-4f);
    EXPECT_NEAR(std::tan(angles.right) * (radius + 0.75f), 2.5f, 1e-4f);
    EXPECT_NEAR(car.Ackermann(-0.3f).left, -0.275851f, 1e-5f);
    EXPECT_NEAR(car.Ackermann(-0.3f).right, -0.328616f, 1e-5f);
    EXPECT_EQ(car.Ackermann(0.0f).left, 0.0f);
    EXPECT_EQ(car.Ackermann(0.0f).right, 0.0f);
}

TEST_F(TestCarLikeKinematics, min_turning_radius_uses_max_steer)
{
    EXPECT_NEAR(car.MinTurningRadius(), 3.654240f, 1e-5f);
    EXPECT_EQ(car.Parameters().wheelBase, 2.5f);
}
