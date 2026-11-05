#include "robotics/mobile/DifferentialDrive.hpp"
#include <cstddef>
#include <gtest/gtest.h>

namespace
{
    using Vector3 = math::Vector3<float>;
    using Wheels = mobile::WheelSpeeds<float>;

    class TestDifferentialDrive
        : public ::testing::Test
    {
    protected:
        static void ExpectNear(const Vector3& actual, const Vector3& expected)
        {
            for (std::size_t i = 0; i < 3; ++i)
                EXPECT_NEAR(actual[i], expected[i], 1e-5f);
        }

        static void ExpectNear(const Wheels& actual, const Wheels& expected)
        {
            EXPECT_NEAR(actual.left, expected.left, 1e-4f);
            EXPECT_NEAR(actual.right, expected.right, 1e-4f);
        }

        mobile::DifferentialDrive<float> drive{ 0.05f, 0.3f, 20.0f };
    };
}

TEST_F(TestDifferentialDrive, forward_maps_wheel_speeds_to_a_unicycle_twist)
{
    ExpectNear(drive.Forward(Wheels{ 10.0f, 14.0f }), Vector3{ 0.6f, 0.0f, 0.666667f });
}

TEST_F(TestDifferentialDrive, inverse_inverts_forward)
{
    ExpectNear(drive.Inverse(0.6f, 0.666667f), Wheels{ 10.0f, 14.0f });
}

TEST_F(TestDifferentialDrive, spin_in_place_counter_rotates_the_wheels)
{
    ExpectNear(drive.Inverse(0.0f, 2.0f), Wheels{ -6.0f, 6.0f });
    ExpectNear(drive.Forward(Wheels{ -6.0f, 6.0f }), Vector3{ 0.0f, 0.0f, 2.0f });
}

TEST_F(TestDifferentialDrive, limit_preserves_curvature)
{
    ExpectNear(drive.Inverse(1.2f, 2.0f), Wheels{ 18.0f, 30.0f });
    ExpectNear(drive.Limit(Wheels{ 18.0f, 30.0f }), Wheels{ 12.0f, 20.0f });
    ExpectNear(drive.Achievable(1.2f, 2.0f), Vector3{ 0.8f, 0.0f, 1.333333f });
}

TEST_F(TestDifferentialDrive, limit_leaves_feasible_speeds_unchanged)
{
    const auto limited{ drive.Limit(Wheels{ 10.0f, -14.0f }) };

    EXPECT_EQ(limited.left, 10.0f);
    EXPECT_EQ(limited.right, -14.0f);
}

TEST_F(TestDifferentialDrive, speed_envelope_corners)
{
    EXPECT_NEAR(drive.MaxLinearSpeed(), 1.0f, 1e-5f);
    EXPECT_NEAR(drive.MaxAngularSpeed(), 6.666667f, 1e-5f);
    ExpectNear(drive.Achievable(1.0f, 0.0f), Vector3{ 1.0f, 0.0f, 0.0f });
    ExpectNear(drive.Achievable(0.0f, 6.666667f), Vector3{ 0.0f, 0.0f, 6.666667f });
}
