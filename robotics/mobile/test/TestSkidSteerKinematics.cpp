#include "robotics/mobile/DifferentialDrive.hpp"
#include "robotics/mobile/SkidSteerKinematics.hpp"
#include <cstddef>
#include <gtest/gtest.h>

namespace
{
    using Vector3 = math::Vector3<float>;
    using Wheels = mobile::WheelSpeeds<float>;
    using SkidSteer = mobile::SkidSteerKinematics<float>;

    class TestSkidSteerKinematics
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

        mobile::IcrParameters<float> identified{ 0.30f, -0.28f, 0.05f };
        SkidSteer vehicle{ identified, 0.1f, 20.0f };
        mobile::IcrEstimator<float> estimator{ 0.1f };
    };
}

TEST_F(TestSkidSteerKinematics, ideal_icr_reduces_to_differential_drive)
{
    const SkidSteer ideal{ SkidSteer::IdealIcr(0.4f), 0.1f, 20.0f };
    const mobile::DifferentialDrive<float> drive{ 0.1f, 0.4f, 20.0f };

    ExpectNear(ideal.Forward(Wheels{ 5.0f, 8.0f }), drive.Forward(Wheels{ 5.0f, 8.0f }));
    ExpectNear(ideal.Forward(Wheels{ 5.0f, 8.0f }), Vector3{ 0.65f, 0.0f, 0.75f });
}

TEST_F(TestSkidSteerKinematics, forward_with_identified_icr_turns_slower_and_slips)
{
    ExpectNear(vehicle.Forward(Wheels{ 5.0f, 8.0f }), Vector3{ 0.655172f, -0.025862f, 0.517241f });
}

TEST_F(TestSkidSteerKinematics, inverse_inverts_forward)
{
    ExpectNear(vehicle.Inverse(0.655172f, 0.517241f), Wheels{ 5.0f, 8.0f });
    ExpectNear(vehicle.Limit(Wheels{ 30.0f, 10.0f }), Wheels{ 20.0f, 6.666667f });
}

TEST_F(TestSkidSteerKinematics, estimator_recovers_the_icr_from_consistent_samples)
{
    for (const auto& sprockets : { Wheels{ 5.0f, 8.0f }, Wheels{ 8.0f, 5.0f }, Wheels{ -3.0f, 4.0f }, Wheels{ 6.0f, 6.0f }, Wheels{ 2.0f, -2.0f } })
        estimator.Add(sprockets, vehicle.Forward(sprockets));

    const auto estimate{ estimator.Estimate(0.01f) };

    EXPECT_EQ(estimator.Samples(), std::size_t{ 5 });
    ASSERT_TRUE(estimate.has_value());
    EXPECT_NEAR(estimate->yLeft, 0.30f, 1e-5f);
    EXPECT_NEAR(estimate->yRight, -0.28f, 1e-5f);
    EXPECT_NEAR(estimate->xIcr, 0.05f, 1e-5f);
}

TEST_F(TestSkidSteerKinematics, estimator_rejects_straight_driving)
{
    estimator.Add(Wheels{ 5.0f, 5.0f }, vehicle.Forward(Wheels{ 5.0f, 5.0f }));
    estimator.Add(Wheels{ 8.0f, 8.0f }, vehicle.Forward(Wheels{ 8.0f, 8.0f }));

    EXPECT_FALSE(estimator.Estimate(0.01f).has_value());
    estimator.Reset();
    EXPECT_EQ(estimator.Samples(), std::size_t{ 0 });
}
