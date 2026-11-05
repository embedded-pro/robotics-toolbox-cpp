#include "numerical/math/SE2Transform.hpp"
#include "robotics/localization/OdometryFusion.hpp"
#include <cmath>
#include <cstddef>
#include <gtest/gtest.h>

namespace
{
    using Transform = math::SE2Transform<float>;
    using Fusion = localization::OdometryFusion<float>;
    using filters::KalmanUpdateStatus;

    class TestOdometryFusion
        : public ::testing::Test
    {
    protected:
        static void ExpectState(const Fusion::StateVector& actual, float x, float y, float heading, float bias)
        {
            EXPECT_NEAR(actual[0], x, 1e-5f);
            EXPECT_NEAR(actual[1], y, 1e-5f);
            EXPECT_NEAR(actual[2], heading, 1e-5f);
            EXPECT_NEAR(actual[3], bias, 1e-5f);
        }

        static void ExpectEqual(const Fusion::CovarianceMatrix& actual, const Fusion::CovarianceMatrix& expected)
        {
            for (std::size_t r = 0; r < 4; ++r)
                for (std::size_t c = 0; c < 4; ++c)
                    EXPECT_EQ(actual.at(r, c), expected.at(r, c));
        }

        localization::OdometryFusionNoise<float> noise{ 0.01f, 0.05f, 0.01f, 1e-4f, 0.02f, 6.63f };
        Fusion::CovarianceMatrix initial{ { 0.01f, 0.0f, 0.0f, 0.0f }, { 0.0f, 0.01f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.01f, 0.0f }, { 0.0f, 0.0f, 0.0f, 0.0025f } };
        Fusion fusion{ Transform::Identity(), initial, noise };
    };
}

TEST_F(TestOdometryFusion, straight_drive_grows_the_covariance_along_and_across)
{
    for (std::size_t k = 0; k < 10; ++k)
        fusion.Predict(1.0f, 0.0f, 0.1f);

    ExpectState(fusion.State(), 1.0f, 0.0f, 0.0f, 0.0f);
    EXPECT_NEAR(fusion.Covariance().at(0, 0), 0.010360f, 1e-6f);
    EXPECT_NEAR(fusion.Covariance().at(1, 1), 0.020628f, 1e-6f);
    EXPECT_NEAR(fusion.Covariance().at(2, 2), 0.012510f, 1e-6f);
    EXPECT_NEAR(fusion.Covariance().at(3, 3), 0.0025f, 1e-6f);
    EXPECT_NEAR(fusion.Covariance().at(1, 2), 0.011255f, 1e-6f);
}

TEST_F(TestOdometryFusion, arc_uses_the_exact_se2_exponential)
{
    for (std::size_t k = 0; k < 20; ++k)
        fusion.Predict(1.0f, 0.5f, 0.1f);

    const auto pose{ fusion.Pose().Pose() };
    const auto exact{ Transform::Exp(math::Vector3<float>{ 1.0f, 0.0f, 0.5f }, 2.0f).Pose() };
    for (std::size_t i = 0; i < 3; ++i)
        EXPECT_NEAR(pose[i], exact[i], 1e-5f);
    EXPECT_NEAR(pose[0], 1.682942f, 1e-5f);
    EXPECT_NEAR(pose[1], 0.919395f, 1e-5f);
    EXPECT_NEAR(pose[2], 1.0f, 1e-5f);
}

TEST_F(TestOdometryFusion, standstill_estimates_the_gyro_bias)
{
    for (std::size_t k = 0; k < 500; ++k)
    {
        fusion.Predict(0.0f, 0.02f, 0.01f);
        fusion.UpdateWheelYawRate(0.0f, 0.02f);
    }

    EXPECT_NEAR(fusion.GyroBias(), 0.019992f, 1e-4f);
    EXPECT_LT(std::abs(fusion.State()[2]), 1e-4f);
    EXPECT_NEAR(fusion.Covariance().at(3, 3), 1.02e-6f, 2e-8f);
}

TEST_F(TestOdometryFusion, slip_is_gated_and_leaves_the_state_unchanged)
{
    fusion.Predict(1.0f, 0.0f, 0.1f);
    const auto state{ fusion.State() };
    const auto covariance{ fusion.Covariance() };

    const auto result{ fusion.UpdateWheelYawRate(1.0f, 0.0f) };

    EXPECT_EQ(result.status, KalmanUpdateStatus::Gated);
    EXPECT_NEAR(result.nis, 333.33f, 0.01f);
    ExpectState(fusion.State(), state[0], state[1], state[2], state[3]);
    ExpectEqual(fusion.Covariance(), covariance);
}

TEST_F(TestOdometryFusion, heading_update_wraps_the_residual)
{
    Fusion reversed{ Transform::FromPose(0.0f, 0.0f, 3.1f), initial, noise };

    const auto result{ reversed.UpdateHeading(-3.1f, 0.05f) };

    EXPECT_EQ(result.status, KalmanUpdateStatus::Applied);
    EXPECT_NEAR(reversed.State()[2], -3.116637f, 1e-5f);
}

TEST_F(TestOdometryFusion, position_update_pulls_the_pose_and_shrinks_the_covariance)
{
    const math::SquareMatrix<float, 2> gnss{ { 0.01f, 0.0f }, { 0.0f, 0.01f } };

    const auto result{ fusion.UpdatePosition(math::Vector2<float>{ 0.1f, -0.05f }, gnss) };

    ExpectState(fusion.State(), 0.05f, -0.025f, 0.0f, 0.0f);
    EXPECT_NEAR(fusion.Covariance().at(0, 0), 0.005f, 1e-6f);
    EXPECT_NEAR(fusion.Covariance().at(1, 1), 0.005f, 1e-6f);
    EXPECT_NEAR(result.nis, 0.625f, 1e-5f);
}
