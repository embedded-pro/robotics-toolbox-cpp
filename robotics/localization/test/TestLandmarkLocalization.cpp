#include "numerical/math/AngleUtilities.hpp"
#include "numerical/math/SE2Transform.hpp"
#include "robotics/localization/LandmarkLocalization.hpp"
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <gtest/gtest.h>
#include <numbers>
#include <optional>

namespace
{
    using Transform = math::SE2Transform<float>;
    using Point = math::Vector2<float>;
    using Vector3 = math::Vector3<float>;
    using Landmark = localization::Landmark<float>;
    using Observation = localization::RangeBearing<float>;
    using Localization = localization::LandmarkLocalization<float, 8>;
    using filters::KalmanUpdateStatus;

    constexpr float pi{ std::numbers::pi_v<float> };

    class TestLandmarkLocalization
        : public ::testing::Test
    {
    protected:
        TestLandmarkLocalization()
        {
            filter.SetMap(landmarks);
        }

        Observation Measure(std::size_t landmark, std::optional<std::uint16_t> id) const
        {
            const float dx{ landmarks[landmark].position[0] - truth[0] };
            const float dy{ landmarks[landmark].position[1] - truth[1] };
            return Observation{ std::hypot(dx, dy), math::WrapToPi(std::atan2(dy, dx) - truth[2]), id };
        }

        static void ExpectPose(const Localization& actual, float x, float y, float heading)
        {
            const auto pose{ actual.Pose().Pose() };
            EXPECT_NEAR(pose[0], x, 1e-5f);
            EXPECT_NEAR(pose[1], y, 1e-5f);
            EXPECT_NEAR(pose[2], heading, 1e-5f);
        }

        static void ExpectDiagonal(const Localization& actual, float xx, float yy, float headingVariance)
        {
            EXPECT_NEAR(actual.Covariance().at(0, 0), xx, 1e-6f);
            EXPECT_NEAR(actual.Covariance().at(1, 1), yy, 1e-6f);
            EXPECT_NEAR(actual.Covariance().at(2, 2), headingVariance, 1e-6f);
        }

        std::array<Landmark, 4> landmarks{ Landmark{ Point{ 5.0f, 0.0f }, 0 }, Landmark{ Point{ 0.0f, 5.0f }, 1 }, Landmark{ Point{ 5.0f, 5.0f }, 2 }, Landmark{ Point{ -3.0f, 2.0f }, 3 } };
        localization::LandmarkLocalizationNoise<float> noise{ { 0.05f, 0.01f, 0.05f, 0.01f }, 0.1f, 0.02f, 9.21f };
        Localization::CovarianceMatrix initial{ { 0.05f, 0.0f, 0.0f }, { 0.0f, 0.05f, 0.0f }, { 0.0f, 0.0f, 0.01f } };
        Localization filter{ Transform::FromPose(1.0f, 1.0f, 0.3f), initial, noise };
        Vector3 truth{ 1.1f, 0.9f, 0.32f };
    };
}

TEST_F(TestLandmarkLocalization, predict_composes_the_increment_and_grows_the_covariance)
{
    filter.Predict(Transform::FromPose(0.5f, 0.0f, 0.1f));

    ExpectPose(filter, 1.477668f, 1.147760f, 0.4f);
    ExpectDiagonal(filter, 0.062818f, 0.064882f, 0.013f);
    EXPECT_NEAR(filter.Covariance().at(0, 1), -0.000706f, 1e-6f);
    EXPECT_NEAR(filter.Covariance().at(1, 2), 0.004777f, 1e-6f);
}

TEST_F(TestLandmarkLocalization, reverse_motion_has_the_same_noise_as_forward)
{
    Localization origin{ Transform::Identity(), Localization::CovarianceMatrix{}, noise };

    origin.Predict(Transform::FromPose(-0.5f, 0.0f, 0.0f));

    ExpectPose(origin, -0.5f, 0.0f, 0.0f);
    ExpectDiagonal(origin, 0.0125f, 0.0125f, 0.0025f);
}

TEST_F(TestLandmarkLocalization, unlabelled_observation_is_associated_by_mahalanobis_distance)
{
    const auto observation{ Measure(2, std::nullopt) };
    EXPECT_NEAR(*filter.Nis(observation, 0), 119.6f, 0.1f);
    EXPECT_NEAR(*filter.Nis(observation, 1), 118.1f, 0.1f);
    EXPECT_NEAR(*filter.Nis(observation, 3), 371.8f, 0.1f);

    const auto update{ filter.Update(observation) };

    ASSERT_TRUE(update.landmark.has_value());
    EXPECT_EQ(*update.landmark, 2u);
    EXPECT_EQ(update.result.status, KalmanUpdateStatus::Applied);
    EXPECT_NEAR(update.result.nis, 0.002138f, 1e-5f);
    ExpectPose(filter, 1.001568f, 0.996349f, 0.295825f);
}

TEST_F(TestLandmarkLocalization, labelled_observations_converge_towards_the_truth)
{
    for (const std::uint16_t id : { 2, 0, 1, 3 })
        filter.Update(Measure(id, id));

    ExpectPose(filter, 1.092522f, 0.904252f, 0.318948f);
    ExpectDiagonal(filter, 0.002722f, 0.002077f, 0.000122f);
}

TEST_F(TestLandmarkLocalization, outlier_is_gated)
{
    const auto covariance{ filter.Covariance() };

    const auto update{ filter.Update(Observation{ 0.5f, 2.5f, std::nullopt }) };

    EXPECT_EQ(update.result.status, KalmanUpdateStatus::Gated);
    EXPECT_NEAR(update.result.nis, 219.48f, 0.01f);
    ASSERT_TRUE(update.landmark.has_value());
    EXPECT_EQ(*update.landmark, 3u);
    ExpectPose(filter, 1.0f, 1.0f, 0.3f);
    ExpectDiagonal(filter, covariance.at(0, 0), covariance.at(1, 1), covariance.at(2, 2));

    const auto unknown{ filter.Update(Observation{ 1.0f, 0.0f, std::uint16_t{ 9 } }) };
    EXPECT_EQ(unknown.result.status, KalmanUpdateStatus::Singular);
    EXPECT_FALSE(unknown.landmark.has_value());

    const std::array<Landmark, 9> tooMany{};
    EXPECT_FALSE(filter.SetMap(tooMany));
}

TEST_F(TestLandmarkLocalization, bearing_residual_is_wrapped_across_pi)
{
    Localization nearSeam{ Transform::FromPose(6.0f, 4.9f, 0.0f), initial, noise };
    nearSeam.SetMap(landmarks);

    const auto update{ nearSeam.Update(Observation{ 6.0f, -pi + 0.01f, std::uint16_t{ 1 } }) };

    EXPECT_EQ(update.result.status, KalmanUpdateStatus::Applied);
    EXPECT_NEAR(update.result.nis, 0.060327f, 1e-5f);
    ExpectPose(nearSeam, 5.999620f, 4.918856f, -0.022620f);
}
