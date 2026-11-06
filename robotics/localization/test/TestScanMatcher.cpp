#include "numerical/math/SE2Transform.hpp"
#include "robotics/localization/ScanMatcher.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <gtest/gtest.h>

namespace
{
    using Transform = math::SE2Transform<float>;
    using Point = math::Vector2<float>;
    using Matcher = localization::ScanMatcher<float, 128>;
    using localization::IcpMetric;

    template<std::size_t N>
    std::array<Point, N> Walls(float rightStart, std::size_t rightCount, float topStart, std::size_t topCount, float bottomStart, std::size_t bottomCount)
    {
        std::array<Point, N> points{};
        std::size_t k{ 0 };
        for (std::size_t i = 0; i < rightCount; ++i)
            points[k++] = Point{ 2.0f, rightStart + 0.1f * static_cast<float>(i) };
        for (std::size_t i = 0; i < topCount; ++i)
            points[k++] = Point{ topStart + 0.1f * static_cast<float>(i), 1.5f };
        for (std::size_t i = 0; i < bottomCount; ++i)
            points[k++] = Point{ bottomStart + 0.1f * static_cast<float>(i), -1.0f };
        return points;
    }

    template<std::size_t N>
    std::array<Point, N> Map(const Transform& transform, const std::array<Point, N>& points)
    {
        std::array<Point, N> mapped{};
        for (std::size_t i = 0; i < N; ++i)
            mapped[i] = transform.Apply(points[i]);
        return mapped;
    }

    class TestScanMatcher
        : public ::testing::Test
    {
    protected:
        static void ExpectNear(const Transform& actual, const Transform& expected)
        {
            const auto a{ actual.Pose() };
            const auto e{ expected.Pose() };
            for (std::size_t i = 0; i < 3; ++i)
                EXPECT_NEAR(a[i], e[i], 1e-4f);
        }

        std::array<Point, 68> reference{ Walls<68>(-1.0f, 26, -0.5f, 26, -0.5f, 16) };
        Transform truth{ Transform::FromPose(0.1f, -0.05f, 0.05f) };
        std::array<Point, 68> sameSpots{ Map(truth.Inverse(), reference) };
        std::array<Point, 65> halfSpots{ Map(truth.Inverse(), Walls<65>(-0.95f, 25, -0.45f, 25, -0.45f, 15)) };
        localization::ScanMatcherParameters<float> parameters{ IcpMetric::PointToLine, 0.5f, 50, 1e-6f, 1e-6f, 3, 3 };
    };
}

TEST_F(TestScanMatcher, both_metrics_recover_the_transform_with_exact_correspondences)
{
    auto pointToPoint{ parameters };
    pointToPoint.metric = IcpMetric::PointToPoint;

    const auto line{ Matcher{ parameters }.Match(reference, sameSpots, Transform::Identity()) };
    const auto point{ Matcher{ pointToPoint }.Match(reference, sameSpots, Transform::Identity()) };

    ASSERT_TRUE(line.has_value());
    ASSERT_TRUE(point.has_value());
    ExpectNear(line->transform, truth);
    ExpectNear(point->transform, truth);
    EXPECT_EQ(line->correspondences, 68u);
    EXPECT_EQ(point->correspondences, 68u);
    EXPECT_TRUE(line->converged);
    EXPECT_TRUE(point->converged);
    EXPECT_NEAR(line->rms, 0.0f, 1e-4f);
    EXPECT_NEAR(point->rms, 0.0f, 1e-4f);
    EXPECT_EQ(point->iterations, 3u);
    EXPECT_EQ(line->iterations, 2u);
}

TEST_F(TestScanMatcher, point_to_line_removes_the_sampling_bias)
{
    auto pointToPoint{ parameters };
    pointToPoint.metric = IcpMetric::PointToPoint;

    const auto line{ Matcher{ parameters }.Match(reference, halfSpots, Transform::Identity()) };
    const auto point{ Matcher{ pointToPoint }.Match(reference, halfSpots, Transform::Identity()) };

    ASSERT_TRUE(line.has_value());
    ExpectNear(line->transform, truth);
    EXPECT_EQ(line->correspondences, 65u);
    ASSERT_TRUE(point.has_value());
    const auto error{ (truth.Inverse() * point->transform).Pose() };
    EXPECT_GT(std::max({ std::abs(error[0]), std::abs(error[1]), std::abs(error[2]) }), 0.01f);
}

TEST_F(TestScanMatcher, point_to_line_converges_from_an_offset_guess)
{
    const auto guess{ Transform::FromPose(-0.05f, 0.05f, -0.05f) };

    const auto result{ Matcher{ parameters }.Match(reference, sameSpots, guess) };

    ASSERT_TRUE(result.has_value());
    ExpectNear(result->transform, truth);

    auto single{ parameters };
    single.maxIterations = 1;
    const auto truncated{ Matcher{ single }.Match(reference, sameSpots, guess) };
    ASSERT_TRUE(truncated.has_value());
    EXPECT_FALSE(truncated->converged);
}

TEST_F(TestScanMatcher, far_outlier_is_rejected)
{
    std::array<Point, 66> withOutlier{};
    for (std::size_t i = 0; i < halfSpots.size(); ++i)
        withOutlier[i] = halfSpots[i];
    withOutlier[65] = Point{ 0.6f, 0.2f };

    const auto result{ Matcher{ parameters }.Match(reference, withOutlier, Transform::Identity()) };

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->correspondences, 65u);
    ExpectNear(result->transform, truth);
}

TEST_F(TestScanMatcher, too_few_correspondences_fail)
{
    auto tight{ parameters };
    tight.maxCorrespondenceDistance = 0.01f;

    EXPECT_FALSE(Matcher{ tight }.Match(reference, halfSpots, Transform::Identity()).has_value());

    const std::array<Point, 129> oversized{};
    EXPECT_FALSE(Matcher{ parameters }.Match(reference, oversized, Transform::Identity()).has_value());
}

TEST_F(TestScanMatcher, corridor_is_reported_as_degenerate)
{
    std::array<Point, 42> corridor{};
    std::array<Point, 42> shifted{};
    for (std::size_t i = 0; i < 21; ++i)
    {
        const float x{ -1.0f + 0.1f * static_cast<float>(i) };
        corridor[i] = Point{ x, -1.0f };
        corridor[21 + i] = Point{ x, 1.0f };
        shifted[i] = Point{ x + 0.1f, -1.0f };
        shifted[21 + i] = Point{ x + 0.1f, 1.0f };
    }

    EXPECT_FALSE(Matcher{ parameters }.Match(corridor, shifted, Transform::Identity()).has_value());
}
