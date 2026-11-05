#include "robotics/mobile/PlanarPath.hpp"
#include <array>
#include <gtest/gtest.h>
#include <numbers>

namespace
{
    using Point = math::Vector2<float>;
    using Path = mobile::PlanarPath<float, 8>;

    constexpr float pi{ std::numbers::pi_v<float> };

    class TestPlanarPath
        : public ::testing::Test
    {
    protected:
        TestPlanarPath()
        {
            path.Assign(waypoints);
        }

        static void ExpectNear(const Point& actual, const Point& expected)
        {
            EXPECT_NEAR(actual[0], expected[0], 1e-5f);
            EXPECT_NEAR(actual[1], expected[1], 1e-5f);
        }

        std::array<Point, 4> waypoints{ Point{ 0.0f, 0.0f }, Point{ 2.0f, 0.0f }, Point{ 4.0f, 2.0f }, Point{ 4.0f, 5.0f } };
        Path path;
    };
}

TEST_F(TestPlanarPath, assign_computes_arc_lengths_and_rejects_bad_input)
{
    const std::array<Point, 1> single{ Point{ 1.0f, 1.0f } };
    const std::array<Point, 9> tooMany{};
    const std::array<Point, 3> repeated{ Point{ 0.0f, 0.0f }, Point{ 1.0f, 0.0f }, Point{ 1.0f, 0.0f } };

    EXPECT_NEAR(path.Length(), 7.828427f, 1e-5f);
    EXPECT_NEAR(path.ArcLength(2), 4.828427f, 1e-5f);
    EXPECT_FALSE(path.Assign(single));
    EXPECT_FALSE(path.Assign(tooMany));
    EXPECT_FALSE(path.Assign(repeated));
    EXPECT_EQ(path.Size(), std::size_t{ 4 });
    EXPECT_NEAR(path.Length(), 7.828427f, 1e-5f);
}

TEST_F(TestPlanarPath, project_reports_the_signed_cross_track)
{
    const auto first{ path.Project(Point{ 1.0f, 0.5f }, 0, 3) };
    const auto second{ path.Project(Point{ 3.5f, 0.5f }, 0, 3) };

    EXPECT_NEAR(first.arcLength, 1.0f, 1e-5f);
    ExpectNear(first.point, Point{ 1.0f, 0.0f });
    EXPECT_NEAR(first.heading, 0.0f, 1e-6f);
    EXPECT_NEAR(first.crossTrack, 0.5f, 1e-6f);
    EXPECT_EQ(first.segment, std::size_t{ 0 });
    EXPECT_NEAR(second.arcLength, 3.414214f, 1e-5f);
    ExpectNear(second.point, Point{ 3.0f, 1.0f });
    EXPECT_NEAR(second.heading, pi / 4.0f, 1e-6f);
    EXPECT_NEAR(second.crossTrack, -0.707107f, 1e-5f);
    EXPECT_EQ(second.segment, std::size_t{ 1 });
}

TEST_F(TestPlanarPath, point_at_clamps_to_the_path)
{
    ExpectNear(path.PointAt(5.5f), Point{ 4.0f, 2.671573f });
    EXPECT_NEAR(path.HeadingAt(5.5f), pi / 2.0f, 1e-6f);
    EXPECT_EQ(path.PointAt(-1.0f)[0], 0.0f);
    EXPECT_EQ(path.PointAt(-1.0f)[1], 0.0f);
    EXPECT_EQ(path.PointAt(99.0f)[0], 4.0f);
    EXPECT_EQ(path.PointAt(99.0f)[1], 5.0f);
}
