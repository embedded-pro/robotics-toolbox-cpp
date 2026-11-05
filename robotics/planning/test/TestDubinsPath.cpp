#include "numerical/math/SE2Transform.hpp"
#include "robotics/planning/CarPath.hpp"
#include "robotics/planning/DubinsPath.hpp"
#include <array>
#include <cstddef>
#include <gtest/gtest.h>
#include <numbers>

namespace
{
    using Transform = math::SE2Transform<float>;
    using Path = planning::CarPath<float>;
    using planning::Steering;

    constexpr float pi{ std::numbers::pi_v<float> };

    class TestDubinsPath
        : public ::testing::Test
    {
    protected:
        static void ExpectPath(const Path& path, const std::array<Steering, 3>& letters, const std::array<float, 3>& lengths)
        {
            ASSERT_EQ(path.count, 3u);
            for (std::size_t k = 0; k < 3; ++k)
            {
                EXPECT_EQ(path.segments[k].steering, letters[k]);
                EXPECT_NEAR(path.segments[k].length, lengths[k], 1e-4f);
            }
        }

        static void ExpectPose(const Transform& actual, const Transform& expected)
        {
            const auto error{ (expected.Inverse() * actual).Pose() };
            for (std::size_t i = 0; i < 3; ++i)
                EXPECT_NEAR(error[i], 0.0f, 1e-4f);
        }

        static void ExpectReaches(const Path& path, const Transform& from, const Transform& goal)
        {
            ExpectPose(path.Sample(from, path.Length()), goal);
        }

        planning::DubinsPlanner<float> planner{ 1.0f };
        Transform start{ Transform::Identity() };
    };
}

TEST_F(TestDubinsPath, straight_ahead_is_a_single_segment)
{
    const auto goal{ Transform::FromPose(4.0f, 0.0f, 0.0f) };
    const auto path{ planner.Shortest(start, goal) };

    EXPECT_NEAR(path.Length(), 4.0f, 1e-4f);
    EXPECT_NEAR(path.segments[0].length, 0.0f, 1e-4f);
    EXPECT_NEAR(path.segments[2].length, 0.0f, 1e-4f);
    ExpectReaches(path, start, goal);
}

TEST_F(TestDubinsPath, csc_shortest_is_lsl)
{
    const auto path{ planner.Shortest(start, Transform::FromPose(4.0f, 4.0f, pi / 2.0f)) };

    ExpectPath(path, { Steering::Left, Steering::Straight, Steering::Left }, { 0.785398f, 4.242641f, 0.785398f });
    EXPECT_NEAR(path.Length(), 5.813437f, 1e-4f);
}

TEST_F(TestDubinsPath, csc_shortest_is_rsl)
{
    const auto path{ planner.Shortest(start, Transform::FromPose(3.0f, -2.0f, pi / 3.0f)) };

    ExpectPath(path, { Steering::Right, Steering::Straight, Steering::Left }, { 1.379520f, 0.896575f, 2.426718f });
    EXPECT_NEAR(path.Length(), 4.702813f, 1e-4f);
}

TEST_F(TestDubinsPath, ccc_shortest_is_rlr)
{
    const auto path{ planner.Shortest(start, Transform::FromPose(0.2f, 0.6f, -2.5f)) };

    ExpectPath(path, { Steering::Right, Steering::Left, Steering::Right }, { 0.752181f, 4.974858f, 0.439492f });
    EXPECT_NEAR(path.Length(), 6.166531f, 1e-4f);
}

TEST_F(TestDubinsPath, turning_on_the_spot_needs_lrl)
{
    const auto path{ planner.Shortest(start, Transform::FromPose(0.0f, 0.0f, pi / 2.0f)) };

    ExpectPath(path, { Steering::Left, Steering::Right, Steering::Left }, { 0.424031f, 5.560451f, 0.424031f });
    EXPECT_NEAR(path.Length(), 6.408513f, 1e-4f);
}

TEST_F(TestDubinsPath, single_arc_goal_survives_the_float_boundary)
{
    const auto goal{ Transform::FromPose(1.0f, 1.0f, pi / 2.0f) };
    const auto path{ planner.Shortest(start, goal) };

    EXPECT_NEAR(path.Length(), pi / 2.0f, 1e-4f);
    ExpectReaches(path, start, goal);
}

TEST_F(TestDubinsPath, mirroring_the_goal_swaps_left_and_right)
{
    const auto path{ planner.Shortest(start, Transform::FromPose(3.0f, 2.0f, -pi / 3.0f)) };

    ExpectPath(path, { Steering::Left, Steering::Straight, Steering::Right }, { 1.379520f, 0.896575f, 2.426718f });
}

TEST_F(TestDubinsPath, radius_scales_every_length)
{
    const planning::DubinsPlanner<float> wide{ 2.0f };

    EXPECT_NEAR(wide.Shortest(start, Transform::FromPose(8.0f, 8.0f, pi / 2.0f)).Length(), 11.626874f, 1e-4f);
}

TEST_F(TestDubinsPath, sample_walks_along_the_path)
{
    const auto goal{ Transform::FromPose(4.0f, 4.0f, pi / 2.0f) };
    const auto path{ planner.Shortest(start, goal) };

    ExpectPose(path.Sample(start, 0.0f), start);
    ExpectPose(path.Sample(start, path.Length() / 2.0f), Transform::FromPose(2.207107f, 1.792893f, pi / 4.0f));
    ExpectPose(path.Sample(start, path.Length()), goal);
    ExpectPose(path.Sample(start, path.Length() + 1.0f), goal);
}

TEST_F(TestDubinsPath, every_shortest_path_reaches_its_goal)
{
    const planning::DubinsPlanner<float> planner15{ 1.5f };
    const auto from{ Transform::FromPose(1.0f, 2.0f, pi / 4.0f) };
    const std::array<float, 4> coordinates{ -3.0f, -1.0f, 0.5f, 2.0f };
    const std::array<float, 5> headings{ -2.5f, -pi / 2.0f, 0.0f, 1.0f, pi };

    for (const float x : coordinates)
        for (const float y : coordinates)
            for (const float heading : headings)
            {
                const auto goal{ Transform::FromPose(x, y, heading) };
                ExpectReaches(planner15.Shortest(from, goal), from, goal);
            }
}
