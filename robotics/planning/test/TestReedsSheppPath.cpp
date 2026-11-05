#include "numerical/math/SE2Transform.hpp"
#include "robotics/planning/CarPath.hpp"
#include "robotics/planning/DubinsPath.hpp"
#include "robotics/planning/ReedsSheppPath.hpp"
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

    class TestReedsSheppPath
        : public ::testing::Test
    {
    protected:
        template<std::size_t N>
        static void ExpectPath(const Path& path, const std::array<Steering, N>& letters, const std::array<float, N>& lengths)
        {
            ASSERT_EQ(path.count, N);
            for (std::size_t k = 0; k < N; ++k)
            {
                EXPECT_EQ(path.segments[k].steering, letters[k]);
                EXPECT_NEAR(path.segments[k].length, lengths[k], 1e-4f);
            }
        }

        static void ExpectReaches(const Path& path, const Transform& from, const Transform& goal)
        {
            const auto error{ (goal.Inverse() * path.Sample(from, path.Length())).Pose() };
            for (std::size_t i = 0; i < 3; ++i)
                EXPECT_NEAR(error[i], 0.0f, 1e-4f);
        }

        planning::ReedsSheppPlanner<float> planner{ 1.0f };
        Transform start{ Transform::Identity() };
    };
}

TEST_F(TestReedsSheppPath, straight_reverse_is_a_single_backward_segment)
{
    const auto goal{ Transform::FromPose(-3.0f, 0.0f, 0.0f) };
    const auto path{ planner.Shortest(start, goal) };

    EXPECT_NEAR(path.Length(), 3.0f, 1e-4f);
    for (std::size_t k = 0; k < path.count; ++k)
        EXPECT_NEAR(path.segments[k].length, path.segments[k].steering == Steering::Straight ? -3.0f : 0.0f, 1e-4f);
    ExpectReaches(path, start, goal);
}

TEST_F(TestReedsSheppPath, forward_path_when_it_is_shortest)
{
    const auto path{ planner.Shortest(start, Transform::FromPose(3.0f, 0.5f, 0.2f)) };

    ExpectPath<3>(path, { Steering::Left, Steering::Straight, Steering::Left }, { 0.169722f, 2.842168f, 0.030278f });
    EXPECT_NEAR(path.Length(), 3.042168f, 1e-4f);
}

TEST_F(TestReedsSheppPath, sideways_shift_uses_cusps)
{
    const auto goal{ Transform::FromPose(0.0f, 1.0f, 0.0f) };
    const auto path{ planner.Shortest(start, goal) };

    EXPECT_NEAR(path.Length(), 2.636232f, 1e-4f);
    bool forward{ false };
    bool reverse{ false };
    for (std::size_t k = 0; k < path.count; ++k)
    {
        forward = forward || path.segments[k].length > 1e-4f;
        reverse = reverse || path.segments[k].length < -1e-4f;
    }
    EXPECT_TRUE(forward);
    EXPECT_TRUE(reverse);
    ExpectReaches(path, start, goal);
}

TEST_F(TestReedsSheppPath, ccsc_word_with_a_reversed_quarter_turn)
{
    const auto path{ planner.Shortest(start, Transform::FromPose(-2.0f, 1.0f, pi / 2.0f)) };

    ExpectPath<4>(path, { Steering::Left, Steering::Straight, Steering::Right, Steering::Left }, { -0.729728f, -0.236068f, -pi / 2.0f, 0.729728f });
    EXPECT_NEAR(path.Length(), 3.266320f, 1e-4f);
}

TEST_F(TestReedsSheppPath, ccc_word_with_a_cusp)
{
    const auto path{ planner.Shortest(start, Transform::FromPose(-1.0f, -1.0f, 0.5f)) };

    ExpectPath<3>(path, { Steering::Left, Steering::Right, Steering::Left }, { 0.166243f, -0.965586f, -0.631829f });
    EXPECT_NEAR(path.Length(), 1.763658f, 1e-4f);
}

TEST_F(TestReedsSheppPath, three_point_turn_reverses_heading_in_place)
{
    const auto goal{ Transform::FromPose(0.0f, 0.0f, pi) };
    const auto path{ planner.Shortest(start, goal) };

    EXPECT_NEAR(path.Length(), pi, 1e-4f);
    ExpectReaches(path, start, goal);
}

TEST_F(TestReedsSheppPath, never_longer_than_dubins_symmetric_and_reaching)
{
    const planning::ReedsSheppPlanner<float> reedsShepp{ 1.5f };
    const planning::DubinsPlanner<float> dubins{ 1.5f };
    const auto from{ Transform::FromPose(1.0f, 2.0f, pi / 4.0f) };
    const std::array<float, 4> coordinates{ -3.0f, -1.0f, 0.5f, 2.0f };
    const std::array<float, 5> headings{ -2.5f, -pi / 2.0f, 0.0f, 1.0f, pi };

    for (const float x : coordinates)
        for (const float y : coordinates)
            for (const float heading : headings)
            {
                const auto goal{ Transform::FromPose(x, y, heading) };
                const auto path{ reedsShepp.Shortest(from, goal) };

                EXPECT_LE(path.Length(), dubins.Shortest(from, goal).Length() + 1e-5f);
                EXPECT_NEAR(reedsShepp.Shortest(goal, from).Length(), path.Length(), 1e-4f);
                ExpectReaches(path, from, goal);
            }
}
