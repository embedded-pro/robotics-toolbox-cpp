#include "robotics/planning/RrtConnect.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <gtest/gtest.h>
#include <numbers>
#include <span>

namespace
{
    using planning::PlanStatus;
    using Joints = math::Vector<float, 2>;

    constexpr float pi{ std::numbers::pi_v<float> };
    constexpr float lowerBound{ 7.496376f };

    std::uint32_t generatorState{ 1u };

    float Uniform()
    {
        generatorState ^= generatorState << 13;
        generatorState ^= generatorState >> 17;
        generatorState ^= generatorState << 5;
        return static_cast<float>(static_cast<double>(generatorState) / 4294967296.0);
    }

    Joints Sample()
    {
        const float first{ -pi + 2.0f * pi * Uniform() };
        return Joints{ first, -pi + 2.0f * pi * Uniform() };
    }

    bool IsFree(const Joints& q)
    {
        return !(std::abs(q[0]) <= 0.3f && q[1] <= 1.0f);
    }

    class TestRrtConnect
        : public ::testing::Test
    {
    protected:
        static float Length(std::span<const Joints> path)
        {
            float length{ 0.0f };
            for (std::size_t i = 1; i < path.size(); ++i)
                length += std::hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1]);
            return length;
        }

        static void ExpectValidPath(std::span<const Joints> path, const Joints& start, const Joints& goal)
        {
            ASSERT_GE(path.size(), std::size_t{ 2 });
            EXPECT_EQ(path.front()[0], start[0]);
            EXPECT_EQ(path.front()[1], start[1]);
            EXPECT_EQ(path.back()[0], goal[0]);
            EXPECT_EQ(path.back()[1], goal[1]);
            for (std::size_t i = 1; i < path.size(); ++i)
            {
                const float length{ std::hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1]) };
                const auto checks{ static_cast<std::size_t>(std::ceil(length / 0.05f)) };
                for (std::size_t k = 0; k <= checks; ++k)
                {
                    const float fraction{ checks == 0 ? 0.0f : static_cast<float>(k) / static_cast<float>(checks) };
                    EXPECT_TRUE(IsFree(Joints{ path[i - 1] + Joints{ path[i] - path[i - 1] } * fraction }));
                }
            }
        }

        planning::RrtConnect<float, 2, 256> planner{ { 0.3f, 0.05f, 500 } };
        Joints start{ -2.0f, -2.0f };
        Joints goal{ 2.0f, -2.0f };
    };
}

TEST_F(TestRrtConnect, finds_a_valid_path_around_the_wall)
{
    generatorState = 1u;

    const auto status{ planner.Plan(start, goal, Sample, IsFree) };

    ASSERT_EQ(status, PlanStatus::Found);
    ExpectValidPath(planner.Path(), start, goal);
    EXPECT_GE(Length(planner.Path()), lowerBound);
    EXPECT_LE(planner.Iterations(), std::size_t{ 500 });
    EXPECT_GT(planner.NodeCount(), std::size_t{ 2 });
}

TEST_F(TestRrtConnect, results_are_reproducible_for_a_seed)
{
    generatorState = 7u;
    ASSERT_EQ(planner.Plan(start, goal, Sample, IsFree), PlanStatus::Found);
    std::array<Joints, 512> first{};
    const auto firstPath{ planner.Path() };
    std::copy(firstPath.begin(), firstPath.end(), first.begin());

    generatorState = 7u;
    ASSERT_EQ(planner.Plan(start, goal, Sample, IsFree), PlanStatus::Found);

    ASSERT_EQ(planner.Path().size(), firstPath.size());
    for (std::size_t i = 0; i < firstPath.size(); ++i)
    {
        EXPECT_EQ(planner.Path()[i][0], first[i][0]);
        EXPECT_EQ(planner.Path()[i][1], first[i][1]);
    }
}

TEST_F(TestRrtConnect, shortcut_shortens_and_stays_valid)
{
    generatorState = 1u;
    ASSERT_EQ(planner.Plan(start, goal, Sample, IsFree), PlanStatus::Found);
    const float raw{ Length(planner.Path()) };

    generatorState = 99u;
    planner.Shortcut(100, Uniform, IsFree);

    ExpectValidPath(planner.Path(), start, goal);
    EXPECT_LT(Length(planner.Path()), raw);
    EXPECT_GE(Length(planner.Path()), lowerBound);
}

TEST_F(TestRrtConnect, trivial_case_returns_the_straight_edge)
{
    const Joints left{ -2.0f, 2.0f };
    const Joints right{ 2.0f, 2.0f };

    ASSERT_EQ(planner.Plan(left, right, Sample, IsFree), PlanStatus::Found);

    ASSERT_EQ(planner.Path().size(), std::size_t{ 2 });
    EXPECT_EQ(planner.Path()[0][0], left[0]);
    EXPECT_EQ(planner.Path()[1][0], right[0]);
    EXPECT_EQ(planner.Iterations(), std::size_t{ 0 });
}

TEST_F(TestRrtConnect, invalid_endpoints_are_reported)
{
    EXPECT_EQ(planner.Plan(Joints{ 0.0f, 0.0f }, Joints{ 2.0f, 2.0f }, Sample, IsFree), PlanStatus::StartInCollision);
    EXPECT_EQ(planner.Plan(Joints{ 2.0f, 2.0f }, Joints{ 0.0f, 0.0f }, Sample, IsFree), PlanStatus::GoalInCollision);
}

TEST_F(TestRrtConnect, small_pool_is_exhausted)
{
    planning::RrtConnect<float, 2, 10> small{ { 0.3f, 0.05f, 500 } };
    generatorState = 1u;

    EXPECT_EQ(small.Plan(start, goal, Sample, IsFree), PlanStatus::PoolExhausted);

    planning::RrtConnect<float, 2, 10> noBudget{ { 0.3f, 0.05f, 0 } };
    EXPECT_EQ(noBudget.Plan(start, goal, Sample, IsFree), PlanStatus::IterationLimit);
}
