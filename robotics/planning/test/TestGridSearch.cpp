#include "robotics/planning/GridSearch.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <gtest/gtest.h>
#include <optional>
#include <span>
#include <utility>

namespace
{
    using planning::Connectivity;
    using planning::GridIndex;
    using planning::SearchMode;
    using Search = planning::GridSearch<float, 8, 6>;
    using Path = std::array<GridIndex, 48>;

    std::optional<float> WallCost(GridIndex cell)
    {
        if (cell.x == 4 && cell.y <= 4)
            return std::nullopt;
        return 1.0f;
    }

    std::optional<float> ClosedGapCost(GridIndex cell)
    {
        if (cell.x == 4)
            return std::nullopt;
        return 1.0f;
    }

    std::optional<float> OpenSpace(GridIndex)
    {
        return 1.0f;
    }

    std::optional<float> WeightedCost(GridIndex cell)
    {
        if ((cell.x == 3 || cell.x == 4) && cell.y <= 2)
            return 5.0f;
        return 1.0f;
    }

    class TestGridSearch
        : public ::testing::Test
    {
    protected:
        template<class CellCost>
        void ExpectValidPath(std::span<const GridIndex> path, GridIndex start, GridIndex goal, CellCost cost, float expectedCost, Connectivity connectivity) const
        {
            ASSERT_FALSE(path.empty());
            EXPECT_EQ(path.front(), start);
            EXPECT_EQ(path.back(), goal);

            float total{ 0.0f };
            for (std::size_t i = 1; i < path.size(); ++i)
            {
                const int dx{ path[i].x - path[i - 1].x };
                const int dy{ path[i].y - path[i - 1].y };
                const int dz{ path[i].z - path[i - 1].z };
                const int axes{ std::abs(dx) + std::abs(dy) + std::abs(dz) };
                EXPECT_LE(std::max({ std::abs(dx), std::abs(dy), std::abs(dz) }), 1);
                EXPECT_GE(axes, 1);
                EXPECT_TRUE(connectivity == Connectivity::Full || axes == 1);
                ASSERT_TRUE(cost(path[i]).has_value());
                if (axes == 2 && dz == 0)
                {
                    EXPECT_TRUE(cost(GridIndex{ path[i].x, path[i - 1].y, path[i].z }).has_value());
                    EXPECT_TRUE(cost(GridIndex{ path[i - 1].x, path[i].y, path[i].z }).has_value());
                }
                total += std::sqrt(static_cast<float>(axes)) * *cost(path[i]);
            }
            EXPECT_NEAR(total, expectedCost, 1e-4f);
        }

        Search search{ { Connectivity::Full, SearchMode::AStar } };
        Search dijkstra{ { Connectivity::Full, SearchMode::Dijkstra } };
        Search faces{ { Connectivity::Faces, SearchMode::AStar } };
        Search facesDijkstra{ { Connectivity::Faces, SearchMode::Dijkstra } };
        GridIndex start{ 1, 1 };
        GridIndex goal{ 6, 1 };
        Path path{};
    };
}

TEST_F(TestGridSearch, eight_connected_path_goes_through_the_gap)
{
    const auto cost{ search.Search(start, goal, WallCost) };

    ASSERT_TRUE(cost.has_value());
    EXPECT_NEAR(*cost, 11.242641f, 1e-4f);
    EXPECT_NEAR(dijkstra.Search(start, goal, WallCost).value_or(-1.0f), *cost, 1e-4f);
    const auto length{ search.ExtractPath(path) };
    EXPECT_EQ(length, std::size_t{ 11 });
    ExpectValidPath(std::span{ path.data(), length }, start, goal, WallCost, *cost, Connectivity::Full);
}

TEST_F(TestGridSearch, four_connected_path_is_longer)
{
    const auto cost{ faces.Search(start, goal, WallCost) };

    ASSERT_TRUE(cost.has_value());
    EXPECT_NEAR(*cost, 13.0f, 1e-5f);
    const auto length{ faces.ExtractPath(path) };
    EXPECT_EQ(length, std::size_t{ 14 });
    ExpectValidPath(std::span{ path.data(), length }, start, goal, WallCost, *cost, Connectivity::Faces);
}

TEST_F(TestGridSearch, astar_and_dijkstra_agree_and_astar_expands_no_more)
{
    for (auto [informed, uninformed] : { std::pair<Search*, Search*>{ &search, &dijkstra }, std::pair<Search*, Search*>{ &faces, &facesDijkstra } })
    {
        const auto informedCost{ informed->Search(start, goal, WallCost) };
        const auto uninformedCost{ uninformed->Search(start, goal, WallCost) };

        ASSERT_TRUE(informedCost.has_value());
        ASSERT_TRUE(uninformedCost.has_value());
        EXPECT_NEAR(*informedCost, *uninformedCost, 1e-4f);
        EXPECT_LE(informed->Expanded(), uninformed->Expanded());
    }
}

TEST_F(TestGridSearch, weighted_cells_are_avoided_when_cheaper)
{
    for (auto* planner : { &search, &dijkstra })
    {
        const auto cost{ planner->Search(start, goal, WeightedCost) };

        ASSERT_TRUE(cost.has_value());
        EXPECT_NEAR(*cost, 6.656854f, 1e-4f);
        const auto length{ planner->ExtractPath(path) };
        const std::span<const GridIndex> route{ path.data(), length };
        for (const auto& cell : route)
            if (cell.x == 3 || cell.x == 4)
                EXPECT_GE(cell.y, 3);
        ExpectValidPath(route, start, goal, WeightedCost, *cost, Connectivity::Full);
    }
}

TEST_F(TestGridSearch, blocked_goal_or_closed_gap_has_no_path)
{
    EXPECT_FALSE(search.Search(start, goal, ClosedGapCost).has_value());
    EXPECT_EQ(search.ExtractPath(path), std::size_t{ 0 });
    EXPECT_FALSE(search.Search(start, GridIndex{ 4, 2 }, WallCost).has_value());
    EXPECT_EQ(search.ExtractPath(path), std::size_t{ 0 });
}

TEST_F(TestGridSearch, start_equal_to_goal_costs_nothing)
{
    const GridIndex cell{ 2, 2 };

    const auto cost{ search.Search(cell, cell, WallCost) };

    ASSERT_TRUE(cost.has_value());
    EXPECT_EQ(*cost, 0.0f);
    ASSERT_EQ(search.ExtractPath(path), std::size_t{ 1 });
    EXPECT_EQ(path[0], cell);
}

TEST_F(TestGridSearch, three_dimensional_grid_uses_diagonal_steps)
{
    planning::GridSearch<float, 3, 3, 3> full{ { Connectivity::Full, SearchMode::AStar } };
    planning::GridSearch<float, 3, 3, 3> axisAligned{ { Connectivity::Faces, SearchMode::AStar } };

    EXPECT_NEAR(full.Search(GridIndex{ 0, 0, 0 }, GridIndex{ 2, 2, 2 }, OpenSpace).value_or(-1.0f), 3.464102f, 1e-5f);
    EXPECT_NEAR(axisAligned.Search(GridIndex{ 0, 0, 0 }, GridIndex{ 2, 2, 2 }, OpenSpace).value_or(-1.0f), 6.0f, 1e-5f);
}

TEST_F(TestGridSearch, extract_path_rejects_a_short_buffer)
{
    ASSERT_TRUE(search.Search(start, goal, WallCost).has_value());
    std::array<GridIndex, 5> shortBuffer{};

    EXPECT_EQ(search.ExtractPath(shortBuffer), std::size_t{ 0 });
}
