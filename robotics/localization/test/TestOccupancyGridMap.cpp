#include "numerical/math/SE2Transform.hpp"
#include "robotics/localization/OccupancyGridMap.hpp"
#include <array>
#include <cmath>
#include <cstddef>
#include <gtest/gtest.h>
#include <initializer_list>
#include <numbers>

namespace
{
    using Transform = math::SE2Transform<float>;
    using Point = math::Vector2<float>;
    using Beam = localization::RangeBeam<float>;
    using Cell = localization::GridCell;
    using Map = localization::OccupancyGridMap<float, 10, 10>;

    constexpr float pi{ std::numbers::pi_v<float> };

    struct ExpectedCell
    {
        int x;
        int y;
        float logOdds;
    };

    class TestOccupancyGridMap
        : public ::testing::Test
    {
    protected:
        static void Integrate(Map& target, const Transform& pose, Beam beam)
        {
            const std::array<Beam, 1> beams{ beam };
            target.Integrate(pose, beams);
        }

        static float Expected(std::initializer_list<ExpectedCell> touched, int x, int y)
        {
            for (const auto& cell : touched)
                if (cell.x == x && cell.y == y)
                    return cell.logOdds;
            return 0.0f;
        }

        static void ExpectCells(const Map& target, std::initializer_list<ExpectedCell> touched)
        {
            for (int y = 0; y < 10; ++y)
                for (int x = 0; x < 10; ++x)
                    EXPECT_FLOAT_EQ(target.LogOdds(Cell{ x, y }), Expected(touched, x, y)) << "cell (" << x << ", " << y << ")";
        }

        localization::OccupancyGridParameters<float> parameters{ 0.1f, Point{ 0.0f, 0.0f }, 0.85f, -0.4f, -2.0f, 3.5f, 4.0f };
        Map map{ parameters };
        Transform sensor{ Transform::FromPose(0.05f, 0.05f, 0.0f) };
    };
}

TEST_F(TestOccupancyGridMap, axis_beam_clears_the_path_and_marks_the_hit)
{
    Integrate(map, sensor, Beam{ 0.5f, 0.0f });

    ExpectCells(map, { { 0, 0, -0.4f }, { 1, 0, -0.4f }, { 2, 0, -0.4f }, { 3, 0, -0.4f }, { 4, 0, -0.4f }, { 5, 0, 0.85f } });
}

TEST_F(TestOccupancyGridMap, diagonal_and_shallow_beams_follow_bresenham)
{
    Map shallow{ parameters };

    Integrate(map, sensor, Beam{ 0.5f * std::numbers::sqrt2_v<float>, pi / 4.0f });
    Integrate(shallow, sensor, Beam{ std::hypot(0.7f, 0.3f), std::atan2(0.3f, 0.7f) });

    ExpectCells(map, { { 0, 0, -0.4f }, { 1, 1, -0.4f }, { 2, 2, -0.4f }, { 3, 3, -0.4f }, { 4, 4, -0.4f }, { 5, 5, 0.85f } });
    ExpectCells(shallow, { { 0, 0, -0.4f }, { 1, 0, -0.4f }, { 2, 1, -0.4f }, { 3, 1, -0.4f }, { 4, 2, -0.4f }, { 5, 2, -0.4f }, { 6, 3, -0.4f }, { 7, 3, 0.85f } });
}

TEST_F(TestOccupancyGridMap, max_range_reading_only_clears)
{
    parameters.maxRange = 0.5f;
    Map shortRange{ parameters };

    Integrate(shortRange, sensor, Beam{ 0.5f, 0.0f });

    ExpectCells(shortRange, { { 0, 0, -0.4f }, { 1, 0, -0.4f }, { 2, 0, -0.4f }, { 3, 0, -0.4f }, { 4, 0, -0.4f }, { 5, 0, -0.4f } });
}

TEST_F(TestOccupancyGridMap, ray_leaving_the_map_stops_at_the_border)
{
    Integrate(map, sensor, Beam{ 2.0f, 0.0f });

    ExpectCells(map, { { 0, 0, -0.4f }, { 1, 0, -0.4f }, { 2, 0, -0.4f }, { 3, 0, -0.4f }, { 4, 0, -0.4f }, { 5, 0, -0.4f }, { 6, 0, -0.4f }, { 7, 0, -0.4f }, { 8, 0, -0.4f }, { 9, 0, -0.4f } });
}

TEST_F(TestOccupancyGridMap, repeated_hits_clamp_the_log_odds)
{
    for (std::size_t k = 0; k < 10; ++k)
        Integrate(map, sensor, Beam{ 0.5f, 0.0f });

    EXPECT_EQ(map.LogOdds(Cell{ 5, 0 }), 3.5f);
    EXPECT_EQ(map.LogOdds(Cell{ 0, 0 }), -2.0f);
    EXPECT_NEAR(map.Probability(Cell{ 5, 0 }), 0.970688f, 1e-6f);
    EXPECT_NEAR(map.Probability(Cell{ 0, 0 }), 0.119203f, 1e-6f);
    EXPECT_TRUE(map.IsOccupied(Cell{ 5, 0 }, 0.0f));
    EXPECT_FALSE(map.IsOccupied(Cell{ 0, 0 }, 0.0f));
}

TEST_F(TestOccupancyGridMap, world_to_cell_respects_the_origin)
{
    parameters.origin = Point{ -0.5f, -0.5f };
    const Map centred{ parameters };

    EXPECT_EQ(centred.WorldToCell(Point{ 0.0f, 0.0f }), (Cell{ 5, 5 }));
    EXPECT_EQ(centred.WorldToCell(Point{ -0.5f, -0.5f }), (Cell{ 0, 0 }));
    EXPECT_EQ(centred.WorldToCell(Point{ 0.449f, -0.051f }), (Cell{ 9, 4 }));
    const auto centre{ centred.CellCenter(Cell{ 5, 5 }) };
    EXPECT_NEAR(centre[0], 0.05f, 1e-6f);
    EXPECT_NEAR(centre[1], 0.05f, 1e-6f);
    EXPECT_EQ(centred.Parameters().origin[0], -0.5f);
}
