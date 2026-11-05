#include "robotics/planning/DynamicWindowApproach.hpp"
#include <array>
#include <cstddef>
#include <gtest/gtest.h>
#include <optional>
#include <span>

namespace
{
    using Point = math::Vector2<float>;
    using Dwa = planning::DynamicWindowApproach<float, 5, 11>;

    class TestDynamicWindowApproach
        : public ::testing::Test
    {
    protected:
        static void ExpectWindow(const planning::VelocityWindow<float>& actual, const planning::VelocityWindow<float>& expected)
        {
            EXPECT_NEAR(actual.vMin, expected.vMin, 1e-6f);
            EXPECT_NEAR(actual.vMax, expected.vMax, 1e-6f);
            EXPECT_NEAR(actual.omegaMin, expected.omegaMin, 1e-6f);
            EXPECT_NEAR(actual.omegaMax, expected.omegaMax, 1e-6f);
        }

        static void ExpectCommand(const std::optional<planning::DwaCommand<float>>& actual, float v, float omega)
        {
            ASSERT_TRUE(actual.has_value());
            EXPECT_NEAR(actual->v, v, 1e-5f);
            EXPECT_NEAR(actual->omega, omega, 1e-5f);
        }

        planning::DwaParameters<float> parameters{ 0.0f, 1.0f, 2.0f, 1.0f, 3.0f, 0.25f, 1.0f, 0.2f, 2.0f, 1.0f, 1.0f, 0.5f };
        Dwa dwa{ parameters };
        Point ahead{ 5.0f, 0.0f };
    };
}

TEST_F(TestDynamicWindowApproach, straight_clearance_stops_at_the_disc_contact)
{
    EXPECT_NEAR(Dwa::ArcClearance(1.0f, 0.0f, Point{ 2.0f, 0.1f }, 0.2f, 2.0f), 1.826795f, 1e-5f);
    EXPECT_EQ(Dwa::ArcClearance(1.0f, 0.0f, Point{ -1.0f, 0.0f }, 0.2f, 2.0f), 2.0f);
}

TEST_F(TestDynamicWindowApproach, arc_clearance_on_a_turn)
{
    EXPECT_NEAR(Dwa::ArcClearance(1.0f, 1.0f, Point{ 1.0f, 1.0f }, 0.2f, 2.0f), 1.370461f, 1e-5f);
    EXPECT_NEAR(Dwa::ArcClearance(1.0f, -1.0f, Point{ 1.0f, -1.0f }, 0.2f, 2.0f), 1.370461f, 1e-5f);
    EXPECT_EQ(Dwa::ArcClearance(1.0f, 1.0f, Point{ 3.0f, 3.0f }, 0.2f, 2.0f), 2.0f);
}

TEST_F(TestDynamicWindowApproach, contact_and_rotation_in_place)
{
    EXPECT_EQ(Dwa::ArcClearance(1.0f, 0.0f, Point{ 0.1f, 0.05f }, 0.2f, 2.0f), 0.0f);
    EXPECT_EQ(Dwa::ArcClearance(0.0f, 1.0f, Point{ 0.5f, 0.0f }, 0.2f, 2.0f), 2.0f);
}

TEST_F(TestDynamicWindowApproach, window_respects_acceleration_and_bounds)
{
    ExpectWindow(dwa.Window(0.5f, 0.0f), { 0.25f, 0.75f, -0.75f, 0.75f });
    ExpectWindow(dwa.Window(0.95f, 1.9f), { 0.7f, 1.0f, 1.15f, 2.0f });
    ExpectWindow(dwa.Window(0.1f, -1.9f), { 0.0f, 0.35f, -2.0f, -1.15f });
}

TEST_F(TestDynamicWindowApproach, free_space_drives_fast_towards_the_goal)
{
    const auto command{ dwa.Compute(0.5f, 0.0f, ahead, {}) };

    ExpectCommand(command, 0.75f, 0.0f);
    EXPECT_EQ(command->clearance, 2.0f);
}

TEST_F(TestDynamicWindowApproach, goal_to_the_left_turns_left)
{
    ExpectCommand(dwa.Compute(0.5f, 0.0f, Point{ 0.0f, 3.0f }, {}), 0.75f, 0.75f);
}

TEST_F(TestDynamicWindowApproach, obstacle_slightly_left_turns_right)
{
    const std::array<Point, 1> left{ Point{ 0.45f, 0.05f } };
    const std::array<Point, 1> right{ Point{ 0.45f, -0.05f } };

    ExpectCommand(dwa.Compute(0.5f, 0.0f, ahead, left), 0.375f, -0.75f);
    ExpectCommand(dwa.Compute(0.5f, 0.0f, ahead, right), 0.375f, 0.75f);
}

TEST_F(TestDynamicWindowApproach, too_fast_to_stop_returns_nullopt)
{
    std::array<Point, 21> wall{};
    for (std::size_t k = 0; k < wall.size(); ++k)
        wall[k] = Point{ 0.3f, -1.0f + 0.1f * static_cast<float>(k) };

    EXPECT_FALSE(dwa.Compute(0.75f, 0.0f, ahead, wall).has_value());
    ExpectCommand(dwa.Compute(0.0f, 0.0f, ahead, wall), 0.0f, 0.0f);
}
