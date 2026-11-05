#include "numerical/math/SE2Transform.hpp"
#include "robotics/mobile/PurePursuit.hpp"
#include <array>
#include <cmath>
#include <cstddef>
#include <gtest/gtest.h>
#include <numbers>

namespace
{
    using Point = math::Vector2<float>;
    using Transform = math::SE2Transform<float>;
    using Path = mobile::PlanarPath<float, 8>;
    using Pursuit = mobile::PurePursuit<float, 8>;

    constexpr float pi{ std::numbers::pi_v<float> };

    class TestPurePursuit
        : public ::testing::Test
    {
    protected:
        TestPurePursuit()
        {
            path.Assign(waypoints);
            uTurn.Assign(uTurnWaypoints);
        }

        static void ExpectNear(const Point& actual, const Point& expected, float tolerance = 1e-5f)
        {
            EXPECT_NEAR(actual[0], expected[0], tolerance);
            EXPECT_NEAR(actual[1], expected[1], tolerance);
        }

        static float Distance(const Point& a, const Point& b)
        {
            return std::hypot(a[0] - b[0], a[1] - b[1]);
        }

        std::array<Point, 4> waypoints{ Point{ 0.0f, 0.0f }, Point{ 2.0f, 0.0f }, Point{ 4.0f, 2.0f }, Point{ 4.0f, 5.0f } };
        std::array<Point, 4> uTurnWaypoints{ Point{ 0.0f, 0.0f }, Point{ 4.0f, 0.0f }, Point{ 4.0f, 1.0f }, Point{ 0.0f, 1.0f } };
        Path path;
        Path uTurn;
        mobile::PurePursuitParameters<float> parameters{ 1.0f, 3.0f, 0.5f, 0.1f, 2 };
        Pursuit pursuit{ path, parameters };
    };
}

TEST_F(TestPurePursuit, goal_on_the_current_segment)
{
    const auto command{ pursuit.Compute(Transform::FromPose(0.5f, -0.2f, 0.0f), 1.0f) };

    ExpectNear(command.goal, Point{ 1.986607f, 0.0f });
    EXPECT_NEAR(command.curvature, 0.177778f, 1e-5f);
}

TEST_F(TestPurePursuit, goal_on_the_next_segment)
{
    const auto command{ pursuit.Compute(Transform::FromPose(1.8f, 0.0f, 0.0f), 1.0f) };

    ExpectNear(command.goal, Point{ 2.955936f, 0.955936f });
    EXPECT_NEAR(command.curvature, 0.849721f, 1e-5f);
}

TEST_F(TestPurePursuit, lookahead_is_speed_scheduled_and_clamped)
{
    const auto start{ Transform::FromPose(0.5f, 0.0f, 0.0f) };

    EXPECT_NEAR(Distance(pursuit.Compute(start, 10.0f).goal, start.translation), 3.0f, 1e-5f);
    EXPECT_NEAR(Distance(pursuit.Compute(start, 0.0f).goal, start.translation), 1.0f, 1e-5f);
}

TEST_F(TestPurePursuit, falls_back_to_a_path_point_when_far_off_the_path)
{
    const auto command{ pursuit.Compute(Transform::FromPose(1.0f, -3.0f, 0.0f), 0.0f) };

    ExpectNear(command.goal, Point{ 2.0f, 0.0f });
    EXPECT_NEAR(command.curvature, 0.6f, 1e-5f);
}

TEST_F(TestPurePursuit, end_of_path_targets_the_last_point_and_reports_reached)
{
    const auto approaching{ pursuit.Compute(Transform::FromPose(4.0f, 4.5f, pi / 2.0f), 1.0f) };
    const auto arrived{ pursuit.Compute(Transform::FromPose(4.0f, 4.95f, pi / 2.0f), 1.0f) };

    ExpectNear(approaching.goal, Point{ 4.0f, 5.0f });
    EXPECT_NEAR(approaching.curvature, 0.0f, 1e-5f);
    EXPECT_FALSE(approaching.reached);
    EXPECT_TRUE(arrived.reached);
    pursuit.Restart();
    EXPECT_EQ(pursuit.Progress(), 0.0f);
}

TEST_F(TestPurePursuit, search_window_keeps_progress_on_the_current_leg)
{
    Pursuit onLoop{ uTurn, parameters };

    const auto command{ onLoop.Compute(Transform::FromPose(1.0f, 0.6f, 0.0f), 0.0f) };

    ExpectNear(command.goal, Point{ 1.8f, 0.0f });
    EXPECT_NEAR(command.curvature, -1.2f, 1e-5f);
}

TEST_F(TestPurePursuit, closed_loop_unicycle_reaches_the_goal)
{
    Pursuit tracker{ path, mobile::PurePursuitParameters<float>{ 0.6f, 2.0f, 0.5f, 0.1f, 2 } };
    auto pose{ Transform::FromPose(0.0f, -0.5f, 0.3f) };
    constexpr float speed{ 0.8f };
    constexpr float dt{ 0.02f };

    std::size_t steps{ 0 };
    for (auto command{ tracker.Compute(pose, speed) }; !command.reached && steps < 600; command = tracker.Compute(pose, speed), ++steps)
        pose = pose * Transform::Exp(math::Vector3<float>{ speed, 0.0f, speed * command.curvature }, dt);

    EXPECT_LT(steps, std::size_t{ 600 });
    EXPECT_LT(Distance(pose.translation, Point{ 4.0f, 5.0f }), 0.1f);
}
