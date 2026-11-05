#include "numerical/math/Geometry3D.hpp"
#include "numerical/math/Tolerance.hpp"
#include "robotics/planning/PotentialField.hpp"
#include <array>
#include <gtest/gtest.h>

namespace
{
    using Vector3 = math::Vector3<float>;
    using Obstacle = planning::ObstacleProximity<float>;
    using planning::FieldStatus;

    class TestPotentialField
        : public ::testing::Test
    {
    protected:
        void ExpectNear(const Vector3& actual, const Vector3& expected) const
        {
            for (std::size_t i = 0; i < 3; ++i)
                EXPECT_NEAR(actual[i], expected[i], math::Tolerance<float>());
        }

        planning::PotentialFieldParameters<float> parameters{ 1.0f, 1.0f, 0.1f, 1.0f, 0.5f, 1e-3f, 0.05f };
        planning::PotentialField<float> field{ parameters };
        Vector3 origin{};
        Vector3 farGoal{ 3.0f, 4.0f, 0.0f };
    };
}

TEST_F(TestPotentialField, near_goal_the_pull_is_quadratic)
{
    const auto command{ field.Compute(origin, Vector3{ 0.4f, 0.3f, 0.0f }, {}) };

    ExpectNear(command.velocity, Vector3{ 0.4f, 0.3f, 0.0f });
    EXPECT_NEAR(command.attractiveMagnitude, 0.5f, math::Tolerance<float>());
    EXPECT_EQ(command.status, FieldStatus::Moving);
}

TEST_F(TestPotentialField, far_goal_the_pull_is_conic_and_clamped)
{
    const auto command{ field.Compute(origin, farGoal, {}) };

    ExpectNear(command.velocity, Vector3{ 0.3f, 0.4f, 0.0f });
    EXPECT_NEAR(command.attractiveMagnitude, 1.0f, math::Tolerance<float>());
}

TEST_F(TestPotentialField, obstacle_inside_the_influence_distance_repels)
{
    const std::array<Obstacle, 1> obstacles{ Obstacle{ Vector3{ 0.0f, -1.0f, 0.0f }, 0.5f } };

    const auto command{ field.Compute(origin, farGoal, obstacles) };

    EXPECT_NEAR(command.repulsiveMagnitude, 0.4f, math::Tolerance<float>());
    ExpectNear(command.velocity, Vector3{ 0.416025f, 0.277350f, 0.0f });
}

TEST_F(TestPotentialField, obstacle_outside_the_influence_distance_is_ignored)
{
    const std::array<Obstacle, 1> obstacles{ Obstacle{ Vector3{ 0.0f, -1.0f, 0.0f }, 1.2f } };

    const auto command{ field.Compute(origin, farGoal, obstacles) };

    EXPECT_NEAR(command.repulsiveMagnitude, 0.0f, math::Tolerance<float>());
    ExpectNear(command.velocity, Vector3{ 0.3f, 0.4f, 0.0f });
}

TEST_F(TestPotentialField, goal_within_tolerance_is_reached)
{
    EXPECT_EQ(field.Compute(origin, Vector3{ 0.03f, 0.0f, 0.0f }, {}).status, FieldStatus::Reached);
}

TEST_F(TestPotentialField, opposing_obstacle_creates_a_detected_local_minimum)
{
    const Vector3 goal{ 3.0f, 0.0f, 0.0f };
    const std::array<Obstacle, 1> balanced{ Obstacle{ Vector3{ -1.0f, 0.0f, 0.0f }, 0.393003f } };
    const std::array<Obstacle, 1> closer{ Obstacle{ Vector3{ -1.0f, 0.0f, 0.0f }, 0.3f } };

    const auto stalled{ field.Compute(origin, goal, balanced) };
    const auto pushed{ field.Compute(origin, goal, closer) };

    EXPECT_NEAR(stalled.attractiveMagnitude, 1.0f, math::Tolerance<float>());
    EXPECT_NEAR(stalled.repulsiveMagnitude, 1.0f, math::Tolerance<float>());
    EXPECT_LT(math::VectorNorm(stalled.velocity), 1e-3f);
    EXPECT_EQ(stalled.status, FieldStatus::Stalled);
    EXPECT_EQ(pushed.status, FieldStatus::Moving);
    ExpectNear(pushed.velocity, Vector3{ -0.5f, 0.0f, 0.0f });
}
