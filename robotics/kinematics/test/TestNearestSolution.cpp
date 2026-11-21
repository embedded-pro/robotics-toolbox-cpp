#include "numerical/math/Tolerance.hpp"
#include "robotics/kinematics/NearestSolution.hpp"
#include <gtest/gtest.h>
#include <numbers>

namespace
{
    using Joints = math::Vector<float, 2>;
    using Candidates = std::array<std::optional<Joints>, 3>;
    using Limits = kinematics::JointLimitArray<float, 2>;

    constexpr float pi{ std::numbers::pi_v<float> };

    class TestNearestSolution
        : public ::testing::Test
    {
    protected:
        Limits halfTurn{ { { -pi, pi }, { -pi, pi } } };
        Limits wide{ { { -4.0f, 4.0f }, { -4.0f, 4.0f } } };
    };
}

TEST_F(TestNearestSolution, picks_the_candidate_closest_to_the_current_joints)
{
    const Candidates candidates{ Joints{ 1.0f, 1.0f }, std::nullopt, Joints{ -0.5f, 0.2f } };

    const auto result{ kinematics::NearestSolution(candidates, Joints{ -0.4f, 0.0f }, halfTurn) };

    ASSERT_EQ(result.status, kinematics::SolutionStatus::found);
    EXPECT_NEAR((*result.q)[0], -0.5f, math::Tolerance<float>());
    EXPECT_NEAR((*result.q)[1], 0.2f, math::Tolerance<float>());
}

TEST_F(TestNearestSolution, turns_a_joint_by_a_full_turn_when_that_is_inside_the_limits_and_closer)
{
    const Candidates candidates{ Joints{ -3.0f, 0.0f }, std::nullopt, std::nullopt };

    const auto result{ kinematics::NearestSolution(candidates, Joints{ 3.0f, 0.0f }, wide) };

    ASSERT_EQ(result.status, kinematics::SolutionStatus::found);
    EXPECT_NEAR((*result.q)[0], -3.0f + 2.0f * pi, math::Tolerance<float>());
}

TEST_F(TestNearestSolution, reports_outside_limits_when_no_candidate_fits_the_limits)
{
    const Limits narrow{ { { -0.5f, 0.5f }, { -0.5f, 0.5f } } };
    const Candidates candidates{ Joints{ 1.0f, 0.0f }, Joints{ 0.0f, -2.0f }, std::nullopt };

    const auto result{ kinematics::NearestSolution(candidates, Joints{}, narrow) };

    EXPECT_EQ(result.status, kinematics::SolutionStatus::outsideLimits);
    EXPECT_FALSE(result.q.has_value());
}

TEST_F(TestNearestSolution, reports_unreachable_without_candidates)
{
    const auto result{ kinematics::NearestSolution(Candidates{}, Joints{}, halfTurn) };

    EXPECT_EQ(result.status, kinematics::SolutionStatus::unreachable);
    EXPECT_FALSE(result.q.has_value());
}
