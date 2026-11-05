#include "numerical/math/Tolerance.hpp"
#include "robotics/planning/SelfCollision.hpp"
#include <array>
#include <cmath>
#include <gtest/gtest.h>

namespace
{
    using Vector3 = math::Vector3<float>;
    using Frames = kinematics::FrameChain<float, 4>;

    constexpr std::array<float, 4> lengths{ 0.4f, 0.8f, 0.7f, 0.5f };

    Frames Planar(float q0, float q1, float q2, float q3)
    {
        const std::array<float, 4> q{ q0, q1, q2, q3 };
        Frames frames{};
        Vector3 origin{};
        float theta{ 0.0f };
        for (std::size_t k = 0; k < 4; ++k)
        {
            frames.jointOrigins[k] = origin;
            frames.jointAxes[k] = Vector3{ 0.0f, 0.0f, 1.0f };
            theta += q[k];
            origin = origin + Vector3{ std::cos(theta), std::sin(theta), 0.0f } * lengths[k];
        }
        frames.tool.translation = origin;
        return frames;
    }

    class TestSelfCollision
        : public ::testing::Test
    {
    protected:
        void ExpectPair(const planning::SelfCollisionResult<float>& result, std::uint8_t first, std::uint8_t second) const
        {
            EXPECT_EQ(result.pair[0], first);
            EXPECT_EQ(result.pair[1], second);
        }

        planning::SelfCollision<float, 4> model{ { 0.1f, 0.08f, 0.06f, 0.05f } };
    };
}

TEST_F(TestSelfCollision, straight_arm_is_clear)
{
    const auto result{ model.Check(Planar(0.0f, 0.0f, 0.0f, 0.0f), 0.0f) };

    EXPECT_NEAR(result.minimumDistance, 0.57f, math::Tolerance<float>());
    ExpectPair(result, 1, 3);
    EXPECT_FALSE(result.colliding);
    EXPECT_EQ(result.checkedPairs, 3u);
}

TEST_F(TestSelfCollision, folded_arm_collides_between_links_0_and_2)
{
    const auto result{ model.Check(Planar(0.0f, 2.4f, 2.4f, 0.5f), 0.0f) };

    EXPECT_NEAR(result.minimumDistance, -0.018095f, math::Tolerance<float>());
    ExpectPair(result, 0, 2);
    EXPECT_TRUE(result.colliding);
}

TEST_F(TestSelfCollision, allowing_a_pair_reports_the_next_closest)
{
    model.Allow(0, 2);

    const auto result{ model.Check(Planar(0.0f, 2.4f, 2.4f, 0.5f), 0.0f) };

    EXPECT_NEAR(result.minimumDistance, 0.052945f, math::Tolerance<float>());
    ExpectPair(result, 0, 3);
    EXPECT_FALSE(result.colliding);
    EXPECT_EQ(result.checkedPairs, 2u);
}

TEST_F(TestSelfCollision, margin_turns_proximity_into_a_collision)
{
    const auto frames{ Planar(0.0f, 2.6f, 2.2f, 1.0f) };

    const auto unguarded{ model.Check(frames, 0.0f) };
    const auto guarded{ model.Check(frames, 0.1f) };

    EXPECT_NEAR(unguarded.minimumDistance, 0.088331f, math::Tolerance<float>());
    EXPECT_FALSE(unguarded.colliding);
    EXPECT_TRUE(guarded.colliding);
}

TEST_F(TestSelfCollision, links_1_and_3_can_also_collide)
{
    const auto result{ model.Check(Planar(0.5f, -1.0f, -2.5f, -2.0f), 0.0f) };

    EXPECT_NEAR(result.minimumDistance, -0.13f, math::Tolerance<float>());
    ExpectPair(result, 1, 3);
    EXPECT_TRUE(result.colliding);
}

TEST_F(TestSelfCollision, adjacent_pairs_are_allowed_by_default)
{
    EXPECT_TRUE(model.IsAllowed(0, 1));
    EXPECT_TRUE(model.IsAllowed(2, 3));
    EXPECT_FALSE(model.IsAllowed(0, 2));

    model.Disallow(0, 1);

    EXPECT_EQ(model.Check(Planar(0.0f, 0.0f, 0.0f, 0.0f), 0.0f).checkedPairs, 4u);
}
