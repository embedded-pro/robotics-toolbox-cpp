#include "numerical/math/Tolerance.hpp"
#include "robotics/dynamics/EffectiveJointInertia.hpp"
#include <gtest/gtest.h>

namespace
{
    using Matrix2 = math::SquareMatrix<float, 2>;

    class TestEffectiveJointInertia
        : public ::testing::Test
    {};
}

TEST_F(TestEffectiveJointInertia, uncoupled_joints_feel_their_own_inertia)
{
    const auto inertia{ dynamics::EffectiveJointInertia(Matrix2{ { 2.0f, 0.0f }, { 0.0f, 0.5f } }) };

    ASSERT_TRUE(inertia.has_value());
    EXPECT_NEAR((*inertia)[0], 2.0f, math::Tolerance<float>());
    EXPECT_NEAR((*inertia)[1], 0.5f, math::Tolerance<float>());
}

TEST_F(TestEffectiveJointInertia, coupling_lowers_the_inertia_below_the_locked_one)
{
    const auto inertia{ dynamics::EffectiveJointInertia(Matrix2{ { 2.0f, 1.0f }, { 1.0f, 1.0f } }) };

    ASSERT_TRUE(inertia.has_value());
    EXPECT_NEAR((*inertia)[0], 1.0f, math::Tolerance<float>());
    EXPECT_NEAR((*inertia)[1], 0.5f, math::Tolerance<float>());
}

TEST_F(TestEffectiveJointInertia, rejects_a_mass_matrix_that_is_not_positive_definite)
{
    EXPECT_FALSE(dynamics::EffectiveJointInertia(Matrix2{ { 1.0f, 2.0f }, { 2.0f, 1.0f } }).has_value());
}
