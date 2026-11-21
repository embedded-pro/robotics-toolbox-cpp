#include "numerical/math/Tolerance.hpp"
#include "robotics/controllers/manipulator/CoupledDamping.hpp"
#include <cmath>
#include <gtest/gtest.h>

namespace
{
    using Matrix2 = math::SquareMatrix<float, 2>;
    using Vector2 = math::Vector<float, 2>;

    class TestCoupledDamping
        : public ::testing::Test
    {
    protected:
        static void ExpectNear(const Matrix2& actual, const Matrix2& expected)
        {
            for (std::size_t row = 0; row < 2; ++row)
                for (std::size_t column = 0; column < 2; ++column)
                    EXPECT_NEAR(actual.at(row, column), expected.at(row, column), math::Tolerance<float>());
        }

        float dampingRatio{ 0.7f };
    };
}

TEST_F(TestCoupledDamping, uncoupled_joints_get_the_damping_of_a_single_spring_and_mass)
{
    const auto damping{ controllers::CoupledDamping(Matrix2{ { 2.0f, 0.0f }, { 0.0f, 0.5f } }, Vector2{ 8.0f, 2.0f }, dampingRatio) };

    ExpectNear(damping, Matrix2{ { 2.0f * dampingRatio * std::sqrt(8.0f * 2.0f), 0.0f }, { 0.0f, 2.0f * dampingRatio * std::sqrt(2.0f * 0.5f) } });
}

TEST_F(TestCoupledDamping, equal_natural_frequencies_scale_the_coupled_mass_matrix)
{
    const Matrix2 inertia{ { 2.0f, 0.6f }, { 0.6f, 0.5f } };
    const float omega{ 3.0f };

    const auto damping{ controllers::CoupledDamping(inertia, Vector2{ omega * omega * 2.0f, omega * omega * 0.5f }, dampingRatio) };

    ExpectNear(damping, inertia * (2.0f * dampingRatio * omega));
}

TEST_F(TestCoupledDamping, a_joint_without_inertia_gets_no_damping)
{
    const auto damping{ controllers::CoupledDamping(Matrix2{ { 2.0f, 0.0f }, { 0.0f, 0.0f } }, Vector2{ 8.0f, 2.0f }, dampingRatio) };

    EXPECT_NEAR(damping.at(1, 1), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(damping.at(0, 1), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(damping.at(1, 0), 0.0f, math::Tolerance<float>());
}
