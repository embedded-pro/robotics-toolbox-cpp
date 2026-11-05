#include "numerical/math/Tolerance.hpp"
#include "robotics/kinematics/DenavitHartenberg.hpp"
#include "robotics/kinematics/DhTaskJacobian.hpp"
#include "robotics/kinematics/RedundancyResolution.hpp"
#include <cmath>
#include <gtest/gtest.h>
#include <numbers>

namespace
{
    using DhLink = kinematics::DhLink<float>;
    using JointVector = math::Vector<float, 7>;
    using PoseVector = math::Vector<float, 6>;

    constexpr float halfPi{ std::numbers::pi_v<float> / 2.0f };
    constexpr float exactTolerance{ 1e-5f };

    class TestRedundancyResolution
        : public ::testing::Test
    {
    protected:
        template<std::size_t Rows>
        void ExpectNearZero(const math::Matrix<float, Rows, 1>& vector, float tolerance) const
        {
            for (std::size_t i = 0; i < Rows; ++i)
                EXPECT_NEAR(vector[i], 0.0f, tolerance);
        }

        template<std::size_t Rows>
        void ExpectNearZero(const math::Matrix<float, Rows, 7>& matrix, float tolerance) const
        {
            for (std::size_t r = 0; r < Rows; ++r)
                for (std::size_t c = 0; c < 7; ++c)
                    EXPECT_NEAR(matrix.at(r, c), 0.0f, tolerance);
        }

        static float Trace(const math::SquareMatrix<float, 7>& matrix)
        {
            float trace{ 0.0f };
            for (std::size_t i = 0; i < 7; ++i)
                trace += matrix.at(i, i);
            return trace;
        }

        kinematics::DenavitHartenberg<float, 7> arm{ { DhLink{ 0.0f, -halfPi, 0.34f, 0.0f }, DhLink{ 0.0f, halfPi, 0.0f, 0.0f }, DhLink{ 0.0f, halfPi, 0.4f, 0.0f },
                                                         DhLink{ 0.0f, -halfPi, 0.0f, 0.0f }, DhLink{ 0.0f, -halfPi, 0.4f, 0.0f }, DhLink{ 0.0f, halfPi, 0.0f, 0.0f },
                                                         DhLink{ 0.0f, 0.0f, 0.126f, 0.0f } },
            kinematics::DhConvention::Standard };
        kinematics::DhTaskJacobian<float, 6, 7> pose{ arm };
        kinematics::DhTaskJacobian<float, 3, 7> position{ arm };
        JointVector q{ 0.1f, 0.5f, -0.3f, -1.2f, 0.4f, 0.8f, -0.2f };
        PoseVector xDot{ 0.1f, -0.05f, 0.02f, 0.1f, 0.2f, -0.1f };
        JointVector qDot0{ 1.0f, -1.0f, 0.5f, 0.3f, -0.2f, 0.7f, -0.4f };
    };
}

TEST_F(TestRedundancyResolution, undamped_primary_task_is_exact)
{
    kinematics::RedundancyResolution<float, 6, 7> resolution{ pose, 0.0f };

    const auto qDot{ resolution.Resolve(q, xDot, qDot0) };
    const auto rightInverse{ pose.Jacobian(q) * resolution.DampedPseudoInverse(q) };

    ExpectNearZero(PoseVector{ pose.Jacobian(q) * qDot - xDot }, exactTolerance);
    for (std::size_t r = 0; r < 6; ++r)
        for (std::size_t c = 0; c < 6; ++c)
            EXPECT_NEAR(rightInverse.at(r, c), r == c ? 1.0f : 0.0f, math::Tolerance<float>());
}

TEST_F(TestRedundancyResolution, damped_resolution_matches_reference)
{
    kinematics::RedundancyResolution<float, 6, 7> resolution{ pose, 0.01f };
    const JointVector expected{ -0.093934f, 0.325324f, -0.092001f, 0.551001f, 0.088089f, 0.448837f, 0.140035f };

    const auto qDot{ resolution.Resolve(q, xDot, qDot0) };

    for (std::size_t i = 0; i < 7; ++i)
        EXPECT_NEAR(qDot[i], expected[i], math::Tolerance<float>());
}

TEST_F(TestRedundancyResolution, null_space_motion_is_task_invisible_despite_damping)
{
    kinematics::RedundancyResolution<float, 6, 7> resolution{ pose, 0.01f };

    const auto projector{ resolution.NullSpaceProjector(q) };

    ExpectNearZero(PoseVector{ pose.Jacobian(q) * (projector * qDot0) }, exactTolerance);
}

TEST_F(TestRedundancyResolution, secondary_objective_reduces_cost_without_moving_the_tool)
{
    kinematics::RedundancyResolution<float, 6, 7> resolution{ pose, 0.0f };
    const JointVector gradient{ q * (1.0f / (2.9f * 2.9f)) };

    const auto qDot{ resolution.Resolve(q, PoseVector{}, JointVector{ gradient * -1.0f }) };

    EXPECT_NEAR((gradient.Transpose() * qDot).at(0, 0), -0.0051809f, 1e-5f);
    ExpectNearZero(PoseVector{ pose.Jacobian(q) * qDot }, exactTolerance);
}

TEST_F(TestRedundancyResolution, position_task_has_four_dimensional_null_space)
{
    kinematics::RedundancyResolution<float, 3, 7> resolution{ position, 0.0f };

    const auto projector{ resolution.NullSpaceProjector(q) };

    EXPECT_NEAR(Trace(projector), 4.0f, math::Tolerance<float>());
    ExpectNearZero(math::Matrix<float, 3, 7>{ position.Jacobian(q) * projector }, exactTolerance);
}

TEST_F(TestRedundancyResolution, rank_loss_grows_the_null_space)
{
    const JointVector stretched{ 0.1f, 0.0f, -0.3f, 0.0f, 0.4f, 0.0f, -0.2f };
    kinematics::RedundancyResolution<float, 6, 7> undamped{ pose, 0.0f };
    kinematics::RedundancyResolution<float, 6, 7> damped{ pose, 0.01f };

    const auto projector{ undamped.NullSpaceProjector(stretched) };
    const auto qDot{ damped.Resolve(stretched, xDot, qDot0) };

    EXPECT_NEAR(Trace(projector), 3.0f, math::Tolerance<float>());
    ExpectNearZero(math::Matrix<float, 6, 7>{ pose.Jacobian(stretched) * projector }, exactTolerance);
    for (std::size_t i = 0; i < 7; ++i)
        EXPECT_TRUE(std::isfinite(qDot[i]));
}
