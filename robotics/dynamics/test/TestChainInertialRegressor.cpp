#include "numerical/math/Tolerance.hpp"
#include "robotics/dynamics/ChainInertialRegressor.hpp"
#include "robotics/dynamics/ModifiedRecursiveNewtonEuler.hpp"
#include <cmath>
#include <gtest/gtest.h>

namespace
{
    using Vector3 = math::Vector<float, 3>;
    using Matrix3 = math::SquareMatrix<float, 3>;
    using Joints = math::Vector<float, 3>;
    using Link = dynamics::JointLink<float>;
    using Regressor = dynamics::ChainInertialRegressor<float, 3>;

    const Vector3 gravity{ 0.0f, 0.0f, -9.81f };

    Vector3 Normalized(float x, float y, float z)
    {
        const float norm{ std::sqrt(x * x + y * y + z * z) };
        return Vector3{ x / norm, y / norm, z / norm };
    }

    class TestChainInertialRegressor
        : public ::testing::Test
    {
    protected:
        std::array<Link, 3> chain{
            Link{ 1.2f, Matrix3{ { 0.020f, 0.001f, -0.002f }, { 0.001f, 0.030f, 0.003f }, { -0.002f, 0.003f, 0.025f } }, Vector3{ 0.0f, 0.0f, 1.0f }, Vector3{ 0.0f, 0.0f, 0.1f }, Vector3{ 0.05f, 0.02f, 0.1f } },
            Link{ 0.9f, Matrix3{ { 0.015f, -0.002f, 0.001f }, { -0.002f, 0.020f, 0.0f }, { 0.001f, 0.0f, 0.012f } }, Normalized(0.0f, 1.0f, 1.0f), Vector3{ 0.1f, 0.0f, 0.2f }, Vector3{ 0.15f, -0.03f, 0.02f } },
            Link{ 0.6f, Matrix3{ { 0.008f, 0.0f, 0.001f }, { 0.0f, 0.010f, -0.001f }, { 0.001f, -0.001f, 0.006f } }, Normalized(1.0f, 0.0f, 1.0f), Vector3{ 0.3f, 0.05f, 0.0f }, Vector3{ 0.1f, 0.04f, -0.05f } }
        };
        Joints q{ 0.4f, -0.7f, 1.1f };
        Joints qDot{ 0.9f, -1.3f, 0.6f };
        Joints qDotReference{ 0.5f, 0.8f, -1.1f };
        Joints qDDotReference{ 1.5f, -2.0f, 0.7f };
    };
}

TEST_F(TestChainInertialRegressor, regressor_times_parameters_reproduces_dynamics)
{
    const Regressor regressor{ chain, gravity };

    const auto Y{ regressor.Compute(q, qDot, qDotReference, qDDotReference) };
    const auto expected{ dynamics::ModifiedRecursiveNewtonEuler<float, 3>{}.InverseDynamics(chain, q, qDot, qDotReference, qDDotReference, gravity) };

    const Joints reproduced{ Y * Regressor::Parameters(chain) };
    for (std::size_t i = 0; i < 3; ++i)
        EXPECT_NEAR(reproduced[i], expected[i], math::Tolerance<float>());
}

TEST_F(TestChainInertialRegressor, regressor_is_independent_of_parameters)
{
    auto heavier{ chain };
    for (auto& link : heavier)
    {
        link.mass *= 2.5f;
        link.inertia = link.inertia * 1.7f;
        link.jointToCoM = link.jointToCoM * 0.6f;
    }
    const Regressor original{ chain, gravity };
    const Regressor modified{ heavier, gravity };

    const auto first{ original.Compute(q, qDot, qDotReference, qDDotReference) };
    const auto second{ modified.Compute(q, qDot, qDotReference, qDDotReference) };

    for (std::size_t r = 0; r < 3; ++r)
        for (std::size_t c = 0; c < 30; ++c)
            EXPECT_EQ(first.at(r, c), second.at(r, c));
}
