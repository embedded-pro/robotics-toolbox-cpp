#include "numerical/math/Tolerance.hpp"
#include "robotics/dynamics/CompositeRigidBodyAlgorithm.hpp"
#include "robotics/dynamics/CoriolisMatrix.hpp"
#include "robotics/dynamics/ModifiedRecursiveNewtonEuler.hpp"
#include "robotics/dynamics/RecursiveNewtonEuler.hpp"
#include <cmath>
#include <gtest/gtest.h>

namespace
{
    using Vector3 = math::Vector<float, 3>;
    using Matrix3 = math::SquareMatrix<float, 3>;
    using Joints = math::Vector<float, 3>;
    using Link = dynamics::JointLink<float>;

    const Vector3 gravity{ 0.0f, 0.0f, -9.81f };
    constexpr float finiteDifferenceTolerance{ 1e-4f };

    Vector3 Normalized(float x, float y, float z)
    {
        const float norm{ std::sqrt(x * x + y * y + z * z) };
        return Vector3{ x / norm, y / norm, z / norm };
    }

    class TestCoriolisMatrix
        : public ::testing::Test
    {
    protected:
        Matrix3 MassMatrixRate(const Joints& position, const Joints& velocity) const
        {
            constexpr float step{ 1e-3f };
            const auto ahead{ crba.Compute(chain, position + velocity * step) };
            const auto behind{ crba.Compute(chain, position - velocity * step) };
            return Matrix3{ (ahead - behind) * (0.5f / step) };
        }

        std::array<Link, 3> chain{
            Link{ 1.2f, Matrix3{ { 0.020f, 0.001f, -0.002f }, { 0.001f, 0.030f, 0.003f }, { -0.002f, 0.003f, 0.025f } }, Vector3{ 0.0f, 0.0f, 1.0f }, Vector3{ 0.0f, 0.0f, 0.1f }, Vector3{ 0.05f, 0.02f, 0.1f } },
            Link{ 0.9f, Matrix3{ { 0.015f, -0.002f, 0.001f }, { -0.002f, 0.020f, 0.0f }, { 0.001f, 0.0f, 0.012f } }, Normalized(0.0f, 1.0f, 1.0f), Vector3{ 0.1f, 0.0f, 0.2f }, Vector3{ 0.15f, -0.03f, 0.02f } },
            Link{ 0.6f, Matrix3{ { 0.008f, 0.0f, 0.001f }, { 0.0f, 0.010f, -0.001f }, { 0.001f, -0.001f, 0.006f } }, Normalized(1.0f, 0.0f, 1.0f), Vector3{ 0.3f, 0.05f, 0.0f }, Vector3{ 0.1f, 0.04f, -0.05f } }
        };
        dynamics::CoriolisMatrix<float, 3> coriolis;
        dynamics::ModifiedRecursiveNewtonEuler<float, 3> modified;
        dynamics::RecursiveNewtonEuler<float, 3> rnea;
        dynamics::CompositeRigidBodyAlgorithm<float, 3> crba;
        Joints q{ 0.4f, -0.7f, 1.1f };
        Joints qDot{ 0.9f, -1.3f, 0.6f };
    };
}

TEST_F(TestCoriolisMatrix, reduces_to_rnea_when_reference_equals_actual)
{
    const Joints qDDot{ 1.5f, -2.0f, 0.7f };

    const auto fromModified{ modified.InverseDynamics(chain, q, qDot, qDot, qDDot, gravity) };
    const auto fromRnea{ rnea.InverseDynamics(chain, q, qDot, qDDot, gravity) };

    for (std::size_t i = 0; i < 3; ++i)
        EXPECT_NEAR(fromModified[i], fromRnea[i], math::Tolerance<float>());
}

TEST_F(TestCoriolisMatrix, coriolis_times_velocity_equals_rnea_bias)
{
    const auto product{ Joints{ coriolis.Compute(chain, q, qDot) * qDot } };
    const auto bias{ rnea.InverseDynamics(chain, q, qDot, Joints{}, Vector3{}) };

    for (std::size_t i = 0; i < 3; ++i)
        EXPECT_NEAR(product[i], bias[i], math::Tolerance<float>());
}

TEST_F(TestCoriolisMatrix, matches_christoffel_symbols_of_the_mass_matrix)
{
    constexpr float step{ 1e-3f };
    std::array<Matrix3, 3> derivative{};
    for (std::size_t k = 0; k < 3; ++k)
    {
        Joints unit{};
        unit[k] = step;
        derivative[k] = Matrix3{ (crba.Compute(chain, q + unit) - crba.Compute(chain, q - unit)) * (0.5f / step) };
    }

    const auto C{ coriolis.Compute(chain, q, qDot) };

    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j)
        {
            float expected{ 0.0f };
            for (std::size_t k = 0; k < 3; ++k)
                expected += 0.5f * (derivative[k].at(i, j) + derivative[j].at(i, k) - derivative[i].at(j, k)) * qDot[k];
            EXPECT_NEAR(C.at(i, j), expected, finiteDifferenceTolerance);
        }
}

TEST_F(TestCoriolisMatrix, mass_matrix_derivative_minus_twice_coriolis_is_skew)
{
    const Matrix3 n{ MassMatrixRate(q, qDot) - coriolis.Compute(chain, q, qDot) * 2.0f };

    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j)
            EXPECT_NEAR(n.at(i, j) + n.at(j, i), 0.0f, finiteDifferenceTolerance);
}

TEST_F(TestCoriolisMatrix, two_link_planar_closed_form)
{
    const float m2{ 0.7f };
    const float l1{ 0.6f };
    const float l2{ 0.45f };
    const auto rod{ [](float mass, float length, const Vector3& parentToJoint)
        {
            const float inertia{ mass * length * length / 12.0f };
            return Link{ mass, Matrix3{ { 0.0f, 0.0f, 0.0f }, { 0.0f, inertia, 0.0f }, { 0.0f, 0.0f, inertia } }, Vector3{ 0.0f, 0.0f, 1.0f }, parentToJoint, Vector3{ length / 2.0f, 0.0f, 0.0f } };
        } };
    const std::array<Link, 2> planar{ rod(1.2f, l1, Vector3{}), rod(m2, l2, Vector3{ l1, 0.0f, 0.0f }) };
    const math::Vector<float, 2> position{ 0.3f, -0.8f };
    const math::Vector<float, 2> velocity{ 1.1f, -0.6f };
    const float h{ m2 * l1 * (l2 / 2.0f) * std::sin(position[1]) };

    const auto C{ dynamics::CoriolisMatrix<float, 2>{}.Compute(planar, position, velocity) };

    EXPECT_NEAR(C.at(0, 0), -h * velocity[1], math::Tolerance<float>());
    EXPECT_NEAR(C.at(0, 1), -h * (velocity[0] + velocity[1]), math::Tolerance<float>());
    EXPECT_NEAR(C.at(1, 0), h * velocity[0], math::Tolerance<float>());
    EXPECT_NEAR(C.at(1, 1), 0.0f, math::Tolerance<float>());
}
