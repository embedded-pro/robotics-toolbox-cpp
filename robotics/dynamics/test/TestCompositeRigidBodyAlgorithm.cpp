#include "numerical/math/Geometry3D.hpp"
#include "numerical/math/Tolerance.hpp"
#include "robotics/dynamics/CompositeRigidBodyAlgorithm.hpp"
#include "robotics/dynamics/RecursiveNewtonEuler.hpp"
#include <cmath>
#include <gtest/gtest.h>

namespace
{
    using Vector3 = math::Vector<float, 3>;
    using Matrix3 = math::SquareMatrix<float, 3>;
    using Link = dynamics::RevoluteJointLink<float>;

    const Vector3 zAxis{ 0.0f, 0.0f, 1.0f };

    Link MakeRod(float mass, float length, const Vector3& parentToJoint)
    {
        const float inertia{ mass * length * length / 12.0f };
        return Link{ mass, Matrix3{ { 0.0f, 0.0f, 0.0f }, { 0.0f, inertia, 0.0f }, { 0.0f, 0.0f, inertia } }, zAxis, parentToJoint,
            Vector3{ length / 2.0f, 0.0f, 0.0f } };
    }

    Vector3 Normalized(float x, float y, float z)
    {
        const float norm{ std::sqrt(x * x + y * y + z * z) };
        return Vector3{ x / norm, y / norm, z / norm };
    }

    float Determinant2(const math::SquareMatrix<float, 3>& M)
    {
        return M.at(0, 0) * M.at(1, 1) - M.at(0, 1) * M.at(1, 0);
    }

    float Determinant3(const math::SquareMatrix<float, 3>& M)
    {
        return M.at(0, 0) * (M.at(1, 1) * M.at(2, 2) - M.at(1, 2) * M.at(2, 1)) - M.at(0, 1) * (M.at(1, 0) * M.at(2, 2) - M.at(1, 2) * M.at(2, 0)) + M.at(0, 2) * (M.at(1, 0) * M.at(2, 1) - M.at(1, 1) * M.at(2, 0));
    }

    class TestCompositeRigidBodyAlgorithm
        : public ::testing::Test
    {
    protected:
        float KineticEnergyOfLinks(const math::Vector<float, 3>& q, const math::Vector<float, 3>& qDot) const
        {
            Vector3 omega{};
            Vector3 velocity{};
            float energy{ 0.0f };
            for (std::size_t i = 0; i < 3; ++i)
            {
                const Matrix3 Rt{ math::RotationAboutAxis(chain[i].jointAxis, q[i]).Transpose() };
                velocity = Rt * (velocity + math::CrossProduct(omega, chain[i].parentToJoint));
                omega = Rt * omega + chain[i].jointAxis * qDot[i];
                const Vector3 velocityCoM{ velocity + math::CrossProduct(omega, chain[i].jointToCoM) };
                energy += 0.5f * chain[i].mass * math::DotProduct(velocityCoM, velocityCoM) + 0.5f * math::DotProduct(omega, chain[i].inertia * omega);
            }
            return energy;
        }

        dynamics::CompositeRigidBodyAlgorithm<float, 2> crba2;
        dynamics::CompositeRigidBodyAlgorithm<float, 3> crba3;
        std::array<Link, 3> chain{
            Link{ 1.2f, Matrix3{ { 0.020f, 0.001f, -0.002f }, { 0.001f, 0.030f, 0.003f }, { -0.002f, 0.003f, 0.025f } }, zAxis, Vector3{ 0.0f, 0.0f, 0.1f }, Vector3{ 0.05f, 0.02f, 0.1f } },
            Link{ 0.9f, Matrix3{ { 0.015f, -0.002f, 0.001f }, { -0.002f, 0.020f, 0.0f }, { 0.001f, 0.0f, 0.012f } }, Normalized(0.0f, 1.0f, 1.0f), Vector3{ 0.1f, 0.0f, 0.2f }, Vector3{ 0.15f, -0.03f, 0.02f } },
            Link{ 0.6f, Matrix3{ { 0.008f, 0.0f, 0.001f }, { 0.0f, 0.010f, -0.001f }, { 0.001f, -0.001f, 0.006f } }, Normalized(1.0f, 0.0f, 1.0f), Vector3{ 0.3f, 0.05f, 0.0f }, Vector3{ 0.1f, 0.04f, -0.05f } }
        };
        math::Vector<float, 3> q{ 0.4f, -0.7f, 1.1f };
    };
}

TEST_F(TestCompositeRigidBodyAlgorithm, single_rod_about_end_is_ml2_over_3)
{
    const float mass{ 2.0f };
    const float length{ 1.5f };
    dynamics::CompositeRigidBodyAlgorithm<float, 1> crba1;

    const auto M{ crba1.Compute({ MakeRod(mass, length, Vector3{}) }, math::Vector<float, 1>{ 0.7f }) };

    EXPECT_NEAR(M.at(0, 0), mass * length * length / 3.0f, math::Tolerance<float>());
}

TEST_F(TestCompositeRigidBodyAlgorithm, two_link_planar_matches_closed_form)
{
    const float m1{ 2.0f };
    const float l1{ 1.2f };
    const float m2{ 0.8f };
    const float l2{ 0.7f };
    const float q2{ -0.5f };

    const auto M{ crba2.Compute({ MakeRod(m1, l1, Vector3{}), MakeRod(m2, l2, Vector3{ l1, 0.0f, 0.0f }) }, math::Vector<float, 2>{ 0.3f, q2 }) };

    EXPECT_NEAR(M.at(0, 0), m1 * l1 * l1 / 3.0f + m2 * (l1 * l1 + l2 * l2 / 3.0f + l1 * l2 * std::cos(q2)), math::Tolerance<float>());
    EXPECT_NEAR(M.at(0, 1), m2 * (l2 * l2 / 3.0f + l1 * l2 * std::cos(q2) / 2.0f), math::Tolerance<float>());
    EXPECT_NEAR(M.at(1, 1), m2 * l2 * l2 / 3.0f, math::Tolerance<float>());
}

TEST_F(TestCompositeRigidBodyAlgorithm, matrix_is_independent_of_first_joint_angle)
{
    const std::array<Link, 2> links{ MakeRod(1.0f, 1.0f, Vector3{}), MakeRod(1.0f, 1.0f, Vector3{ 1.0f, 0.0f, 0.0f }) };

    const auto first{ crba2.Compute(links, math::Vector<float, 2>{ 0.3f, -0.5f }) };
    const auto second{ crba2.Compute(links, math::Vector<float, 2>{ -1.2f, -0.5f }) };

    for (std::size_t r = 0; r < 2; ++r)
        for (std::size_t c = 0; c < 2; ++c)
            EXPECT_NEAR(first.at(r, c), second.at(r, c), math::Tolerance<float>());
}

TEST_F(TestCompositeRigidBodyAlgorithm, columns_match_recursive_newton_euler)
{
    dynamics::RecursiveNewtonEuler<float, 3> rnea;

    const auto M{ crba3.Compute(chain, q) };

    for (std::size_t j = 0; j < 3; ++j)
    {
        math::Vector<float, 3> unit{};
        unit[j] = 1.0f;
        const auto column{ rnea.InverseDynamics(chain, q, math::Vector<float, 3>{}, unit, Vector3{}) };
        for (std::size_t i = 0; i < 3; ++i)
            EXPECT_NEAR(M.at(i, j), column[i], math::Tolerance<float>());
    }
}

TEST_F(TestCompositeRigidBodyAlgorithm, matrix_is_symmetric_positive_definite)
{
    const auto M{ crba3.Compute(chain, q) };

    for (std::size_t r = 0; r < 3; ++r)
        for (std::size_t c = 0; c < 3; ++c)
            EXPECT_EQ(M.at(r, c), M.at(c, r));
    EXPECT_GT(M.at(0, 0), 0.0f);
    EXPECT_GT(Determinant2(M), 0.0f);
    EXPECT_GT(Determinant3(M), 0.0f);
}

TEST_F(TestCompositeRigidBodyAlgorithm, kinetic_energy_matches_link_sum)
{
    const math::Vector<float, 3> qDot{ 0.8f, -1.3f, 2.1f };

    const auto M{ crba3.Compute(chain, q) };

    EXPECT_NEAR(0.5f * math::DotProduct(qDot, M * qDot), KineticEnergyOfLinks(q, qDot), math::Tolerance<float>());
}
