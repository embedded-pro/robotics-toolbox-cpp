#include "robotics/dynamics/ArticulatedBodyAlgorithm.hpp"
#include "robotics/dynamics/RecursiveNewtonEuler.hpp"
#include "numerical/math/Tolerance.hpp"
#include <cmath>
#include <gtest/gtest.h>

namespace
{
    using Vector3 = math::Vector<float, 3>;
    using Link = dynamics::RevoluteJointLink<float>;

    constexpr float gravity{ 9.81f };
    const Vector3 gravityVector{ 0.0f, 0.0f, -gravity };
    const Vector3 yAxis{ 0.0f, 1.0f, 0.0f };
    const Vector3 zAxis{ 0.0f, 0.0f, 1.0f };

    Link MakeRod(float mass, float length, const Vector3& axis, const Vector3& parentToJoint)
    {
        const float inertia{ mass * length * length / 12.0f };

        return Link{
            mass,
            math::SquareMatrix<float, 3>{
                { 0.0f, 0.0f, 0.0f },
                { 0.0f, inertia, 0.0f },
                { 0.0f, 0.0f, inertia } },
            axis,
            parentToJoint,
            Vector3{ length / 2.0f, 0.0f, 0.0f }
        };
    }

    Vector3 Normalized(float x, float y, float z)
    {
        const float norm{ std::sqrt(x * x + y * y + z * z) };
        return Vector3{ x / norm, y / norm, z / norm };
    }

    class TestArticulatedBodyAlgorithm
        : public ::testing::Test
    {
    protected:
        dynamics::ArticulatedBodyAlgorithm<float, 1> aba1;
        dynamics::ArticulatedBodyAlgorithm<float, 2> aba2;
        dynamics::ArticulatedBodyAlgorithm<float, 3> aba3;
    };
}

TEST_F(TestArticulatedBodyAlgorithm, zero_torque_without_gravity_gives_zero_acceleration)
{
    std::array<Link, 1> links{ MakeRod(1.0f, 1.0f, zAxis, Vector3{}) };

    auto qDDot = aba1.ForwardDynamics(links, math::Vector<float, 1>{ 0.3f }, math::Vector<float, 1>{}, math::Vector<float, 1>{}, Vector3{});

    EXPECT_NEAR(qDDot.at(0, 0), 0.0f, math::Tolerance<float>());
}

TEST_F(TestArticulatedBodyAlgorithm, torque_divided_by_end_inertia_gives_acceleration)
{
    const float mass{ 2.0f };
    const float length{ 1.0f };
    std::array<Link, 1> links{ MakeRod(mass, length, zAxis, Vector3{}) };

    auto qDDot = aba1.ForwardDynamics(links, math::Vector<float, 1>{ 0.0f }, math::Vector<float, 1>{}, math::Vector<float, 1>{ 1.0f }, Vector3{});

    EXPECT_NEAR(qDDot.at(0, 0), 3.0f / (mass * length * length), math::Tolerance<float>());
}

TEST_F(TestArticulatedBodyAlgorithm, horizontal_link_released_falls_at_three_g_over_two_l)
{
    const float length{ 1.0f };
    std::array<Link, 1> links{ MakeRod(1.0f, length, yAxis, Vector3{}) };

    auto qDDot = aba1.ForwardDynamics(links, math::Vector<float, 1>{ 0.0f }, math::Vector<float, 1>{}, math::Vector<float, 1>{}, gravityVector);

    EXPECT_NEAR(qDDot.at(0, 0), 3.0f * gravity / (2.0f * length), math::Tolerance<float>());
}

TEST_F(TestArticulatedBodyAlgorithm, analytic_two_link_torque_produces_commanded_acceleration)
{
    const float q2{ -0.5f };
    const float qd1{ 1.0f };
    const float qd2{ -0.5f };
    const float qdd1{ 2.0f };
    const float qdd2{ -1.0f };
    const float c2{ std::cos(q2) };
    const float h{ -std::sin(q2) };
    const float m11{ 1.0f / 3.0f + 1.0f + 1.0f / 3.0f + c2 };
    const float m12{ 1.0f / 3.0f + c2 / 2.0f };
    const float m22{ 1.0f / 3.0f };
    math::Vector<float, 2> tau{
        m11 * qdd1 + m12 * qdd2 + h * (2.0f * qd1 * qd2 + qd2 * qd2) / 2.0f,
        m12 * qdd1 + m22 * qdd2 - h * qd1 * qd1 / 2.0f
    };
    std::array<Link, 2> links{ MakeRod(1.0f, 1.0f, zAxis, Vector3{}), MakeRod(1.0f, 1.0f, zAxis, Vector3{ 1.0f, 0.0f, 0.0f }) };

    auto qDDot = aba2.ForwardDynamics(links, math::Vector<float, 2>{ 0.3f, q2 }, math::Vector<float, 2>{ qd1, qd2 }, tau, Vector3{});

    EXPECT_NEAR(qDDot.at(0, 0), qdd1, math::Tolerance<float>());
    EXPECT_NEAR(qDDot.at(1, 0), qdd2, math::Tolerance<float>());
}

TEST_F(TestArticulatedBodyAlgorithm, two_link_released_horizontal_matches_analytic_acceleration)
{
    std::array<Link, 2> links{ MakeRod(1.0f, 1.0f, yAxis, Vector3{}), MakeRod(1.0f, 1.0f, yAxis, Vector3{ 1.0f, 0.0f, 0.0f }) };

    auto qDDot = aba2.ForwardDynamics(links, math::Vector<float, 2>{ 0.0f, 0.0f }, math::Vector<float, 2>{}, math::Vector<float, 2>{}, gravityVector);

    EXPECT_NEAR(qDDot.at(0, 0), 9.0f * gravity / 7.0f, math::Tolerance<float>());
    EXPECT_NEAR(qDDot.at(1, 0), -12.0f * gravity / 7.0f, math::Tolerance<float>());
}

TEST_F(TestArticulatedBodyAlgorithm, spatial_chain_inverts_recursive_newton_euler)
{
    const math::SquareMatrix<float, 3> inertia{
        { 0.05f, 0.01f, -0.005f },
        { 0.01f, 0.04f, 0.008f },
        { -0.005f, 0.008f, 0.03f }
    };
    std::array<Link, 3> links{
        Link{ 1.4f, inertia, Normalized(0.0f, 0.2f, 1.0f), Vector3{ 0.0f, 0.0f, 0.1f }, Vector3{ 0.02f, -0.01f, 0.15f } },
        Link{ 0.9f, inertia * 0.8f, Normalized(0.1f, 1.0f, 0.0f), Vector3{ 0.05f, 0.0f, 0.3f }, Vector3{ 0.2f, 0.03f, -0.02f } },
        Link{ 0.5f, inertia * 0.5f, Normalized(1.0f, 0.3f, -0.4f), Vector3{ 0.4f, -0.05f, 0.02f }, Vector3{ 0.1f, 0.02f, 0.03f } }
    };
    dynamics::RecursiveNewtonEuler<float, 3> rnea;
    math::Vector<float, 3> q{ 0.4f, -0.7f, 1.1f };
    math::Vector<float, 3> qDot{ 0.9f, -1.3f, 0.6f };
    math::Vector<float, 3> expected{ 1.5f, -2.0f, 0.7f };

    auto tau = rnea.InverseDynamics(links, q, qDot, expected, gravityVector);
    auto qDDot = aba3.ForwardDynamics(links, q, qDot, tau, gravityVector);

    EXPECT_NEAR(qDDot.at(0, 0), expected.at(0, 0), math::Tolerance<float>());
    EXPECT_NEAR(qDDot.at(1, 0), expected.at(1, 0), math::Tolerance<float>());
    EXPECT_NEAR(qDDot.at(2, 0), expected.at(2, 0), math::Tolerance<float>());
}
