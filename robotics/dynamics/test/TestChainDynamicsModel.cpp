#include "numerical/math/Tolerance.hpp"
#include "robotics/dynamics/ArticulatedBodyAlgorithm.hpp"
#include "robotics/dynamics/ChainDynamicsModel.hpp"
#include "robotics/dynamics/CompositeRigidBodyAlgorithm.hpp"
#include "robotics/dynamics/EulerLagrangeSolver.hpp"
#include <cmath>
#include <gtest/gtest.h>

namespace
{
    using Vector2 = math::Vector<float, 2>;
    using Vector3 = math::Vector<float, 3>;
    using Link = dynamics::JointLink<float>;
    using LinkArray = std::array<Link, 2>;

    constexpr float gravity{ 9.81f };
    constexpr float m1{ 1.2f };
    constexpr float l1{ 0.6f };
    constexpr float m2{ 0.7f };
    constexpr float l2{ 0.45f };
    const Vector3 gravityVector{ 0.0f, 0.0f, -gravity };
    const Vector3 yAxis{ 0.0f, 1.0f, 0.0f };

    Link MakeRod(float mass, float length, const Vector3& parentToJoint)
    {
        const float inertia{ mass * length * length / 12.0f };
        return Link{ mass, math::SquareMatrix<float, 3>{ { 0.0f, 0.0f, 0.0f }, { 0.0f, inertia, 0.0f }, { 0.0f, 0.0f, inertia } }, yAxis, parentToJoint,
            Vector3{ length / 2.0f, 0.0f, 0.0f } };
    }

    LinkArray MakeRodChain(float mass1, float length1, float mass2, float length2)
    {
        return LinkArray{ MakeRod(mass1, length1, Vector3{}), MakeRod(mass2, length2, Vector3{ length1, 0.0f, 0.0f }) };
    }

    Vector2 ClosedFormGravity(float mass2, const Vector2& q)
    {
        const float c1{ std::cos(q[0]) };
        const float c12{ std::cos(q[0] + q[1]) };
        return Vector2{ -gravity * (m1 * l1 / 2.0f * c1 + mass2 * (l1 * c1 + l2 / 2.0f * c12)), -gravity * mass2 * l2 / 2.0f * c12 };
    }

    class TestChainDynamicsModel
        : public ::testing::Test
    {
    protected:
        void ExpectNear(const Vector2& actual, const Vector2& expected) const
        {
            EXPECT_NEAR(actual[0], expected[0], math::Tolerance<float>());
            EXPECT_NEAR(actual[1], expected[1], math::Tolerance<float>());
        }

        LinkArray links{ MakeRodChain(m1, l1, m2, l2) };
        dynamics::ChainDynamicsModel<float, 2> model{ links, gravityVector };
        Vector2 q{ 0.4f, -0.9f };
        Vector2 qDot{ 1.1f, -0.6f };
        Vector2 qDDot{ 0.8f, -1.7f };
    };
}

TEST_F(TestChainDynamicsModel, gravity_terms_match_closed_form)
{
    ExpectNear(model.ComputeGravityTerms(q), ClosedFormGravity(m2, q));
}

TEST_F(TestChainDynamicsModel, coriolis_terms_match_closed_form)
{
    const float h{ m2 * l1 * l2 / 2.0f * std::sin(q[1]) };

    ExpectNear(model.ComputeCoriolisTerms(q, qDot), Vector2{ -h * (2.0f * qDot[0] * qDot[1] + qDot[1] * qDot[1]), h * qDot[0] * qDot[0] });
}

TEST_F(TestChainDynamicsModel, mass_matrix_matches_crba)
{
    const auto M{ model.ComputeMassMatrix(q) };
    const auto expected{ dynamics::CompositeRigidBodyAlgorithm<float, 2>{}.Compute(links, q) };

    for (std::size_t r = 0; r < 2; ++r)
        for (std::size_t c = 0; c < 2; ++c)
            EXPECT_EQ(M.at(r, c), expected.at(r, c));
}

TEST_F(TestChainDynamicsModel, inverse_dynamics_equals_manipulator_equation)
{
    const auto tau{ model.ComputeInverseDynamics(q, qDot, qDDot) };

    ExpectNear(tau, Vector2{ model.ComputeMassMatrix(q) * qDDot + model.ComputeCoriolisTerms(q, qDot) + model.ComputeGravityTerms(q) });
    ExpectNear(tau, Vector2{ -8.2064f, -1.4410f });
}

TEST_F(TestChainDynamicsModel, euler_lagrange_forward_dynamics_matches_articulated_body_algorithm)
{
    const Vector2 tau{ 1.5f, -0.4f };

    const auto fromModel{ dynamics::EulerLagrangeSolver<float, 2>{}.ForwardDynamics(model, q, qDot, tau) };
    const auto fromAba{ dynamics::ArticulatedBodyAlgorithm<float, 2>{}.ForwardDynamics(links, q, qDot, tau, gravityVector) };

    ExpectNear(fromModel, fromAba);
}

TEST_F(TestChainDynamicsModel, set_links_updates_every_term)
{
    LinkArray heavier{ links };
    heavier[1].mass = 2.0f * m2;

    model.SetLinks(heavier);

    ExpectNear(model.ComputeGravityTerms(q), ClosedFormGravity(2.0f * m2, q));
}
