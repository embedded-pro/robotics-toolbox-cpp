#include "robotics/dynamics/EulerLagrangeDynamics.hpp"
#include "robotics/dynamics/EulerLagrangeSolver.hpp"
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

    class UniformRodTwoLinkModel
        : public dynamics::EulerLagrangeDynamics<float, 2>
    {
    public:
        UniformRodTwoLinkModel(float m1, float l1, float m2, float l2)
            : m1{ m1 }
            , l1{ l1 }
            , m2{ m2 }
            , l2{ l2 }
        {}

        MassMatrix ComputeMassMatrix(const StateVector& q) const override
        {
            const float c2{ std::cos(q.at(1, 0)) };
            const float m12{ m2 * (l2 * l2 / 3.0f + l1 * l2 * c2 / 2.0f) };

            return MassMatrix{
                { m1 * l1 * l1 / 3.0f + m2 * (l1 * l1 + l2 * l2 / 3.0f + l1 * l2 * c2), m12 },
                { m12, m2 * l2 * l2 / 3.0f }
            };
        }

        StateVector ComputeCoriolisTerms(const StateVector& q, const StateVector& qDot) const override
        {
            const float h{ m2 * l1 * l2 / 2.0f * std::sin(q.at(1, 0)) };
            const float qd1{ qDot.at(0, 0) };
            const float qd2{ qDot.at(1, 0) };

            return StateVector{ -h * (2.0f * qd1 * qd2 + qd2 * qd2), h * qd1 * qd1 };
        }

        StateVector ComputeGravityTerms(const StateVector& q) const override
        {
            const float c1{ std::cos(q.at(0, 0)) };
            const float c12{ std::cos(q.at(0, 0) + q.at(1, 0)) };

            return StateVector{
                -gravity * (m1 * l1 / 2.0f * c1 + m2 * (l1 * c1 + l2 / 2.0f * c12)),
                -gravity * m2 * l2 / 2.0f * c12
            };
        }

    private:
        float m1;
        float l1;
        float m2;
        float l2;
    };

    class TestRecursiveNewtonEuler
        : public ::testing::Test
    {
    protected:
        dynamics::RecursiveNewtonEuler<float, 1> rnea1;
        dynamics::RecursiveNewtonEuler<float, 2> rnea2;
    };
}

TEST_F(TestRecursiveNewtonEuler, horizontal_link_needs_half_weight_moment_to_hold)
{
    const float mass{ 1.0f };
    const float length{ 1.0f };
    std::array<Link, 1> links{ MakeRod(mass, length, yAxis, Vector3{}) };

    auto tau = rnea1.InverseDynamics(links, math::Vector<float, 1>{ 0.0f }, math::Vector<float, 1>{}, math::Vector<float, 1>{}, gravityVector);

    EXPECT_NEAR(tau.at(0, 0), -mass * gravity * length / 2.0f, math::Tolerance<float>());
}

TEST_F(TestRecursiveNewtonEuler, rest_without_gravity_needs_no_torque)
{
    std::array<Link, 1> links{ MakeRod(1.0f, 1.0f, zAxis, Vector3{}) };

    auto tau = rnea1.InverseDynamics(links, math::Vector<float, 1>{ 0.5f }, math::Vector<float, 1>{}, math::Vector<float, 1>{}, Vector3{});

    EXPECT_NEAR(tau.at(0, 0), 0.0f, math::Tolerance<float>());
}

TEST_F(TestRecursiveNewtonEuler, angular_acceleration_needs_end_inertia_torque)
{
    const float mass{ 2.0f };
    const float length{ 1.0f };
    std::array<Link, 1> links{ MakeRod(mass, length, zAxis, Vector3{}) };

    auto tau = rnea1.InverseDynamics(links, math::Vector<float, 1>{ 0.0f }, math::Vector<float, 1>{}, math::Vector<float, 1>{ 1.0f }, Vector3{});

    EXPECT_NEAR(tau.at(0, 0), mass * length * length / 3.0f, math::Tolerance<float>());
}

TEST_F(TestRecursiveNewtonEuler, constant_spin_needs_no_torque)
{
    std::array<Link, 1> links{ MakeRod(1.0f, 1.0f, zAxis, Vector3{}) };

    auto tau = rnea1.InverseDynamics(links, math::Vector<float, 1>{ 0.0f }, math::Vector<float, 1>{ 2.0f }, math::Vector<float, 1>{}, Vector3{});

    EXPECT_NEAR(tau.at(0, 0), 0.0f, math::Tolerance<float>());
}

TEST_F(TestRecursiveNewtonEuler, elbow_acceleration_couples_into_shoulder_through_mass_matrix)
{
    const float m2{ 1.0f };
    const float l1{ 1.0f };
    const float l2{ 1.0f };
    std::array<Link, 2> links{ MakeRod(1.0f, l1, zAxis, Vector3{}), MakeRod(m2, l2, zAxis, Vector3{ l1, 0.0f, 0.0f }) };

    auto tau = rnea2.InverseDynamics(links, math::Vector<float, 2>{ 0.0f, 0.0f }, math::Vector<float, 2>{}, math::Vector<float, 2>{ 0.0f, 1.0f }, Vector3{});

    EXPECT_NEAR(tau.at(0, 0), m2 * (l2 * l2 / 3.0f + l1 * l2 / 2.0f), math::Tolerance<float>());
    EXPECT_NEAR(tau.at(1, 0), m2 * l2 * l2 / 3.0f, math::Tolerance<float>());
}

TEST_F(TestRecursiveNewtonEuler, matches_euler_lagrange_model_under_gravity_and_motion)
{
    const float m1{ 1.2f };
    const float l1{ 0.6f };
    const float m2{ 0.7f };
    const float l2{ 0.45f };
    std::array<Link, 2> links{ MakeRod(m1, l1, yAxis, Vector3{}), MakeRod(m2, l2, yAxis, Vector3{ l1, 0.0f, 0.0f }) };
    UniformRodTwoLinkModel model{ m1, l1, m2, l2 };
    dynamics::EulerLagrangeSolver<float, 2> eulerLagrange;
    math::Vector<float, 2> q{ 0.4f, -0.9f };
    math::Vector<float, 2> qDot{ 1.1f, -0.6f };
    math::Vector<float, 2> qDDot{ 0.8f, -1.7f };

    auto tau = rnea2.InverseDynamics(links, q, qDot, qDDot, gravityVector);
    auto reference = eulerLagrange.InverseDynamics(model, q, qDot, qDDot);

    EXPECT_NEAR(tau.at(0, 0), reference.at(0, 0), math::Tolerance<float>());
    EXPECT_NEAR(tau.at(1, 0), reference.at(1, 0), math::Tolerance<float>());
}
