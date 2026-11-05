#include "numerical/math/Tolerance.hpp"
#include "robotics/dynamics/EulerLagrangeDynamics.hpp"
#include "robotics/dynamics/EulerLagrangeSolver.hpp"
#include "robotics/dynamics/RecursiveNewtonEuler.hpp"
#include "robotics/kinematics/ChainPoseKinematics.hpp"
#include "robotics/kinematics/GeometricJacobian.hpp"
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
        dynamics::RecursiveNewtonEuler<float, 3> rnea3;
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

TEST_F(TestRecursiveNewtonEuler, vertical_slider_needs_weight_plus_inertial_force)
{
    const float mass{ 2.0f };
    const float acceleration{ 1.0f };
    std::array<Link, 1> links{ Link{ mass, math::SquareMatrix<float, 3>{}, zAxis, Vector3{}, Vector3{}, dynamics::JointType::Prismatic } };

    auto tau = rnea1.InverseDynamics(links, math::Vector<float, 1>{ 0.3f }, math::Vector<float, 1>{ 0.5f }, math::Vector<float, 1>{ acceleration }, gravityVector);

    EXPECT_NEAR(tau.at(0, 0), mass * (gravity + acceleration), math::Tolerance<float>());
}

TEST_F(TestRecursiveNewtonEuler, prismatic_coriolis_term_on_rotating_base)
{
    const float mass{ 2.0f };
    const float radius{ 0.5f };
    const float extensionRate{ 0.3f };
    const float spinRate{ 1.5f };
    const float spinAcceleration{ 0.4f };
    const float extensionAcceleration{ 0.2f };
    std::array<Link, 2> links{
        Link{ 0.0f, math::SquareMatrix<float, 3>{}, zAxis, Vector3{}, Vector3{} },
        Link{ mass, math::SquareMatrix<float, 3>{}, Vector3{ 1.0f, 0.0f, 0.0f }, Vector3{}, Vector3{}, dynamics::JointType::Prismatic }
    };

    auto tau = rnea2.InverseDynamics(links, math::Vector<float, 2>{ 0.7f, radius }, math::Vector<float, 2>{ spinRate, extensionRate },
        math::Vector<float, 2>{ spinAcceleration, extensionAcceleration }, Vector3{});

    EXPECT_NEAR(tau.at(0, 0), 2.0f * mass * radius * extensionRate * spinRate + mass * radius * radius * spinAcceleration, math::Tolerance<float>());
    EXPECT_NEAR(tau.at(1, 0), mass * (extensionAcceleration - radius * spinRate * spinRate), math::Tolerance<float>());
}

namespace
{
    const Vector3 toolOffset{ 0.3f, 0.05f, 0.0f };

    Vector3 Normalized(float x, float y, float z)
    {
        const float norm{ std::sqrt(x * x + y * y + z * z) };
        return Vector3{ x / norm, y / norm, z / norm };
    }

    std::array<Link, 3> SkewedChain()
    {
        const math::SquareMatrix<float, 3> inertia{ { 0.05f, 0.01f, -0.005f }, { 0.01f, 0.04f, 0.008f }, { -0.005f, 0.008f, 0.03f } };
        return std::array<Link, 3>{
            Link{ 1.4f, inertia, Normalized(0.0f, 0.2f, 1.0f), Vector3{ 0.0f, 0.0f, 0.1f }, Vector3{ 0.02f, -0.01f, 0.15f } },
            Link{ 0.9f, inertia * 0.8f, Normalized(0.1f, 1.0f, 0.0f), Vector3{ 0.05f, 0.0f, 0.3f }, Vector3{ 0.2f, 0.03f, -0.02f } },
            Link{ 0.5f, inertia * 0.5f, Normalized(1.0f, 0.3f, -0.4f), Vector3{ 0.4f, -0.05f, 0.02f }, Vector3{ 0.1f, 0.02f, 0.03f } }
        };
    }
}

TEST_F(TestRecursiveNewtonEuler, zero_wrench_matches_shipped_overload)
{
    const auto chain{ SkewedChain() };
    const math::Vector<float, 3> q{ 0.4f, -0.7f, 1.1f };
    const math::Vector<float, 3> qDot{ 0.9f, -1.3f, 0.6f };
    const math::Vector<float, 3> qDDot{ 1.5f, -2.0f, 0.7f };

    const auto withWrench{ rnea3.InverseDynamics(chain, q, qDot, qDDot, gravityVector, dynamics::ToolWrench<float>{ Vector3{}, Vector3{}, toolOffset }) };
    const auto shipped{ rnea3.InverseDynamics(chain, q, qDot, qDDot, gravityVector) };

    for (std::size_t i = 0; i < 3; ++i)
        EXPECT_NEAR(withWrench.at(i, 0), shipped.at(i, 0), math::Tolerance<float>());
}

TEST_F(TestRecursiveNewtonEuler, wrench_enters_as_minus_jacobian_transpose)
{
    const auto chain{ SkewedChain() };
    const math::Vector<float, 3> q{ 0.4f, -0.7f, 1.1f };
    const math::Vector<float, 3> qDot{ 0.9f, -1.3f, 0.6f };
    const math::Vector<float, 3> qDDot{ 1.5f, -2.0f, 0.7f };
    const dynamics::ToolWrench<float> wrench{ Vector3{ 1.0f, -2.0f, 0.5f }, Vector3{ 0.1f, 0.0f, -0.2f }, toolOffset };
    const kinematics::ChainPoseKinematics<float, 3> kinematics{ math::SE3Transform<float>{ math::SquareMatrix<float, 3>::Identity(), toolOffset } };
    math::Vector6<float> stacked{};
    stacked.SetBlock(wrench.force, 0, 0);
    stacked.SetBlock(wrench.moment, 3, 0);

    const auto loaded{ rnea3.InverseDynamics(chain, q, qDot, qDDot, gravityVector, wrench) };
    const auto free{ rnea3.InverseDynamics(chain, q, qDot, qDDot, gravityVector) };

    using Jacobian = kinematics::GeometricJacobian<float, 3>;
    const auto expected{ Jacobian::JointTorques(Jacobian::Compute(kinematics.Compute(chain, q)), stacked) };
    for (std::size_t i = 0; i < 3; ++i)
        EXPECT_NEAR(loaded.at(i, 0) - free.at(i, 0), -expected.at(i, 0), math::Tolerance<float>());
}

TEST_F(TestRecursiveNewtonEuler, horizontal_link_holding_a_payload)
{
    const float mass{ 1.0f };
    const float length{ 1.0f };
    const float payload{ 0.5f };
    std::array<Link, 1> links{ MakeRod(mass, length, yAxis, Vector3{}) };
    const dynamics::ToolWrench<float> wrench{ Vector3{ 0.0f, 0.0f, -payload * gravity }, Vector3{}, Vector3{ length, 0.0f, 0.0f } };

    auto tau = rnea1.InverseDynamics(links, math::Vector<float, 1>{ 0.0f }, math::Vector<float, 1>{}, math::Vector<float, 1>{}, gravityVector, wrench);

    EXPECT_NEAR(tau.at(0, 0), -(mass * length / 2.0f + payload * length) * gravity, math::Tolerance<float>());
}

TEST_F(TestRecursiveNewtonEuler, pure_moment_projects_on_the_axis)
{
    std::array<Link, 1> links{ MakeRod(1.0f, 1.0f, zAxis, Vector3{}) };
    const dynamics::ToolWrench<float> wrench{ Vector3{}, Vector3{ 0.0f, 0.0f, 2.0f }, Vector3{ 1.0f, 0.0f, 0.0f } };

    auto tau = rnea1.InverseDynamics(links, math::Vector<float, 1>{ 0.6f }, math::Vector<float, 1>{}, math::Vector<float, 1>{}, gravityVector, wrench);

    EXPECT_NEAR(tau.at(0, 0), -2.0f, math::Tolerance<float>());
}
