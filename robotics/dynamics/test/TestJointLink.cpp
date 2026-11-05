#include "numerical/math/Tolerance.hpp"
#include "robotics/dynamics/ArticulatedBodyAlgorithm.hpp"
#include "robotics/dynamics/CompositeRigidBodyAlgorithm.hpp"
#include "robotics/dynamics/JointLink.hpp"
#include "robotics/dynamics/RecursiveNewtonEuler.hpp"
#include <cmath>
#include <gtest/gtest.h>
#include <numbers>

namespace
{
    using Vector3 = math::Vector<float, 3>;
    using Matrix3 = math::SquareMatrix<float, 3>;
    using Link = dynamics::JointLink<float>;

    class TestJointLink
        : public ::testing::Test
    {
    protected:
        Link MakeRevolute(const Vector3& axis, const Vector3& parentToJoint) const
        {
            return Link{ 1.0f, Matrix3::Identity(), axis, parentToJoint, Vector3{} };
        }

        Link MakePrismatic(const Vector3& axis, const Vector3& parentToJoint) const
        {
            return Link{ 1.0f, Matrix3::Identity(), axis, parentToJoint, Vector3{}, dynamics::JointType::Prismatic };
        }

        void ExpectNear(const Vector3& actual, const Vector3& expected) const
        {
            for (std::size_t i = 0; i < 3; ++i)
                EXPECT_NEAR(actual[i], expected[i], math::Tolerance<float>());
        }

        Vector3 x{ 1.0f, 0.0f, 0.0f };
        Vector3 z{ 0.0f, 0.0f, 1.0f };
        Vector3 offset{ 0.1f, -0.2f, 0.3f };
    };
}

TEST_F(TestJointLink, existing_aggregate_initializers_default_to_revolute)
{
    const dynamics::RevoluteJointLink<float> link{ 2.0f, Matrix3::Identity(), z, offset, x };

    EXPECT_EQ(link.type, dynamics::JointType::Revolute);
    EXPECT_EQ(link.armature, 0.0f);
    EXPECT_TRUE(std::isinf(link.limits.positionMin) && link.limits.positionMin < 0.0f);
    EXPECT_TRUE(std::isinf(link.limits.positionMax) && link.limits.positionMax > 0.0f);
    EXPECT_TRUE(std::isinf(link.limits.velocityMax));
    EXPECT_TRUE(std::isinf(link.limits.effortMax));
}

TEST_F(TestJointLink, revolute_transform_rotates_about_axis)
{
    const auto transform{ MakeRevolute(z, offset).JointTransform(std::numbers::pi_v<float> / 2.0f) };

    ExpectNear(transform.rotation * x, Vector3{ 0.0f, 1.0f, 0.0f });
    ExpectNear(transform.translation, offset);
}

TEST_F(TestJointLink, prismatic_transform_slides_along_axis)
{
    const auto transform{ MakePrismatic(x, offset).JointTransform(0.3f) };

    for (std::size_t r = 0; r < 3; ++r)
        for (std::size_t c = 0; c < 3; ++c)
            EXPECT_NEAR(transform.rotation.at(r, c), r == c ? 1.0f : 0.0f, math::Tolerance<float>());
    ExpectNear(transform.translation, Vector3{ 0.4f, -0.2f, 0.3f });
}

TEST_F(TestJointLink, motion_subspace_selects_one_channel)
{
    const Link revolute{ MakeRevolute(z, offset) };
    const Link prismatic{ MakePrismatic(z, offset) };

    ExpectNear(revolute.AngularAxis(), z);
    ExpectNear(revolute.LinearAxis(), Vector3{});
    ExpectNear(prismatic.AngularAxis(), Vector3{});
    ExpectNear(prismatic.LinearAxis(), z);
}

TEST_F(TestJointLink, armature_adds_to_diagonal_inertia)
{
    const float armature{ 0.2f };
    const float rodInertia{ 1.0f / 3.0f };
    const float qDDot{ 1.5f };
    Link rod{ 1.0f, Matrix3{ { 0.0f, 0.0f, 0.0f }, { 0.0f, 1.0f / 12.0f, 0.0f }, { 0.0f, 0.0f, 1.0f / 12.0f } }, z, Vector3{}, Vector3{ 0.5f, 0.0f, 0.0f } };
    rod.armature = armature;
    const std::array<Link, 1> links{ rod };
    const math::Vector<float, 1> q{ 0.4f };

    const auto tau{ dynamics::RecursiveNewtonEuler<float, 1>{}.InverseDynamics(links, q, math::Vector<float, 1>{}, math::Vector<float, 1>{ qDDot }, Vector3{}) };
    const auto acceleration{ dynamics::ArticulatedBodyAlgorithm<float, 1>{}.ForwardDynamics(links, q, math::Vector<float, 1>{}, tau, Vector3{}) };
    const auto M{ dynamics::CompositeRigidBodyAlgorithm<float, 1>{}.Compute(links, q) };

    EXPECT_NEAR(tau[0], (rodInertia + armature) * qDDot, math::Tolerance<float>());
    EXPECT_NEAR(acceleration[0], qDDot, math::Tolerance<float>());
    EXPECT_NEAR(M.at(0, 0), rodInertia + armature, math::Tolerance<float>());
}
