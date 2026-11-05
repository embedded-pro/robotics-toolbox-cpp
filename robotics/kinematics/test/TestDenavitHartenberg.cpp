#include "numerical/math/Geometry3D.hpp"
#include "numerical/math/SE3Transform.hpp"
#include "numerical/math/Tolerance.hpp"
#include "robotics/kinematics/DenavitHartenberg.hpp"
#include "robotics/kinematics/DhTaskJacobian.hpp"
#include <cmath>
#include <gtest/gtest.h>
#include <numbers>

namespace
{
    using Vector3 = math::Vector3<float>;
    using Matrix3 = math::Matrix3<float>;
    using Transform = math::SE3Transform<float>;
    using DhLink = kinematics::DhLink<float>;
    using kinematics::DhConvention;

    constexpr float pi{ std::numbers::pi_v<float> };
    constexpr dynamics::JointType revolute{ dynamics::JointType::Revolute };
    constexpr dynamics::JointType prismatic{ dynamics::JointType::Prismatic };
    const Vector3 xAxis{ 1.0f, 0.0f, 0.0f };
    const Vector3 zAxis{ 0.0f, 0.0f, 1.0f };

    class TestDenavitHartenberg
        : public ::testing::Test
    {
    protected:
        void ExpectNear(const Vector3& actual, const Vector3& expected, float tolerance = math::Tolerance<float>()) const
        {
            for (std::size_t i = 0; i < 3; ++i)
                EXPECT_NEAR(actual[i], expected[i], tolerance);
        }

        void ExpectNear(const Matrix3& actual, const Matrix3& expected) const
        {
            for (std::size_t r = 0; r < 3; ++r)
                for (std::size_t c = 0; c < 3; ++c)
                    EXPECT_NEAR(actual.at(r, c), expected.at(r, c), math::Tolerance<float>());
        }

        kinematics::DenavitHartenberg<float, 2> standard{ { DhLink{ 1.0f, 0.0f, 0.0f, 0.0f, revolute }, DhLink{ 1.0f, 0.0f, 0.0f, 0.0f, revolute } }, DhConvention::Standard };
        kinematics::DenavitHartenberg<float, 2> modified{ { DhLink{ 0.0f, 0.0f, 0.0f, 0.0f, revolute }, DhLink{ 1.0f, 0.0f, 0.0f, 0.0f, revolute } }, DhConvention::Modified,
            Transform{ Matrix3::Identity(), xAxis } };
    };
}

TEST_F(TestDenavitHartenberg, zero_config_is_stretched_along_x)
{
    const auto pose{ standard.Forward(math::Vector<float, 2>{}) };

    ExpectNear(pose.translation, Vector3{ 2.0f, 0.0f, 0.0f });
    ExpectNear(pose.rotation, Matrix3::Identity());
}

TEST_F(TestDenavitHartenberg, planar_elbow_ninety_deg)
{
    ExpectNear(standard.Forward(math::Vector<float, 2>{ 0.0f, pi / 2.0f }).translation, Vector3{ 1.0f, 1.0f, 0.0f });
}

TEST_F(TestDenavitHartenberg, single_link_revolute_rotates)
{
    const kinematics::DenavitHartenberg<float, 1> single{ { DhLink{ 1.0f, 0.0f, 0.0f, 0.0f, revolute } }, DhConvention::Standard };

    ExpectNear(single.Forward(math::Vector<float, 1>{ pi / 2.0f }).translation, Vector3{ 0.0f, 1.0f, 0.0f });
}

TEST_F(TestDenavitHartenberg, theta_offset_is_added_to_the_joint_variable)
{
    const kinematics::DenavitHartenberg<float, 2> offset{ { DhLink{ 1.0f, 0.0f, 0.0f, pi / 2.0f, revolute }, DhLink{ 1.0f, 0.0f, 0.0f, 0.0f, revolute } }, DhConvention::Standard };

    ExpectNear(offset.Forward(math::Vector<float, 2>{}).translation, Vector3{ 0.0f, 2.0f, 0.0f });
    ExpectNear(offset.Forward(math::Vector<float, 2>{ 0.0f, pi / 2.0f }).translation, Vector3{ -1.0f, 1.0f, 0.0f });
}

TEST_F(TestDenavitHartenberg, prismatic_variable_adds_to_d)
{
    const kinematics::DenavitHartenberg<float, 1> slider{ { DhLink{ 0.0f, 0.0f, 0.2f, 0.0f, prismatic } }, DhConvention::Standard };

    ExpectNear(slider.Forward(math::Vector<float, 1>{ 0.5f }).translation, Vector3{ 0.0f, 0.0f, 0.7f });
}

TEST_F(TestDenavitHartenberg, link_twist_maps_z_to_minus_y)
{
    const DhLink twisted{ 0.0f, pi / 2.0f, 0.0f, 0.0f, revolute };
    const kinematics::DenavitHartenberg<float, 1> standardTwist{ { twisted }, DhConvention::Standard };
    const kinematics::DenavitHartenberg<float, 1> modifiedTwist{ { twisted }, DhConvention::Modified };

    ExpectNear(standardTwist.LinkTransform(0, 0.0f).rotation * zAxis, Vector3{ 0.0f, -1.0f, 0.0f });
    ExpectNear(modifiedTwist.LinkTransform(0, 0.0f).rotation * zAxis, Vector3{ 0.0f, -1.0f, 0.0f });
}

TEST_F(TestDenavitHartenberg, modified_link_transform_matches_elementary_product)
{
    const kinematics::DenavitHartenberg<float, 1> link{ { DhLink{ 0.3f, 0.7f, 0.2f, 0.0f, revolute } }, DhConvention::Modified };

    const auto transform{ link.LinkTransform(0, 1.1f) };

    ExpectNear(transform.translation, Vector3{ 0.3f, -0.128844f, 0.152968f });
    ExpectNear(transform.rotation, Matrix3{ math::RotationAboutAxis(xAxis, 0.7f) * math::RotationAboutAxis(zAxis, 1.1f) });
}

TEST_F(TestDenavitHartenberg, standard_and_modified_give_the_same_pose)
{
    for (const auto& q : { math::Vector<float, 2>{}, math::Vector<float, 2>{ 0.0f, pi / 2.0f }, math::Vector<float, 2>{ 0.4f, -0.7f }, math::Vector<float, 2>{ 1.3f, 2.1f } })
    {
        const auto fromStandard{ standard.Forward(q) };
        const auto fromModified{ modified.Forward(q) };
        ExpectNear(fromStandard.translation, fromModified.translation);
        ExpectNear(fromStandard.rotation, fromModified.rotation);
    }
    ExpectNear(standard.Forward(math::Vector<float, 2>{ 0.4f, -0.7f }).translation, Vector3{ 1.876397f, 0.093898f, 0.0f });
    ExpectNear(standard.Forward(math::Vector<float, 2>{ 1.3f, 2.1f }).translation, Vector3{ -0.699299f, 0.708017f, 0.0f });
}

TEST_F(TestDenavitHartenberg, standard_and_modified_give_the_same_frame_chain)
{
    const math::Vector<float, 2> q{ 0.4f, -0.7f };

    const auto fromStandard{ standard.Frames(q) };
    const auto fromModified{ modified.Frames(q) };

    for (std::size_t i = 0; i < 2; ++i)
    {
        ExpectNear(fromStandard.jointOrigins[i], fromModified.jointOrigins[i]);
        ExpectNear(fromStandard.jointAxes[i], zAxis);
        ExpectNear(fromModified.jointAxes[i], zAxis);
    }
    ExpectNear(fromStandard.jointOrigins[0], Vector3{});
    ExpectNear(fromStandard.jointOrigins[1], Vector3{ std::cos(q[0]), std::sin(q[0]), 0.0f });
    ExpectNear(fromStandard.tool.translation, fromModified.tool.translation);
}

TEST_F(TestDenavitHartenberg, frame_chain_jacobian_matches_finite_difference)
{
    const kinematics::DenavitHartenberg<float, 6> puma{ { DhLink{ 0.0f, pi / 2.0f, 0.0f, 0.0f, revolute }, DhLink{ 0.4318f, 0.0f, 0.0f, 0.0f, revolute },
                                                            DhLink{ 0.0203f, -pi / 2.0f, 0.15005f, 0.0f, revolute }, DhLink{ 0.0f, pi / 2.0f, 0.4318f, 0.0f, revolute },
                                                            DhLink{ 0.0f, -pi / 2.0f, 0.0f, 0.0f, revolute }, DhLink{ 0.0f, 0.0f, 0.0f, 0.0f, revolute } },
        DhConvention::Standard };
    const kinematics::DhTaskJacobian<float, 6, 6> provider{ puma };
    const math::Vector<float, 6> q{ 0.3f, -0.4f, 0.5f, 0.6f, -0.7f, 0.8f };
    const math::Vector<float, 6> qDot{ 0.3f, -0.2f, 0.5f, 0.1f, -0.4f, 0.7f };
    constexpr float epsilon{ 1e-3f };

    const auto twist{ math::Vector6<float>{ provider.Jacobian(q) * qDot } };

    const auto now{ provider.ToolPose(q) };
    const auto ahead{ puma.Forward(q + qDot * epsilon) };
    ExpectNear(now.translation, Vector3{ 0.402407f, -0.032586f, 0.263519f });
    ExpectNear(twist.GetBlock<3, 1>(0, 0), Vector3{ (ahead.translation - now.translation) * (1.0f / epsilon) });
    ExpectNear(twist.GetBlock<3, 1>(3, 0), Vector3{ Transform::LogRotation(Matrix3{ ahead.rotation * now.rotation.Transpose() }) * (1.0f / epsilon) });
}
