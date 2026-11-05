#include "numerical/math/Geometry3D.hpp"
#include "numerical/math/SE2Transform.hpp"
#include "numerical/math/SE3Transform.hpp"
#include "robotics/kinematics/ChainTaskJacobian.hpp"
#include "robotics/kinematics/MobileManipulatorKinematics.hpp"
#include <array>
#include <cmath>
#include <cstddef>
#include <gtest/gtest.h>
#include <numbers>

namespace
{
    using Vector3 = math::Vector3<float>;
    using Matrix3 = math::Matrix3<float>;
    using Link = dynamics::RevoluteJointLink<float>;
    using Pose = math::SE3Transform<float>;
    using Planar = math::SE2Transform<float>;
    using Kinematics = kinematics::MobileManipulatorKinematics<float, 3>;
    using Command = math::Vector<float, 5>;

    constexpr float pi{ std::numbers::pi_v<float> };
    const Vector3 yAxis{ 0.0f, 1.0f, 0.0f };
    const Vector3 zAxis{ 0.0f, 0.0f, 1.0f };

    Link MakeLink(const Vector3& axis, const Vector3& parentToJoint)
    {
        return Link{ 1.0f, Matrix3::Identity(), axis, parentToJoint, Vector3{} };
    }

    class TestMobileManipulatorKinematics
        : public ::testing::Test
    {
    protected:
        static void ExpectNear(const Vector3& actual, const Vector3& expected, float tolerance)
        {
            for (std::size_t i = 0; i < 3; ++i)
                EXPECT_NEAR(actual[i], expected[i], tolerance);
        }

        static Vector3 Linear(const math::Vector<float, 6>& twist)
        {
            return Vector3{ twist[0], twist[1], twist[2] };
        }

        static Vector3 Angular(const math::Vector<float, 6>& twist)
        {
            return Vector3{ twist[3], twist[4], twist[5] };
        }

        std::array<Link, 3> links{ MakeLink(zAxis, Vector3{ 0.0f, 0.0f, 0.1f }), MakeLink(yAxis, Vector3{ 0.0f, 0.0f, 0.2f }), MakeLink(yAxis, Vector3{ 0.3f, 0.0f, 0.0f }) };
        kinematics::ChainTaskJacobian<float, 6, 3> arm{ links, Pose{ Matrix3::Identity(), Vector3{ 0.25f, 0.0f, 0.0f } } };
        Pose mount{ math::RotationAboutAxis(zAxis, pi / 2.0f), Vector3{ 0.1f, 0.0f, 0.3f } };
        Kinematics mobile{ arm, mount };
        Planar base{ Planar::FromPose(1.0f, 2.0f, pi / 6.0f) };
        Planar alongX{ Planar::FromPose(1.0f, 2.0f, 0.0f) };
        Kinematics::JointVector q{ 0.4f, -0.6f, 0.9f };
        Command u{ 0.3f, -0.5f, 0.2f, -0.1f, 0.4f };
    };
}

TEST_F(TestMobileManipulatorKinematics, tool_pose_composes_base_mount_and_arm)
{
    const auto tool{ mobile.ToolPose(base, q) };
    const auto composed{ base.ToSE3() * mount * arm.ToolPose(q) };

    ExpectNear(tool.translation, Vector3{ 0.698536f, 2.343297f, 0.695513f }, 1e-5f);
    for (std::size_t r = 0; r < 3; ++r)
        for (std::size_t c = 0; c < 3; ++c)
            EXPECT_NEAR(tool.rotation.at(r, c), composed.rotation.at(r, c), 1e-6f);
}

TEST_F(TestMobileManipulatorKinematics, twist_matches_finite_difference)
{
    const float epsilon{ 1e-3f };
    const auto heading{ base.Heading() };
    const auto movedBase{ Planar::FromPose(base.translation[0] + epsilon * u[0] * std::cos(heading), base.translation[1] + epsilon * u[0] * std::sin(heading), heading + epsilon * u[1]) };
    const Kinematics::JointVector movedQ{ q[0] + epsilon * u[2], q[1] + epsilon * u[3], q[2] + epsilon * u[4] };
    const auto before{ mobile.ToolPose(base, q) };
    const auto after{ mobile.ToolPose(movedBase, movedQ) };

    const auto twist{ mobile.Compute(base, q) * u };

    ExpectNear(Linear(twist), Vector3{ after.translation - before.translation } * (1.0f / epsilon), 1e-3f);
    ExpectNear(Angular(twist), Pose::LogRotation(after.rotation * before.rotation.Transpose()) * (1.0f / epsilon), 1e-3f);
    ExpectNear(Linear(twist), Vector3{ 0.403993f, 0.199541f, -0.046890f }, 1e-5f);
    ExpectNear(Angular(twist), Vector3{ -0.180886f, -0.239333f, -0.3f }, 1e-5f);
}

TEST_F(TestMobileManipulatorKinematics, arm_block_is_rotated_into_the_world)
{
    const auto jacobian{ mobile.Compute(base, q) };
    const Matrix3 armRotation{ math::RotationAboutAxis(zAxis, pi / 6.0f) * math::RotationAboutAxis(zAxis, pi / 2.0f) };
    const auto armJacobian{ arm.Jacobian(q) };
    const auto linear{ armRotation * armJacobian.GetBlock<3, 3>(0, 0) };
    const auto angular{ armRotation * armJacobian.GetBlock<3, 3>(3, 0) };

    for (std::size_t r = 0; r < 3; ++r)
        for (std::size_t c = 0; c < 3; ++c)
        {
            EXPECT_NEAR(jacobian.at(r, c + 2), linear.at(r, c), 1e-5f);
            EXPECT_NEAR(jacobian.at(r + 3, c + 2), angular.at(r, c), 1e-5f);
        }
}

TEST_F(TestMobileManipulatorKinematics, forward_drive_moves_tool_along_heading)
{
    const auto twist{ mobile.Compute(alongX, q) * Command{ 1.0f, 0.0f, 0.0f, 0.0f, 0.0f } };

    ExpectNear(Linear(twist), Vector3{ 1.0f, 0.0f, 0.0f }, 1e-6f);
    ExpectNear(Angular(twist), Vector3{}, 1e-6f);
}

TEST_F(TestMobileManipulatorKinematics, yaw_moves_tool_by_lever_arm)
{
    const auto tool{ mobile.ToolPose(alongX, q).translation };
    const Vector3 lever{ tool[0] - alongX.translation[0], tool[1] - alongX.translation[1], tool[2] };

    const auto twist{ mobile.Compute(alongX, q) * Command{ 0.0f, 1.0f, 0.0f, 0.0f, 0.0f } };

    ExpectNear(Linear(twist), math::CrossProduct(zAxis, lever), 1e-6f);
    EXPECT_NEAR(std::hypot(twist[0], twist[1], twist[2]), 0.456874f, 1e-5f);
    ExpectNear(Angular(twist), zAxis, 1e-6f);
}

TEST_F(TestMobileManipulatorKinematics, base_constraint_blocks_sideslip)
{
    const math::Vector<float, 2> command{ 0.7f, -0.3f };

    for (const float phi : { 0.0f, pi / 6.0f, pi })
    {
        const auto planar{ Kinematics::BaseConstraint(phi) * command };
        EXPECT_NEAR(-std::sin(phi) * planar[0] + std::cos(phi) * planar[1], 0.0f, 1e-6f);
    }
}
