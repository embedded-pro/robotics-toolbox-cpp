#include "numerical/math/Geometry3D.hpp"
#include "numerical/math/Tolerance.hpp"
#include "robotics/kinematics/ChainPoseKinematics.hpp"
#include "robotics/kinematics/ForwardKinematics.hpp"
#include <gtest/gtest.h>
#include <numbers>

namespace
{
    using Vector3 = math::Vector3<float>;
    using Matrix3 = math::Matrix3<float>;
    using Link = dynamics::RevoluteJointLink<float>;
    using Transform = math::SE3Transform<float>;

    constexpr float halfPi{ std::numbers::pi_v<float> / 2.0f };
    const Vector3 xAxis{ 1.0f, 0.0f, 0.0f };
    const Vector3 yAxis{ 0.0f, 1.0f, 0.0f };
    const Vector3 zAxis{ 0.0f, 0.0f, 1.0f };

    Link MakeLink(const Vector3& axis, const Vector3& parentToJoint)
    {
        return Link{ 1.0f, Matrix3::Identity(), axis, parentToJoint, Vector3{} };
    }

    class TestChainPoseKinematics
        : public ::testing::Test
    {
    protected:
        void ExpectNear(const Vector3& actual, const Vector3& expected) const
        {
            for (std::size_t i = 0; i < 3; ++i)
                EXPECT_NEAR(actual[i], expected[i], math::Tolerance<float>());
        }

        void ExpectNear(const Matrix3& actual, const Matrix3& expected) const
        {
            for (std::size_t r = 0; r < 3; ++r)
                for (std::size_t c = 0; c < 3; ++c)
                    EXPECT_NEAR(actual.at(r, c), expected.at(r, c), math::Tolerance<float>());
        }

        std::array<Link, 3> arm{ MakeLink(zAxis, Vector3{ 0.0f, 0.0f, 0.2f }), MakeLink(yAxis, Vector3{ 0.0f, 0.0f, 0.5f }), MakeLink(yAxis, Vector3{ 0.4f, 0.0f, 0.0f }) };
        kinematics::ChainPoseKinematics<float, 3> kinematics{ Transform{ Matrix3::Identity(), Vector3{ 0.3f, 0.0f, 0.0f } } };
    };
}

TEST_F(TestChainPoseKinematics, origins_and_tool_match_shipped_forward_kinematics)
{
    const math::Vector<float, 3> q{ 0.3f, -0.7f, 1.1f };
    const kinematics::ForwardKinematics<float, 3> fk{ Vector3{ 0.3f, 0.0f, 0.0f } };

    const auto frames{ kinematics.Compute(arm, q) };
    const auto positions{ fk.Compute(arm, q) };

    for (std::size_t i = 0; i < 3; ++i)
        ExpectNear(frames.jointOrigins[i], positions[i]);
    ExpectNear(frames.tool.translation, positions[3]);
}

TEST_F(TestChainPoseKinematics, axes_are_expressed_in_the_base_frame)
{
    const auto frames{ kinematics.Compute(arm, math::Vector<float, 3>{ halfPi, 0.0f, 0.0f }) };

    ExpectNear(frames.jointAxes[1], Vector3{ -1.0f, 0.0f, 0.0f });
}

TEST_F(TestChainPoseKinematics, tool_orientation_composes_joint_rotations)
{
    const auto frames{ kinematics.Compute(arm, math::Vector<float, 3>{ halfPi, halfPi, 0.0f }) };

    ExpectNear(frames.tool.rotation, Matrix3{ math::RotationAboutAxis(zAxis, halfPi) * math::RotationAboutAxis(yAxis, halfPi) });
    ExpectNear(frames.tool.rotation * xAxis, Vector3{ 0.0f, 0.0f, -1.0f });
}

TEST_F(TestChainPoseKinematics, tool_frame_rotation_is_applied_last)
{
    const kinematics::ChainPoseKinematics<float, 3> rotatedTool{ Transform{ math::RotationAboutAxis(xAxis, halfPi), Vector3{} } };
    const math::Vector<float, 3> q{ 0.4f, -0.3f, 0.9f };

    const auto tool{ rotatedTool.ToolPose(arm, q) };

    const Matrix3 expected{ math::RotationAboutAxis(zAxis, q[0]) * math::RotationAboutAxis(yAxis, q[1]) * math::RotationAboutAxis(yAxis, q[2]) * math::RotationAboutAxis(xAxis, halfPi) };
    ExpectNear(tool.rotation, expected);
}

TEST_F(TestChainPoseKinematics, base_offset_moves_every_origin)
{
    for (const auto& q : { math::Vector<float, 3>{}, math::Vector<float, 3>{ 1.2f, -0.4f, 2.0f }, math::Vector<float, 3>{ -2.5f, 0.8f, -1.1f } })
        ExpectNear(kinematics.Compute(arm, q).jointOrigins[0], arm[0].parentToJoint);
}
