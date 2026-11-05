#include "numerical/math/Geometry3D.hpp"
#include "numerical/math/SE3Transform.hpp"
#include "numerical/math/Tolerance.hpp"
#include "robotics/kinematics/ChainPoseKinematics.hpp"
#include "robotics/kinematics/ChainTaskJacobian.hpp"
#include "robotics/kinematics/DenavitHartenberg.hpp"
#include "robotics/kinematics/GeometricJacobian.hpp"
#include <cmath>
#include <gtest/gtest.h>

namespace
{
    using Vector3 = math::Vector3<float>;
    using Matrix3 = math::Matrix3<float>;
    using Link = dynamics::RevoluteJointLink<float>;
    using Transform = math::SE3Transform<float>;
    using Planar = kinematics::GeometricJacobian<float, 2>;
    using Spatial = kinematics::GeometricJacobian<float, 3>;

    constexpr float epsilon{ 1e-3f };
    const Vector3 xAxis{ 1.0f, 0.0f, 0.0f };
    const Vector3 yAxis{ 0.0f, 1.0f, 0.0f };
    const Vector3 zAxis{ 0.0f, 0.0f, 1.0f };

    Link MakeLink(const Vector3& axis, const Vector3& parentToJoint, dynamics::JointType type = dynamics::JointType::Revolute)
    {
        return Link{ 1.0f, Matrix3::Identity(), axis, parentToJoint, Vector3{}, type };
    }

    Vector3 Normalized(float x, float y, float z)
    {
        const float norm{ std::sqrt(x * x + y * y + z * z) };
        return Vector3{ x / norm, y / norm, z / norm };
    }

    class TestGeometricJacobian
        : public ::testing::Test
    {
    protected:
        template<std::size_t N>
        math::Vector<float, N> Step(const math::Vector<float, N>& q, const math::Vector<float, N>& qDot, float scale) const
        {
            return q + qDot * scale;
        }

        void ExpectNear(const Vector3& actual, const Vector3& expected) const
        {
            for (std::size_t i = 0; i < 3; ++i)
                EXPECT_NEAR(actual[i], expected[i], math::Tolerance<float>());
        }

        std::array<Link, 2> planar{ MakeLink(zAxis, Vector3{}), MakeLink(zAxis, Vector3{ 1.0f, 0.0f, 0.0f }) };
        kinematics::ChainPoseKinematics<float, 2> kinematics{ Transform{ Matrix3::Identity(), Vector3{ 1.0f, 0.0f, 0.0f } } };
        std::array<Link, 3> spatial{ MakeLink(zAxis, Vector3{ 0.0f, 0.0f, 0.2f }), MakeLink(Normalized(1.0f, 0.3f, 0.2f), Vector3{ 0.0f, 0.1f, 0.4f }, dynamics::JointType::Prismatic),
            MakeLink(Normalized(0.2f, 1.0f, 0.4f), Vector3{ 0.3f, 0.0f, 0.1f }) };
        kinematics::ChainPoseKinematics<float, 3> spatialKinematics{ Transform{ math::RotationAboutAxis(xAxis, 0.4f), Vector3{ 0.25f, 0.05f, -0.1f } } };
        math::Vector<float, 3> spatialQ{ 0.5f, 0.15f, -0.8f };
        math::Vector<float, 3> spatialQDot{ 0.7f, -0.4f, 1.1f };
    };
}

TEST_F(TestGeometricJacobian, stretched_planar_arm_columns)
{
    const auto J{ Planar::Compute(kinematics.Compute(planar, math::Vector<float, 2>{})) };

    const math::Matrix<float, 6, 2> expected{ { 0.0f, 0.0f }, { 2.0f, 1.0f }, { 0.0f, 0.0f }, { 0.0f, 0.0f }, { 0.0f, 0.0f }, { 1.0f, 1.0f } };
    for (std::size_t r = 0; r < 6; ++r)
        for (std::size_t c = 0; c < 2; ++c)
            EXPECT_NEAR(J.at(r, c), expected.at(r, c), math::Tolerance<float>());
}

TEST_F(TestGeometricJacobian, linear_rows_match_finite_difference_of_tool_position)
{
    const math::Vector<float, 2> q{ 0.4f, -0.7f };
    const math::Vector<float, 2> qDot{ 0.3f, -0.5f };

    const auto velocity{ Planar::TaskTwist<3>(Planar::Twist{ Planar::Compute(kinematics.Compute(planar, q)) * qDot }) };

    const Vector3 ahead{ kinematics.ToolPose(planar, Step(q, qDot, epsilon)).translation };
    const Vector3 behind{ kinematics.ToolPose(planar, Step(q, qDot, -epsilon)).translation };
    ExpectNear(velocity, Vector3{ (ahead - behind) * (0.5f / epsilon) });
}

TEST_F(TestGeometricJacobian, angular_rows_match_finite_difference_of_tool_rotation)
{
    const auto twist{ Spatial::Twist{ Spatial::Compute(spatialKinematics.Compute(spatial, spatialQ)) * spatialQDot } };

    const Matrix3 ahead{ spatialKinematics.ToolPose(spatial, Step(spatialQ, spatialQDot, epsilon)).rotation };
    const Matrix3 behind{ spatialKinematics.ToolPose(spatial, Step(spatialQ, spatialQDot, -epsilon)).rotation };
    const Vector3 rotation{ Transform::LogRotation(Matrix3{ ahead * behind.Transpose() }) };
    ExpectNear(twist.GetBlock<3, 1>(3, 0), Vector3{ rotation * (0.5f / epsilon) });
}

TEST_F(TestGeometricJacobian, bias_acceleration_matches_finite_difference)
{
    const auto bias{ Spatial::BiasAcceleration(spatialKinematics.Compute(spatial, spatialQ), spatialQDot) };

    const auto ahead{ Spatial::Compute(spatialKinematics.Compute(spatial, Step(spatialQ, spatialQDot, epsilon))) };
    const auto behind{ Spatial::Compute(spatialKinematics.Compute(spatial, Step(spatialQ, spatialQDot, -epsilon))) };
    const Spatial::Twist expected{ (ahead - behind) * spatialQDot * (0.5f / epsilon) };
    for (std::size_t i = 0; i < 6; ++i)
        EXPECT_NEAR(bias[i], expected[i], math::Tolerance<float>());
}

TEST_F(TestGeometricJacobian, prismatic_column_is_pure_translation)
{
    kinematics::FrameChain<float, 1> frames{};
    frames.jointOrigins[0] = Vector3{ 0.3f, -0.2f, 0.5f };
    frames.jointAxes[0] = yAxis;
    frames.jointTypes[0] = dynamics::JointType::Prismatic;
    frames.tool = Transform{ Matrix3::Identity(), Vector3{ 1.0f, 2.0f, 3.0f } };

    const auto J{ kinematics::GeometricJacobian<float, 1>::Compute(frames) };

    ExpectNear(J.GetBlock<3, 1>(0, 0), yAxis);
    ExpectNear(J.GetBlock<3, 1>(3, 0), Vector3{});
}

TEST_F(TestGeometricJacobian, wrench_along_link_line_needs_no_torque)
{
    const auto J{ Planar::Compute(kinematics.Compute(planar, math::Vector<float, 2>{})) };
    Planar::Twist wrench{};
    wrench[0] = 1.0f;

    const auto tau{ Planar::JointTorques(J, wrench) };

    EXPECT_NEAR(tau[0], 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(tau[1], 0.0f, math::Tolerance<float>());
}

TEST_F(TestGeometricJacobian, link_model_and_dh_frames_give_the_same_jacobian)
{
    const kinematics::DenavitHartenberg<float, 2> dh{ { kinematics::DhLink<float>{ 1.0f, 0.0f, 0.0f, 0.0f }, kinematics::DhLink<float>{ 1.0f, 0.0f, 0.0f, 0.0f } }, kinematics::DhConvention::Standard };
    const math::Vector<float, 2> q{ 0.4f, -0.7f };

    const auto fromDh{ Planar::Compute(dh.Frames(q)) };
    const auto fromLinks{ Planar::Compute(kinematics.Compute(planar, q)) };

    for (std::size_t r = 0; r < 6; ++r)
        for (std::size_t c = 0; c < 2; ++c)
            EXPECT_NEAR(fromDh.at(r, c), fromLinks.at(r, c), math::Tolerance<float>());
}

TEST_F(TestGeometricJacobian, task_jacobian_position_rows_match_six_dof_rows)
{
    const Transform tool{ Matrix3::Identity(), Vector3{ 1.0f, 0.0f, 0.0f } };
    const kinematics::ChainTaskJacobian<float, 3, 2> position{ planar, tool };
    const kinematics::ChainTaskJacobian<float, 6, 2> pose{ planar, tool };
    const math::Vector<float, 2> q{ 0.4f, -0.7f };
    const math::Vector<float, 2> qDot{ 0.3f, -0.5f };

    const auto positionRows{ position.Jacobian(q) };
    const auto poseRows{ pose.Jacobian(q) };
    const auto positionBias{ position.BiasAcceleration(q, qDot) };
    const auto poseBias{ pose.BiasAcceleration(q, qDot) };

    for (std::size_t r = 0; r < 3; ++r)
    {
        for (std::size_t c = 0; c < 2; ++c)
            EXPECT_EQ(positionRows.at(r, c), poseRows.at(r, c));
        EXPECT_EQ(positionBias[r], poseBias[r]);
    }
    ExpectNear(position.ToolPose(q).translation, pose.ToolPose(q).translation);
}
