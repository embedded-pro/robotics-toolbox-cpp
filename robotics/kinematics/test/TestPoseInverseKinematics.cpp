#include "numerical/math/SE3Transform.hpp"
#include "robotics/kinematics/DenavitHartenberg.hpp"
#include "robotics/kinematics/DhTaskJacobian.hpp"
#include "robotics/kinematics/PoseInverseKinematics.hpp"
#include "robotics/kinematics/test_doubles/MockJacobianProvider.hpp"
#include <cmath>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <limits>
#include <numbers>

namespace
{
    using ::testing::_;
    using ::testing::Return;
    using Vector3 = math::Vector3<float>;
    using Joints = math::Vector<float, 6>;
    using Jacobian = math::Matrix<float, 6, 6>;
    using Transform = math::SE3Transform<float>;
    using DhLink = kinematics::DhLink<float>;
    using Ik = kinematics::PoseInverseKinematics<float, 6>;

    constexpr float pi{ std::numbers::pi_v<float> };
    constexpr auto revolute{ dynamics::JointType::Revolute };

    Transform Pose(const Vector3& rotationVector, const Vector3& translation)
    {
        math::Vector6<float> twist{};
        twist.SetBlock(rotationVector, 3, 0);
        return Transform{ Transform::Exp(twist, 1.0f).rotation, translation };
    }

    Ik::Config MakeConfig()
    {
        Ik::Config config{};
        config.damping = 0.05f;
        config.characteristicLength = 0.3f;
        config.tolerance = 1e-4f;
        config.maxIterations = 100;
        return config;
    }

    class TestPoseInverseKinematics
        : public ::testing::Test
    {
    protected:
        float WeightedError(const Transform& target, const Joints& q) const
        {
            const auto error{ Transform::PoseError(target, puma.Forward(q)) };
            float sum{ 0.0f };
            for (std::size_t i = 0; i < 6; ++i)
            {
                const float weighted{ i < 3 ? error[i] : config.characteristicLength * error[i] };
                sum += weighted * weighted;
            }
            return std::sqrt(sum);
        }

        Ik::Result SolveWithMock(const Transform& target, const Transform& firstPose, const Transform& secondPose, const Jacobian& jacobian)
        {
            config.maxIterations = 1;
            EXPECT_CALL(mock, ToolPose(_)).WillOnce(Return(firstPose)).WillOnce(Return(secondPose));
            EXPECT_CALL(mock, Jacobian(_)).WillOnce(Return(jacobian));
            return Ik{ mock, config }.Solve(target, qTrue);
        }

        void ExpectStep(const Ik::Result& result, const Joints& expected) const
        {
            for (std::size_t i = 0; i < 6; ++i)
                EXPECT_NEAR(result.q[i] - qTrue[i], expected[i], 1e-6f);
        }

        static void ExpectFinite(const Joints& q)
        {
            for (std::size_t i = 0; i < 6; ++i)
                EXPECT_TRUE(std::isfinite(q[i]));
        }

        kinematics::DenavitHartenberg<float, 6> puma{ { DhLink{ 0.0f, pi / 2.0f, 0.0f, 0.0f, revolute }, DhLink{ 0.4318f, 0.0f, 0.0f, 0.0f, revolute },
                                                          DhLink{ 0.0203f, -pi / 2.0f, 0.15005f, 0.0f, revolute }, DhLink{ 0.0f, pi / 2.0f, 0.4318f, 0.0f, revolute },
                                                          DhLink{ 0.0f, -pi / 2.0f, 0.0f, 0.0f, revolute }, DhLink{ 0.0f, 0.0f, 0.0f, 0.0f, revolute } },
            kinematics::DhConvention::Standard };
        kinematics::DhTaskJacobian<float, 6, 6> arm{ puma };
        ::testing::StrictMock<kinematics::MockJacobianProvider<float, 6, 6>> mock;
        Ik::Config config{ MakeConfig() };
        Joints qTrue{ 0.3f, -0.4f, 0.5f, 0.6f, -0.7f, 0.8f };
        Joints qNear{ 0.4f, -0.3f, 0.6f, 0.7f, -0.6f, 0.9f };
        Transform nudge{ Pose(Vector3{ 0.0f, 0.0f, 0.1f }, Vector3{ 0.1f, 0.0f, 0.0f }) };
    };
}

TEST_F(TestPoseInverseKinematics, round_trip_reachable_pose)
{
    const auto target{ puma.Forward(qTrue) };

    const auto result{ Ik{ arm, config }.Solve(target, qNear) };

    EXPECT_TRUE(result.converged);
    EXPECT_LE(result.iterations, std::size_t{ 10 });
    EXPECT_LT(WeightedError(target, result.q), 1e-4f);
}

TEST_F(TestPoseInverseKinematics, damping_slows_but_does_not_bias)
{
    const auto target{ puma.Forward(qTrue) };
    config.tolerance = 1e-5f;
    config.maxIterations = 200;
    const auto light{ Ik{ arm, config }.Solve(target, qNear) };
    config.damping = 0.2f;

    const auto heavy{ Ik{ arm, config }.Solve(target, qNear) };

    EXPECT_TRUE(heavy.converged);
    for (std::size_t i = 0; i < 6; ++i)
        EXPECT_NEAR(heavy.q[i], qTrue[i], 1e-3f);
    EXPECT_GT(heavy.iterations, light.iterations);
}

TEST_F(TestPoseInverseKinematics, zero_error_is_immediate)
{
    const auto result{ Ik{ arm, config }.Solve(puma.Forward(qNear), qNear) };

    EXPECT_TRUE(result.converged);
    EXPECT_EQ(result.iterations, std::size_t{ 0 });
}

TEST_F(TestPoseInverseKinematics, final_error_is_measured_after_the_last_update)
{
    config.damping = 0.0f;
    config.characteristicLength = 1.0f;

    const auto result{ SolveWithMock(nudge, Transform::Identity(), Pose(Vector3{ 0.0f, 0.0f, 0.08f }, Vector3{ 0.08f, 0.0f, 0.0f }), Jacobian::Identity()) };

    ExpectStep(result, Joints{ 0.1f, 0.0f, 0.0f, 0.0f, 0.0f, 0.1f });
    EXPECT_NEAR(result.finalError, 0.028284f, 1e-5f);
    EXPECT_FALSE(result.converged);
}

TEST_F(TestPoseInverseKinematics, rotation_rows_are_weighted_in_the_step)
{
    config.damping = 0.1f;
    config.characteristicLength = 0.1f;

    const auto result{ SolveWithMock(nudge, Transform::Identity(), Transform::Identity(), Jacobian::Identity()) };

    ExpectStep(result, Joints{ 0.0990099f, 0.0f, 0.0f, 0.0f, 0.0f, 0.05f });
    EXPECT_NEAR(result.finalError, 0.100499f, 1e-5f);
}

TEST_F(TestPoseInverseKinematics, adaptive_damping_vanishes_away_from_singularity)
{
    config.manipulabilityThreshold = 0.5f;
    config.damping = 0.1f;
    config.characteristicLength = 1.0f;
    const auto error{ Transform::PoseError(nudge, Transform::Identity()) };

    const auto result{ SolveWithMock(nudge, Transform::Identity(), Transform::Identity(), Jacobian::Identity()) };

    for (std::size_t i = 0; i < 6; ++i)
        EXPECT_FLOAT_EQ(result.q[i], qTrue[i] + error[i]);
}

TEST_F(TestPoseInverseKinematics, adaptive_damping_engages_near_singularity)
{
    config.manipulabilityThreshold = 0.5f;
    config.damping = 0.1f;
    config.characteristicLength = 1.0f;
    auto nearlySingular{ Jacobian::Identity() };
    nearlySingular.at(5, 5) = 0.1f;

    const auto result{ SolveWithMock(Pose(Vector3{ 0.01f, 0.01f, 0.01f }, Vector3{ 0.01f, 0.01f, 0.01f }), Transform::Identity(), Transform::Identity(), nearlySingular) };

    ExpectStep(result, Joints{ 0.0099049f, 0.0099049f, 0.0099049f, 0.0099049f, 0.0099049f, 0.0510204f });
}

TEST_F(TestPoseInverseKinematics, step_is_clamped)
{
    config.maxStep = 0.1f;

    const auto result{ SolveWithMock(Pose(Vector3{}, Vector3{ 0.3f, 0.4f, 0.0f }), Transform::Identity(), Transform::Identity(), Jacobian::Identity()) };

    ExpectStep(result, Joints{ 0.06f, 0.08f, 0.0f, 0.0f, 0.0f, 0.0f });
}

TEST_F(TestPoseInverseKinematics, joint_limits_are_enforced)
{
    config.upperLimits[0] = qTrue[0] + 0.05f;

    const auto result{ SolveWithMock(Pose(Vector3{}, Vector3{ 0.3f, 0.0f, 0.0f }), Transform::Identity(), Transform::Identity(), Jacobian::Identity()) };

    EXPECT_EQ(result.q[0], config.upperLimits[0]);
}

TEST_F(TestPoseInverseKinematics, damping_survives_wrist_singularity)
{
    config.manipulabilityThreshold = 1e-3f;
    config.maxStep = 0.2f;
    const Joints wristSingular{ 0.3f, -0.4f, 0.5f, 0.6f, 0.0f, 0.8f };

    const auto result{ Ik{ arm, config }.Solve(puma.Forward(Joints{ 0.3f, -0.4f, 0.5f, 0.9f, 0.3f, 0.5f }), wristSingular) };

    EXPECT_TRUE(result.converged);
    ExpectFinite(result.q);
}

TEST_F(TestPoseInverseKinematics, unreachable_target_fails_gracefully)
{
    auto target{ puma.Forward(qTrue) };
    target.translation = Vector3{ 3.0f, 0.0f, 0.0f };

    const auto result{ Ik{ arm, config }.Solve(target, qTrue) };

    EXPECT_FALSE(result.converged);
    EXPECT_GT(result.finalError, config.tolerance);
    ExpectFinite(result.q);
}
