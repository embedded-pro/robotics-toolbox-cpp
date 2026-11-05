#include "numerical/math/SE3Transform.hpp"
#include "robotics/kinematics/StewartGoughKinematics.hpp"
#include <cmath>
#include <gtest/gtest.h>
#include <numbers>

namespace
{
    using Vector3 = math::Vector3<float>;
    using Matrix3 = math::Matrix3<float>;
    using Transform = math::SE3Transform<float>;
    using Hexapod = kinematics::StewartGoughKinematics<float>;

    constexpr float pi{ std::numbers::pi_v<float> };

    Matrix3 RotationZ(float angle)
    {
        return Matrix3{ { std::cos(angle), -std::sin(angle), 0.0f }, { std::sin(angle), std::cos(angle), 0.0f }, { 0.0f, 0.0f, 1.0f } };
    }

    Matrix3 RotationX(float angle)
    {
        return Matrix3{ { 1.0f, 0.0f, 0.0f }, { 0.0f, std::cos(angle), -std::sin(angle) }, { 0.0f, std::sin(angle), std::cos(angle) } };
    }

    kinematics::StewartGoughGeometry<float> MakeGeometry()
    {
        kinematics::StewartGoughGeometry<float> geometry{};
        for (std::size_t k = 0; k < 6; ++k)
        {
            const float side{ k % 2 == 0 ? -1.0f : 1.0f };
            const float sector{ 2.0f * pi / 3.0f * static_cast<float>(k / 2) };
            const float base{ sector + side * pi / 12.0f };
            const float platform{ sector + side * pi / 4.0f };
            geometry.baseAnchors[k] = Vector3{ std::cos(base), std::sin(base), 0.0f };
            geometry.platformAnchors[k] = Vector3{ 0.6f * std::cos(platform), 0.6f * std::sin(platform), 0.0f };
        }
        return geometry;
    }

    class TestStewartGoughKinematics
        : public ::testing::Test
    {
    protected:
        static void ExpectLegs(const Hexapod::LegLengths& actual, const Hexapod::LegLengths& expected, float tolerance = 1e-5f)
        {
            for (std::size_t i = 0; i < 6; ++i)
                EXPECT_NEAR(actual[i], expected[i], tolerance);
        }

        static void ExpectPose(const Transform& actual, const Transform& expected, float tolerance)
        {
            for (std::size_t r = 0; r < 3; ++r)
            {
                EXPECT_NEAR(actual.translation[r], expected.translation[r], tolerance);
                for (std::size_t c = 0; c < 3; ++c)
                    EXPECT_NEAR(actual.rotation.at(r, c), expected.rotation.at(r, c), tolerance);
            }
        }

        kinematics::StewartGoughGeometry<float> geometry{ MakeGeometry() };
        Hexapod hexapod{ geometry, { 1e-5f, 0.0f, 50 } };
        Transform home{ Matrix3::Identity(), Vector3{ 0.0f, 0.0f, 0.8f } };
        Transform tilted{ Matrix3{ RotationZ(0.1f) * RotationX(0.05f) }, Vector3{ 0.05f, -0.03f, 0.85f } };
    };
}

TEST_F(TestStewartGoughKinematics, home_legs_are_equal)
{
    ExpectLegs(hexapod.Inverse(home), { 0.980189f, 0.980189f, 0.980189f, 0.980189f, 0.980189f, 0.980189f });
}

TEST_F(TestStewartGoughKinematics, raising_the_platform_lengthens_every_leg)
{
    ExpectLegs(hexapod.Inverse(Transform{ Matrix3::Identity(), Vector3{ 0.0f, 0.0f, 0.9f } }), { 1.063376f, 1.063376f, 1.063376f, 1.063376f, 1.063376f, 1.063376f });
}

TEST_F(TestStewartGoughKinematics, inverse_of_a_general_pose)
{
    ExpectLegs(hexapod.Inverse(tilted), { 0.955703f, 1.037816f, 1.049122f, 1.083232f, 0.981789f, 1.042429f });
}

TEST_F(TestStewartGoughKinematics, leg_jacobian_matches_finite_difference)
{
    math::Vector6<float> twist{ 0.1f, -0.2f, 0.05f, 0.3f, -0.1f, 0.2f };
    math::Vector6<float> rotation{};
    constexpr float epsilon{ 1e-3f };
    for (std::size_t i = 3; i < 6; ++i)
        rotation[i] = twist[i] * epsilon;
    const Transform perturbed{ Matrix3{ Transform::Exp(rotation, 1.0f).rotation * tilted.rotation },
        Vector3{ tilted.translation + Vector3{ twist[0], twist[1], twist[2] } * epsilon } };

    const math::Vector6<float> predicted{ hexapod.LegJacobian(tilted) * twist };

    const auto ahead{ hexapod.Inverse(perturbed) };
    const auto here{ hexapod.Inverse(tilted) };
    for (std::size_t i = 0; i < 6; ++i)
        EXPECT_NEAR(predicted[i], (ahead[i] - here[i]) / epsilon, 1e-3f);
}

TEST_F(TestStewartGoughKinematics, forward_recovers_the_pose)
{
    const auto result{ hexapod.Forward(hexapod.Inverse(tilted), Transform{ RotationZ(0.15f), Vector3{ 0.0f, 0.0f, 0.8f } }) };

    EXPECT_TRUE(result.converged);
    EXPECT_LE(result.iterations, std::size_t{ 6 });
    ExpectPose(result.pose, tilted, 1e-4f);
}

TEST_F(TestStewartGoughKinematics, assembly_mode_is_selected_by_the_guess)
{
    const auto lengths{ hexapod.Inverse(home) };

    const auto upper{ hexapod.Forward(lengths, Transform{ Matrix3::Identity(), Vector3{ 0.0f, 0.0f, 0.7f } }) };
    const auto lower{ hexapod.Forward(lengths, Transform{ Matrix3::Identity(), Vector3{ 0.0f, 0.0f, -0.7f } }) };

    ExpectPose(upper.pose, home, 1e-4f);
    ExpectPose(lower.pose, Transform{ Matrix3::Identity(), Vector3{ 0.0f, 0.0f, -0.8f } }, 1e-4f);
}

TEST_F(TestStewartGoughKinematics, singular_pose_step_stays_finite)
{
    const Hexapod damped{ geometry, { 1e-5f, 1e-3f, 50 } };
    const auto lengths{ damped.Inverse(Transform{ RotationZ(pi / 2.0f), Vector3{ 0.0f, 0.0f, 0.8f } }) };

    const auto result{ damped.Forward(lengths, Transform{ RotationZ(80.0f * pi / 180.0f), Vector3{ 0.0f, 0.0f, 0.8f } }) };

    for (std::size_t r = 0; r < 3; ++r)
    {
        EXPECT_TRUE(std::isfinite(result.pose.translation[r]));
        for (std::size_t c = 0; c < 3; ++c)
            EXPECT_TRUE(std::isfinite(result.pose.rotation.at(r, c)));
    }
    const auto reached{ damped.Inverse(result.pose) };
    float residual{ 0.0f };
    for (std::size_t i = 0; i < 6; ++i)
        residual += (reached[i] - lengths[i]) * (reached[i] - lengths[i]);
    EXPECT_LT(std::sqrt(residual), 1e-4f);
}
