#include "numerical/math/Geometry3D.hpp"
#include "numerical/math/Tolerance.hpp"
#include "numerical/motion_profiles/PolynomialProfile.hpp"
#include "robotics/trajectory/CartesianSlerpInterpolation.hpp"
#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>
#include <numbers>

namespace
{
    using Law = motion_profiles::PolynomialProfile<float>;
    using Path = trajectory::CartesianSlerpInterpolation<float, Law>;
    using Transform = math::SE3Transform<float>;
    using Vector3 = math::Vector3<float>;
    using Matrix3 = math::Matrix3<float>;

    constexpr float pi{ std::numbers::pi_v<float> };
    const Vector3 zAxis{ 0.0f, 0.0f, 1.0f };

    Matrix3 Rz(float angle)
    {
        return math::RotationAboutAxis(zAxis, angle);
    }

    float AngleBetween(const Matrix3& a, const Matrix3& b)
    {
        const Matrix3 relative{ a.Transpose() * b };
        return std::acos(std::clamp((relative.Trace() - 1.0f) / 2.0f, -1.0f, 1.0f));
    }

    class TestCartesianSlerp
        : public ::testing::Test
    {
    protected:
        void ExpectNear(const Matrix3& actual, const Matrix3& expected) const
        {
            for (std::size_t r = 0; r < 3; ++r)
                for (std::size_t c = 0; c < 3; ++c)
                    EXPECT_NEAR(actual.at(r, c), expected.at(r, c), math::Tolerance<float>());
        }

        void ExpectNear(const Vector3& actual, const Vector3& expected) const
        {
            for (std::size_t i = 0; i < 3; ++i)
                EXPECT_NEAR(actual[i], expected[i], math::Tolerance<float>());
        }

        void ExpectPose(const Transform& actual, const Transform& expected) const
        {
            ExpectNear(actual.rotation, expected.rotation);
            ExpectNear(actual.translation, expected.translation);
        }

        Law linearLaw{ { 0.0f, 1.0f, 1.0f, 1.0f }, 1.0f, motion_profiles::Degree::Cubic };
        Transform start{ Matrix3::Identity(), Vector3{} };
        Transform goal{ Rz(pi / 2.0f), Vector3{ 1.0f, 0.0f, 0.0f } };
        Path path{ start, goal, linearLaw };
    };
}

TEST_F(TestCartesianSlerp, endpoints_match_start_and_goal)
{
    ExpectPose(path.Sample(0.0f).pose, start);
    ExpectPose(path.Sample(path.Duration()).pose, goal);
}

TEST_F(TestCartesianSlerp, position_is_straight_line)
{
    ExpectNear(path.Sample(0.5f).pose.translation, Vector3{ 0.5f, 0.0f, 0.0f });
}

TEST_F(TestCartesianSlerp, rotation_stays_orthonormal)
{
    const Matrix3 rotation{ path.Sample(0.5f).pose.rotation };

    ExpectNear(Matrix3{ rotation * rotation.Transpose() }, Matrix3::Identity());
    const float determinant{ rotation.at(0, 0) * (rotation.at(1, 1) * rotation.at(2, 2) - rotation.at(1, 2) * rotation.at(2, 1)) - rotation.at(0, 1) * (rotation.at(1, 0) * rotation.at(2, 2) - rotation.at(1, 2) * rotation.at(2, 0)) + rotation.at(0, 2) * (rotation.at(1, 0) * rotation.at(2, 1) - rotation.at(1, 1) * rotation.at(2, 0)) };
    EXPECT_NEAR(determinant, 1.0f, math::Tolerance<float>());
}

TEST_F(TestCartesianSlerp, slerp_midpoint_is_half_angle)
{
    ExpectNear(path.Sample(0.5f).pose.rotation, Rz(pi / 4.0f));
}

TEST_F(TestCartesianSlerp, shortest_path_takes_short_arc)
{
    const Path reflex{ start, Transform{ Rz(3.0f * pi / 2.0f), goal.translation }, linearLaw };

    const auto state{ reflex.Sample(0.5f) };

    ExpectNear(state.pose.rotation, Rz(-pi / 4.0f));
    ExpectNear(state.twist.GetBlock<3, 1>(3, 0), Vector3{ 0.0f, 0.0f, -pi / 2.0f });
}

TEST_F(TestCartesianSlerp, near_parallel_orientation_is_finite)
{
    const float angle{ 0.01f * pi / 180.0f };
    const Path tiny{ start, Transform{ Rz(angle), goal.translation }, linearLaw };

    const auto state{ tiny.Sample(0.5f) };

    for (std::size_t i = 0; i < 6; ++i)
        EXPECT_TRUE(std::isfinite(state.twist[i]));
    for (std::size_t r = 0; r < 3; ++r)
        for (std::size_t c = 0; c < 3; ++c)
            EXPECT_TRUE(std::isfinite(state.pose.rotation.at(r, c)));
    ExpectNear(state.pose.rotation, Matrix3::Identity());
    EXPECT_NEAR(state.twist[5], angle, 1e-6f);
}

TEST_F(TestCartesianSlerp, constant_angular_rate_for_linear_time_law)
{
    for (const float t : { 0.25f, 0.5f, 0.75f })
        EXPECT_NEAR(AngleBetween(start.rotation, path.Sample(t).pose.rotation), t * pi / 2.0f, math::Tolerance<float>());
}

TEST_F(TestCartesianSlerp, twist_is_sdot_times_path_tangent)
{
    const Path quintic{ start, goal, Law{ { 0.0f, 1.0f }, 2.0f, motion_profiles::Degree::Quintic } };

    const auto middle{ quintic.Sample(1.0f).twist };
    const auto atRest{ quintic.Sample(0.0f).twist };

    ExpectNear(middle.GetBlock<3, 1>(0, 0), Vector3{ 0.9375f, 0.0f, 0.0f });
    ExpectNear(middle.GetBlock<3, 1>(3, 0), Vector3{ 0.0f, 0.0f, 1.472622f });
    for (std::size_t i = 0; i < 6; ++i)
        EXPECT_NEAR(atRest[i], 0.0f, math::Tolerance<float>());
}

TEST_F(TestCartesianSlerp, sample_clamps_outside_domain)
{
    ExpectPose(path.Sample(-1.0f).pose, start);
    ExpectPose(path.Sample(path.Duration() + 1.0f).pose, goal);
}
