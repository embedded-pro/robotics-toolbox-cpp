#include "numerical/math/SE3Transform.hpp"
#include "robotics/kinematics/ContinuumKinematics.hpp"
#include <cmath>
#include <gtest/gtest.h>
#include <numbers>

namespace
{
    using Vector3 = math::Vector3<float>;
    using Arc = kinematics::ArcParameters<float>;
    using Continuum = kinematics::ContinuumKinematics<float, 2>;

    constexpr float pi{ std::numbers::pi_v<float> };

    class TestContinuumKinematics
        : public ::testing::Test
    {
    protected:
        static void ExpectNear(const Vector3& actual, const Vector3& expected, float tolerance = 1e-5f)
        {
            for (std::size_t i = 0; i < 3; ++i)
                EXPECT_NEAR(actual[i], expected[i], tolerance);
        }

        static void ExpectIdentityRotation(const math::Matrix3<float>& rotation)
        {
            for (std::size_t r = 0; r < 3; ++r)
                for (std::size_t c = 0; c < 3; ++c)
                    EXPECT_NEAR(rotation.at(r, c), r == c ? 1.0f : 0.0f, 1e-6f);
        }

        static float Norm(const Vector3& v)
        {
            return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        }

        static void ExpectArc(const Arc& actual, const Arc& expected)
        {
            EXPECT_NEAR(actual.kappa, expected.kappa, 1e-4f);
            EXPECT_NEAR(actual.phi, expected.phi, 1e-5f);
            EXPECT_NEAR(actual.length, expected.length, 1e-5f);
        }

        Arc quarter{ pi / 2.0f, 0.0f, 1.0f };
        Arc general{ 2.0f, 0.7f, 1.2f };
        Arc negative{ -1.5f, 0.3f, 0.8f };
        Continuum twoSections{ { general, general } };
    };
}

TEST_F(TestContinuumKinematics, straight_section_is_pure_translation)
{
    const auto pose{ Continuum::SectionTransform(Arc{ 0.0f, 0.0f, 1.0f }) };

    ExpectIdentityRotation(pose.rotation);
    ExpectNear(pose.translation, Vector3{ 0.0f, 0.0f, 1.0f });
}

TEST_F(TestContinuumKinematics, quarter_circle_bend)
{
    const auto pose{ Continuum::SectionTransform(quarter) };

    ExpectNear(pose.translation, Vector3{ 0.636620f, 0.0f, 0.636620f });
    ExpectNear(Vector3{ pose.rotation * Vector3{ 0.0f, 0.0f, 1.0f } }, Vector3{ 1.0f, 0.0f, 0.0f });
    ExpectNear(Vector3{ pose.rotation * Vector3{ 1.0f, 0.0f, 0.0f } }, Vector3{ 0.0f, 0.0f, -1.0f });
}

TEST_F(TestContinuumKinematics, bending_plane_angle_rotates_tip)
{
    ExpectNear(Continuum::SectionTransform(Arc{ pi / 2.0f, pi / 2.0f, 1.0f }).translation, Vector3{ 0.0f, 0.636620f, 0.636620f });
}

TEST_F(TestContinuumKinematics, tip_distance_equals_chord)
{
    EXPECT_NEAR(Norm(Continuum::SectionTransform(quarter).translation), 0.900316f, 1e-5f);
    EXPECT_NEAR(Norm(Continuum::SectionTransform(general).translation), 0.932039f, 1e-5f);
    EXPECT_NEAR(Norm(Continuum::SectionTransform(negative).translation), 0.752857f, 1e-5f);
}

TEST_F(TestContinuumKinematics, tip_tangent_matches_bend)
{
    const auto pose{ Continuum::SectionTransform(general) };

    ExpectNear(Vector3{ pose.rotation * Vector3{ 0.0f, 0.0f, 1.0f } }, Vector3{ 0.516623f, 0.435145f, -0.737394f });
    ExpectNear(pose.translation, Vector3{ 0.664416f, 0.559630f, 0.337732f });
}

TEST_F(TestContinuumKinematics, forward_composes_sections)
{
    const auto single{ Continuum::SectionTransform(general) };
    const auto composed{ single * single };

    const auto tip{ twoSections.Forward() };

    ExpectNear(tip.translation, composed.translation);
    ExpectNear(tip.translation, Vector3{ 0.348960f, 0.293925f, -0.498082f });
    ExpectNear(Vector3{ tip.rotation * Vector3{ 0.0f, 0.0f, 1.0f } }, Vector3{ composed.rotation * Vector3{ 0.0f, 0.0f, 1.0f } });
}

TEST_F(TestContinuumKinematics, inverse_recovers_arc_parameters)
{
    ExpectArc(Continuum::InverseSection(Continuum::SectionTransform(quarter).translation), quarter);
    ExpectArc(Continuum::InverseSection(Continuum::SectionTransform(general).translation), general);
    ExpectArc(Continuum::InverseSection(Continuum::SectionTransform(negative).translation), Arc{ 1.5f, 0.3f - pi, 0.8f });
    ExpectArc(Continuum::InverseSection(Vector3{ 0.0f, 0.0f, 0.7f }), Arc{ 0.0f, 0.0f, 0.7f });
}

TEST_F(TestContinuumKinematics, near_zero_curvature_uses_series)
{
    const auto straight{ Continuum::SectionTransform(Arc{ 1e-9f, 0.4f, 1.0f }) };
    const auto slight{ Continuum::SectionTransform(Arc{ 1e-5f, 0.4f, 1.0f }) };

    ExpectIdentityRotation(straight.rotation);
    ExpectNear(straight.translation, Vector3{ 0.0f, 0.0f, 1.0f });
    ExpectNear(slight.translation, Vector3{ 4.605e-6f, 1.947e-6f, 1.0f }, 1e-8f);
    ExpectArc(Continuum::InverseSection(slight.translation), Arc{ 1e-5f, 0.4f, 1.0f });
}
