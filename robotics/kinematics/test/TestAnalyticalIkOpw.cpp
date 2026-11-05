#include "numerical/math/AngleUtilities.hpp"
#include "numerical/math/SE3Transform.hpp"
#include "robotics/kinematics/AnalyticalIkOpw.hpp"
#include <array>
#include <cmath>
#include <cstdint>
#include <gtest/gtest.h>
#include <numbers>

namespace
{
    using Vector3 = math::Vector3<float>;
    using Matrix3 = math::Matrix3<float>;
    using Joints = math::Vector<float, 6>;
    using Transform = math::SE3Transform<float>;
    using Parameters = kinematics::OpwParameters<float>;
    using Ik = kinematics::AnalyticalIkOpw<float>;

    constexpr float pi{ std::numbers::pi_v<float> };

    Matrix3 RotationZ(float angle)
    {
        return Matrix3{ { std::cos(angle), -std::sin(angle), 0.0f }, { std::sin(angle), std::cos(angle), 0.0f }, { 0.0f, 0.0f, 1.0f } };
    }

    Matrix3 RotationY(float angle)
    {
        return Matrix3{ { std::cos(angle), 0.0f, std::sin(angle) }, { 0.0f, 1.0f, 0.0f }, { -std::sin(angle), 0.0f, std::cos(angle) } };
    }

    Transform Rotate(const Matrix3& rotation)
    {
        return Transform{ rotation, Vector3{} };
    }

    Transform Translate(float x, float y, float z)
    {
        return Transform{ Matrix3::Identity(), Vector3{ x, y, z } };
    }

    class TestAnalyticalIkOpw
        : public ::testing::Test
    {
    protected:
        static std::array<float, 6> ModelAngles(const Parameters& parameters, const Joints& q)
        {
            std::array<float, 6> theta{};
            for (std::size_t i = 0; i < 6; ++i)
                theta[i] = static_cast<float>(parameters.signs[i]) * q[i] - parameters.offsets[i];
            return theta;
        }

        static Transform ElementaryChain(const Parameters& parameters, const Joints& q)
        {
            const auto theta{ ModelAngles(parameters, q) };
            return Rotate(RotationZ(theta[0])) * Translate(parameters.a1, parameters.b, parameters.c1) * Rotate(RotationY(theta[1])) * Translate(0.0f, 0.0f, parameters.c2) *
                   Rotate(RotationY(theta[2])) * Translate(parameters.a2, 0.0f, parameters.c3) * Rotate(RotationZ(theta[3])) * Rotate(RotationY(theta[4])) * Rotate(RotationZ(theta[5])) *
                   Translate(0.0f, 0.0f, parameters.c4);
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

        static bool SameAngles(const Joints& lhs, const Joints& rhs, float tolerance)
        {
            for (std::size_t i = 0; i < 6; ++i)
                if (std::abs(math::WrapToPi(lhs[i] - rhs[i])) > tolerance)
                    return false;
            return true;
        }

        static void ExpectRoundTrips(const Parameters& parameters)
        {
            const Ik solver{ parameters };
            std::uint32_t state{ 12345u };
            const auto next{ [&state]
                {
                    state = state * 1664525u + 1013904223u;
                    return static_cast<float>(state) / 4294967296.0f * 2.0f * pi - pi;
                } };

            for (std::size_t sample = 0; sample < 200; ++sample)
            {
                const Joints q{ next(), next(), next(), next(), next(), next() };
                const auto target{ solver.Forward(q) };
                bool found{ false };
                for (const auto& solution : solver.Solve(target))
                    if (solution)
                    {
                        ExpectPose(solver.Forward(solution->q), target, 1e-3f);
                        found = found || SameAngles(solution->q, q, 1e-3f);
                    }
                const bool wristWellConditioned{ std::abs(std::sin(ModelAngles(parameters, q)[4])) > 1e-2f };
                EXPECT_TRUE(found || !wristWellConditioned) << "sample " << sample;
            }
        }

        Parameters kr6{ 0.025f, -0.035f, 0.0f, 0.400f, 0.315f, 0.365f, 0.080f, { 0.0f, -pi / 2.0f, 0.0f, 0.0f, 0.0f, 0.0f }, { -1, 1, 1, -1, 1, -1 } };
        Ik ik{ kr6 };
        Joints qTrue{ 0.3f, -1.0f, 1.2f, 0.5f, 0.6f, 0.7f };
    };
}

TEST_F(TestAnalyticalIkOpw, forward_matches_elementary_chain)
{
    ExpectPose(ik.Forward(qTrue), ElementaryChain(kr6, qTrue), 1e-5f);
    ExpectPose(ik.Forward(Joints{}), ElementaryChain(kr6, Joints{}), 1e-5f);
}

TEST_F(TestAnalyticalIkOpw, home_flange_pose)
{
    ExpectPose(ik.Forward(Joints{}), Transform{ Matrix3{ { 0.0f, 0.0f, 1.0f }, { 0.0f, 1.0f, 0.0f }, { -1.0f, 0.0f, 0.0f } }, Vector3{ 0.785f, 0.0f, 0.435f } }, 1e-5f);
}

TEST_F(TestAnalyticalIkOpw, every_solution_reproduces_target)
{
    const auto target{ ik.Forward(qTrue) };

    const auto solutions{ ik.Solve(target) };

    for (const auto& solution : solutions)
    {
        ASSERT_TRUE(solution.has_value());
        EXPECT_FALSE(solution->wristSingular);
        ExpectPose(ik.Forward(solution->q), target, 1e-4f);
    }
}

TEST_F(TestAnalyticalIkOpw, finds_the_known_configuration)
{
    const auto solutions{ ik.Solve(ik.Forward(qTrue)) };

    ASSERT_TRUE(solutions[0].has_value());
    EXPECT_TRUE(SameAngles(solutions[0]->q, qTrue, 1e-4f));
}

TEST_F(TestAnalyticalIkOpw, wrist_flip_pairs)
{
    const auto solutions{ ik.Solve(ik.Forward(qTrue)) };

    for (std::size_t j = 0; j < 4; ++j)
    {
        ASSERT_TRUE(solutions[j].has_value());
        ASSERT_TRUE(solutions[j + 4].has_value());
        const auto regular{ ModelAngles(kr6, solutions[j]->q) };
        const auto flipped{ ModelAngles(kr6, solutions[j + 4]->q) };
        for (std::size_t i = 0; i < 3; ++i)
            EXPECT_NEAR(math::WrapToPi(flipped[i] - regular[i]), 0.0f, 1e-5f);
        EXPECT_NEAR(math::WrapToPi(flipped[3] - regular[3] - pi), 0.0f, 1e-5f);
        EXPECT_NEAR(math::WrapToPi(flipped[4] + regular[4]), 0.0f, 1e-5f);
        EXPECT_NEAR(math::WrapToPi(flipped[5] - regular[5] + pi), 0.0f, 1e-5f);
    }
}

TEST_F(TestAnalyticalIkOpw, random_poses_round_trip)
{
    ExpectRoundTrips(kr6);
}

TEST_F(TestAnalyticalIkOpw, general_offsets_signs_and_lateral_offset)
{
    ExpectRoundTrips(Parameters{ 0.10f, -0.05f, 0.07f, 0.50f, 0.60f, 0.55f, 0.10f, { 0.1f, -0.2f, 0.3f, 0.0f, 0.4f, -0.5f }, { 1, -1, 1, 1, -1, 1 } });
}

TEST_F(TestAnalyticalIkOpw, wrist_singularity_keeps_the_sum)
{
    const auto target{ ik.Forward(Joints{ 0.3f, -1.0f, 1.2f, 0.5f, 0.0f, 0.7f }) };

    const auto solutions{ ik.Solve(target) };

    ASSERT_TRUE(solutions[0].has_value());
    EXPECT_TRUE(solutions[0]->wristSingular);
    EXPECT_TRUE(SameAngles(solutions[0]->q, Joints{ 0.3f, -1.0f, 1.2f, 0.0f, 0.0f, 1.2f }, 1e-3f));
    EXPECT_FALSE(solutions[4].has_value());
    ExpectPose(ik.Forward(solutions[0]->q), target, 1e-4f);

    const auto flippedTarget{ ik.Forward(Joints{ 0.3f, -1.0f, 1.2f, 0.5f, pi, 0.7f }) };
    const auto flipped{ ik.Solve(flippedTarget) };
    ASSERT_TRUE(flipped[0].has_value());
    EXPECT_TRUE(flipped[0]->wristSingular);
    EXPECT_FALSE(flipped[4].has_value());
    ExpectPose(ik.Forward(flipped[0]->q), flippedTarget, 1e-4f);
}

TEST_F(TestAnalyticalIkOpw, stretched_arm_elbows_coincide)
{
    const auto target{ ik.Forward(Joints{ 0.3f, -0.3f, 0.095598f, 0.5f, 0.6f, 0.7f }) };

    const auto solutions{ ik.Solve(target) };

    ASSERT_TRUE(solutions[0].has_value());
    ASSERT_TRUE(solutions[1].has_value());
    EXPECT_TRUE(SameAngles(solutions[0]->q, solutions[1]->q, 1e-3f));
    ExpectPose(ik.Forward(solutions[0]->q), target, 1e-3f);
    ExpectPose(ik.Forward(solutions[1]->q), target, 1e-3f);
}

TEST_F(TestAnalyticalIkOpw, unreachable_target_returns_no_solution)
{
    const auto solutions{ ik.Solve(Translate(1.5f, 0.0f, 0.4f)) };

    for (const auto& solution : solutions)
        EXPECT_FALSE(solution.has_value());

    const Ik lateral{ Parameters{ 0.10f, -0.05f, 0.07f, 0.50f, 0.60f, 0.55f, 0.10f, {}, { 1, 1, 1, 1, 1, 1 } } };
    for (const auto& solution : lateral.Solve(Translate(0.01f, 0.0f, 0.6f)))
        EXPECT_FALSE(solution.has_value());

    const Ik foldedOnShoulder{ Parameters{ 0.0f, -0.05f, 0.0f, 0.5f, 0.6f, 0.55f, 0.25f, {}, { 1, 1, 1, 1, 1, 1 } } };
    for (const auto& solution : foldedOnShoulder.Solve(Translate(0.0f, 0.0f, 0.75f)))
        EXPECT_FALSE(solution.has_value());
}
