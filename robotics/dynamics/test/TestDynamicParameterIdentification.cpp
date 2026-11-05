#include "robotics/dynamics/ChainInertialRegressor.hpp"
#include "robotics/dynamics/DynamicParameterIdentification.hpp"
#include "robotics/dynamics/RecursiveNewtonEuler.hpp"
#include <cmath>
#include <gtest/gtest.h>
#include <numbers>

namespace
{
    using Vector2 = math::Vector<float, 2>;
    using Vector3 = math::Vector<float, 3>;
    using Link = dynamics::JointLink<float>;
    using Links = std::array<Link, 2>;
    using Regressor = dynamics::ChainInertialRegressor<float, 2>;
    using Identification = dynamics::DynamicParameterIdentification<float, 2>;

    constexpr float samplePeriod{ 0.01f };
    constexpr float baseFrequency{ std::numbers::pi_v<float> };
    const Vector3 gravity{ 0.0f, 0.0f, -9.81f };

    struct Motion
    {
        Vector2 q;
        Vector2 qDot;
        Vector2 qDDot;
    };

    Links MakeArm(const Vector3& axis)
    {
        const auto rod{ [&axis](float mass, float length, const Vector3& parentToJoint)
            {
                const float inertia{ mass * length * length / 12.0f };
                return Link{ mass, math::SquareMatrix<float, 3>{ { 0.0f, 0.0f, 0.0f }, { 0.0f, inertia, 0.0f }, { 0.0f, 0.0f, inertia } }, axis, parentToJoint, Vector3{ length / 2.0f, 0.0f, 0.0f } };
            } };
        return Links{ rod(2.0f, 1.0f, Vector3{}), rod(1.5f, 0.8f, Vector3{ 1.0f, 0.0f, 0.0f }) };
    }

    Motion Excitation(float time)
    {
        constexpr std::array<std::array<float, 3>, 2> amplitude{ { { 0.8f, 0.4f, 0.2f }, { 1.0f, 0.5f, 0.3f } } };
        constexpr std::array<std::array<float, 3>, 2> phase{ { { 0.0f, 1.1f, 2.3f }, { 0.7f, 2.9f, 0.4f } } };
        constexpr std::array<float, 2> offset{ 0.3f, -0.6f };

        Motion motion{};
        for (std::size_t joint = 0; joint < 2; ++joint)
        {
            motion.q[joint] = offset[joint];
            for (std::size_t k = 0; k < 3; ++k)
            {
                const float frequency{ baseFrequency * static_cast<float>(k + 1) };
                const float angle{ frequency * time + phase[joint][k] };
                motion.q[joint] += amplitude[joint][k] * std::sin(angle);
                motion.qDot[joint] += amplitude[joint][k] * frequency * std::cos(angle);
                motion.qDDot[joint] -= amplitude[joint][k] * frequency * frequency * std::sin(angle);
            }
        }
        return motion;
    }

    class TestDynamicParameterIdentification
        : public ::testing::Test
    {
    protected:
        Vector2 Torque(const Links& links, const Motion& motion) const
        {
            return rnea.InverseDynamics(links, motion.q, motion.qDot, motion.qDDot, gravity);
        }

        void Excite(Identification& target, const Links& links) const
        {
            for (std::size_t k = 0; k < 200; ++k)
            {
                const auto motion{ Excitation(static_cast<float>(k) * samplePeriod) };
                target.AddSample(motion.q, motion.qDot, motion.qDDot, Torque(links, motion));
            }
        }

        dynamics::RecursiveNewtonEuler<float, 2> rnea;
        Links horizontal{ MakeArm(Vector3{ 0.0f, 1.0f, 0.0f }) };
        Links vertical{ MakeArm(Vector3{ 0.0f, 0.0f, 1.0f }) };
        Regressor regressor{ horizontal, gravity };
        Regressor verticalRegressor{ vertical, gravity };
        Identification identification{ regressor };
        float tolerance{ 1e-4f };
    };
}

TEST_F(TestDynamicParameterIdentification, base_parameter_count_with_gravity_is_six)
{
    Excite(identification, horizontal);

    const auto result{ identification.Solve(tolerance) };

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->rank, std::size_t{ 6 });
}

TEST_F(TestDynamicParameterIdentification, base_parameter_count_without_gravity_is_four)
{
    Identification withoutGravity{ verticalRegressor };
    Excite(withoutGravity, vertical);

    const auto result{ withoutGravity.Solve(tolerance) };

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->rank, std::size_t{ 4 });
}

TEST_F(TestDynamicParameterIdentification, noiseless_data_reproduces_torques)
{
    Excite(identification, horizontal);
    const auto result{ identification.Solve(tolerance) };
    ASSERT_TRUE(result.has_value());

    for (std::size_t k = 0; k < 20; ++k)
    {
        const auto motion{ Excitation(0.0137f + static_cast<float>(k) * 0.093f) };
        const auto expected{ Torque(horizontal, motion) };
        const auto predicted{ identification.Predict(*result, motion.q, motion.qDot, motion.qDDot) };
        const float scale{ std::abs(expected[0]) + std::abs(expected[1]) };
        for (std::size_t i = 0; i < 2; ++i)
            EXPECT_NEAR(predicted[i], expected[i], 1e-3f * scale);
    }
    EXPECT_LT(result->residualRms, 1e-2f);
}

TEST_F(TestDynamicParameterIdentification, identifiable_combinations_match_the_true_parameters)
{
    Excite(identification, horizontal);
    const auto result{ identification.Solve(tolerance) };
    ASSERT_TRUE(result.has_value());

    const auto truth{ Regressor::Parameters(horizontal) };
    float truthNorm{ 0.0f };
    for (std::size_t c = 0; c < Identification::ParameterCount; ++c)
        truthNorm += truth[c] * truth[c];

    for (std::size_t r = 0; r < result->rank; ++r)
    {
        float estimated{ 0.0f };
        float expected{ 0.0f };
        for (std::size_t c = 0; c < Identification::ParameterCount; ++c)
        {
            estimated += result->baseDirections.at(r, c) * result->parameters[c];
            expected += result->baseDirections.at(r, c) * truth[c];
        }
        EXPECT_NEAR(estimated, expected, 1e-3f * std::sqrt(truthNorm));
    }
}

TEST_F(TestDynamicParameterIdentification, no_samples_returns_nullopt)
{
    Excite(identification, horizontal);

    identification.Reset();

    EXPECT_FALSE(identification.Solve(tolerance).has_value());
}
