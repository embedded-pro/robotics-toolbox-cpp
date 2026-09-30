#include "robotics/kinematics/ForwardKinematics.hpp"
#include "robotics/kinematics/InverseKinematics.hpp"
#include "numerical/math/Tolerance.hpp"
#include <cmath>
#include <gtest/gtest.h>
#include <numbers>

namespace
{
    using Vector3 = math::Vector<float, 3>;
    using Link = dynamics::RevoluteJointLink<float>;

    constexpr float pi = std::numbers::pi_v<float>;

    const Vector3 xAxis{ 1.0f, 0.0f, 0.0f };
    const Vector3 yAxis{ 0.0f, 1.0f, 0.0f };
    const Vector3 zAxis{ 0.0f, 0.0f, 1.0f };

    Link MakeLink(const Vector3& axis, const Vector3& parentToJoint, const Vector3& jointToCoM)
    {
        return Link{ 1.0f, math::SquareMatrix<float, 3>::Identity(), axis, parentToJoint, jointToCoM };
    }

    class TestInverseKinematics
        : public ::testing::Test
    {
    protected:
        const Vector3 planarTool{ 0.8f, 0.0f, 0.0f };
        const std::array<Link, 2> planarArm{
            MakeLink(zAxis, Vector3{}, Vector3{ 0.5f, 0.0f, 0.0f }),
            MakeLink(zAxis, Vector3{ 1.0f, 0.0f, 0.0f }, Vector3{ 0.4f, 0.0f, 0.0f })
        };

        template<std::size_t N>
        void ExpectToolAt(const std::array<Link, N>& links, const Vector3& tool,
            const math::Vector<float, N>& q, const Vector3& target) const
        {
            kinematics::ForwardKinematics<float, N> fk{ tool };
            auto positions = fk.Compute(links, q);

            EXPECT_NEAR(positions[N].at(0, 0), target.at(0, 0), math::Tolerance<float>());
            EXPECT_NEAR(positions[N].at(1, 0), target.at(1, 0), math::Tolerance<float>());
            EXPECT_NEAR(positions[N].at(2, 0), target.at(2, 0), math::Tolerance<float>());
        }
    };
}

TEST_F(TestInverseKinematics, single_link_recovers_reference_angle)
{
    std::array<Link, 1> links{ MakeLink(zAxis, Vector3{}, Vector3{ 0.5f, 0.0f, 0.0f }) };
    kinematics::InverseKinematics<float, 1> ik{ xAxis };
    Vector3 target{ std::cos(pi / 3.0f), std::sin(pi / 3.0f), 0.0f };

    auto result = ik.Solve(links, target, math::Vector<float, 1>{ 0.0f });

    EXPECT_TRUE(result.converged);
    EXPECT_NEAR(result.q.at(0, 0), pi / 3.0f, math::Tolerance<float>());
}

TEST_F(TestInverseKinematics, initial_guess_at_solution_returns_without_iterating)
{
    kinematics::InverseKinematics<float, 2> ik{ planarTool };
    math::Vector<float, 2> initialQ{ pi / 4.0f, -pi / 6.0f };
    kinematics::ForwardKinematics<float, 2> fk{ planarTool };
    Vector3 target{ fk.Compute(planarArm, initialQ)[2] };

    auto result = ik.Solve(planarArm, target, initialQ);

    EXPECT_TRUE(result.converged);
    EXPECT_EQ(result.iterations, 0u);
    EXPECT_LT(result.finalError, 1e-4f);
}

TEST_F(TestInverseKinematics, two_link_planar_reaches_reachable_target)
{
    kinematics::InverseKinematics<float, 2> ik{ planarTool };
    Vector3 target{ 0.5f, 1.0f, 0.0f };

    auto result = ik.Solve(planarArm, target, math::Vector<float, 2>{ 0.0f, 0.0f });

    EXPECT_TRUE(result.converged);
    ExpectToolAt(planarArm, planarTool, result.q, target);
}

TEST_F(TestInverseKinematics, target_near_full_extension_converges)
{
    kinematics::InverseKinematics<float, 2> ik{ planarTool };
    Vector3 target{ 1.75f, 0.0f, 0.0f };

    auto result = ik.Solve(planarArm, target, math::Vector<float, 2>{ 0.1f, 0.1f });

    EXPECT_TRUE(result.converged);
    ExpectToolAt(planarArm, planarTool, result.q, target);
}

TEST_F(TestInverseKinematics, target_near_base_converges)
{
    kinematics::InverseKinematics<float, 2> ik{ planarTool };
    Vector3 target{ 0.4f, 0.0f, 0.0f };

    auto result = ik.Solve(planarArm, target, math::Vector<float, 2>{ pi / 3.0f, -2.0f * pi / 3.0f });

    EXPECT_TRUE(result.converged);
    ExpectToolAt(planarArm, planarTool, result.q, target);
}

TEST_F(TestInverseKinematics, unreachable_target_exhausts_iterations)
{
    kinematics::InverseKinematicsConfig<float> config;
    config.maxIterations = 50;
    kinematics::InverseKinematics<float, 2> ik{ planarTool, config };

    auto result = ik.Solve(planarArm, Vector3{ 10.0f, 10.0f, 10.0f }, math::Vector<float, 2>{ 0.0f, 0.0f });

    EXPECT_FALSE(result.converged);
    EXPECT_EQ(result.iterations, 50u);
    EXPECT_GT(result.finalError, config.tolerance);
}

TEST_F(TestInverseKinematics, heavy_damping_still_converges_to_exact_target)
{
    kinematics::InverseKinematicsConfig<float> config;
    config.dampingFactor = 0.5f;
    config.maxIterations = 500;
    kinematics::InverseKinematics<float, 2> ik{ planarTool, config };
    Vector3 target{ 0.5f, 1.0f, 0.0f };

    auto result = ik.Solve(planarArm, target, math::Vector<float, 2>{ 0.0f, 0.0f });

    EXPECT_TRUE(result.converged);
    EXPECT_LT(result.finalError, config.tolerance);
}

TEST_F(TestInverseKinematics, spatial_chain_with_base_and_tool_offsets_reaches_fk_target)
{
    const Vector3 tool{ 0.25f, 0.0f, 0.05f };
    const std::array<Link, 3> links{
        MakeLink(zAxis, Vector3{ 0.0f, 0.0f, 0.2f }, Vector3{ 0.0f, 0.0f, 0.1f }),
        MakeLink(yAxis, Vector3{ 0.0f, 0.0f, 0.5f }, Vector3{ 0.05f, 0.0f, 0.0f }),
        MakeLink(yAxis, Vector3{ 0.4f, 0.0f, 0.0f }, Vector3{ 0.05f, 0.0f, 0.0f })
    };
    kinematics::InverseKinematicsConfig<float> config;
    config.dampingFactor = 0.05f;
    config.maxIterations = 300;
    kinematics::InverseKinematics<float, 3> ik{ tool, config };
    kinematics::ForwardKinematics<float, 3> fk{ tool };
    Vector3 target{ fk.Compute(links, math::Vector<float, 3>{ pi / 4.0f, pi / 6.0f, -pi / 3.0f })[3] };

    auto result = ik.Solve(links, target, math::Vector<float, 3>{ 0.0f, 0.0f, 0.0f });

    EXPECT_TRUE(result.converged);
    ExpectToolAt(links, tool, result.q, target);
}
