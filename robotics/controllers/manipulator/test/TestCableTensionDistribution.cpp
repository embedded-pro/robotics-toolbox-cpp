#include "numerical/math/SE3Transform.hpp"
#include "robotics/controllers/manipulator/CableTensionDistribution.hpp"
#include <cmath>
#include <gtest/gtest.h>
#include <numbers>

namespace
{
    using Vector3 = math::Vector3<float>;
    using Matrix3 = math::Matrix3<float>;
    using Transform = math::SE3Transform<float>;
    using Planar = controllers::CableTensionDistribution<float, 3, 2>;
    using Square = controllers::CableTensionDistribution<float, 2, 2>;
    using Spatial = controllers::CableTensionDistribution<float, 8, 6>;

    const float root3{ std::numbers::sqrt3_v<float> };

    template<std::size_t N>
    void ExpectNear(const math::Vector<float, N>& actual, const math::Vector<float, N>& expected, float tolerance)
    {
        for (std::size_t i = 0; i < N; ++i)
            EXPECT_NEAR(actual[i], expected[i], tolerance);
    }

    template<std::size_t Rows, std::size_t Cables>
    void ExpectWrench(const math::Matrix<float, Rows, Cables>& structure, const math::Vector<float, Cables>& tensions, const math::Vector<float, Rows>& wrench, float tolerance)
    {
        ExpectNear(math::Vector<float, Rows>{ structure * tensions }, wrench, tolerance);
    }

    template<std::size_t Cables>
    void ExpectWithinBounds(const math::Vector<float, Cables>& tensions)
    {
        for (std::size_t i = 0; i < Cables; ++i)
        {
            EXPECT_GE(tensions[i], 10.0f - 1e-3f);
            EXPECT_LE(tensions[i], 200.0f + 1e-3f);
        }
    }

    class TestCableTensionDistribution
        : public ::testing::Test
    {
    protected:
        Planar distributor{ { Vector3{ 0.0f, 2.0f, 0.0f }, Vector3{ -root3, -1.0f, 0.0f }, Vector3{ root3, -1.0f, 0.0f } }, {}, 10.0f, 200.0f };
        Transform pose{ Transform::Identity() };
    };

    class TestCableTensionDistributionSquare
        : public ::testing::Test
    {
    protected:
        Square distributor{ { Vector3{ -1.0f, 1.0f, 0.0f }, Vector3{ 1.0f, 1.0f, 0.0f } }, {}, 10.0f, 200.0f };
        Transform pose{ Transform::Identity() };
    };

    class TestCableTensionDistributionSpatial
        : public ::testing::Test
    {
    protected:
        Spatial distributor{ { Vector3{ 2.0f, 2.0f, 2.0f }, Vector3{ -2.0f, 2.0f, 2.0f }, Vector3{ -2.0f, -2.0f, 2.0f }, Vector3{ 2.0f, -2.0f, 2.0f }, Vector3{ 2.0f, 2.0f, 0.0f },
                                 Vector3{ -2.0f, 2.0f, 0.0f }, Vector3{ -2.0f, -2.0f, 0.0f }, Vector3{ 2.0f, -2.0f, 0.0f } },
            { Vector3{ 0.2f, 0.0f, 0.0f }, Vector3{ -0.2f, 0.2f, 0.1f }, Vector3{ -0.2f, -0.2f, 0.1f }, Vector3{ 0.2f, -0.2f, 0.1f }, Vector3{ 0.2f, 0.2f, -0.1f }, Vector3{ -0.2f, 0.2f, -0.1f },
                Vector3{ -0.2f, -0.2f, -0.1f }, Vector3{ 0.2f, -0.2f, -0.1f } },
            10.0f, 200.0f };
    };

    class TestCableTensionDistributionCrossed
        : public ::testing::Test
    {
    protected:
        Spatial distributor{ { Vector3{ -2.0f, -2.0f, 0.0f }, Vector3{ -2.0f, -2.0f, 3.0f }, Vector3{ -2.0f, 2.0f, 0.0f }, Vector3{ -2.0f, 2.0f, 3.0f }, Vector3{ 2.0f, -2.0f, 0.0f },
                                 Vector3{ 2.0f, -2.0f, 3.0f }, Vector3{ 2.0f, 2.0f, 0.0f }, Vector3{ 2.0f, 2.0f, 3.0f } },
            { Vector3{ 0.2f, -0.2f, -0.1f }, Vector3{ 0.2f, -0.2f, 0.1f }, Vector3{ 0.2f, 0.2f, -0.1f }, Vector3{ 0.2f, 0.2f, 0.1f }, Vector3{ -0.2f, -0.2f, -0.1f }, Vector3{ -0.2f, -0.2f, 0.1f },
                Vector3{ -0.2f, 0.2f, -0.1f }, Vector3{ -0.2f, 0.2f, 0.1f } },
            10.0f, 200.0f };
        Transform pose{ Matrix3::Identity(), Vector3{ 0.3f, -0.2f, 1.4f } };
    };
}

TEST_F(TestCableTensionDistribution, structure_matrix_from_geometry)
{
    const auto structure{ distributor.ComputeStructureMatrix(pose) };

    ASSERT_TRUE(structure.has_value());
    const math::Matrix<float, 2, 3> expected{ { 0.0f, -0.866025f, 0.866025f }, { 1.0f, -0.5f, -0.5f } };
    for (std::size_t r = 0; r < 2; ++r)
        for (std::size_t c = 0; c < 3; ++c)
            EXPECT_NEAR(structure->at(r, c), expected.at(r, c), 1e-5f);
}

TEST_F(TestCableTensionDistributionSpatial, spatial_structure_matrix_includes_moment_arm)
{
    const Transform rotated{ Matrix3{ { 0.0f, -1.0f, 0.0f }, { 1.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 1.0f } }, Vector3{} };

    const auto structure{ distributor.ComputeStructureMatrix(rotated) };

    ASSERT_TRUE(structure.has_value());
    const math::Vector<float, 6> expected{ 0.59655f, 0.53689f, 0.59655f, 0.11931f, 0.0f, -0.11931f };
    for (std::size_t r = 0; r < 6; ++r)
        EXPECT_NEAR(structure->at(r, 0), expected[r], 1e-5f);
}

TEST_F(TestCableTensionDistribution, zero_wrench_gives_centred_pretension)
{
    const auto tensions{ distributor.Distribute(math::Vector<float, 2>{}, pose) };

    ASSERT_TRUE(tensions.has_value());
    ExpectNear(*tensions, math::Vector<float, 3>{ 105.0f, 105.0f, 105.0f }, 1e-3f);
}

TEST_F(TestCableTensionDistribution, redundant_case_centres_tensions)
{
    const auto tensions{ distributor.Distribute(math::Vector<float, 2>{ 0.0f, 60.0f }, pose) };

    ASSERT_TRUE(tensions.has_value());
    ExpectNear(*tensions, math::Vector<float, 3>{ 145.0f, 85.0f, 85.0f }, 1e-3f);
}

TEST_F(TestCableTensionDistribution, wrench_is_reproduced)
{
    const math::Vector<float, 2> wrench{ 30.0f, -40.0f };

    const auto tensions{ distributor.Distribute(wrench, pose) };

    ASSERT_TRUE(tensions.has_value());
    ExpectWrench(*distributor.ComputeStructureMatrix(pose), *tensions, wrench, 1e-3f);
    ExpectWithinBounds(*tensions);
    ExpectNear(*tensions, math::Vector<float, 3>{ 78.333f, 101.013f, 135.654f }, 1e-3f);
}

TEST_F(TestCableTensionDistribution, upper_bound_fixes_worst_cable)
{
    const math::Vector<float, 2> wrench{ 0.0f, 150.0f };

    const auto tensions{ distributor.Distribute(wrench, pose) };

    ASSERT_TRUE(tensions.has_value());
    ExpectNear(*tensions, math::Vector<float, 3>{ 200.0f, 50.0f, 50.0f }, 1e-3f);
    ExpectWrench(*distributor.ComputeStructureMatrix(pose), *tensions, wrench, 1e-3f);
}

TEST_F(TestCableTensionDistribution, lower_bound_fixes_slack_cable)
{
    const math::Vector<float, 2> wrench{ 0.0f, -150.0f };

    const auto tensions{ distributor.Distribute(wrench, pose) };

    ASSERT_TRUE(tensions.has_value());
    ExpectNear(*tensions, math::Vector<float, 3>{ 10.0f, 160.0f, 160.0f }, 1e-3f);
    ExpectWrench(*distributor.ComputeStructureMatrix(pose), *tensions, wrench, 1e-3f);
}

TEST_F(TestCableTensionDistribution, infeasible_wrench_returns_nullopt)
{
    EXPECT_FALSE(distributor.Distribute(math::Vector<float, 2>{ 0.0f, 300.0f }, pose).has_value());
}

TEST_F(TestCableTensionDistributionSquare, square_case_unique_solution)
{
    const auto tensions{ distributor.Distribute(math::Vector<float, 2>{ 0.0f, 100.0f }, pose) };

    ASSERT_TRUE(tensions.has_value());
    ExpectNear(*tensions, math::Vector<float, 2>{ 70.7107f, 70.7107f }, 1e-3f);
}

TEST_F(TestCableTensionDistributionSquare, collinear_cables_return_nullopt)
{
    const math::Vector<float, 2> wrench{ 0.0f, 100.0f };

    EXPECT_FALSE(distributor.Distribute(wrench, Transform{ Matrix3::Identity(), Vector3{ 0.0f, 1.0f, 0.0f } }).has_value());
    EXPECT_FALSE(distributor.Distribute(wrench, Transform{ Matrix3::Identity(), Vector3{ -1.0f, 1.0f, 0.0f } }).has_value());
    EXPECT_FALSE(distributor.DistributeExact(wrench, Transform{ Matrix3::Identity(), Vector3{ -1.0f, 1.0f, 0.0f } }).has_value());
}

TEST_F(TestCableTensionDistribution, exact_distribution_matches_closed_form_with_active_bound)
{
    const math::Vector<float, 2> wrench{ 0.0f, 150.0f };

    const auto exact{ distributor.DistributeExact(wrench, pose) };
    const auto closed{ distributor.Distribute(wrench, pose) };

    ASSERT_TRUE(exact.has_value());
    ASSERT_TRUE(closed.has_value());
    ExpectNear(*exact, math::Vector<float, 3>{ 200.0f, 50.0f, 50.0f }, 1e-3f);
    ExpectNear(*exact, *closed, 1e-3f);
}

TEST_F(TestCableTensionDistributionCrossed, exact_distribution_succeeds_where_closed_form_fails)
{
    const math::Vector<float, 6> wrench{ 56.8f, -183.5f, -35.3f, -2.0f, 41.7f, -2.7f };

    const auto closed{ distributor.Distribute(wrench, pose) };
    const auto exact{ distributor.DistributeExact(wrench, pose) };

    EXPECT_FALSE(closed.has_value());
    ASSERT_TRUE(exact.has_value());
    ExpectNear(*exact, math::Vector<float, 8>{ 200.0f, 21.0187f, 23.9864f, 10.0f, 88.1208f, 197.9432f, 76.4249f, 23.7712f }, 1e-2f);
    ExpectWrench(*distributor.ComputeStructureMatrix(pose), *exact, wrench, 1e-2f);
}

TEST_F(TestCableTensionDistribution, exact_distribution_reports_infeasible_wrench)
{
    EXPECT_FALSE(distributor.DistributeExact(math::Vector<float, 2>{ 0.0f, 300.0f }, pose).has_value());
}
