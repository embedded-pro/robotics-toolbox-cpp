#include "numerical/math/SE2Transform.hpp"
#include "robotics/mobile/PlanarOdometry.hpp"
#include <cstddef>
#include <gtest/gtest.h>
#include <numbers>

namespace
{
    using Transform = math::SE2Transform<float>;
    using Vector3 = math::Vector3<float>;
    using Odometry = mobile::PlanarOdometry<float>;

    constexpr float pi{ std::numbers::pi_v<float> };

    class TestPlanarOdometry
        : public ::testing::Test
    {
    protected:
        static void ExpectNear(const Vector3& actual, const Vector3& expected, float tolerance = 1e-5f)
        {
            for (std::size_t i = 0; i < 3; ++i)
                EXPECT_NEAR(actual[i], expected[i], tolerance);
        }

        Odometry odometry;
        Vector3 arc{ 0.5f, 0.0f, 0.25f };
    };
}

TEST_F(TestPlanarOdometry, straight_line_accumulates_distance)
{
    for (std::size_t k = 0; k < 4; ++k)
        odometry.Integrate(Vector3{ 1.0f, 0.0f, 0.0f }, 0.25f);

    ExpectNear(odometry.Pose().Pose(), Vector3{ 1.0f, 0.0f, 0.0f });
}

TEST_F(TestPlanarOdometry, quarter_arc_is_exact_in_one_step)
{
    odometry.Integrate(arc, 2.0f * pi);

    ExpectNear(odometry.Pose().Pose(), Vector3{ 2.0f, 2.0f, pi / 2.0f });
}

TEST_F(TestPlanarOdometry, full_circle_closes_independent_of_step_count)
{
    Odometry fine;

    for (std::size_t k = 0; k < 7; ++k)
        odometry.Integrate(arc, 8.0f * pi / 7.0f);
    for (std::size_t k = 0; k < 1000; ++k)
        fine.Integrate(arc, 8.0f * pi / 1000.0f);

    ExpectNear(odometry.Pose().Pose(), Vector3{}, 1e-5f);
    ExpectNear(fine.Pose().Pose(), Vector3{}, 1e-4f);
}

TEST_F(TestPlanarOdometry, increment_recovers_the_integrated_twist)
{
    const auto start{ Transform::FromPose(1.0f, 2.0f, 0.3f) };
    Odometry moving{ start };

    moving.Integrate(Vector3{ 0.4f, 0.1f, -0.6f }, 0.5f);

    ExpectNear(Odometry::Increment(start, moving.Pose()), Vector3{ 0.2f, 0.05f, -0.3f });
}

TEST_F(TestPlanarOdometry, reset_replaces_the_pose)
{
    odometry.Integrate(arc, 1.0f);

    odometry.Reset(Transform::FromPose(-1.0f, 3.0f, 2.0f));

    ExpectNear(odometry.Pose().Pose(), Vector3{ -1.0f, 3.0f, 2.0f });
}

TEST_F(TestPlanarOdometry, increment_deviation_grows_with_distance_and_rotation)
{
    const mobile::OdometryNoise<float> noise{ 0.05f, 0.01f, 0.05f, 0.01f };

    ExpectNear(mobile::IncrementDeviation(Transform::FromPose(0.5f, 0.0f, 0.1f), noise), Vector3{ 0.112250f, 0.112250f, 0.054772f });
    ExpectNear(mobile::IncrementDeviation(Transform::FromPose(-0.5f, 0.0f, 0.0f), noise), mobile::IncrementDeviation(Transform::FromPose(0.5f, 0.0f, 0.0f), noise), 0.0f);
}

TEST_F(TestPlanarOdometry, long_runs_keep_a_unit_rotation)
{
    for (std::size_t k = 0; k < 100000; ++k)
        odometry.Integrate(arc, 1e-3f);

    const auto& pose{ odometry.Pose() };
    EXPECT_NEAR(pose.cosine * pose.cosine + pose.sine * pose.sine, 1.0f, 1e-6f);
}
