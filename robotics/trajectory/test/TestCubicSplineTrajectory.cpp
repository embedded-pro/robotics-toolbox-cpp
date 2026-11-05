#include "numerical/math/CubicSpline.hpp"
#include "numerical/math/Tolerance.hpp"
#include "numerical/motion_profiles/PolynomialProfile.hpp"
#include "robotics/trajectory/CubicSplineTrajectory.hpp"
#include <gtest/gtest.h>

namespace
{
    using Vector3 = math::Vector<float, 3>;
    using Spline = trajectory::CubicSplineTrajectory<float, 3, 8>;

    class TestCubicSplineTrajectory
        : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            ASSERT_TRUE(spline.Plan(times, positions, 5));
        }

        void ExpectNear(const Vector3& actual, const Vector3& expected) const
        {
            for (std::size_t j = 0; j < 3; ++j)
                EXPECT_NEAR(actual[j], expected[j], math::Tolerance<float>());
        }

        Spline::Times times{ 0.0f, 1.0f, 2.5f, 3.0f, 4.5f };
        Spline::Positions positions{ Vector3{ 0.0f, 0.5f, 0.0f }, Vector3{ 1.0f, 0.2f, 0.0f }, Vector3{ -0.5f, 0.2f, 0.0f },
            Vector3{ 0.3f, -0.4f, 0.0f }, Vector3{ 2.0f, 0.0f, 0.0f } };
        Spline spline;
    };
}

TEST_F(TestCubicSplineTrajectory, default_plan_is_rest_to_rest)
{
    const auto start{ spline.Sample(0.0f) };
    const auto end{ spline.Sample(4.5f) };

    ExpectNear(start.position, positions[0]);
    ExpectNear(end.position, positions[4]);
    ExpectNear(start.velocity, Vector3{});
    ExpectNear(end.velocity, Vector3{});
}

TEST_F(TestCubicSplineTrajectory, state_maps_value_and_derivatives_to_position_velocity_acceleration)
{
    const std::array<float, 5> knotAccelerations{ 5.665169f, -5.330337f, 5.991011f, -0.737079f, -1.898127f };
    math::CubicSpline<float, 3, 8> reference;
    ASSERT_TRUE(reference.Fit(times, positions, 5));

    for (std::size_t k = 0; k < knotAccelerations.size(); ++k)
        EXPECT_NEAR(spline.Sample(times[k]).acceleration[0], knotAccelerations[k], math::Tolerance<float>());

    const auto state{ spline.Sample(0.5f) };
    const auto point{ reference.Evaluate(0.5f) };
    ExpectNear(state.position, point.value);
    ExpectNear(state.velocity, point.firstDerivative);
    ExpectNear(state.acceleration, point.secondDerivative);
}

TEST_F(TestCubicSplineTrajectory, start_time_and_duration_follow_the_knots)
{
    Spline::Times shifted{ 2.0f, 3.0f, 4.5f, 5.0f, 6.5f };

    ASSERT_TRUE(spline.Plan(shifted, positions, 5));

    EXPECT_NEAR(spline.StartTime(), 2.0f, math::Tolerance<float>());
    EXPECT_NEAR(spline.Duration(), 4.5f, math::Tolerance<float>());
    ExpectNear(spline.Sample(2.0f).position, positions[0]);
}

TEST_F(TestCubicSplineTrajectory, two_knots_reduce_to_the_cubic_polynomial_profile)
{
    Spline::Times twoTimes{ 0.0f, 2.0f };
    Spline::Positions twoPositions{ Vector3{}, Vector3{ 1.0f, 0.0f, 0.0f } };
    Spline::Boundary boundary{ math::SplineEnd::Clamped, Vector3{ 0.5f, 0.0f, 0.0f }, Vector3{ -0.5f, 0.0f, 0.0f } };
    motion_profiles::PolynomialProfile<float> profile{ { 0.0f, 1.0f, 0.5f, -0.5f }, 2.0f, motion_profiles::Degree::Cubic };

    ASSERT_TRUE(spline.Plan(twoTimes, twoPositions, 2, boundary));

    for (const float t : { 0.3f, 1.0f, 1.7f })
    {
        const auto state{ spline.Sample(t) };
        const auto expected{ profile.Sample(t) };
        EXPECT_NEAR(state.position[0], expected.position, math::Tolerance<float>());
        EXPECT_NEAR(state.velocity[0], expected.velocity, math::Tolerance<float>());
        EXPECT_NEAR(state.acceleration[0], expected.acceleration, math::Tolerance<float>());
    }
    EXPECT_NEAR(spline.Sample(0.0f).acceleration[0], 1.0f, math::Tolerance<float>());
    EXPECT_NEAR(spline.Sample(2.0f).acceleration[0], -2.0f, math::Tolerance<float>());
}

TEST_F(TestCubicSplineTrajectory, plan_rejects_invalid_knots)
{
    Spline::Times repeated{ 0.0f, 1.0f, 1.0f, 2.0f };

    EXPECT_FALSE(spline.Plan(times, positions, 1));
    EXPECT_FALSE(spline.Plan(times, positions, 9));
    EXPECT_FALSE(spline.Plan(repeated, positions, 4));
}
