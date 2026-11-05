#include "numerical/math/SE2Transform.hpp"
#include "robotics/mobile/CarLikeKinematics.hpp"
#include "robotics/mobile/PlanarPath.hpp"
#include "robotics/mobile/StanleyController.hpp"
#include <array>
#include <cmath>
#include <cstddef>
#include <gtest/gtest.h>

namespace
{
    using Point = math::Vector2<float>;
    using Transform = math::SE2Transform<float>;
    using Car = mobile::CarLikeKinematics<float>;
    using Stanley = mobile::StanleyController<float, 4>;

    class TestStanleyController
        : public ::testing::Test
    {
    protected:
        TestStanleyController()
        {
            road.Assign(waypoints);
        }

        std::array<Point, 2> waypoints{ Point{ 0.0f, 0.0f }, Point{ 50.0f, 0.0f } };
        mobile::PlanarPath<float, 4> road;
        Car car{ { 2.5f, 1.5f, 0.6f, 10.0f } };
        mobile::StanleyParameters<float> parameters{ 2.5f, 1.0f, 2 };
        Stanley stanley{ road, car, parameters };
    };
}

TEST_F(TestStanleyController, lateral_offset_steers_back_towards_the_path)
{
    EXPECT_NEAR(stanley.Compute(Transform::FromPose(0.0f, 0.5f, 0.0f), 5.0f, 0.0f, 1.0f), -0.205395f, 1e-5f);
}

TEST_F(TestStanleyController, heading_error_adds_to_the_correction)
{
    EXPECT_NEAR(stanley.Compute(Transform::FromPose(0.0f, 0.0f, 0.2f), 5.0f, 0.0f, 1.0f), -0.404067f, 1e-5f);
}

TEST_F(TestStanleyController, error_is_measured_at_the_front_axle)
{
    const auto error{ stanley.Error(Transform::FromPose(0.0f, 0.0f, 0.2f)) };

    EXPECT_NEAR(error.crossTrack, 0.496673f, 1e-5f);
    EXPECT_NEAR(error.headingError, -0.2f, 1e-5f);
}

TEST_F(TestStanleyController, output_saturates_at_max_steer)
{
    EXPECT_EQ(stanley.Compute(Transform::FromPose(0.0f, -3.0f, 0.0f), 5.0f, 0.0f, 1.0f), 0.6f);
}

TEST_F(TestStanleyController, steering_rate_is_limited)
{
    const Car slowSteering{ { 2.5f, 1.5f, 0.6f, 0.5f } };
    Stanley limited{ road, slowSteering, parameters };

    EXPECT_NEAR(limited.Compute(Transform::FromPose(0.0f, 0.5f, 0.0f), 5.0f, 0.0f, 0.1f), -0.05f, 1e-6f);
}

TEST_F(TestStanleyController, closed_loop_converges)
{
    const float speed{ 5.0f };
    const float dt{ 0.01f };
    auto pose{ Transform::FromPose(0.0f, 1.0f, 0.0f) };
    float steer{ 0.0f };

    for (std::size_t k = 0; k < 400; ++k)
    {
        steer = stanley.Compute(pose, speed, steer, dt);
        pose = (pose * Transform::Exp(car.Forward(speed, steer), dt)).Normalized();
    }

    EXPECT_LT(std::abs(stanley.Error(pose).crossTrack), 1e-3f);

    stanley.Restart();
    EXPECT_NEAR(stanley.Compute(Transform::FromPose(0.0f, 0.5f, 0.0f), speed, 0.0f, 1.0f), -0.205395f, 1e-5f);
}
