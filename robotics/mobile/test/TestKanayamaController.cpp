#include "numerical/math/SE2Transform.hpp"
#include "robotics/mobile/KanayamaController.hpp"
#include <cstddef>
#include <gtest/gtest.h>
#include <numbers>

namespace
{
    using Transform = math::SE2Transform<float>;
    using Vector3 = math::Vector3<float>;
    using Command = mobile::UnicycleCommand<float>;

    constexpr float pi{ std::numbers::pi_v<float> };

    class TestKanayamaController
        : public ::testing::Test
    {
    protected:
        static void ExpectNear(const Command& actual, const Command& expected)
        {
            EXPECT_NEAR(actual.v, expected.v, 1e-5f);
            EXPECT_NEAR(actual.omega, expected.omega, 1e-5f);
        }

        static float Lyapunov(const Vector3& error, float ky)
        {
            return error[0] * error[0] + error[1] * error[1] + 2.0f * (1.0f - math::Cos(error[2])) / ky;
        }

        mobile::KanayamaController<float> controller{ { 1.0f, 4.0f, 4.0f } };
        float vr{ 0.5f };
    };
}

TEST_F(TestKanayamaController, zero_error_returns_the_feedforward)
{
    ExpectNear(controller.Compute(Transform::Identity(), Transform::Identity(), vr, 0.1f), Command{ 0.5f, 0.1f });
}

TEST_F(TestKanayamaController, longitudinal_error_changes_speed)
{
    ExpectNear(controller.Compute(Transform::Identity(), Transform::FromPose(0.2f, 0.0f, 0.0f), vr, 0.0f), Command{ 0.7f, 0.0f });
}

TEST_F(TestKanayamaController, lateral_error_turns_towards_the_reference)
{
    ExpectNear(controller.Compute(Transform::Identity(), Transform::FromPose(0.0f, 0.1f, 0.0f), vr, 0.0f), Command{ 0.5f, 0.2f });
}

TEST_F(TestKanayamaController, heading_error_enters_through_sine_and_cosine)
{
    ExpectNear(controller.Compute(Transform::Identity(), Transform::FromPose(0.0f, 0.0f, 0.3f), vr, 0.0f), Command{ 0.477668f, 0.591040f });
}

TEST_F(TestKanayamaController, error_is_expressed_in_the_robot_frame)
{
    const auto pose{ Transform::FromPose(1.0f, 1.0f, pi / 2.0f) };
    const auto reference{ Transform::FromPose(1.0f, 2.0f, pi / 2.0f) };
    const auto error{ mobile::KanayamaController<float>::TrackingError(pose, reference) };

    EXPECT_NEAR(error[0], 1.0f, 1e-5f);
    EXPECT_NEAR(error[1], 0.0f, 1e-5f);
    EXPECT_NEAR(error[2], 0.0f, 1e-5f);
    ExpectNear(controller.Compute(pose, reference, vr, 0.0f), Command{ 1.5f, 0.0f });
}

TEST_F(TestKanayamaController, critically_damped_gains)
{
    const auto gains{ mobile::KanayamaController<float>::CriticallyDamped(1.0f, 4.0f) };

    EXPECT_EQ(gains.kx, 1.0f);
    EXPECT_EQ(gains.ky, 4.0f);
    EXPECT_EQ(gains.ktheta, 4.0f);
}

TEST_F(TestKanayamaController, closed_loop_converges_with_a_non_increasing_lyapunov_function)
{
    const float dt{ 0.01f };
    auto pose{ Transform::FromPose(-0.3f, 0.5f, -0.4f) };
    auto reference{ Transform::Identity() };
    float lyapunov{ Lyapunov(mobile::KanayamaController<float>::TrackingError(pose, reference), 4.0f) };
    EXPECT_NEAR(lyapunov, 0.379470f, 1e-5f);

    for (std::size_t k = 0; k < 2000; ++k)
    {
        const auto command{ controller.Compute(pose, reference, vr, 0.0f) };
        pose = (pose * Transform::Exp(Vector3{ command.v, 0.0f, command.omega }, dt)).Normalized();
        reference = (reference * Transform::Exp(Vector3{ vr, 0.0f, 0.0f }, dt)).Normalized();

        const float next{ Lyapunov(mobile::KanayamaController<float>::TrackingError(pose, reference), 4.0f) };
        EXPECT_LE(next, lyapunov + 1e-6f);
        lyapunov = next;
    }

    const auto error{ mobile::KanayamaController<float>::TrackingError(pose, reference) };
    EXPECT_LT(math::Hypot(math::Hypot(error[0], error[1]), error[2]), 1e-4f);
}
