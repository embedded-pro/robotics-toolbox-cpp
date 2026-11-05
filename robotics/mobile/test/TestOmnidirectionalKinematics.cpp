#include "robotics/mobile/OmnidirectionalKinematics.hpp"
#include <cmath>
#include <cstddef>
#include <gtest/gtest.h>
#include <numbers>

namespace
{
    using Vector2 = math::Vector2<float>;
    using Vector3 = math::Vector3<float>;
    using Vector4 = math::Vector<float, 4>;
    using Wheel = mobile::SwedishWheel<float>;

    constexpr float pi{ std::numbers::pi_v<float> };

    Wheel KiwiWheel(float degrees)
    {
        const float angle{ degrees * pi / 180.0f };
        return Wheel{ Vector2{ 0.2f * std::cos(angle), 0.2f * std::sin(angle) }, angle + pi / 2.0f, 0.0f, 0.05f };
    }

    class TestOmnidirectionalKinematics
        : public ::testing::Test
    {
    protected:
        template<std::size_t Rows, std::size_t Cols>
        static void ExpectNear(const math::Matrix<float, Rows, Cols>& actual, const math::Matrix<float, Rows, Cols>& expected, float tolerance = 1e-4f)
        {
            for (std::size_t r = 0; r < Rows; ++r)
                for (std::size_t c = 0; c < Cols; ++c)
                    EXPECT_NEAR(actual.at(r, c), expected.at(r, c), tolerance);
        }

        mobile::OmnidirectionalKinematics<float, 4> mecanum{ { Wheel{ Vector2{ 0.2f, 0.15f }, 0.0f, -pi / 4.0f, 0.05f }, Wheel{ Vector2{ 0.2f, -0.15f }, 0.0f, pi / 4.0f, 0.05f },
                                                                 Wheel{ Vector2{ -0.2f, 0.15f }, 0.0f, pi / 4.0f, 0.05f }, Wheel{ Vector2{ -0.2f, -0.15f }, 0.0f, -pi / 4.0f, 0.05f } },
            20.0f };
        mobile::OmnidirectionalKinematics<float, 3> kiwi{ { KiwiWheel(90.0f), KiwiWheel(210.0f), KiwiWheel(330.0f) }, 20.0f };
    };
}

TEST_F(TestOmnidirectionalKinematics, mecanum_jacobian_matches_the_closed_form)
{
    ExpectNear(mecanum.WheelJacobian(), math::Matrix<float, 4, 3>{ { 20.0f, -20.0f, -7.0f }, { 20.0f, 20.0f, 7.0f }, { 20.0f, 20.0f, -7.0f }, { 20.0f, -20.0f, 7.0f } });
}

TEST_F(TestOmnidirectionalKinematics, inverse_then_forward_recovers_the_twist)
{
    const Vector4 speeds{ 6.5f, 5.5f, -1.5f, 13.5f };

    ExpectNear(mecanum.Inverse(Vector3{ 0.3f, -0.2f, 0.5f }), speeds);
    ExpectNear(mecanum.Forward(speeds), Vector3{ 0.3f, -0.2f, 0.5f }, 1e-5f);
    EXPECT_NEAR(mecanum.SlipResidual(speeds), 0.0f, 1e-4f);
}

TEST_F(TestOmnidirectionalKinematics, forward_is_least_squares_and_flags_slip)
{
    const Vector4 speeds{ 10.0f, 0.0f, 0.0f, 0.0f };

    ExpectNear(mecanum.Forward(speeds), Vector3{ 0.125f, -0.125f, -0.357143f }, 1e-5f);
    EXPECT_NEAR(mecanum.SlipResidual(speeds), 5.0f, 1e-4f);
}

TEST_F(TestOmnidirectionalKinematics, kiwi_is_exactly_determined)
{
    ExpectNear(kiwi.WheelJacobian(), math::Matrix<float, 3, 3>{ { -20.0f, 0.0f, 4.0f }, { 10.0f, -17.320508f, 4.0f }, { 10.0f, 17.320508f, 4.0f } });
    ExpectNear(kiwi.Forward(Vector3{ 5.0f, 5.0f, 5.0f }), Vector3{ 0.0f, 0.0f, 1.25f }, 1e-5f);
    EXPECT_NEAR(kiwi.SlipResidual(Vector3{ 3.0f, -1.0f, 7.0f }), 0.0f, 1e-4f);
}

TEST_F(TestOmnidirectionalKinematics, limit_scales_all_wheels_uniformly)
{
    const auto speeds{ mecanum.Inverse(Vector3{ 1.0f, 0.5f, 1.0f }) };

    ExpectNear(speeds, Vector4{ 3.0f, 37.0f, 23.0f, 17.0f });
    ExpectNear(mecanum.Limit(speeds), Vector4{ speeds * (20.0f / 37.0f) });
    ExpectNear(mecanum.Forward(mecanum.Limit(speeds)), Vector3{ 0.540541f, 0.270270f, 0.540541f }, 1e-5f);
    ExpectNear(mecanum.Limit(Vector4{ 1.0f, -2.0f, 3.0f, -4.0f }), Vector4{ 1.0f, -2.0f, 3.0f, -4.0f }, 0.0f);
}
