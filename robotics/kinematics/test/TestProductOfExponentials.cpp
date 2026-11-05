#include "numerical/math/SE3Transform.hpp"
#include "numerical/math/Tolerance.hpp"
#include "robotics/kinematics/DenavitHartenberg.hpp"
#include "robotics/kinematics/GeometricJacobian.hpp"
#include "robotics/kinematics/ProductOfExponentials.hpp"
#include <gtest/gtest.h>
#include <numbers>

namespace
{
    using Vector3 = math::Vector3<float>;
    using Matrix3 = math::Matrix3<float>;
    using Screw = math::Vector6<float>;
    using Transform = math::SE3Transform<float>;
    using Joints2 = math::Vector<float, 2>;
    using Joints6 = math::Vector<float, 6>;

    constexpr float pi{ std::numbers::pi_v<float> };
    constexpr float w1{ 0.109f };
    constexpr float w2{ 0.082f };
    constexpr float l1{ 0.425f };
    constexpr float l2{ 0.392f };
    constexpr float h1{ 0.089f };
    constexpr float h2{ 0.095f };

    Screw MakeScrew(const Vector3& linear, const Vector3& angular)
    {
        Screw screw{};
        screw.SetBlock(linear, 0, 0);
        screw.SetBlock(angular, 3, 0);
        return screw;
    }

    class TestProductOfExponentials
        : public ::testing::Test
    {
    protected:
        void ExpectNear(const Vector3& actual, const Vector3& expected) const
        {
            for (std::size_t i = 0; i < 3; ++i)
                EXPECT_NEAR(actual[i], expected[i], math::Tolerance<float>());
        }

        template<std::size_t N>
        void ExpectNear(const math::Matrix<float, 6, N>& actual, const math::Matrix<float, 6, N>& expected) const
        {
            for (std::size_t r = 0; r < 6; ++r)
                for (std::size_t c = 0; c < N; ++c)
                    EXPECT_NEAR(actual.at(r, c), expected.at(r, c), math::Tolerance<float>());
        }

        Vector3 x{ 1.0f, 0.0f, 0.0f };
        Vector3 y{ 0.0f, 1.0f, 0.0f };
        Vector3 z{ 0.0f, 0.0f, 1.0f };
        kinematics::ProductOfExponentials<float, 2> poe{ { MakeScrew(Vector3{}, z), MakeScrew(Vector3{ 0.0f, -1.0f, 0.0f }, z) }, Transform{ Matrix3::Identity(), Vector3{ 2.0f, 0.0f, 0.0f } } };
        kinematics::ProductOfExponentials<float, 6> ur5{ { MakeScrew(Vector3{}, z), MakeScrew(Vector3{ -h1, 0.0f, 0.0f }, y), MakeScrew(Vector3{ -h1, 0.0f, l1 }, y),
                                                             MakeScrew(Vector3{ -h1, 0.0f, l1 + l2 }, y), MakeScrew(Vector3{ -w1, l1 + l2, 0.0f }, Vector3{ 0.0f, 0.0f, -1.0f }),
                                                             MakeScrew(Vector3{ h2 - h1, 0.0f, l1 + l2 }, y) },
            Transform{ Matrix3{ { -1.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 1.0f }, { 0.0f, 1.0f, 0.0f } }, Vector3{ l1 + l2, w1 + w2, h1 - h2 } } };
        Joints6 ur5Q{ 0.3f, -0.5f, 0.8f, -0.2f, 0.6f, 1.1f };
    };
}

TEST_F(TestProductOfExponentials, zero_config_returns_home)
{
    const auto pose{ poe.Compute(Joints2{}) };

    ExpectNear(pose.translation, Vector3{ 2.0f, 0.0f, 0.0f });
    ExpectNear(Vector3{ pose.rotation * x }, x);
    ExpectNear(ur5.Compute(Joints6{}).translation, Vector3{ 0.817f, 0.191f, -0.006f });
}

TEST_F(TestProductOfExponentials, single_joint_rotation)
{
    ExpectNear(poe.Compute(Joints2{ pi / 2.0f, 0.0f }).translation, Vector3{ 0.0f, 2.0f, 0.0f });
}

TEST_F(TestProductOfExponentials, elbow_rotation)
{
    ExpectNear(poe.Compute(Joints2{ 0.0f, pi / 2.0f }).translation, Vector3{ 1.0f, 1.0f, 0.0f });
}

TEST_F(TestProductOfExponentials, agrees_with_denavit_hartenberg)
{
    const kinematics::DenavitHartenberg<float, 2> dh{ { kinematics::DhLink<float>{ 1.0f, 0.0f, 0.0f, 0.0f }, kinematics::DhLink<float>{ 1.0f, 0.0f, 0.0f, 0.0f } }, kinematics::DhConvention::Standard };

    for (const auto& q : { Joints2{ 0.4f, -0.7f }, Joints2{ 1.3f, 2.1f } })
    {
        const auto fromScrews{ poe.Compute(q) };
        const auto fromDh{ dh.Forward(q) };
        ExpectNear(fromScrews.translation, fromDh.translation);
        ExpectNear(Vector3{ fromScrews.rotation * x }, Vector3{ fromDh.rotation * x });
    }
}

TEST_F(TestProductOfExponentials, space_jacobian_linear_part_is_base_origin_velocity)
{
    const math::Matrix<float, 6, 2> expected{ { 0.0f, 0.389418f }, { 0.0f, -0.921061f }, { 0.0f, 0.0f }, { 0.0f, 0.0f }, { 0.0f, 0.0f }, { 1.0f, 1.0f } };

    ExpectNear(poe.SpaceJacobian(Joints2{ 0.4f, -0.7f }), expected);
}

TEST_F(TestProductOfExponentials, space_to_geometric_matches_frame_chain_jacobian)
{
    const auto geometric{ decltype(ur5)::SpaceToGeometric(ur5.SpaceJacobian(ur5Q), ur5.Compute(ur5Q).translation) };

    ExpectNear(geometric, kinematics::GeometricJacobian<float, 6>::Compute(ur5.Frames(ur5Q)));
}

TEST_F(TestProductOfExponentials, geometric_jacobian_matches_finite_difference)
{
    const Joints6 qDot{ 0.2f, -0.4f, 0.3f, 0.5f, -0.1f, 0.7f };
    constexpr float epsilon{ 1e-3f };

    const auto twist{ math::Vector6<float>{ decltype(ur5)::SpaceToGeometric(ur5.SpaceJacobian(ur5Q), ur5.Compute(ur5Q).translation) * qDot } };

    const auto ahead{ ur5.Compute(ur5Q + qDot * epsilon) };
    const auto behind{ ur5.Compute(ur5Q - qDot * epsilon) };
    ExpectNear(ur5.Compute(ur5Q).translation, Vector3{ 0.696820f, 0.400489f, 0.077764f });
    ExpectNear(twist.GetBlock<3, 1>(0, 0), Vector3{ (ahead.translation - behind.translation) * (0.5f / epsilon) });
    ExpectNear(twist.GetBlock<3, 1>(3, 0), Vector3{ Transform::LogRotation(Matrix3{ ahead.rotation * behind.rotation.Transpose() }) * (0.5f / epsilon) });
}

TEST_F(TestProductOfExponentials, frames_place_joint_origins_on_the_axes)
{
    const auto frames{ poe.Frames(Joints2{ 0.4f, -0.7f }) };

    ExpectNear(frames.jointOrigins[0], Vector3{});
    ExpectNear(frames.jointOrigins[1], Vector3{ 0.921061f, 0.389418f, 0.0f });
    ExpectNear(frames.jointAxes[0], z);
    ExpectNear(frames.jointAxes[1], z);
}

TEST_F(TestProductOfExponentials, prismatic_screw_translates)
{
    const kinematics::ProductOfExponentials<float, 1> slider{ { MakeScrew(x, Vector3{}) }, Transform::Identity() };

    const auto frames{ slider.Frames(math::Vector<float, 1>{ 0.3f }) };

    ExpectNear(slider.Compute(math::Vector<float, 1>{ 0.3f }).translation, Vector3{ 0.3f, 0.0f, 0.0f });
    EXPECT_EQ(frames.jointTypes[0], dynamics::JointType::Prismatic);
    ExpectNear(frames.jointAxes[0], x);
}
