#include "robotics/kinematics/ForwardKinematics.hpp"
#include "numerical/math/Tolerance.hpp"
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

    Vector3 Along(const Vector3& axis, float length)
    {
        return axis * length;
    }

    class TestForwardKinematics
        : public ::testing::Test
    {
    protected:
        void ExpectNear(const Vector3& actual, const Vector3& expected) const
        {
            EXPECT_NEAR(actual.at(0, 0), expected.at(0, 0), math::Tolerance<float>());
            EXPECT_NEAR(actual.at(1, 0), expected.at(1, 0), math::Tolerance<float>());
            EXPECT_NEAR(actual.at(2, 0), expected.at(2, 0), math::Tolerance<float>());
        }
    };
}

TEST_F(TestForwardKinematics, single_link_at_zero_angle_places_tool_along_offset)
{
    std::array<Link, 1> links{ MakeLink(zAxis, Vector3{}, Along(xAxis, 0.5f)) };
    kinematics::ForwardKinematics<float, 1> fk{ Along(xAxis, 1.0f) };

    auto positions = fk.Compute(links, math::Vector<float, 1>{ 0.0f });

    ExpectNear(positions[0], Vector3{});
    ExpectNear(positions[1], Vector3{ 1.0f, 0.0f, 0.0f });
}

TEST_F(TestForwardKinematics, z_axis_rotation_follows_right_hand_rule)
{
    std::array<Link, 1> links{ MakeLink(zAxis, Vector3{}, Along(xAxis, 0.5f)) };
    kinematics::ForwardKinematics<float, 1> fk{ Along(xAxis, 1.0f) };

    auto positions = fk.Compute(links, math::Vector<float, 1>{ pi / 2.0f });

    ExpectNear(positions[1], Vector3{ 0.0f, 1.0f, 0.0f });
}

TEST_F(TestForwardKinematics, y_axis_rotation_follows_right_hand_rule)
{
    std::array<Link, 1> links{ MakeLink(yAxis, Vector3{}, Along(xAxis, 0.5f)) };
    kinematics::ForwardKinematics<float, 1> fk{ Along(xAxis, 1.0f) };

    auto positions = fk.Compute(links, math::Vector<float, 1>{ pi / 2.0f });

    ExpectNear(positions[1], Vector3{ 0.0f, 0.0f, -1.0f });
}

TEST_F(TestForwardKinematics, two_link_planar_elbow_at_ninety_degrees)
{
    std::array<Link, 2> links{
        MakeLink(zAxis, Vector3{}, Along(xAxis, 0.5f)),
        MakeLink(zAxis, Along(xAxis, 1.0f), Along(xAxis, 0.4f))
    };
    kinematics::ForwardKinematics<float, 2> fk{ Along(xAxis, 0.8f) };

    auto positions = fk.Compute(links, math::Vector<float, 2>{ 0.0f, pi / 2.0f });

    ExpectNear(positions[1], Vector3{ 1.0f, 0.0f, 0.0f });
    ExpectNear(positions[2], Vector3{ 1.0f, 0.8f, 0.0f });
}

TEST_F(TestForwardKinematics, two_link_planar_folded_back)
{
    std::array<Link, 2> links{
        MakeLink(zAxis, Vector3{}, Along(xAxis, 0.5f)),
        MakeLink(zAxis, Along(xAxis, 1.0f), Along(xAxis, 0.4f))
    };
    kinematics::ForwardKinematics<float, 2> fk{ Along(xAxis, 0.8f) };

    auto positions = fk.Compute(links, math::Vector<float, 2>{ 0.0f, pi });

    ExpectNear(positions[2], Vector3{ 0.2f, 0.0f, 0.0f });
}

TEST_F(TestForwardKinematics, base_offset_translates_the_whole_chain)
{
    std::array<Link, 2> links{
        MakeLink(zAxis, Along(zAxis, 0.3f), Along(xAxis, 0.5f)),
        MakeLink(zAxis, Along(xAxis, 1.0f), Along(xAxis, 0.4f))
    };
    kinematics::ForwardKinematics<float, 2> fk{ Along(xAxis, 0.8f) };

    auto positions = fk.Compute(links, math::Vector<float, 2>{ pi / 2.0f, 0.0f });

    ExpectNear(positions[0], Vector3{ 0.0f, 0.0f, 0.3f });
    ExpectNear(positions[1], Vector3{ 0.0f, 1.0f, 0.3f });
    ExpectNear(positions[2], Vector3{ 0.0f, 1.8f, 0.3f });
}

TEST_F(TestForwardKinematics, tool_point_is_independent_of_center_of_mass)
{
    std::array<Link, 1> links{ MakeLink(zAxis, Vector3{}, Along(xAxis, 0.1f)) };
    kinematics::ForwardKinematics<float, 1> fk{ Along(xAxis, 1.0f) };

    auto positions = fk.Compute(links, math::Vector<float, 1>{ 0.0f });

    ExpectNear(positions[1], Vector3{ 1.0f, 0.0f, 0.0f });
}

TEST_F(TestForwardKinematics, tool_offset_is_expressed_in_last_link_frame)
{
    std::array<Link, 1> links{ MakeLink(zAxis, Vector3{}, Along(xAxis, 0.5f)) };
    kinematics::ForwardKinematics<float, 1> fk{ Vector3{ 1.0f, 0.2f, 0.0f } };

    auto positions = fk.Compute(links, math::Vector<float, 1>{ pi / 2.0f });

    ExpectNear(positions[1], Vector3{ -0.2f, 1.0f, 0.0f });
}

TEST_F(TestForwardKinematics, mixed_axes_compose_in_joint_order)
{
    std::array<Link, 2> links{
        MakeLink(zAxis, Vector3{}, Along(xAxis, 0.25f)),
        MakeLink(yAxis, Along(xAxis, 0.5f), Along(xAxis, 0.2f))
    };
    kinematics::ForwardKinematics<float, 2> fk{ Along(xAxis, 0.4f) };

    auto positions = fk.Compute(links, math::Vector<float, 2>{ pi / 2.0f, pi / 2.0f });

    ExpectNear(positions[1], Vector3{ 0.0f, 0.5f, 0.0f });
    ExpectNear(positions[2], Vector3{ 0.0f, 0.5f, -0.4f });
}

TEST_F(TestForwardKinematics, spatial_three_link_vertical_base)
{
    std::array<Link, 3> links{
        MakeLink(zAxis, Vector3{}, Along(zAxis, 0.25f)),
        MakeLink(yAxis, Along(zAxis, 0.5f), Along(xAxis, 0.2f)),
        MakeLink(yAxis, Along(xAxis, 0.4f), Along(xAxis, 0.15f))
    };
    kinematics::ForwardKinematics<float, 3> fk{ Along(xAxis, 0.3f) };

    auto positions = fk.Compute(links, math::Vector<float, 3>{ pi / 2.0f, -pi / 2.0f, 0.0f });

    ExpectNear(positions[1], Vector3{ 0.0f, 0.0f, 0.5f });
    ExpectNear(positions[2], Vector3{ 0.0f, 0.0f, 0.9f });
    ExpectNear(positions[3], Vector3{ 0.0f, 0.0f, 1.2f });
}
