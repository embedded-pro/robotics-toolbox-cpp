#include "numerical/math/Tolerance.hpp"
#include "robotics/kinematics/ChainTaskJacobian.hpp"
#include "robotics/kinematics/ManipulabilityIndex.hpp"
#include "robotics/kinematics/test_doubles/MockJacobianProvider.hpp"
#include <cmath>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <numbers>

namespace
{
    using Vector3 = math::Vector3<float>;
    using Matrix3 = math::Matrix3<float>;
    using Link = dynamics::RevoluteJointLink<float>;
    using Transform = math::SE3Transform<float>;
    using Planar = math::Vector<float, 2>;

    constexpr float halfPi{ std::numbers::pi_v<float> / 2.0f };
    const Vector3 zAxis{ 0.0f, 0.0f, 1.0f };

    Link MakeLink(const Vector3& parentToJoint)
    {
        return Link{ 1.0f, Matrix3::Identity(), zAxis, parentToJoint, Vector3{} };
    }

    class TestManipulabilityIndex
        : public ::testing::Test
    {
    protected:
        std::array<Link, 2> planar{ MakeLink(Vector3{}), MakeLink(Vector3{ 1.0f, 0.0f, 0.0f }) };
        Transform tool{ Matrix3::Identity(), Vector3{ 1.0f, 0.0f, 0.0f } };
        kinematics::ChainTaskJacobian<float, 3, 2> position{ planar, tool };
        kinematics::ChainTaskJacobian<float, 6, 2> full{ planar, tool };
        kinematics::ManipulabilityIndex<float, 3, 2> w{ position };
        testing::StrictMock<kinematics::MockJacobianProvider<float, 2, 3>> planarTask;
        kinematics::ManipulabilityIndex<float, 2, 3> wRedundant{ planarTask };
    };
}

TEST_F(TestManipulabilityIndex, position_manipulability_is_abs_sin_elbow)
{
    for (const float elbow : { 0.3f, 1.2f, 2.5f })
        EXPECT_NEAR(w.Compute(Planar{ 0.0f, elbow }), std::abs(std::sin(elbow)), math::Tolerance<float>());
}

TEST_F(TestManipulabilityIndex, right_angle_elbow_is_most_dexterous)
{
    const float best{ w.Compute(Planar{ 0.0f, halfPi }) };

    EXPECT_NEAR(best, 1.0f, math::Tolerance<float>());
    EXPECT_GT(best, w.Compute(Planar{ 0.0f, halfPi + 0.1f }));
    EXPECT_GT(best, w.Compute(Planar{ 0.0f, halfPi - 0.1f }));
}

TEST_F(TestManipulabilityIndex, stretched_and_folded_arm_are_singular)
{
    EXPECT_NEAR(w.Compute(Planar{ 0.0f, 0.0f }), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(w.Compute(Planar{ 0.0f, std::numbers::pi_v<float> }), 0.0f, math::Tolerance<float>());
}

TEST_F(TestManipulabilityIndex, redundant_task_uses_j_jt_branch)
{
    const math::Vector<float, 3> q{ 0.0f, halfPi, halfPi };
    EXPECT_CALL(planarTask, Jacobian(testing::_)).Times(2).WillRepeatedly(testing::Return(math::Matrix<float, 2, 3>{ { -1.0f, -1.0f, 0.0f }, { 0.0f, -1.0f, -1.0f } }));

    const float value{ wRedundant.Compute(q) };
    const auto axes{ wRedundant.EllipsoidAxes(q) };

    EXPECT_NEAR(value, std::sqrt(3.0f), math::Tolerance<float>());
    EXPECT_NEAR(axes[0], std::sqrt(3.0f), math::Tolerance<float>());
    EXPECT_NEAR(axes[1], 1.0f, math::Tolerance<float>());
}

TEST_F(TestManipulabilityIndex, six_row_task_measures_units_separately)
{
    const kinematics::ManipulabilityIndex<float, 6, 2> sixRow{ full };

    EXPECT_NEAR(sixRow.Compute(Planar{ 0.0f, 1.2f }), 0.932039f, math::Tolerance<float>());
    EXPECT_NEAR(sixRow.ComputeRotational(Planar{ 0.0f, 1.2f }), 0.0f, math::Tolerance<float>());
}

TEST_F(TestManipulabilityIndex, ellipsoid_axes_are_sorted_singular_values)
{
    const auto axes{ w.EllipsoidAxes(Planar{ 0.0f, halfPi }) };

    EXPECT_NEAR(axes[0], 1.618034f, math::Tolerance<float>());
    EXPECT_NEAR(axes[1], 0.618034f, math::Tolerance<float>());
    EXPECT_NEAR(axes[0] * axes[1], w.Compute(Planar{ 0.0f, halfPi }), math::Tolerance<float>());
}

TEST_F(TestManipulabilityIndex, condition_number_grows_near_singularity)
{
    EXPECT_NEAR(w.ConditionNumber(Planar{ 0.0f, halfPi }), 2.618034f, 2.618034f * 1e-3f);
    EXPECT_NEAR(w.ConditionNumber(Planar{ 0.0f, 0.1f }), 49.963f, 49.963f * 1e-3f);
    EXPECT_NEAR(w.ConditionNumber(Planar{ 0.0f, 0.01f }), 499.996f, 499.996f * 1e-3f);
}

TEST_F(TestManipulabilityIndex, exactly_singular_condition_number_is_infinite)
{
    const float condition{ w.ConditionNumber(Planar{ 0.0f, 0.0f }) };

    EXPECT_TRUE(std::isinf(condition) && condition > 0.0f);
}

TEST_F(TestManipulabilityIndex, near_singular_flag_trips)
{
    EXPECT_TRUE(w.NearSingular(Planar{ 0.0f, 0.01f }, 0.05f));
    EXPECT_FALSE(w.NearSingular(Planar{ 0.0f, halfPi }, 0.05f));
}
