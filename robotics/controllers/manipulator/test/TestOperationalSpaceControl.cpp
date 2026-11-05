#include "numerical/math/SE3Transform.hpp"
#include "numerical/solvers/GaussianElimination.hpp"
#include "robotics/controllers/manipulator/OperationalSpaceControl.hpp"
#include "robotics/dynamics/test_doubles/MockEulerLagrangeDynamics.hpp"
#include "robotics/kinematics/test_doubles/MockJacobianProvider.hpp"
#include <cmath>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace
{
    using ::testing::_;
    using ::testing::Return;
    using Vector2 = math::Vector<float, 2>;
    using Vector3 = math::Vector<float, 3>;
    using Matrix2 = math::SquareMatrix<float, 2>;
    using Matrix3 = math::SquareMatrix<float, 3>;
    using Transform = math::SE3Transform<float>;

    Transform At(float x, float y)
    {
        return Transform{ math::Matrix3<float>::Identity(), math::Vector3<float>{ x, y, 0.0f } };
    }

    template<std::size_t N>
    void ExpectNear(const math::Vector<float, N>& actual, const math::Vector<float, N>& expected, float tolerance = 1e-4f)
    {
        for (std::size_t i = 0; i < N; ++i)
            EXPECT_NEAR(actual[i], expected[i], tolerance);
    }

    class TestOperationalSpaceControl
        : public ::testing::Test
    {
    protected:
        void ExpectDependencies(const Matrix2& mass, const Matrix2& jacobianMatrix, const Vector2& bias, const Vector2& coriolis, const Vector2& gravity)
        {
            EXPECT_CALL(model, ComputeMassMatrix(_)).WillOnce(Return(mass));
            EXPECT_CALL(model, ComputeCoriolisTerms(_, _)).WillOnce(Return(coriolis));
            EXPECT_CALL(model, ComputeGravityTerms(_)).WillOnce(Return(gravity));
            EXPECT_CALL(jacobian, Jacobian(_)).WillOnce(Return(jacobianMatrix));
            EXPECT_CALL(jacobian, BiasAcceleration(_, _)).WillOnce(Return(bias));
            EXPECT_CALL(jacobian, ToolPose(_)).WillOnce(Return(current));
        }

        Vector2 TaskCommand() const
        {
            return Vector2{ xdDdot + kd * Vector2{ xdDot - qDot } + kp * error };
        }

        ::testing::StrictMock<dynamics::MockEulerLagrangeDynamics<float, 2>> model;
        ::testing::StrictMock<kinematics::MockJacobianProvider<float, 2, 2>> jacobian;
        Matrix2 kp{ { 100.0f, 0.0f }, { 0.0f, 100.0f } };
        Matrix2 kd{ { 20.0f, 0.0f }, { 0.0f, 20.0f } };
        controllers::OperationalSpaceControl<float, 2, 2> controller{ model, jacobian, kp, kd, 0.0f };
        Transform current{ At(0.3f, 0.2f) };
        Transform desired{ At(0.32f, 0.17f) };
        Vector2 error{ 0.02f, -0.03f };
        Vector2 q{ 0.3f, 0.2f };
        Vector2 qDot{ 0.1f, -0.2f };
        Vector2 xdDot{ 0.05f, 0.1f };
        Vector2 xdDdot{ 0.4f, -0.3f };
    };

    class TestOperationalSpaceControlRedundant
        : public ::testing::Test
    {
    protected:
        void ExpectRestState()
        {
            EXPECT_CALL(model, ComputeMassMatrix(_)).WillRepeatedly(Return(mass));
            EXPECT_CALL(model, ComputeCoriolisTerms(_, _)).WillRepeatedly(Return(Vector3{}));
            EXPECT_CALL(model, ComputeGravityTerms(_)).WillRepeatedly(Return(gravity));
            EXPECT_CALL(jacobian, Jacobian(_)).WillRepeatedly(Return(jacobianMatrix));
            EXPECT_CALL(jacobian, BiasAcceleration(_, _)).WillRepeatedly(Return(Vector2{}));
            EXPECT_CALL(jacobian, ToolPose(_)).WillRepeatedly(Return(At(0.4f, 0.1f)));
        }

        Vector3 Torque(const Vector3& tauSecondary) const
        {
            return controller.ComputeTorque(Vector3{ 0.1f, 0.2f, 0.3f }, Vector3{}, At(0.4f, 0.1f), Vector2{}, Vector2{}, tauSecondary);
        }

        ::testing::StrictMock<dynamics::MockEulerLagrangeDynamics<float, 3>> model;
        ::testing::StrictMock<kinematics::MockJacobianProvider<float, 2, 3>> jacobian;
        Matrix3 mass{ { 3.0f, 0.5f, 0.2f }, { 0.5f, 2.0f, 0.3f }, { 0.2f, 0.3f, 1.0f } };
        Vector3 gravity{ 4.0f, -2.0f, 1.5f };
        math::Matrix<float, 2, 3> jacobianMatrix{ { 1.0f, 0.5f, 0.2f }, { 0.0f, 1.0f, 0.7f } };
        controllers::OperationalSpaceControl<float, 3, 2> controller{ model, jacobian, Matrix2{ { 100.0f, 0.0f }, { 0.0f, 100.0f } }, Matrix2{ { 20.0f, 0.0f }, { 0.0f, 20.0f } }, 0.0f };
    };
}

TEST_F(TestOperationalSpaceControl, unit_inertia_gives_task_pd)
{
    ExpectDependencies(Matrix2::Identity(), Matrix2::Identity(), Vector2{}, Vector2{}, Vector2{});

    const auto tau{ controller.ComputeTorque(q, qDot, desired, xdDot, xdDdot, Vector2{}) };

    ExpectNear(tau, TaskCommand());
}

TEST_F(TestOperationalSpaceControl, task_inertia_scales_command)
{
    const Matrix2 mass{ { 2.0f, 0.0f }, { 0.0f, 3.0f } };
    ExpectDependencies(mass, Matrix2::Identity(), Vector2{}, Vector2{}, Vector2{});

    const auto tau{ controller.ComputeTorque(q, qDot, desired, xdDot, xdDdot, Vector2{}) };

    ExpectNear(tau, Vector2{ mass * TaskCommand() });
}

TEST_F(TestOperationalSpaceControl, bias_acceleration_subtracted)
{
    const Vector2 bias{ 0.7f, -0.4f };
    current = desired;
    ExpectDependencies(Matrix2::Identity(), Matrix2::Identity(), bias, Vector2{}, Vector2{});

    const auto tau{ controller.ComputeTorque(q, Vector2{}, desired, Vector2{}, Vector2{}, Vector2{}) };

    ExpectNear(tau, Vector2{ bias * -1.0f });
}

TEST_F(TestOperationalSpaceControl, coriolis_and_gravity_added_in_joint_space)
{
    const Vector2 coriolis{ 0.3f, -0.1f };
    const Vector2 gravity{ 5.0f, 2.0f };
    current = desired;
    ExpectDependencies(Matrix2::Identity(), Matrix2::Identity(), Vector2{}, coriolis, gravity);

    const auto tau{ controller.ComputeTorque(q, Vector2{}, desired, Vector2{}, Vector2{}, Vector2{}) };

    ExpectNear(tau, Vector2{ coriolis + gravity });
}

TEST_F(TestOperationalSpaceControl, jacobian_transpose_realises_wrench)
{
    current = desired;
    ExpectDependencies(Matrix2::Identity(), Matrix2{ { 1.0f, 0.0f }, { 1.0f, 2.0f } }, Vector2{}, Vector2{}, Vector2{});

    const auto tau{ controller.ComputeTorque(q, Vector2{}, desired, Vector2{}, Vector2{ 1.0f, 2.0f }, Vector2{}) };

    ExpectNear(tau, Vector2{ 1.0f, 0.5f });
}

TEST_F(TestOperationalSpaceControlRedundant, redundant_at_rest_holds_gravity_exactly)
{
    ExpectRestState();

    ExpectNear(Torque(Vector3{}), gravity);
}

TEST_F(TestOperationalSpaceControlRedundant, secondary_torque_invisible_to_task)
{
    ExpectRestState();
    const Vector3 secondary{ 1.0f, -2.0f, 0.5f };

    const Vector3 difference{ Torque(secondary) - Torque(Vector3{}) };

    ExpectNear(difference, Vector3{ 0.38863f, -1.32780f, 1.06224f });
    ExpectNear(math::Vector<float, 2>{ jacobianMatrix * solvers::SolveSystem(mass, difference) }, Vector2{}, 1e-5f);
}

TEST_F(TestOperationalSpaceControl, singular_jacobian_uses_damping)
{
    const controllers::OperationalSpaceControl<float, 2, 2> damped{ model, jacobian, kp, kd, 0.1f };
    EXPECT_CALL(model, ComputeMassMatrix(_)).WillOnce(Return(Matrix2::Identity()));
    EXPECT_CALL(model, ComputeCoriolisTerms(_, _)).WillOnce(Return(Vector2{}));
    EXPECT_CALL(model, ComputeGravityTerms(_)).WillOnce(Return(Vector2{}));
    EXPECT_CALL(jacobian, Jacobian(_)).WillOnce(Return(Matrix2{ { 1.0f, 1.0f }, { 1.0f, 1.0f } }));
    EXPECT_CALL(jacobian, BiasAcceleration(_, _)).WillOnce(Return(Vector2{}));
    EXPECT_CALL(jacobian, ToolPose(_)).WillOnce(Return(current));

    const auto tau{ damped.ComputeTorque(q, qDot, desired, xdDot, xdDdot, Vector2{}) };

    EXPECT_TRUE(std::isfinite(tau[0]));
    EXPECT_TRUE(std::isfinite(tau[1]));
}

TEST_F(TestOperationalSpaceControl, dependencies_queried_once)
{
    ExpectDependencies(Matrix2::Identity(), Matrix2::Identity(), Vector2{}, Vector2{}, Vector2{});

    static_cast<void>(controller.ComputeTorque(q, qDot, desired, xdDot, xdDdot, Vector2{}));
}
