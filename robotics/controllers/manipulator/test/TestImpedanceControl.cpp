#include "numerical/math/SE3Transform.hpp"
#include "robotics/controllers/manipulator/ImpedanceControl.hpp"
#include "robotics/dynamics/test_doubles/MockEulerLagrangeDynamics.hpp"
#include "robotics/kinematics/test_doubles/MockJacobianProvider.hpp"
#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace
{
    using ::testing::_;
    using ::testing::Invoke;
    using ::testing::Return;
    using Vector2 = math::Vector<float, 2>;
    using Vector3 = math::Vector3<float>;
    using Matrix2 = math::SquareMatrix<float, 2>;
    using Vector6 = math::Vector<float, 6>;
    using Matrix6 = math::SquareMatrix<float, 6>;
    using Transform = math::SE3Transform<float>;
    using Controller = controllers::ImpedanceControl<float, 2, 2>;

    constexpr float dt{ 1e-3f };

    Transform At(float x, float y)
    {
        return Transform{ math::Matrix3<float>::Identity(), Vector3{ x, y, 0.0f } };
    }

    Matrix2 Diagonal(float first, float second)
    {
        return Matrix2{ { first, 0.0f }, { 0.0f, second } };
    }

    Matrix6 Diagonal6(float linear, float angular)
    {
        Matrix6 diagonal{};
        for (std::size_t i = 0; i < 6; ++i)
            diagonal.at(i, i) = i < 3 ? linear : angular;
        return diagonal;
    }

    class TestImpedanceControl
        : public ::testing::Test
    {
    protected:
        void ExpectNear(const Vector2& actual, const Vector2& expected, float tolerance = 1e-4f) const
        {
            for (std::size_t i = 0; i < 2; ++i)
                EXPECT_NEAR(actual[i], expected[i], tolerance);
        }

        void ExpectArm(const Matrix2& jacobianMatrix)
        {
            EXPECT_CALL(jacobian, Jacobian(_)).WillOnce(Return(jacobianMatrix));
            EXPECT_CALL(jacobian, ToolPose(_)).WillOnce(Return(current));
        }

        void ExpectModel(const Matrix2& mass)
        {
            EXPECT_CALL(model, ComputeMassMatrix(_)).WillOnce(Return(mass));
            EXPECT_CALL(model, ComputeCoriolisTerms(_, _)).WillOnce(Return(coriolis));
            EXPECT_CALL(model, ComputeGravityTerms(_)).WillOnce(Return(gravity));
            EXPECT_CALL(jacobian, BiasAcceleration(_, _)).WillOnce(Return(bias));
        }

        void ExpectSimulatedPlant()
        {
            EXPECT_CALL(jacobian, Jacobian(_)).WillRepeatedly(Return(Matrix2::Identity()));
            EXPECT_CALL(jacobian, ToolPose(_)).WillRepeatedly(Invoke([](const Vector2& position)
                {
                    return At(position[0], position[1]);
                }));
            EXPECT_CALL(model, ComputeGravityTerms(_)).WillRepeatedly(Return(plantGravity));
        }

        void StepPlant(const Vector2& tau, const Vector2& fExternal)
        {
            const Vector2 net{ tau + fExternal - plantGravity };
            qDot = qDot + Vector2{ net[0] / plantMass.at(0, 0), net[1] / plantMass.at(1, 1) } * dt;
            q = q + qDot * dt;
        }

        ::testing::StrictMock<dynamics::MockEulerLagrangeDynamics<float, 2>> model;
        ::testing::StrictMock<kinematics::MockJacobianProvider<float, 2, 2>> jacobian;
        Matrix2 stiffness{ Diagonal(500.0f, 300.0f) };
        Matrix2 damping{ Diagonal(40.0f, 30.0f) };
        Matrix2 targetInertia{ Diagonal(2.0f, 1.5f) };
        Controller controller{ model, jacobian, stiffness, damping, targetInertia };
        Transform current{ At(0.3f, 0.2f) };
        Transform desired{ At(0.35f, 0.15f) };
        Vector2 error{ 0.05f, -0.05f };
        Vector2 q{ 0.3f, 0.2f };
        Vector2 qDot{ 0.2f, -0.1f };
        Vector2 xdDot{ 0.1f, 0.05f };
        Vector2 xdDdot{ 0.5f, -0.4f };
        Vector2 fExternal{ 3.0f, -4.0f };
        Vector2 gravity{ 1.5f, -2.0f };
        Vector2 coriolis{ 0.4f, -0.3f };
        Vector2 bias{ 0.2f, 0.1f };
        Matrix2 plantMass{ Diagonal(1.0f, 2.0f) };
        Vector2 plantGravity{ 0.0f, 19.62f };
    };

    class TestImpedanceControlPose
        : public ::testing::Test
    {
    protected:
        ::testing::StrictMock<dynamics::MockEulerLagrangeDynamics<float, 6>> model;
        ::testing::StrictMock<kinematics::MockJacobianProvider<float, 6, 6>> jacobian;
        Matrix6 stiffness{ Diagonal6(500.0f, 20.0f) };
        controllers::ImpedanceControl<float, 6, 6> controller{ model, jacobian, stiffness, Diagonal6(40.0f, 2.0f), Matrix6::Identity() };
    };
}

TEST_F(TestImpedanceControl, compliant_law_with_identity_jacobian)
{
    ExpectArm(Matrix2::Identity());
    EXPECT_CALL(model, ComputeGravityTerms(_)).WillOnce(Return(gravity));

    const auto tau{ controller.ComputeTorque(q, qDot, desired, xdDot) };

    ExpectNear(tau, Vector2{ stiffness * error + damping * Vector2{ xdDot - qDot } + gravity });
}

TEST_F(TestImpedanceControl, jacobian_transpose_maps_wrench)
{
    const Matrix2 skewed{ { 1.0f, 0.5f }, { 0.0f, 2.0f } };
    ExpectArm(skewed);
    EXPECT_CALL(model, ComputeGravityTerms(_)).WillOnce(Return(Vector2{}));

    const auto tau{ controller.ComputeTorque(q, Vector2{}, desired, Vector2{}) };

    ExpectNear(tau, Vector2{ 25.0f, -17.5f });
}

TEST_F(TestImpedanceControl, compliant_law_queries_only_gravity)
{
    ExpectArm(Matrix2::Identity());
    EXPECT_CALL(model, ComputeGravityTerms(_)).WillOnce(Return(gravity));

    static_cast<void>(controller.ComputeTorque(q, qDot, desired, xdDot));
}

TEST_F(TestImpedanceControl, static_deflection_equals_compliance)
{
    ExpectSimulatedPlant();
    const auto target{ At(q[0], q[1]) };
    qDot = Vector2{};

    for (std::size_t k = 0; k < 5000; ++k)
        StepPlant(controller.ComputeTorque(q, qDot, target, Vector2{}), fExternal);

    ExpectNear(Vector2{ q - target.translation.GetBlock<2, 1>(0, 0) }, Vector2{ 0.006f, -0.0133333f }, 1e-5f);
    ExpectNear(qDot, Vector2{}, 1e-4f);
}

TEST_F(TestImpedanceControlPose, pose_error_uses_rotation_vector)
{
    math::Vector6<float> rotation{};
    rotation[5] = 0.2f;
    EXPECT_CALL(jacobian, Jacobian(_)).WillOnce(Return(Matrix6::Identity()));
    EXPECT_CALL(jacobian, ToolPose(_)).WillOnce(Return(Transform{ Transform::Exp(rotation, 1.0f).rotation, Vector3{} }));
    EXPECT_CALL(model, ComputeGravityTerms(_)).WillOnce(Return(Vector6{}));

    const auto tau{ controller.ComputeTorque(Vector6{}, Vector6{}, Transform::Identity(), Vector6{}) };

    for (std::size_t i = 0; i < 5; ++i)
        EXPECT_NEAR(tau[i], 0.0f, 1e-5f);
    EXPECT_NEAR(tau[5], 20.0f * -0.2f, 1e-5f);
}

TEST_F(TestImpedanceControl, inertia_shaping_with_unit_model)
{
    ExpectArm(Matrix2::Identity());
    EXPECT_CALL(model, ComputeMassMatrix(_)).WillOnce(Return(Matrix2::Identity()));
    EXPECT_CALL(model, ComputeCoriolisTerms(_, _)).WillOnce(Return(Vector2{}));
    EXPECT_CALL(model, ComputeGravityTerms(_)).WillOnce(Return(Vector2{}));
    EXPECT_CALL(jacobian, BiasAcceleration(_, _)).WillOnce(Return(Vector2{}));

    const auto tau{ controller.ComputeTorqueWithInertiaShaping(q, qDot, desired, xdDot, xdDdot, fExternal) };

    const Vector2 shaped{ damping * Vector2{ xdDot - qDot } + stiffness * error + fExternal };
    const Vector2 acceleration{ xdDdot + Vector2{ shaped[0] / 2.0f, shaped[1] / 1.5f } };
    ExpectNear(tau, Vector2{ acceleration - fExternal });
}

TEST_F(TestImpedanceControl, inertia_shaping_uses_task_inertia_bias_and_model)
{
    const auto taskInertia{ Diagonal(2.0f, 3.0f) };
    ExpectArm(Matrix2::Identity());
    ExpectModel(taskInertia);

    const auto tau{ controller.ComputeTorqueWithInertiaShaping(q, qDot, desired, xdDot, xdDdot, fExternal) };

    const Vector2 shaped{ damping * Vector2{ xdDot - qDot } + stiffness * error + fExternal };
    const Vector2 acceleration{ xdDdot + Vector2{ shaped[0] / 2.0f, shaped[1] / 1.5f } };
    ExpectNear(tau, Vector2{ taskInertia * Vector2{ acceleration - bias } - fExternal + coriolis + gravity });
}

TEST_F(TestImpedanceControl, matched_inertia_reduces_to_compliant_law)
{
    const auto taskInertia{ Diagonal(2.0f, 3.0f) };
    const Controller matched{ model, jacobian, stiffness, damping, taskInertia };
    EXPECT_CALL(jacobian, Jacobian(_)).WillRepeatedly(Return(Matrix2::Identity()));
    EXPECT_CALL(jacobian, ToolPose(_)).WillRepeatedly(Return(current));
    EXPECT_CALL(jacobian, BiasAcceleration(_, _)).WillRepeatedly(Return(bias));
    EXPECT_CALL(model, ComputeMassMatrix(_)).WillRepeatedly(Return(taskInertia));
    EXPECT_CALL(model, ComputeCoriolisTerms(_, _)).WillRepeatedly(Return(coriolis));
    EXPECT_CALL(model, ComputeGravityTerms(_)).WillRepeatedly(Return(gravity));

    const Vector2 expected{ matched.ComputeTorque(q, qDot, desired, xdDot) + taskInertia * Vector2{ xdDdot - bias } + coriolis };

    ExpectNear(matched.ComputeTorqueWithInertiaShaping(q, qDot, desired, xdDot, xdDdot, fExternal), expected);
    ExpectNear(matched.ComputeTorqueWithInertiaShaping(q, qDot, desired, xdDot, xdDdot, Vector2{ -7.0f, 2.5f }), expected);
}

TEST_F(TestImpedanceControl, rendered_impedance_matches_target)
{
    ExpectSimulatedPlant();
    EXPECT_CALL(jacobian, BiasAcceleration(_, _)).WillRepeatedly(Return(Vector2{}));
    EXPECT_CALL(model, ComputeMassMatrix(_)).WillRepeatedly(Return(plantMass));
    EXPECT_CALL(model, ComputeCoriolisTerms(_, _)).WillRepeatedly(Return(Vector2{}));
    const auto target{ At(q[0], q[1]) };
    const Vector2 origin{ q };
    qDot = Vector2{};
    Vector2 offset{};
    Vector2 offsetRate{};

    for (std::size_t k = 0; k < 2000; ++k)
    {
        StepPlant(controller.ComputeTorqueWithInertiaShaping(q, qDot, target, Vector2{}, Vector2{}, fExternal), fExternal);

        const Vector2 targetForce{ fExternal - damping * offsetRate - stiffness * offset };
        offsetRate = offsetRate + Vector2{ targetForce[0] / 2.0f, targetForce[1] / 1.5f } * dt;
        offset = offset + offsetRate * dt;

        ExpectNear(Vector2{ q - origin }, offset, 1e-5f);
    }
}
