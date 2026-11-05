#include "numerical/math/SE3Transform.hpp"
#include "robotics/controllers/manipulator/HybridPositionForceControl.hpp"
#include "robotics/dynamics/test_doubles/MockEulerLagrangeDynamics.hpp"
#include "robotics/kinematics/test_doubles/MockJacobianProvider.hpp"
#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace
{
    using ::testing::_;
    using ::testing::Return;
    using Vector2 = math::Vector<float, 2>;
    using Matrix2 = math::SquareMatrix<float, 2>;
    using Transform = math::SE3Transform<float>;
    using Controller = controllers::HybridPositionForceControl<float, 2, 2>;

    Transform At(float x, float y)
    {
        return Transform{ math::Matrix3<float>::Identity(), math::Vector3<float>{ x, y, 0.0f } };
    }

    Matrix2 Diagonal(float first, float second)
    {
        return Matrix2{ { first, 0.0f }, { 0.0f, second } };
    }

    class TestHybridPositionForceControl
        : public ::testing::Test
    {
    protected:
        void ExpectNear(const Vector2& actual, const Vector2& expected) const
        {
            for (std::size_t i = 0; i < 2; ++i)
                EXPECT_NEAR(actual[i], expected[i], 1e-4f);
        }

        void ExpectDependencies(const Matrix2& jacobianMatrix, const Transform& current, std::size_t calls = 1, const Vector2& coriolis = Vector2{}, const Vector2& gravity = Vector2{})
        {
            EXPECT_CALL(jacobian, Jacobian(_)).Times(static_cast<int>(calls)).WillRepeatedly(Return(jacobianMatrix));
            EXPECT_CALL(jacobian, ToolPose(_)).Times(static_cast<int>(calls)).WillRepeatedly(Return(current));
            EXPECT_CALL(model, ComputeCoriolisTerms(_, _)).Times(static_cast<int>(calls)).WillRepeatedly(Return(coriolis));
            EXPECT_CALL(model, ComputeGravityTerms(_)).Times(static_cast<int>(calls)).WillRepeatedly(Return(gravity));
        }

        Vector2 Step(Controller& target, const Vector2& qDot, const Vector2& xdDot, const Vector2& fMeasured, const Vector2& fd)
        {
            return target.ComputeTorque(q, qDot, desired, xdDot, fMeasured, fd);
        }

        ::testing::StrictMock<dynamics::MockEulerLagrangeDynamics<float, 2>> model;
        ::testing::StrictMock<kinematics::MockJacobianProvider<float, 2, 2>> jacobian;
        Controller::Gains gains{ Diagonal(100.0f, 100.0f), Diagonal(20.0f, 20.0f), Diagonal(2.0f, 2.0f), Diagonal(5.0f, 5.0f), Diagonal(3.0f, 3.0f), Vector2{ 1.0f, 1.0f } };
        float dt{ 0.001f };
        Controller controller{ model, jacobian, Matrix2::Identity(), Diagonal(1.0f, 0.0f), gains, dt };
        Vector2 q{ 0.3f, 0.2f };
        Transform desired{ At(0.3f, 0.2f) };
    };
}

TEST_F(TestHybridPositionForceControl, motion_axis_does_position_pd)
{
    ExpectDependencies(Matrix2::Identity(), At(0.25f, 0.2f));

    const auto tau{ Step(controller, Vector2{ 0.1f, 0.0f }, Vector2{ 0.3f, 0.0f }, Vector2{}, Vector2{}) };

    ExpectNear(tau, Vector2{ 100.0f * 0.05f + 20.0f * 0.2f, 0.0f });
}

TEST_F(TestHybridPositionForceControl, force_axis_does_damped_force_pi)
{
    ExpectDependencies(Matrix2::Identity(), desired);

    const auto tau{ Step(controller, Vector2{ 0.0f, 0.4f }, Vector2{}, Vector2{ 0.0f, 3.0f }, Vector2{ 0.0f, 5.0f }) };

    EXPECT_NEAR(tau[1], 5.0f + 2.0f * 2.0f + 5.0f * 2.0f * dt - 3.0f * 0.4f, 1e-4f);
}

TEST_F(TestHybridPositionForceControl, selection_masks_cross_terms)
{
    ExpectDependencies(Matrix2::Identity(), At(0.3f, 0.15f));

    const auto tau{ Step(controller, Vector2{}, Vector2{}, Vector2{ 2.0f, 0.0f }, Vector2{}) };

    ExpectNear(tau, Vector2{});
}

TEST_F(TestHybridPositionForceControl, constraint_rotation_maps_axes)
{
    Controller rotated{ model, jacobian, Matrix2{ { 0.0f, -1.0f }, { 1.0f, 0.0f } }, Diagonal(1.0f, 0.0f), gains, dt };
    desired = At(0.02f, 0.1f);
    ExpectDependencies(Matrix2::Identity(), At(0.0f, 0.0f));

    const auto tau{ Step(rotated, Vector2{}, Vector2{}, Vector2{ -3.0f, 0.0f }, Vector2{ -5.0f, 0.0f }) };

    ExpectNear(tau, Vector2{ -9.01f, 10.0f });
}

TEST_F(TestHybridPositionForceControl, force_integral_accumulates)
{
    ExpectDependencies(Matrix2::Identity(), desired, 100);

    Vector2 tau{};
    for (std::size_t k = 0; k < 100; ++k)
        tau = Step(controller, Vector2{}, Vector2{}, Vector2{ 0.0f, 3.0f }, Vector2{ 0.0f, 5.0f });

    EXPECT_NEAR(tau[1] - (5.0f + 2.0f * 2.0f), 5.0f * 2.0f * 100.0f * dt, 1e-4f);
}

TEST_F(TestHybridPositionForceControl, force_integral_clamped)
{
    ExpectDependencies(Matrix2::Identity(), desired, 1000);

    Vector2 tau{};
    for (std::size_t k = 0; k < 1000; ++k)
        tau = Step(controller, Vector2{}, Vector2{}, Vector2{}, Vector2{ 0.0f, 10.0f });

    EXPECT_NEAR(tau[1], 10.0f + 2.0f * 10.0f + 5.0f * 1.0f, 1e-4f);
}

TEST_F(TestHybridPositionForceControl, reset_clears_force_integral)
{
    ExpectDependencies(Matrix2::Identity(), desired, 101);
    for (std::size_t k = 0; k < 100; ++k)
        static_cast<void>(Step(controller, Vector2{}, Vector2{}, Vector2{ 0.0f, 3.0f }, Vector2{ 0.0f, 5.0f }));

    controller.Reset();
    const auto tau{ Step(controller, Vector2{}, Vector2{}, Vector2{ 0.0f, 3.0f }, Vector2{ 0.0f, 5.0f }) };

    EXPECT_NEAR(tau[1], 5.0f + 2.0f * 2.0f + 5.0f * 2.0f * dt, 1e-4f);
}

TEST_F(TestHybridPositionForceControl, jacobian_transpose_maps_wrench)
{
    ExpectDependencies(Matrix2{ { 1.0f, 0.5f }, { 0.0f, 2.0f } }, At(0.25f, 0.2f));

    const auto tau{ Step(controller, Vector2{}, Vector2{}, Vector2{ 0.0f, 5.0f }, Vector2{ 0.0f, 5.0f }) };

    ExpectNear(tau, Vector2{ 5.0f, 12.5f });
}

TEST_F(TestHybridPositionForceControl, gravity_and_coriolis_added)
{
    const Vector2 coriolis{ 0.3f, -0.2f };
    const Vector2 gravity{ 4.0f, 1.5f };
    ExpectDependencies(Matrix2::Identity(), desired, 1, coriolis, gravity);

    const auto tau{ Step(controller, Vector2{}, Vector2{}, Vector2{}, Vector2{}) };

    ExpectNear(tau, Vector2{ coriolis + gravity });
}

TEST_F(TestHybridPositionForceControl, dependencies_queried_once)
{
    ExpectDependencies(Matrix2::Identity(), desired);

    static_cast<void>(Step(controller, Vector2{ 0.1f, 0.2f }, Vector2{}, Vector2{ 1.0f, 2.0f }, Vector2{ 0.0f, 3.0f }));
}
