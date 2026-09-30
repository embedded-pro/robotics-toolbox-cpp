#include "robotics/dynamics/NewtonEulerBody.hpp"
#include "robotics/dynamics/NewtonEulerSolver.hpp"
#include <cmath>
#include <gtest/gtest.h>

namespace
{
    class UniformSphere : public dynamics::NewtonEulerBody<float>
    {
    public:
        static constexpr float mass = 2.0f;
        static constexpr float radius = 0.5f;

        float ComputeMass() const override
        {
            return mass;
        }

        InertiaMatrix ComputeInertia() const override
        {
            float i = 0.4f * mass * radius * radius;
            return InertiaMatrix{
                { i, 0.0f, 0.0f },
                { 0.0f, i, 0.0f },
                { 0.0f, 0.0f, i }
            };
        }
    };

    class AsymmetricBody : public dynamics::NewtonEulerBody<float>
    {
    public:
        static constexpr float mass = 3.0f;

        float ComputeMass() const override
        {
            return mass;
        }

        InertiaMatrix ComputeInertia() const override
        {
            return InertiaMatrix{
                { 1.0f, 0.0f, 0.0f },
                { 0.0f, 2.0f, 0.0f },
                { 0.0f, 0.0f, 3.0f }
            };
        }
    };

    class TestNewtonEulerSolver : public ::testing::Test
    {
    protected:
        dynamics::NewtonEulerSolver<float> solver;
        UniformSphere sphere;
        AsymmetricBody asymmetricBody;

        math::Vector<float, 3> zero3{};
    };
}

TEST_F(TestNewtonEulerSolver, forward_dynamics_pure_translation_no_rotation)
{
    math::Vector<float, 3> force{ 6.0f, 0.0f, 0.0f };
    math::Vector<float, 3> torque{};

    auto result = solver.ForwardDynamics(sphere, force, torque, zero3, zero3);

    EXPECT_NEAR(result.linear.at(0, 0), 3.0f, 1e-4f);
    EXPECT_NEAR(result.linear.at(1, 0), 0.0f, 1e-4f);
    EXPECT_NEAR(result.linear.at(2, 0), 0.0f, 1e-4f);

    EXPECT_NEAR(result.angular.at(0, 0), 0.0f, 1e-4f);
    EXPECT_NEAR(result.angular.at(1, 0), 0.0f, 1e-4f);
    EXPECT_NEAR(result.angular.at(2, 0), 0.0f, 1e-4f);
}

TEST_F(TestNewtonEulerSolver, forward_dynamics_pure_rotation_no_translation)
{
    math::Vector<float, 3> force{};
    math::Vector<float, 3> torque{ 0.0f, 0.0f, 1.0f };

    auto result = solver.ForwardDynamics(sphere, force, torque, zero3, zero3);

    float inertia = 0.4f * UniformSphere::mass * UniformSphere::radius * UniformSphere::radius;
    EXPECT_NEAR(result.angular.at(2, 0), 1.0f / inertia, 1e-3f);
    EXPECT_NEAR(result.linear.at(0, 0), 0.0f, 1e-4f);
    EXPECT_NEAR(result.linear.at(1, 0), 0.0f, 1e-4f);
    EXPECT_NEAR(result.linear.at(2, 0), 0.0f, 1e-4f);
}

TEST_F(TestNewtonEulerSolver, inverse_dynamics_at_rest_returns_zero)
{
    auto result = solver.InverseDynamics(sphere, zero3, zero3, zero3, zero3);

    EXPECT_NEAR(result.force.at(0, 0), 0.0f, 1e-6f);
    EXPECT_NEAR(result.force.at(1, 0), 0.0f, 1e-6f);
    EXPECT_NEAR(result.force.at(2, 0), 0.0f, 1e-6f);
    EXPECT_NEAR(result.torque.at(0, 0), 0.0f, 1e-6f);
    EXPECT_NEAR(result.torque.at(1, 0), 0.0f, 1e-6f);
    EXPECT_NEAR(result.torque.at(2, 0), 0.0f, 1e-6f);
}

TEST_F(TestNewtonEulerSolver, forward_inverse_roundtrip_consistency)
{
    math::Vector<float, 3> force{ 1.0f, -2.0f, 3.0f };
    math::Vector<float, 3> torque{ 0.5f, -0.3f, 0.8f };
    math::Vector<float, 3> linearVel{ 0.1f, 0.2f, -0.1f };
    math::Vector<float, 3> angularVel{ 0.3f, -0.5f, 0.2f };

    auto accel = solver.ForwardDynamics(sphere, force, torque, linearVel, angularVel);
    auto recovered = solver.InverseDynamics(sphere, accel.linear, accel.angular, linearVel, angularVel);

    EXPECT_NEAR(recovered.force.at(0, 0), force.at(0, 0), 1e-3f);
    EXPECT_NEAR(recovered.force.at(1, 0), force.at(1, 0), 1e-3f);
    EXPECT_NEAR(recovered.force.at(2, 0), force.at(2, 0), 1e-3f);
    EXPECT_NEAR(recovered.torque.at(0, 0), torque.at(0, 0), 1e-3f);
    EXPECT_NEAR(recovered.torque.at(1, 0), torque.at(1, 0), 1e-3f);
    EXPECT_NEAR(recovered.torque.at(2, 0), torque.at(2, 0), 1e-3f);
}

TEST_F(TestNewtonEulerSolver, gyroscopic_effect_on_asymmetric_body)
{
    math::Vector<float, 3> force{};
    math::Vector<float, 3> torque{};
    math::Vector<float, 3> angularVel{ 0.0f, 0.0f, 1.0f };

    auto result = solver.ForwardDynamics(asymmetricBody, force, torque, zero3, angularVel);

    EXPECT_NEAR(result.angular.at(0, 0), 0.0f, 1e-4f);
    EXPECT_NEAR(result.angular.at(1, 0), 0.0f, 1e-4f);
    EXPECT_NEAR(result.angular.at(2, 0), 0.0f, 1e-4f);
}

TEST_F(TestNewtonEulerSolver, gyroscopic_effect_off_principal_axis)
{
    math::Vector<float, 3> force{};
    math::Vector<float, 3> torque{};
    math::Vector<float, 3> angularVel{ 1.0f, 1.0f, 0.0f };

    auto result = solver.ForwardDynamics(asymmetricBody, force, torque, zero3, angularVel);

    EXPECT_NEAR(result.angular.at(0, 0), 0.0f, 1e-4f);
    EXPECT_NEAR(result.angular.at(1, 0), 0.0f, 1e-4f);
    EXPECT_NEAR(result.angular.at(2, 0), -1.0f / 3.0f, 1e-3f);
}

TEST_F(TestNewtonEulerSolver, body_frame_coriolis_effect)
{
    math::Vector<float, 3> force{};
    math::Vector<float, 3> torque{};
    math::Vector<float, 3> linearVel{ 1.0f, 0.0f, 0.0f };
    math::Vector<float, 3> angularVel{ 0.0f, 0.0f, 1.0f };

    auto result = solver.ForwardDynamics(sphere, force, torque, linearVel, angularVel);

    EXPECT_NEAR(result.linear.at(0, 0), 0.0f, 1e-4f);
    EXPECT_NEAR(result.linear.at(1, 0), -1.0f, 1e-4f);
    EXPECT_NEAR(result.linear.at(2, 0), 0.0f, 1e-4f);
}

TEST_F(TestNewtonEulerSolver, asymmetric_body_forward_inverse_roundtrip)
{
    math::Vector<float, 3> force{ 3.0f, -1.0f, 2.0f };
    math::Vector<float, 3> torque{ 1.0f, 0.5f, -0.7f };
    math::Vector<float, 3> linearVel{ 0.5f, -0.3f, 0.8f };
    math::Vector<float, 3> angularVel{ 0.2f, 0.4f, -0.6f };

    auto accel = solver.ForwardDynamics(asymmetricBody, force, torque, linearVel, angularVel);
    auto recovered = solver.InverseDynamics(asymmetricBody, accel.linear, accel.angular, linearVel, angularVel);

    EXPECT_NEAR(recovered.force.at(0, 0), force.at(0, 0), 1e-3f);
    EXPECT_NEAR(recovered.force.at(1, 0), force.at(1, 0), 1e-3f);
    EXPECT_NEAR(recovered.force.at(2, 0), force.at(2, 0), 1e-3f);
    EXPECT_NEAR(recovered.torque.at(0, 0), torque.at(0, 0), 1e-3f);
    EXPECT_NEAR(recovered.torque.at(1, 0), torque.at(1, 0), 1e-3f);
    EXPECT_NEAR(recovered.torque.at(2, 0), torque.at(2, 0), 1e-3f);
}
