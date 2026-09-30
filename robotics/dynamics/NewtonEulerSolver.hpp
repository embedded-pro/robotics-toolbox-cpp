#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "infra/util/ReallyAssert.hpp"
#include "numerical/math/CholeskyDecomposition.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Geometry3D.hpp"
#include "numerical/math/Matrix.hpp"
#include "robotics/dynamics/NewtonEulerBody.hpp"

namespace dynamics
{
    template<typename T>
    struct BodyAcceleration
    {
        math::Vector<T, 3> linear;
        math::Vector<T, 3> angular;
    };

    template<typename T>
    struct BodyWrench
    {
        math::Vector<T, 3> force;
        math::Vector<T, 3> torque;
    };

    template<typename T>
    using SpatialAcceleration [[deprecated("use BodyAcceleration")]] = BodyAcceleration<T>;

    template<typename T>
    using SpatialForce [[deprecated("use BodyWrench")]] = BodyWrench<T>;

    template<typename T>
    class NewtonEulerSolver
    {
        static_assert(std::is_floating_point_v<T>,
            "NewtonEulerSolver only supports floating-point types");

    public:
        using Vector3 = math::Vector<T, 3>;
        using InertiaMatrix = math::SquareMatrix<T, 3>;

        NewtonEulerSolver() = default;

        OPTIMIZE_FOR_SPEED BodyAcceleration<T> ForwardDynamics(const NewtonEulerBody<T>& body,
            const Vector3& force, const Vector3& torque,
            const Vector3& linearVelocity, const Vector3& angularVelocity) const;

        OPTIMIZE_FOR_SPEED BodyWrench<T> InverseDynamics(const NewtonEulerBody<T>& body,
            const Vector3& linearAcceleration, const Vector3& angularAcceleration,
            const Vector3& linearVelocity, const Vector3& angularVelocity) const;
    };

    template<typename T>
    OPTIMIZE_FOR_SPEED
        BodyAcceleration<T>
        NewtonEulerSolver<T>::ForwardDynamics(const NewtonEulerBody<T>& body,
            const Vector3& force, const Vector3& torque,
            const Vector3& linearVelocity, const Vector3& angularVelocity) const
    {
        const T mass{ body.ComputeMass() };
        const InertiaMatrix inertia{ body.ComputeInertia() };
        really_assert(mass > T(0));

        const Vector3 gyroscopic{ math::CrossProduct(angularVelocity, Vector3{ inertia * angularVelocity }) };
        const auto angularAcceleration{ math::CholeskyDecomposition<T, 3>::Solve(inertia, Vector3{ torque - gyroscopic }) };
        really_assert(angularAcceleration.has_value());

        const Vector3 linearAcceleration{ force * (T(1) / mass) - math::CrossProduct(angularVelocity, linearVelocity) };

        return BodyAcceleration<T>{ linearAcceleration, *angularAcceleration };
    }

    template<typename T>
    OPTIMIZE_FOR_SPEED
        BodyWrench<T>
        NewtonEulerSolver<T>::InverseDynamics(const NewtonEulerBody<T>& body,
            const Vector3& linearAcceleration, const Vector3& angularAcceleration,
            const Vector3& linearVelocity, const Vector3& angularVelocity) const
    {
        const T mass{ body.ComputeMass() };
        const InertiaMatrix inertia{ body.ComputeInertia() };

        const Vector3 resultForce{ (linearAcceleration + math::CrossProduct(angularVelocity, linearVelocity)) * mass };
        const Vector3 resultTorque{ inertia * angularAcceleration + math::CrossProduct(angularVelocity, Vector3{ inertia * angularVelocity }) };

        return BodyWrench<T>{ resultForce, resultTorque };
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class NewtonEulerSolver<float>;
#endif
}
