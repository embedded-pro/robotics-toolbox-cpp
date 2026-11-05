#pragma once

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Geometry3D.hpp"
#include "numerical/math/Matrix.hpp"
#include "robotics/dynamics/JointLink.hpp"
#include "robotics/dynamics/SpatialInertia.hpp"
#include <array>
#include <cassert>
#include <cstddef>
#include <type_traits>

namespace dynamics
{
    template<typename T, std::size_t NumLinks>
    struct ReferenceMotion
    {
        static_assert(std::is_floating_point_v<T>, "ReferenceMotion supports floating-point types");

        std::array<SpatialMotion<T>, NumLinks> velocity{};
        std::array<SpatialMotion<T>, NumLinks> referenceVelocity{};
        std::array<SpatialMotion<T>, NumLinks> acceleration{};
        std::array<math::SquareMatrix<T, 3>, NumLinks> rotation{};
        std::array<math::Vector<T, 3>, NumLinks> offset{};
    };

    template<typename T, std::size_t NumLinks>
    class ModifiedRecursiveNewtonEuler
    {
        static_assert(std::is_floating_point_v<T>, "ModifiedRecursiveNewtonEuler supports floating-point types");
        static_assert(NumLinks > 0, "Number of links must be positive");

    public:
        using Vector3 = math::Vector<T, 3>;
        using JointVector = math::Vector<T, NumLinks>;
        using LinkArray = std::array<JointLink<T>, NumLinks>;
        using Motion = ReferenceMotion<T, NumLinks>;

        OPTIMIZE_FOR_SPEED JointVector InverseDynamics(const LinkArray& links, const JointVector& q, const JointVector& qDot,
            const JointVector& qDotReference, const JointVector& qDDotReference, const Vector3& gravity) const;

        static Motion ComputeReferenceMotion(const LinkArray& links, const JointVector& q, const JointVector& qDot,
            const JointVector& qDotReference, const JointVector& qDDotReference, const Vector3& gravity);
        static SpatialMotion<T> MotionSubspace(const JointLink<T>& link);

    private:
        static SpatialWrench<T> BodyWrench(const SpatialInertia<T>& inertia, const Motion& motion, std::size_t link);
    };

    template<typename T, std::size_t NumLinks>
    OPTIMIZE_FOR_SPEED typename ModifiedRecursiveNewtonEuler<T, NumLinks>::JointVector ModifiedRecursiveNewtonEuler<T, NumLinks>::InverseDynamics(const LinkArray& links,
        const JointVector& q, const JointVector& qDot, const JointVector& qDotReference, const JointVector& qDDotReference, const Vector3& gravity) const
    {
        const auto motion{ ComputeReferenceMotion(links, q, qDot, qDotReference, qDDotReference, gravity) };

        std::array<SpatialWrench<T>, NumLinks> wrench{};
        for (std::size_t i = 0; i < NumLinks; ++i)
            wrench[i] = BodyWrench(RigidBodyInertia(links[i]), motion, i);

        JointVector tau{};
        for (std::size_t j = NumLinks; j > 0; --j)
        {
            const std::size_t i{ j - 1 };
            const auto subspace{ MotionSubspace(links[i]) };
            tau[i] = math::DotProduct(subspace.angular, wrench[i].torque) + math::DotProduct(subspace.linear, wrench[i].force) + links[i].armature * qDDotReference[i];

            if (i > 0)
            {
                const auto parent{ TransformWrenchToParent(wrench[i], motion.rotation[i], motion.offset[i]) };
                wrench[i - 1].force = wrench[i - 1].force + parent.force;
                wrench[i - 1].torque = wrench[i - 1].torque + parent.torque;
            }
        }

        return tau;
    }

    template<typename T, std::size_t NumLinks>
    typename ModifiedRecursiveNewtonEuler<T, NumLinks>::Motion ModifiedRecursiveNewtonEuler<T, NumLinks>::ComputeReferenceMotion(const LinkArray& links,
        const JointVector& q, const JointVector& qDot, const JointVector& qDotReference, const JointVector& qDDotReference, const Vector3& gravity)
    {
        Motion motion{};
        SpatialMotion<T> velocity{};
        SpatialMotion<T> referenceVelocity{};
        SpatialMotion<T> acceleration{ Vector3{}, gravity * T{ -1 } };

        for (std::size_t i = 0; i < NumLinks; ++i)
        {
            assert(HasUnitJointAxis(links[i]));
            const auto transform{ links[i].JointTransform(q[i]) };
            const auto subspace{ MotionSubspace(links[i]) };
            motion.rotation[i] = transform.rotation;
            motion.offset[i] = transform.translation;

            velocity = TransformMotionToChild(velocity, transform.rotation, transform.translation);
            velocity += subspace * qDot[i];
            referenceVelocity = TransformMotionToChild(referenceVelocity, transform.rotation, transform.translation);
            referenceVelocity += subspace * qDotReference[i];
            acceleration = TransformMotionToChild(acceleration, transform.rotation, transform.translation);
            acceleration += subspace * qDDotReference[i];
            acceleration += MotionCross(velocity, subspace) * qDotReference[i];

            motion.velocity[i] = velocity;
            motion.referenceVelocity[i] = referenceVelocity;
            motion.acceleration[i] = acceleration;
        }

        return motion;
    }

    template<typename T, std::size_t NumLinks>
    SpatialMotion<T> ModifiedRecursiveNewtonEuler<T, NumLinks>::MotionSubspace(const JointLink<T>& link)
    {
        return SpatialMotion<T>{ link.AngularAxis(), link.LinearAxis() };
    }

    template<typename T, std::size_t NumLinks>
    SpatialWrench<T> ModifiedRecursiveNewtonEuler<T, NumLinks>::BodyWrench(const SpatialInertia<T>& inertia, const Motion& motion, std::size_t link)
    {
        const auto& v{ motion.velocity[link] };
        const auto& vr{ motion.referenceVelocity[link] };
        const auto inertial{ Apply(inertia, motion.acceleration[link]) };
        const auto first{ ForceCross(v, Apply(inertia, vr)) };
        const auto second{ Apply(inertia, MotionCross(v, vr)) };
        const auto third{ ForceCross(vr, Apply(inertia, v)) };
        const T half{ T{ 0.5f } };

        return SpatialWrench<T>{
            inertial.force + (first.force - second.force + third.force) * half,
            inertial.torque + (first.torque - second.torque + third.torque) * half
        };
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class ModifiedRecursiveNewtonEuler<float, 2>;
    extern template class ModifiedRecursiveNewtonEuler<float, 3>;
#endif
}
