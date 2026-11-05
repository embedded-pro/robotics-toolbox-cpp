#pragma once

#include "numerical/math/Geometry3D.hpp"
#include "numerical/math/Matrix.hpp"
#include "robotics/dynamics/JointLink.hpp"
#include <type_traits>

namespace dynamics
{
    template<typename T>
    struct SpatialInertia
    {
        static_assert(std::is_floating_point_v<T>, "SpatialInertia supports floating-point types");

        math::SquareMatrix<T, 3> rot;
        math::SquareMatrix<T, 3> cross;
        math::SquareMatrix<T, 3> lin;

        SpatialInertia& operator+=(const SpatialInertia& other)
        {
            rot = rot + other.rot;
            cross = cross + other.cross;
            lin = lin + other.lin;
            return *this;
        }
    };

    template<typename T>
    struct SpatialWrench
    {
        static_assert(std::is_floating_point_v<T>, "SpatialWrench supports floating-point types");

        math::Vector<T, 3> force;
        math::Vector<T, 3> torque;
    };

    template<typename T>
    struct SpatialMotion
    {
        static_assert(std::is_floating_point_v<T>, "SpatialMotion supports floating-point types");

        math::Vector<T, 3> angular;
        math::Vector<T, 3> linear;

        SpatialMotion& operator+=(const SpatialMotion& other)
        {
            angular = angular + other.angular;
            linear = linear + other.linear;
            return *this;
        }

        SpatialMotion operator*(T scale) const
        {
            return SpatialMotion{ angular * scale, linear * scale };
        }
    };

    template<typename T>
    SpatialMotion<T> MotionCross(const SpatialMotion<T>& velocity, const SpatialMotion<T>& motion)
    {
        return SpatialMotion<T>{
            math::CrossProduct(velocity.angular, motion.angular),
            math::CrossProduct(velocity.linear, motion.angular) + math::CrossProduct(velocity.angular, motion.linear)
        };
    }

    template<typename T>
    SpatialWrench<T> ForceCross(const SpatialMotion<T>& velocity, const SpatialWrench<T>& wrench)
    {
        return SpatialWrench<T>{
            math::CrossProduct(velocity.angular, wrench.force),
            math::CrossProduct(velocity.angular, wrench.torque) + math::CrossProduct(velocity.linear, wrench.force)
        };
    }

    template<typename T>
    SpatialWrench<T> Apply(const SpatialInertia<T>& inertia, const SpatialMotion<T>& motion)
    {
        return SpatialWrench<T>{
            inertia.cross.Transpose() * motion.angular + inertia.lin * motion.linear,
            inertia.rot * motion.angular + inertia.cross * motion.linear
        };
    }

    template<typename T>
    SpatialMotion<T> TransformMotionToChild(const SpatialMotion<T>& motion, const math::SquareMatrix<T, 3>& R, const math::Vector<T, 3>& parentToJoint)
    {
        const auto Rt{ R.Transpose() };
        return SpatialMotion<T>{ Rt * motion.angular, Rt * (motion.linear + math::CrossProduct(motion.angular, parentToJoint)) };
    }

    template<typename T>
    SpatialInertia<T> RigidBodyInertia(const JointLink<T>& link)
    {
        const auto rSkew{ math::SkewSymmetric(link.jointToCoM) };

        return SpatialInertia<T>{
            link.inertia + rSkew.Transpose() * rSkew * link.mass,
            rSkew * link.mass,
            math::ScaledIdentity(link.mass)
        };
    }

    template<typename T>
    SpatialInertia<T> TransformInertiaToParent(const SpatialInertia<T>& inertia,
        const math::SquareMatrix<T, 3>& R, const math::Vector<T, 3>& parentToJoint)
    {
        const auto Rt{ R.Transpose() };
        const auto rotP{ R * inertia.rot * Rt };
        const auto crossP{ R * inertia.cross * Rt };
        const auto linP{ R * inertia.lin * Rt };
        const auto rSkew{ math::SkewSymmetric(parentToJoint) };

        return SpatialInertia<T>{
            rotP - crossP * rSkew + rSkew * crossP.Transpose() - rSkew * linP * rSkew,
            crossP + rSkew * linP,
            linP
        };
    }

    template<typename T>
    SpatialWrench<T> TransformWrenchToParent(const SpatialWrench<T>& wrench,
        const math::SquareMatrix<T, 3>& R, const math::Vector<T, 3>& parentToJoint)
    {
        const auto forceP{ R * wrench.force };

        return SpatialWrench<T>{
            forceP,
            R * wrench.torque + math::CrossProduct(parentToJoint, forceP)
        };
    }
}
