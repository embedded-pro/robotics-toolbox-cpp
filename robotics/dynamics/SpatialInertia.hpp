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
