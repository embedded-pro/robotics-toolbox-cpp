#pragma once

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Geometry3D.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/math/SE3Transform.hpp"
#include <cstdint>
#include <limits>
#include <type_traits>

namespace dynamics
{
    enum class JointType : std::uint8_t
    {
        Revolute,
        Prismatic
    };

    template<typename T>
    struct JointLimits
    {
        static_assert(std::is_floating_point_v<T>, "JointLimits supports floating-point types");

        T positionMin{ -std::numeric_limits<T>::infinity() };
        T positionMax{ std::numeric_limits<T>::infinity() };
        T velocityMax{ std::numeric_limits<T>::infinity() };
        T effortMax{ std::numeric_limits<T>::infinity() };
    };

    template<typename T>
    struct JointLink
    {
        static_assert(std::is_floating_point_v<T>, "JointLink supports floating-point types");

        T mass;
        math::SquareMatrix<T, 3> inertia;
        math::Vector<T, 3> jointAxis;
        math::Vector<T, 3> parentToJoint;
        math::Vector<T, 3> jointToCoM;
        JointType type{ JointType::Revolute };
        JointLimits<T> limits{};
        T armature{};

        OPTIMIZE_FOR_SPEED math::SE3Transform<T> JointTransform(T q) const;
        math::Vector<T, 3> AngularAxis() const;
        math::Vector<T, 3> LinearAxis() const;
    };

    template<typename T>
    using RevoluteJointLink = JointLink<T>;

    template<typename T>
    OPTIMIZE_FOR_SPEED math::SE3Transform<T> JointLink<T>::JointTransform(T q) const
    {
        if (type == JointType::Revolute)
            return { math::RotationAboutAxis(jointAxis, q), parentToJoint };

        return { math::SquareMatrix<T, 3>::Identity(), parentToJoint + jointAxis * q };
    }

    template<typename T>
    math::Vector<T, 3> JointLink<T>::AngularAxis() const
    {
        return type == JointType::Revolute ? jointAxis : math::Vector<T, 3>{};
    }

    template<typename T>
    math::Vector<T, 3> JointLink<T>::LinearAxis() const
    {
        return type == JointType::Prismatic ? jointAxis : math::Vector<T, 3>{};
    }

    template<typename T>
    bool HasUnitJointAxis(const JointLink<T>& link)
    {
        return math::Abs(math::DotProduct(link.jointAxis, link.jointAxis) - T(1)) < T(1e-3);
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template struct JointLink<float>;
#endif
}
