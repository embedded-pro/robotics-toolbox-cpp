#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/Geometry3D.hpp"
#include "numerical/math/Matrix.hpp"
#include <array>
#include <cmath>

namespace dynamics
{
    template<typename T>
    struct RevoluteJointLink
    {
        static_assert(std::is_floating_point_v<T>,
            "RevoluteJointLink only supports floating-point types");

        T mass;
        math::SquareMatrix<T, 3> inertia;
        math::Vector<T, 3> jointAxis;
        math::Vector<T, 3> parentToJoint;
        math::Vector<T, 3> jointToCoM;
    };

    template<typename T>
    bool HasUnitJointAxis(const RevoluteJointLink<T>& link)
    {
        return std::abs(math::DotProduct(link.jointAxis, link.jointAxis) - T(1)) < T(1e-3);
    }
}
