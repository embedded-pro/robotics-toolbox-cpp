#pragma once

#include "numerical/math/Matrix.hpp"
#include <cstddef>
#include <type_traits>

namespace dynamics
{
    template<typename T, std::size_t Dof, std::size_t NumParams>
    class InertialRegressor
    {
        static_assert(std::is_floating_point_v<T>, "InertialRegressor supports floating-point types");

    public:
        using JointVector = math::Vector<T, Dof>;
        using Regressor = math::Matrix<T, Dof, NumParams>;

        virtual Regressor Compute(const JointVector& q, const JointVector& qDot, const JointVector& qDotReference, const JointVector& qDDotReference) const = 0;

    protected:
        ~InertialRegressor() = default;
    };
}
