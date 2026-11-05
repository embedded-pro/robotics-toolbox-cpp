#pragma once

#include "numerical/math/Matrix.hpp"
#include <cstddef>
#include <type_traits>

namespace dynamics
{
    template<typename T, std::size_t Dof>
    class InverseDynamicsModel
    {
        static_assert(std::is_floating_point_v<T>, "InverseDynamicsModel supports floating-point types");
        static_assert(Dof > 0, "Degrees of freedom must be positive");

    public:
        using StateVector = math::Vector<T, Dof>;

        virtual StateVector ComputeInverseDynamics(const StateVector& q, const StateVector& qDot, const StateVector& qDDot) const = 0;

    protected:
        ~InverseDynamicsModel() = default;
    };
}
