#pragma once

#include "numerical/math/Matrix.hpp"

namespace dynamics
{
    template<typename T>
    class NewtonEulerBody
    {
        static_assert(std::is_floating_point_v<T>,
            "NewtonEulerBody only supports floating-point types");

    public:
        using Vector3 = math::Vector<T, 3>;
        using InertiaMatrix = math::SquareMatrix<T, 3>;

        virtual ~NewtonEulerBody() = default;

        virtual T ComputeMass() const = 0;
        virtual InertiaMatrix ComputeInertia() const = 0;
    };
}
