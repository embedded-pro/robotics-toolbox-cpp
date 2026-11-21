#pragma once

#include "numerical/math/CholeskyDecomposition.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/math/MatrixNorms.hpp"
#include "numerical/math/TriangularSolve.hpp"
#include <cstddef>
#include <optional>
#include <type_traits>

namespace dynamics
{
    template<typename T, std::size_t N>
    std::optional<math::Vector<T, N>> EffectiveJointInertia(const math::SquareMatrix<T, N>& massMatrix)
    {
        static_assert(std::is_floating_point_v<T>, "EffectiveJointInertia supports floating-point types");

        const auto factor{ math::CholeskyDecomposition<T, N>::TryFactor(massMatrix) };
        if (!factor.has_value())
            return std::nullopt;

        math::Vector<T, N> inertia{};
        for (std::size_t joint = 0; joint < N; ++joint)
        {
            math::Vector<T, N> unit{};
            unit[joint] = T{ 1 };
            const auto column{ math::SolveLowerTriangular(*factor, unit) };
            inertia[joint] = T{ 1 } / math::DotProduct(column, column);
        }

        return inertia;
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template std::optional<math::Vector<float, 2>> EffectiveJointInertia<float, 2>(const math::SquareMatrix<float, 2>&);
    extern template std::optional<math::Vector<float, 6>> EffectiveJointInertia<float, 6>(const math::SquareMatrix<float, 6>&);
#endif
}
