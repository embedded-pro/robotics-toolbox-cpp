#pragma once

#include "numerical/math/Math.hpp"
#include "numerical/math/Matrix.hpp"
#include <array>
#include <cstddef>
#include <type_traits>

namespace controllers
{
    template<typename T, std::size_t N>
    math::SquareMatrix<T, N> CoupledDamping(const math::SquareMatrix<T, N>& inertia, const math::Vector<T, N>& stiffness, T dampingRatio)
    {
        static_assert(std::is_floating_point_v<T>, "CoupledDamping supports floating-point types");

        std::array<T, N> weight{};
        for (std::size_t joint = 0; joint < N; ++joint)
            weight[joint] = inertia.at(joint, joint) > T{} ? math::Sqrt(math::Sqrt(stiffness[joint] / inertia.at(joint, joint))) : T{};

        math::SquareMatrix<T, N> damping{};
        for (std::size_t row = 0; row < N; ++row)
            for (std::size_t column = 0; column < N; ++column)
                damping.at(row, column) = T{ 2 } * dampingRatio * weight[row] * inertia.at(row, column) * weight[column];
        return damping;
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template math::SquareMatrix<float, 2> CoupledDamping<float, 2>(const math::SquareMatrix<float, 2>&, const math::Vector<float, 2>&, float);
    extern template math::SquareMatrix<float, 6> CoupledDamping<float, 6>(const math::SquareMatrix<float, 6>&, const math::Vector<float, 6>&, float);
#endif
}
