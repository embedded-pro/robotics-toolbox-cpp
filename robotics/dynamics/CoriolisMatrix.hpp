#pragma once

#include "numerical/math/Matrix.hpp"
#include "robotics/dynamics/JointLink.hpp"
#include "robotics/dynamics/ModifiedRecursiveNewtonEuler.hpp"
#include <array>
#include <cstddef>
#include <type_traits>

namespace dynamics
{
    template<typename T, std::size_t NumLinks>
    class CoriolisMatrix
    {
        static_assert(std::is_floating_point_v<T>, "CoriolisMatrix supports floating-point types");

    public:
        using JointVector = math::Vector<T, NumLinks>;
        using Matrix = math::SquareMatrix<T, NumLinks>;
        using LinkArray = std::array<JointLink<T>, NumLinks>;

        Matrix Compute(const LinkArray& links, const JointVector& q, const JointVector& qDot) const;
        JointVector TransposeTimesVelocity(const LinkArray& links, const JointVector& q, const JointVector& qDot) const;

    private:
        [[no_unique_address]] ModifiedRecursiveNewtonEuler<T, NumLinks> modified;
    };

    template<typename T, std::size_t NumLinks>
    typename CoriolisMatrix<T, NumLinks>::Matrix CoriolisMatrix<T, NumLinks>::Compute(const LinkArray& links, const JointVector& q, const JointVector& qDot) const
    {
        Matrix coriolis{};
        for (std::size_t j = 0; j < NumLinks; ++j)
        {
            JointVector unit{};
            unit[j] = T{ 1 };
            coriolis.SetBlock(modified.InverseDynamics(links, q, qDot, unit, JointVector{}, math::Vector<T, 3>{}), 0, j);
        }

        return coriolis;
    }

    template<typename T, std::size_t NumLinks>
    typename CoriolisMatrix<T, NumLinks>::JointVector CoriolisMatrix<T, NumLinks>::TransposeTimesVelocity(const LinkArray& links, const JointVector& q, const JointVector& qDot) const
    {
        return Compute(links, q, qDot).Transpose() * qDot;
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class CoriolisMatrix<float, 2>;
    extern template class CoriolisMatrix<float, 3>;
#endif
}
