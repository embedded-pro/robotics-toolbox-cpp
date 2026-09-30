#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "infra/util/ReallyAssert.hpp"
#include "numerical/math/CholeskyDecomposition.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Matrix.hpp"
#include "robotics/dynamics/EulerLagrangeDynamics.hpp"

namespace dynamics
{
    template<typename T, std::size_t Dof>
    class EulerLagrangeSolver
    {
        static_assert(std::is_floating_point_v<T>,
            "EulerLagrangeSolver only supports floating-point types");
        static_assert(Dof > 0, "Degrees of freedom must be positive");

    public:
        using StateVector = math::Vector<T, Dof>;
        using MassMatrix = math::SquareMatrix<T, Dof>;

        EulerLagrangeSolver() = default;

        OPTIMIZE_FOR_SPEED StateVector ForwardDynamics(const EulerLagrangeDynamics<T, Dof>& model,
            const StateVector& q, const StateVector& qDot, const StateVector& tau) const;

        OPTIMIZE_FOR_SPEED StateVector InverseDynamics(const EulerLagrangeDynamics<T, Dof>& model,
            const StateVector& q, const StateVector& qDot, const StateVector& qDDot) const;
    };

    template<typename T, std::size_t Dof>
    OPTIMIZE_FOR_SPEED
        typename EulerLagrangeSolver<T, Dof>::StateVector
        EulerLagrangeSolver<T, Dof>::ForwardDynamics(const EulerLagrangeDynamics<T, Dof>& model,
            const StateVector& q, const StateVector& qDot, const StateVector& tau) const
    {
        auto M = model.ComputeMassMatrix(q);
        auto C = model.ComputeCoriolisTerms(q, qDot);
        auto g = model.ComputeGravityTerms(q);

        const auto qDDot{ math::CholeskyDecomposition<T, Dof>::Solve(M, StateVector{ tau - C - g }) };
        really_assert(qDDot.has_value());

        return *qDDot;
    }

    template<typename T, std::size_t Dof>
    OPTIMIZE_FOR_SPEED
        typename EulerLagrangeSolver<T, Dof>::StateVector
        EulerLagrangeSolver<T, Dof>::InverseDynamics(const EulerLagrangeDynamics<T, Dof>& model,
            const StateVector& q, const StateVector& qDot, const StateVector& qDDot) const
    {
        auto M = model.ComputeMassMatrix(q);
        auto C = model.ComputeCoriolisTerms(q, qDot);
        auto g = model.ComputeGravityTerms(q);

        return M * qDDot + C + g;
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class EulerLagrangeSolver<float, 1>;
    extern template class EulerLagrangeSolver<float, 2>;
    extern template class EulerLagrangeSolver<float, 3>;
#endif
}
