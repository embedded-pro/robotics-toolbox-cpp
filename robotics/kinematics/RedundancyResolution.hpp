#pragma once

#include "infra/util/ReallyAssert.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/solvers/DampedPseudoInverse.hpp"
#include "robotics/kinematics/JacobianProvider.hpp"
#include <cstddef>
#include <type_traits>

namespace kinematics
{
    template<typename T, std::size_t TaskDim, std::size_t Dof>
    class RedundancyResolution
    {
        static_assert(std::is_floating_point_v<T>, "RedundancyResolution supports floating-point types");
        static_assert(Dof > TaskDim, "Redundancy resolution needs more joints than task dimensions");

    public:
        using JointVector = math::Vector<T, Dof>;
        using TaskVector = math::Vector<T, TaskDim>;

        RedundancyResolution(const JacobianProvider<T, TaskDim, Dof>& jacobian, T damping, T rankTolerance = static_cast<T>(1e-4));

        OPTIMIZE_FOR_SPEED JointVector Resolve(const JointVector& q, const TaskVector& xDot, const JointVector& qDot0);
        math::Matrix<T, Dof, TaskDim> DampedPseudoInverse(const JointVector& q);
        math::SquareMatrix<T, Dof> NullSpaceProjector(const JointVector& q);

    private:
        void Factor(const JointVector& q);

        const JacobianProvider<T, TaskDim, Dof>& jacobian;
        solvers::DampedPseudoInverse<T, TaskDim, Dof> inverse;
    };

    template<typename T, std::size_t TaskDim, std::size_t Dof>
    RedundancyResolution<T, TaskDim, Dof>::RedundancyResolution(const JacobianProvider<T, TaskDim, Dof>& jacobian, T damping, T rankTolerance)
        : jacobian{ jacobian }
        , inverse{ damping, rankTolerance }
    {}

    template<typename T, std::size_t TaskDim, std::size_t Dof>
    OPTIMIZE_FOR_SPEED typename RedundancyResolution<T, TaskDim, Dof>::JointVector RedundancyResolution<T, TaskDim, Dof>::Resolve(const JointVector& q, const TaskVector& xDot, const JointVector& qDot0)
    {
        Factor(q);
        return inverse.Solve(xDot) + inverse.Project(qDot0);
    }

    template<typename T, std::size_t TaskDim, std::size_t Dof>
    math::Matrix<T, Dof, TaskDim> RedundancyResolution<T, TaskDim, Dof>::DampedPseudoInverse(const JointVector& q)
    {
        Factor(q);
        return inverse.Inverse();
    }

    template<typename T, std::size_t TaskDim, std::size_t Dof>
    math::SquareMatrix<T, Dof> RedundancyResolution<T, TaskDim, Dof>::NullSpaceProjector(const JointVector& q)
    {
        Factor(q);
        return inverse.NullSpaceProjector();
    }

    template<typename T, std::size_t TaskDim, std::size_t Dof>
    void RedundancyResolution<T, TaskDim, Dof>::Factor(const JointVector& q)
    {
        const bool factored{ inverse.Factor(jacobian.Jacobian(q)) };
        really_assert(factored);
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class RedundancyResolution<float, 6, 7>;
    extern template class RedundancyResolution<float, 3, 7>;
#endif
}
