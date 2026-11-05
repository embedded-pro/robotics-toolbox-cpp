#pragma once

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/solvers/LuDecomposition.hpp"
#include "numerical/solvers/SingularValueDecomposition.hpp"
#include "robotics/kinematics/JacobianProvider.hpp"
#include <algorithm>
#include <array>
#include <cstddef>
#include <limits>
#include <type_traits>

namespace kinematics
{
    template<typename T, std::size_t TaskDim, std::size_t Dof>
    class ManipulabilityIndex
    {
        static_assert(std::is_floating_point_v<T>, "ManipulabilityIndex supports floating-point types");

    public:
        static constexpr std::size_t Rows{ TaskDim == 6 ? 3 : TaskDim };
        static constexpr std::size_t Axes{ std::min(Rows, Dof) };

        using JointVector = math::Vector<T, Dof>;
        using Block = math::Matrix<T, Rows, Dof>;
        using AxesArray = std::array<T, Axes>;

        explicit ManipulabilityIndex(const JacobianProvider<T, TaskDim, Dof>& jacobian);

        OPTIMIZE_FOR_SPEED T Compute(const JointVector& q) const;
        T ComputeRotational(const JointVector& q) const
        requires(TaskDim == 6);
        AxesArray EllipsoidAxes(const JointVector& q) const;
        T ConditionNumber(const JointVector& q) const;
        bool NearSingular(const JointVector& q, T epsilon) const;

    private:
        static constexpr T singularRatio{ static_cast<T>(1e-6) };

        template<std::size_t BlockRows>
        static T GramRoot(const math::Matrix<T, BlockRows, Dof>& block);

        Block TranslationalBlock(const JointVector& q) const;

        const JacobianProvider<T, TaskDim, Dof>& jacobian;
    };

    template<typename T, std::size_t TaskDim, std::size_t Dof>
    ManipulabilityIndex<T, TaskDim, Dof>::ManipulabilityIndex(const JacobianProvider<T, TaskDim, Dof>& jacobian)
        : jacobian{ jacobian }
    {}

    template<typename T, std::size_t TaskDim, std::size_t Dof>
    OPTIMIZE_FOR_SPEED T ManipulabilityIndex<T, TaskDim, Dof>::Compute(const JointVector& q) const
    {
        return GramRoot(TranslationalBlock(q));
    }

    template<typename T, std::size_t TaskDim, std::size_t Dof>
    T ManipulabilityIndex<T, TaskDim, Dof>::ComputeRotational(const JointVector& q) const
    requires(TaskDim == 6)
    {
        return GramRoot(math::Matrix<T, 3, Dof>{ jacobian.Jacobian(q).template GetBlock<3, Dof>(3, 0) });
    }

    template<typename T, std::size_t TaskDim, std::size_t Dof>
    typename ManipulabilityIndex<T, TaskDim, Dof>::AxesArray ManipulabilityIndex<T, TaskDim, Dof>::EllipsoidAxes(const JointVector& q) const
    {
        solvers::SingularValueDecomposition<T, Rows, Dof> svd;
        svd.Decompose(TranslationalBlock(q));

        AxesArray axes{};
        for (std::size_t i = 0; i < Axes; ++i)
            axes[i] = svd.SingularValues()[i];
        return axes;
    }

    template<typename T, std::size_t TaskDim, std::size_t Dof>
    T ManipulabilityIndex<T, TaskDim, Dof>::ConditionNumber(const JointVector& q) const
    {
        const auto axes{ EllipsoidAxes(q) };
        const T largest{ axes.front() };
        const T smallest{ axes.back() };

        if (smallest <= singularRatio * largest)
            return std::numeric_limits<T>::infinity();

        return largest / smallest;
    }

    template<typename T, std::size_t TaskDim, std::size_t Dof>
    bool ManipulabilityIndex<T, TaskDim, Dof>::NearSingular(const JointVector& q, T epsilon) const
    {
        return Compute(q) < epsilon;
    }

    template<typename T, std::size_t TaskDim, std::size_t Dof>
    template<std::size_t BlockRows>
    T ManipulabilityIndex<T, TaskDim, Dof>::GramRoot(const math::Matrix<T, BlockRows, Dof>& block)
    {
        constexpr std::size_t gramSize{ std::min(BlockRows, Dof) };
        math::SquareMatrix<T, gramSize> gram{};
        if constexpr (BlockRows <= Dof)
            gram = block * block.Transpose();
        else
            gram = block.Transpose() * block;

        solvers::LuDecomposition<T, gramSize> lu;
        if (!lu.Decompose(gram))
            return T{};

        return math::Sqrt(std::max(T{}, lu.Determinant()));
    }

    template<typename T, std::size_t TaskDim, std::size_t Dof>
    typename ManipulabilityIndex<T, TaskDim, Dof>::Block ManipulabilityIndex<T, TaskDim, Dof>::TranslationalBlock(const JointVector& q) const
    {
        return jacobian.Jacobian(q).template GetBlock<Rows, Dof>(0, 0);
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class ManipulabilityIndex<float, 3, 2>;
    extern template class ManipulabilityIndex<float, 2, 3>;
    extern template class ManipulabilityIndex<float, 6, 2>;
    extern template class ManipulabilityIndex<float, 6, 6>;
#endif
}
