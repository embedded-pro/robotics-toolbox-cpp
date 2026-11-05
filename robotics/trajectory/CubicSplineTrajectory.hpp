#pragma once

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/CubicSpline.hpp"
#include "numerical/math/Matrix.hpp"
#include "robotics/trajectory/TrajectoryTypes.hpp"
#include <array>
#include <cstddef>
#include <type_traits>

namespace trajectory
{
    template<typename T, std::size_t Dof, std::size_t MaxPoints>
    class CubicSplineTrajectory
    {
        static_assert(std::is_floating_point_v<T>, "CubicSplineTrajectory supports floating-point types");
        static_assert(MaxPoints >= 2, "CubicSplineTrajectory requires MaxPoints >= 2");

    public:
        using JointVector = math::Vector<T, Dof>;
        using Times = std::array<T, MaxPoints>;
        using Positions = std::array<JointVector, MaxPoints>;
        using Boundary = math::SplineBoundary<T, Dof>;
        using State = JointTrajectoryState<T, Dof>;

        bool Plan(const Times& times, const Positions& positions, std::size_t count, const Boundary& boundary = {});
        OPTIMIZE_FOR_SPEED State Sample(T t) const;
        T StartTime() const;
        T Duration() const;

    private:
        math::CubicSpline<T, Dof, MaxPoints> spline;
    };

    template<typename T, std::size_t Dof, std::size_t MaxPoints>
    bool CubicSplineTrajectory<T, Dof, MaxPoints>::Plan(const Times& times, const Positions& positions, std::size_t count, const Boundary& boundary)
    {
        return spline.Fit(times, positions, count, boundary);
    }

    template<typename T, std::size_t Dof, std::size_t MaxPoints>
    OPTIMIZE_FOR_SPEED typename CubicSplineTrajectory<T, Dof, MaxPoints>::State CubicSplineTrajectory<T, Dof, MaxPoints>::Sample(T t) const
    {
        const auto point{ spline.Evaluate(t) };
        return State{ point.value, point.firstDerivative, point.secondDerivative };
    }

    template<typename T, std::size_t Dof, std::size_t MaxPoints>
    T CubicSplineTrajectory<T, Dof, MaxPoints>::StartTime() const
    {
        return spline.First();
    }

    template<typename T, std::size_t Dof, std::size_t MaxPoints>
    T CubicSplineTrajectory<T, Dof, MaxPoints>::Duration() const
    {
        return spline.Last() - spline.First();
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class CubicSplineTrajectory<float, 3, 8>;
#endif
}
