#pragma once

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/math/Tolerance.hpp"
#include "robotics/dynamics/InverseDynamicsModel.hpp"
#include "robotics/trajectory/TrajectoryTypes.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <type_traits>

namespace trajectory
{
    template<typename T, std::size_t Dof>
    class PathGeometry
    {
        static_assert(std::is_floating_point_v<T>, "PathGeometry supports floating-point types");

    public:
        using JointVector = math::Vector<T, Dof>;

        virtual JointVector Position(T s) const = 0;
        virtual JointVector FirstDerivative(T s) const = 0;
        virtual JointVector SecondDerivative(T s) const = 0;

    protected:
        ~PathGeometry() = default;
    };

    template<typename T, std::size_t Dof>
    struct PathCoefficients
    {
        math::Vector<T, Dof> a{};
        math::Vector<T, Dof> b{};
        math::Vector<T, Dof> c{};
    };

    template<typename T, std::size_t Dof>
    struct JointBounds
    {
        math::Vector<T, Dof> torqueMin{};
        math::Vector<T, Dof> torqueMax{};
        math::Vector<T, Dof> velocityMax{};
    };

    template<typename T, std::size_t Dof>
    class PathConstraints
    {
        static_assert(std::is_floating_point_v<T>, "PathConstraints supports floating-point types");

    public:
        virtual PathCoefficients<T, Dof> Coefficients(T s) const = 0;
        virtual const JointBounds<T, Dof>& Bounds() const = 0;

    protected:
        ~PathConstraints() = default;
    };

    template<typename T, std::size_t Dof>
    class InverseDynamicsPathConstraints
        : public PathConstraints<T, Dof>
    {
    public:
        InverseDynamicsPathConstraints(const PathGeometry<T, Dof>& path, const dynamics::InverseDynamicsModel<T, Dof>& model, const JointBounds<T, Dof>& bounds);

        PathCoefficients<T, Dof> Coefficients(T s) const override;
        const JointBounds<T, Dof>& Bounds() const override;

    private:
        const PathGeometry<T, Dof>& path;
        const dynamics::InverseDynamicsModel<T, Dof>& model;
        JointBounds<T, Dof> bounds;
    };

    template<typename T, std::size_t Dof, std::size_t Grid>
    class TimeOptimalPathParameterization
    {
        static_assert(std::is_floating_point_v<T>, "TimeOptimalPathParameterization supports floating-point types");
        static_assert(Grid >= 3, "TimeOptimalPathParameterization needs at least two stages");

    public:
        using JointVector = math::Vector<T, Dof>;
        using State = JointTrajectoryState<T, Dof>;

        TimeOptimalPathParameterization(const PathGeometry<T, Dof>& path, const PathConstraints<T, Dof>& constraints);

        bool Parameterize();
        OPTIMIZE_FOR_SPEED std::optional<State> Sample(T t) const;
        T TotalTime() const;

    private:
        struct StageRow
        {
            T alpha;
            T beta;
            T gamma;
        };

        struct Interval
        {
            T lo;
            T hi;
            bool empty;
        };

        static constexpr std::size_t stages{ Grid - 1 };
        static constexpr std::size_t rowCount{ 2 * Dof + 4 };
        static constexpr T step{ T{ 1 } / static_cast<T>(stages) };
        static constexpr T epsilon{ std::numeric_limits<T>::epsilon() * T{ 64 } };

        using StageRows = std::array<StageRow, rowCount>;

        static T GridPoint(std::size_t i);
        static void Tighten(Interval& interval, T alpha, T gamma);
        static std::optional<Interval> ProjectOntoX(const StageRows& rows);
        static T MaximumControl(const StageRows& rows, T x);
        static T SpeedLimit(const JointVector& firstDerivative, const JointBounds<T, Dof>& bounds);
        static T Ceiling();

        StageRows Rows(std::size_t i, const JointBounds<T, Dof>& bounds) const;
        bool Integrate();

        const PathGeometry<T, Dof>& path;
        const PathConstraints<T, Dof>& constraints;
        std::array<PathCoefficients<T, Dof>, Grid> coefficients{};
        std::array<T, Grid> xMax{};
        std::array<T, Grid> lo{};
        std::array<T, Grid> hi{};
        std::array<T, Grid> sDot{};
        std::array<T, Grid> u{};
        std::array<T, Grid> stageStartTime{};
        bool parameterized{ false };
    };

    template<typename T, std::size_t Dof>
    InverseDynamicsPathConstraints<T, Dof>::InverseDynamicsPathConstraints(const PathGeometry<T, Dof>& path, const dynamics::InverseDynamicsModel<T, Dof>& model, const JointBounds<T, Dof>& bounds)
        : path(path)
        , model(model)
        , bounds(bounds)
    {}

    template<typename T, std::size_t Dof>
    PathCoefficients<T, Dof> InverseDynamicsPathConstraints<T, Dof>::Coefficients(T s) const
    {
        const auto q{ path.Position(s) };
        const auto firstDerivative{ path.FirstDerivative(s) };
        const auto secondDerivative{ path.SecondDerivative(s) };
        const math::Vector<T, Dof> zero{};

        const auto c{ model.ComputeInverseDynamics(q, zero, zero) };
        const auto a{ model.ComputeInverseDynamics(q, zero, firstDerivative) - c };
        const auto b{ model.ComputeInverseDynamics(q, firstDerivative, secondDerivative) - c };
        return PathCoefficients<T, Dof>{ a, b, c };
    }

    template<typename T, std::size_t Dof>
    const JointBounds<T, Dof>& InverseDynamicsPathConstraints<T, Dof>::Bounds() const
    {
        return bounds;
    }

    template<typename T, std::size_t Dof, std::size_t Grid>
    TimeOptimalPathParameterization<T, Dof, Grid>::TimeOptimalPathParameterization(const PathGeometry<T, Dof>& path, const PathConstraints<T, Dof>& constraints)
        : path(path)
        , constraints(constraints)
    {}

    template<typename T, std::size_t Dof, std::size_t Grid>
    bool TimeOptimalPathParameterization<T, Dof, Grid>::Parameterize()
    {
        parameterized = false;
        const auto& bounds{ constraints.Bounds() };
        bool moving{ false };
        for (std::size_t i = 0; i < Grid; ++i)
        {
            const auto firstDerivative{ path.FirstDerivative(GridPoint(i)) };
            coefficients[i] = constraints.Coefficients(GridPoint(i));
            xMax[i] = SpeedLimit(firstDerivative, bounds);
            for (std::size_t j = 0; j < Dof; ++j)
                moving = moving || std::abs(firstDerivative[j]) > epsilon;
        }

        if (!moving)
        {
            sDot.fill(T{});
            u.fill(T{});
            stageStartTime.fill(T{});
            parameterized = true;
            return true;
        }

        lo[stages] = T{};
        hi[stages] = T{};
        for (std::size_t k = stages; k > 0; --k)
        {
            const auto interval{ ProjectOntoX(Rows(k - 1, bounds)) };
            if (!interval)
                return false;
            lo[k - 1] = interval->lo;
            hi[k - 1] = interval->hi;
        }

        if (lo[0] > epsilon)
            return false;

        parameterized = Integrate();
        return parameterized;
    }

    template<typename T, std::size_t Dof, std::size_t Grid>
    bool TimeOptimalPathParameterization<T, Dof, Grid>::Integrate()
    {
        const auto& bounds{ constraints.Bounds() };
        T x{};
        sDot[0] = T{};
        stageStartTime[0] = T{};
        for (std::size_t i = 0; i < stages; ++i)
        {
            const T reach{ x + T{ 2 } * step * MaximumControl(Rows(i, bounds), x) };
            const T xNext{ std::max(std::min(reach, hi[i + 1]), lo[i + 1]) };
            u[i] = (xNext - x) / (T{ 2 } * step);
            sDot[i + 1] = math::Sqrt(xNext);
            if (sDot[i] + sDot[i + 1] <= epsilon)
                return false;
            stageStartTime[i + 1] = stageStartTime[i] + T{ 2 } * step / (sDot[i] + sDot[i + 1]);
            x = xNext;
        }
        return true;
    }

    template<typename T, std::size_t Dof, std::size_t Grid>
    OPTIMIZE_FOR_SPEED auto TimeOptimalPathParameterization<T, Dof, Grid>::Sample(T t) const -> std::optional<State>
    {
        if (!parameterized)
            return std::nullopt;

        const T time{ std::clamp(t, T{}, stageStartTime[stages]) };
        const auto first{ stageStartTime.begin() };
        const auto next{ std::upper_bound(first, first + stages, time) };
        const auto i{ static_cast<std::size_t>(next - first) - 1 };

        const T tau{ time - stageStartTime[i] };
        const T s{ GridPoint(i) + sDot[i] * tau + u[i] * tau * tau / T{ 2 } };
        const T speed{ sDot[i] + u[i] * tau };
        const auto firstDerivative{ path.FirstDerivative(s) };
        return State{ path.Position(s), firstDerivative * speed, firstDerivative * u[i] + path.SecondDerivative(s) * (speed * speed) };
    }

    template<typename T, std::size_t Dof, std::size_t Grid>
    T TimeOptimalPathParameterization<T, Dof, Grid>::TotalTime() const
    {
        return stageStartTime[stages];
    }

    template<typename T, std::size_t Dof, std::size_t Grid>
    T TimeOptimalPathParameterization<T, Dof, Grid>::GridPoint(std::size_t i)
    {
        return static_cast<T>(i) / static_cast<T>(stages);
    }

    template<typename T, std::size_t Dof, std::size_t Grid>
    T TimeOptimalPathParameterization<T, Dof, Grid>::Ceiling()
    {
        return T{ 1 } / static_cast<T>(math::Tolerance<T>());
    }

    template<typename T, std::size_t Dof, std::size_t Grid>
    T TimeOptimalPathParameterization<T, Dof, Grid>::SpeedLimit(const JointVector& firstDerivative, const JointBounds<T, Dof>& bounds)
    {
        T limit{ Ceiling() };
        for (std::size_t j = 0; j < Dof; ++j)
            if (std::abs(firstDerivative[j]) > epsilon)
            {
                const T ratio{ bounds.velocityMax[j] / firstDerivative[j] };
                limit = std::min(limit, ratio * ratio);
            }
        return limit;
    }

    template<typename T, std::size_t Dof, std::size_t Grid>
    auto TimeOptimalPathParameterization<T, Dof, Grid>::Rows(std::size_t i, const JointBounds<T, Dof>& bounds) const -> StageRows
    {
        const auto& coefficient{ coefficients[i] };
        StageRows rows{};
        for (std::size_t j = 0; j < Dof; ++j)
        {
            rows[2 * j] = StageRow{ coefficient.b[j], coefficient.a[j], bounds.torqueMax[j] - coefficient.c[j] };
            rows[2 * j + 1] = StageRow{ -coefficient.b[j], -coefficient.a[j], coefficient.c[j] - bounds.torqueMin[j] };
        }

        rows[2 * Dof] = StageRow{ T{ 1 }, T{}, xMax[i] };
        rows[2 * Dof + 1] = StageRow{ T{ -1 }, T{}, T{} };
        rows[2 * Dof + 2] = StageRow{ T{ 1 }, T{ 2 } * step, hi[i + 1] };
        rows[2 * Dof + 3] = StageRow{ T{ -1 }, T{ -2 } * step, -lo[i + 1] };
        return rows;
    }

    template<typename T, std::size_t Dof, std::size_t Grid>
    void TimeOptimalPathParameterization<T, Dof, Grid>::Tighten(Interval& interval, T alpha, T gamma)
    {
        if (alpha > epsilon)
            interval.hi = std::min(interval.hi, gamma / alpha);
        else if (alpha < -epsilon)
            interval.lo = std::max(interval.lo, gamma / alpha);
        else if (gamma < -epsilon)
            interval.empty = true;
    }

    template<typename T, std::size_t Dof, std::size_t Grid>
    auto TimeOptimalPathParameterization<T, Dof, Grid>::ProjectOntoX(const StageRows& rows) -> std::optional<Interval>
    {
        Interval interval{ T{}, Ceiling(), false };
        for (const auto& row : rows)
            if (std::abs(row.beta) <= epsilon)
                Tighten(interval, row.alpha, row.gamma);

        for (const auto& upper : rows)
            if (upper.beta > epsilon)
                for (const auto& lower : rows)
                    if (lower.beta < -epsilon)
                        Tighten(interval, upper.beta * lower.alpha - lower.beta * upper.alpha, upper.beta * lower.gamma - lower.beta * upper.gamma);

        if (interval.empty || interval.lo > interval.hi + epsilon)
            return std::nullopt;
        return interval;
    }

    template<typename T, std::size_t Dof, std::size_t Grid>
    T TimeOptimalPathParameterization<T, Dof, Grid>::MaximumControl(const StageRows& rows, T x)
    {
        T control{ std::numeric_limits<T>::max() };
        for (const auto& row : rows)
            if (row.beta > epsilon)
                control = std::min(control, (row.gamma - row.alpha * x) / row.beta);
        return control;
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class InverseDynamicsPathConstraints<float, 2>;
    extern template class TimeOptimalPathParameterization<float, 2, 64>;
#endif
}
