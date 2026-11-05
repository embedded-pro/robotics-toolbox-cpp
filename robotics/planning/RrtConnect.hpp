#pragma once

#include "numerical/math/Math.hpp"
#include "numerical/math/Matrix.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <type_traits>
#include <utility>

namespace planning
{
    template<typename T>
    struct RrtConnectParameters
    {
        static_assert(std::is_floating_point_v<T>, "RrtConnectParameters supports floating-point types");

        T stepSize{};
        T collisionResolution{};
        std::size_t maxIterations{};
    };

    enum class PlanStatus : std::uint8_t
    {
        Found,
        StartInCollision,
        GoalInCollision,
        PoolExhausted,
        IterationLimit
    };

    template<typename T, std::size_t Dof, std::size_t MaxNodes>
    class RrtConnect
    {
        static_assert(std::is_floating_point_v<T>, "RrtConnect supports floating-point types");
        static_assert(MaxNodes > 0 && MaxNodes < 65535, "Node indices are 16-bit");

    public:
        using JointVector = math::Vector<T, Dof>;

        explicit RrtConnect(const RrtConnectParameters<T>& parameters);

        template<class Sample, class IsFree>
        PlanStatus Plan(const JointVector& start, const JointVector& goal, Sample&& sample, IsFree&& isFree);

        template<class Uniform, class IsFree>
        void Shortcut(std::size_t attempts, Uniform&& uniform, IsFree&& isFree);

        std::span<const JointVector> Path() const;
        std::size_t Iterations() const;
        std::size_t NodeCount() const;

    private:
        enum class Growth : std::uint8_t
        {
            Trapped,
            Advanced,
            Reached,
            Full
        };

        struct Tree
        {
            std::array<JointVector, MaxNodes> configuration{};
            std::array<std::uint16_t, MaxNodes> parent{};
            std::size_t size{ 0 };
        };

        static constexpr std::uint16_t root{ std::numeric_limits<std::uint16_t>::max() };

        static T Distance(const JointVector& a, const JointVector& b);

        template<class IsFree>
        bool EdgeFree(const JointVector& from, const JointVector& to, IsFree& isFree) const;
        template<class IsFree>
        Growth Extend(Tree& tree, const JointVector& target, IsFree& isFree) const;
        template<class IsFree>
        Growth Connect(Tree& tree, const JointVector& target, IsFree& isFree) const;

        static void Seed(Tree& tree, const JointVector& configuration);
        void BuildPath();

        RrtConnectParameters<T> parameters;
        std::array<Tree, 2> trees{};
        std::array<JointVector, 2 * MaxNodes> path{};
        std::size_t pathLength{ 0 };
        std::size_t iterations{ 0 };
    };

    template<typename T, std::size_t Dof, std::size_t MaxNodes>
    RrtConnect<T, Dof, MaxNodes>::RrtConnect(const RrtConnectParameters<T>& parameters)
        : parameters{ parameters }
    {}

    template<typename T, std::size_t Dof, std::size_t MaxNodes>
    template<class Sample, class IsFree>
    PlanStatus RrtConnect<T, Dof, MaxNodes>::Plan(const JointVector& start, const JointVector& goal, Sample&& sample, IsFree&& isFree)
    {
        pathLength = 0;
        iterations = 0;
        Seed(trees[0], start);
        Seed(trees[1], goal);

        if (!isFree(start))
            return PlanStatus::StartInCollision;
        if (!isFree(goal))
            return PlanStatus::GoalInCollision;

        if (EdgeFree(start, goal, isFree))
        {
            path[0] = start;
            path[1] = goal;
            pathLength = 2;
            return PlanStatus::Found;
        }

        std::size_t grow{ 0 };
        while (iterations < parameters.maxIterations)
        {
            ++iterations;
            Tree& active{ trees[grow] };
            Tree& other{ trees[1 - grow] };

            const auto extended{ Extend(active, sample(), isFree) };
            const auto growth{ (extended == Growth::Advanced || extended == Growth::Reached) ? Connect(other, active.configuration[active.size - 1], isFree) : extended };
            if (growth == Growth::Full)
                return PlanStatus::PoolExhausted;
            if (growth == Growth::Reached)
            {
                BuildPath();
                return PlanStatus::Found;
            }

            grow = 1 - grow;
        }

        return PlanStatus::IterationLimit;
    }

    template<typename T, std::size_t Dof, std::size_t MaxNodes>
    template<class Uniform, class IsFree>
    void RrtConnect<T, Dof, MaxNodes>::Shortcut(std::size_t attempts, Uniform&& uniform, IsFree&& isFree)
    {
        const auto pick{ [this, &uniform]
            {
                return std::min(static_cast<std::size_t>(uniform() * static_cast<T>(pathLength)), pathLength - 1);
            } };

        for (std::size_t attempt = 0; attempt < attempts && pathLength > 2; ++attempt)
        {
            auto first{ pick() };
            auto last{ pick() };
            if (last < first)
                std::swap(first, last);
            if (last - first < 2 || !EdgeFree(path[first], path[last], isFree))
                continue;

            std::copy(path.begin() + static_cast<std::ptrdiff_t>(last), path.begin() + static_cast<std::ptrdiff_t>(pathLength), path.begin() + static_cast<std::ptrdiff_t>(first + 1));
            pathLength -= last - first - 1;
        }
    }

    template<typename T, std::size_t Dof, std::size_t MaxNodes>
    auto RrtConnect<T, Dof, MaxNodes>::Path() const -> std::span<const JointVector>
    {
        return std::span<const JointVector>{ path.data(), pathLength };
    }

    template<typename T, std::size_t Dof, std::size_t MaxNodes>
    std::size_t RrtConnect<T, Dof, MaxNodes>::Iterations() const
    {
        return iterations;
    }

    template<typename T, std::size_t Dof, std::size_t MaxNodes>
    std::size_t RrtConnect<T, Dof, MaxNodes>::NodeCount() const
    {
        return trees[0].size + trees[1].size;
    }

    template<typename T, std::size_t Dof, std::size_t MaxNodes>
    T RrtConnect<T, Dof, MaxNodes>::Distance(const JointVector& a, const JointVector& b)
    {
        T sum{};
        for (std::size_t i = 0; i < Dof; ++i)
            sum += (b[i] - a[i]) * (b[i] - a[i]);
        return math::Sqrt(sum);
    }

    template<typename T, std::size_t Dof, std::size_t MaxNodes>
    template<class IsFree>
    bool RrtConnect<T, Dof, MaxNodes>::EdgeFree(const JointVector& from, const JointVector& to, IsFree& isFree) const
    {
        const auto checks{ static_cast<std::size_t>(std::ceil(Distance(from, to) / parameters.collisionResolution)) };
        for (std::size_t k = 1; k <= checks; ++k)
            if (!isFree(JointVector{ from + JointVector{ to - from } * (static_cast<T>(k) / static_cast<T>(checks)) }))
                return false;
        return true;
    }

    template<typename T, std::size_t Dof, std::size_t MaxNodes>
    template<class IsFree>
    auto RrtConnect<T, Dof, MaxNodes>::Extend(Tree& tree, const JointVector& target, IsFree& isFree) const -> Growth
    {
        std::size_t nearest{ 0 };
        T nearestDistance{ Distance(tree.configuration[0], target) };
        for (std::size_t i = 1; i < tree.size; ++i)
        {
            const T distance{ Distance(tree.configuration[i], target) };
            if (distance < nearestDistance)
            {
                nearest = i;
                nearestDistance = distance;
            }
        }

        const bool reaches{ nearestDistance <= parameters.stepSize };
        const JointVector candidate{ reaches ? target : JointVector{ tree.configuration[nearest] + JointVector{ target - tree.configuration[nearest] } * (parameters.stepSize / nearestDistance) } };
        if (!EdgeFree(tree.configuration[nearest], candidate, isFree))
            return Growth::Trapped;
        if (tree.size == MaxNodes)
            return Growth::Full;

        tree.configuration[tree.size] = candidate;
        tree.parent[tree.size] = static_cast<std::uint16_t>(nearest);
        ++tree.size;
        return reaches ? Growth::Reached : Growth::Advanced;
    }

    template<typename T, std::size_t Dof, std::size_t MaxNodes>
    template<class IsFree>
    auto RrtConnect<T, Dof, MaxNodes>::Connect(Tree& tree, const JointVector& target, IsFree& isFree) const -> Growth
    {
        auto growth{ Extend(tree, target, isFree) };
        while (growth == Growth::Advanced)
            growth = Extend(tree, target, isFree);
        return growth;
    }

    template<typename T, std::size_t Dof, std::size_t MaxNodes>
    void RrtConnect<T, Dof, MaxNodes>::Seed(Tree& tree, const JointVector& configuration)
    {
        tree.configuration[0] = configuration;
        tree.parent[0] = root;
        tree.size = 1;
    }

    template<typename T, std::size_t Dof, std::size_t MaxNodes>
    void RrtConnect<T, Dof, MaxNodes>::BuildPath()
    {
        const auto& fromStart{ trees[0] };
        const auto& fromGoal{ trees[1] };

        std::size_t depth{ 0 };
        for (auto node{ static_cast<std::uint16_t>(fromStart.size - 1) }; node != root; node = fromStart.parent[node])
            ++depth;

        std::size_t slot{ depth };
        for (auto node{ static_cast<std::uint16_t>(fromStart.size - 1) }; node != root; node = fromStart.parent[node])
            path[--slot] = fromStart.configuration[node];

        pathLength = depth;
        for (auto node{ fromGoal.parent[fromGoal.size - 1] }; node != root; node = fromGoal.parent[node])
            path[pathLength++] = fromGoal.configuration[node];
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class RrtConnect<float, 2, 256>;
    extern template class RrtConnect<float, 2, 10>;
    extern template PlanStatus RrtConnect<float, 2, 256>::Plan<math::Vector<float, 2> (&)(), bool (&)(const math::Vector<float, 2>&)>(
        const math::Vector<float, 2>&, const math::Vector<float, 2>&, math::Vector<float, 2> (&)(), bool (&)(const math::Vector<float, 2>&));
    extern template PlanStatus RrtConnect<float, 2, 10>::Plan<math::Vector<float, 2> (&)(), bool (&)(const math::Vector<float, 2>&)>(
        const math::Vector<float, 2>&, const math::Vector<float, 2>&, math::Vector<float, 2> (&)(), bool (&)(const math::Vector<float, 2>&));
    extern template void RrtConnect<float, 2, 256>::Shortcut<float (&)(), bool (&)(const math::Vector<float, 2>&)>(std::size_t, float (&)(), bool (&)(const math::Vector<float, 2>&));
#endif
}
