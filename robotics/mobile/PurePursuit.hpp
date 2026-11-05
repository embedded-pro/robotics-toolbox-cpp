#pragma once

#include "infra/util/ReallyAssert.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/math/SE2Transform.hpp"
#include "robotics/mobile/PlanarPath.hpp"
#include <algorithm>
#include <cstddef>
#include <optional>
#include <type_traits>

namespace mobile
{
    template<typename T>
    struct PurePursuitParameters
    {
        static_assert(std::is_floating_point_v<T>, "PurePursuitParameters supports floating-point types");

        T minLookahead{};
        T maxLookahead{};
        T lookaheadGain{};
        T goalTolerance{};
        std::size_t searchWindow{ 1 };
    };

    template<typename T>
    struct PursuitCommand
    {
        static_assert(std::is_floating_point_v<T>, "PursuitCommand supports floating-point types");

        T curvature{};
        math::Vector2<T> goal{};
        bool reached{ false };
    };

    template<typename T, std::size_t MaxPoints>
    class PurePursuit
    {
        static_assert(std::is_floating_point_v<T>, "PurePursuit supports floating-point types");

    public:
        using Path = PlanarPath<T, MaxPoints>;

        PurePursuit(const Path& path, const PurePursuitParameters<T>& parameters);

        OPTIMIZE_FOR_SPEED PursuitCommand<T> Compute(const math::SE2Transform<T>& pose, T speed);
        void Restart();
        T Progress() const;

    private:
        std::optional<math::Vector2<T>> CircleExit(const math::Vector2<T>& centre, T radius) const;
        math::Vector2<T> Goal(const math::Vector2<T>& position, T lookahead) const;

        const Path& path;
        PurePursuitParameters<T> parameters;
        T progress{};
        std::size_t segment{ 0 };
    };

    template<typename T, std::size_t MaxPoints>
    PurePursuit<T, MaxPoints>::PurePursuit(const Path& path, const PurePursuitParameters<T>& parameters)
        : path{ path }
        , parameters{ parameters }
    {}

    template<typename T, std::size_t MaxPoints>
    OPTIMIZE_FOR_SPEED PursuitCommand<T> PurePursuit<T, MaxPoints>::Compute(const math::SE2Transform<T>& pose, T speed)
    {
        really_assert(path.Size() >= 2);

        const T lookahead{ std::clamp(parameters.minLookahead + parameters.lookaheadGain * math::Abs(speed), parameters.minLookahead, parameters.maxLookahead) };
        const auto& position{ pose.translation };
        const auto projection{ path.Project(position, segment, parameters.searchWindow) };
        if (projection.arcLength >= progress)
        {
            progress = projection.arcLength;
            segment = projection.segment;
        }

        const auto goal{ Goal(position, lookahead) };
        const auto body{ pose.Inverse().Apply(goal) };
        const T squared{ body[0] * body[0] + body[1] * body[1] };
        const auto& last{ path.Point(path.Size() - 1) };
        const bool reached{ math::Hypot(position[0] - last[0], position[1] - last[1]) <= parameters.goalTolerance };
        return PursuitCommand<T>{ squared < T{ 1e-12f } ? T{} : T{ 2 } * body[1] / squared, goal, reached };
    }

    template<typename T, std::size_t MaxPoints>
    void PurePursuit<T, MaxPoints>::Restart()
    {
        progress = T{};
        segment = 0;
    }

    template<typename T, std::size_t MaxPoints>
    T PurePursuit<T, MaxPoints>::Progress() const
    {
        return progress;
    }

    template<typename T, std::size_t MaxPoints>
    std::optional<math::Vector2<T>> PurePursuit<T, MaxPoints>::CircleExit(const math::Vector2<T>& centre, T radius) const
    {
        for (std::size_t k = segment; k < path.Segments(); ++k)
        {
            const auto& start{ path.Point(k) };
            const math::Vector2<T> delta{ path.Point(k + 1) - start };
            const math::Vector2<T> offset{ start - centre };
            const T a{ delta[0] * delta[0] + delta[1] * delta[1] };
            const T b{ T{ 2 } * (offset[0] * delta[0] + offset[1] * delta[1]) };
            const T c{ offset[0] * offset[0] + offset[1] * offset[1] - radius * radius };
            const T discriminant{ b * b - T{ 4 } * a * c };
            if (discriminant < T{})
                continue;

            const T t{ (-b + math::Sqrt(discriminant)) / (T{ 2 } * a) };
            if (t >= T{} && t <= T{ 1 } && path.ArcLength(k) + t * math::Sqrt(a) >= progress)
                return math::Vector2<T>{ start + delta * t };
        }
        return std::nullopt;
    }

    template<typename T, std::size_t MaxPoints>
    math::Vector2<T> PurePursuit<T, MaxPoints>::Goal(const math::Vector2<T>& position, T lookahead) const
    {
        const auto exit{ CircleExit(position, lookahead) };
        if (exit)
            return *exit;

        const auto& last{ path.Point(path.Size() - 1) };
        if (math::Hypot(position[0] - last[0], position[1] - last[1]) <= lookahead)
            return last;
        return path.PointAt(std::min(progress + lookahead, path.Length()));
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class PurePursuit<float, 8>;
    extern template class PurePursuit<float, 32>;
#endif
}
