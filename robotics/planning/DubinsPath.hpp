#pragma once

#include "infra/util/ReallyAssert.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/math/SE2Transform.hpp"
#include "robotics/planning/CarPath.hpp"
#include <algorithm>
#include <array>
#include <cstddef>
#include <numbers>
#include <optional>
#include <type_traits>

namespace planning
{
    template<typename T>
    class DubinsPlanner
    {
        static_assert(std::is_floating_point_v<T>, "DubinsPlanner supports floating-point types");

    public:
        explicit DubinsPlanner(T turningRadius);

        CarPath<T> Shortest(const math::SE2Transform<T>& start, const math::SE2Transform<T>& goal) const;

    private:
        struct Geometry
        {
            T d;
            T alpha;
            T beta;
            T sa;
            T ca;
            T sb;
            T cb;
            T cab;
        };

        using Lengths = std::array<T, 3>;
        using Word = std::array<Steering, 3>;

        static constexpr T twoPi{ T{ 2 } * std::numbers::pi_v<T> };
        static constexpr T tolerance{ T{ 1e-5f } };
        static constexpr std::array<Word, 6> words{ { { Steering::Left, Steering::Straight, Steering::Left },
            { Steering::Right, Steering::Straight, Steering::Right },
            { Steering::Left, Steering::Straight, Steering::Right },
            { Steering::Right, Steering::Straight, Steering::Left },
            { Steering::Right, Steering::Left, Steering::Right },
            { Steering::Left, Steering::Right, Steering::Left } } };

        static T Mod2Pi(T angle);
        static std::optional<T> Straight(T squared);
        static std::optional<T> Middle(T cosine);
        static Lengths Lsl(const Geometry& g);
        static Lengths Rsr(const Geometry& g);
        static std::optional<Lengths> Lsr(const Geometry& g);
        static std::optional<Lengths> Rsl(const Geometry& g);
        static std::optional<Lengths> Rlr(const Geometry& g);
        static std::optional<Lengths> Lrl(const Geometry& g);

        T turningRadius;
    };

    template<typename T>
    DubinsPlanner<T>::DubinsPlanner(T turningRadius)
        : turningRadius{ turningRadius }
    {
        really_assert(turningRadius > T{});
    }

    template<typename T>
    CarPath<T> DubinsPlanner<T>::Shortest(const math::SE2Transform<T>& start, const math::SE2Transform<T>& goal) const
    {
        const T dx{ goal.translation[0] - start.translation[0] };
        const T dy{ goal.translation[1] - start.translation[1] };
        const T distance{ math::Hypot(dx, dy) };
        const T theta{ distance > T{} ? math::Atan2(dy, dx) : start.Heading() };
        const T alpha{ Mod2Pi(start.Heading() - theta) };
        const T beta{ Mod2Pi(goal.Heading() - theta) };
        const Geometry geometry{ distance / turningRadius, alpha, beta, math::Sin(alpha), math::Cos(alpha), math::Sin(beta), math::Cos(beta), math::Cos(alpha - beta) };

        const std::array<std::optional<Lengths>, 6> candidates{ Lsl(geometry), Rsr(geometry), Lsr(geometry), Rsl(geometry), Rlr(geometry), Lrl(geometry) };

        std::size_t best{ candidates.size() };
        T bestLength{};
        for (std::size_t k = 0; k < candidates.size(); ++k)
        {
            if (!candidates[k])
                continue;

            const auto& lengths{ *candidates[k] };
            const T total{ lengths[0] + lengths[1] + lengths[2] };
            if (best == candidates.size() || total < bestLength)
            {
                best = k;
                bestLength = total;
            }
        }

        really_assert(best < candidates.size());

        CarPath<T> path{};
        path.count = 3;
        path.turningRadius = turningRadius;
        for (std::size_t k = 0; k < 3; ++k)
            path.segments[k] = CarPathSegment<T>{ words[best][k], (*candidates[best])[k] * turningRadius };
        return path;
    }

    template<typename T>
    T DubinsPlanner<T>::Mod2Pi(T angle)
    {
        const T wrapped{ angle - twoPi * math::Floor(angle / twoPi) };
        return wrapped > twoPi - tolerance ? T{} : wrapped;
    }

    template<typename T>
    std::optional<T> DubinsPlanner<T>::Straight(T squared)
    {
        if (squared < -tolerance)
            return std::nullopt;
        return math::Sqrt(std::max(squared, T{}));
    }

    template<typename T>
    std::optional<T> DubinsPlanner<T>::Middle(T cosine)
    {
        if (math::Abs(cosine) > T{ 1 } + tolerance)
            return std::nullopt;
        return Mod2Pi(twoPi - math::Acos(std::clamp(cosine, T{ -1 }, T{ 1 })));
    }

    template<typename T>
    auto DubinsPlanner<T>::Lsl(const Geometry& g) -> Lengths
    {
        const T x{ g.d + g.sa - g.sb };
        const T y{ g.cb - g.ca };
        const T tau{ math::Atan2(y, x) };
        return Lengths{ Mod2Pi(tau - g.alpha), math::Hypot(x, y), Mod2Pi(g.beta - tau) };
    }

    template<typename T>
    auto DubinsPlanner<T>::Rsr(const Geometry& g) -> Lengths
    {
        const T x{ g.d - g.sa + g.sb };
        const T y{ g.ca - g.cb };
        const T tau{ math::Atan2(y, x) };
        return Lengths{ Mod2Pi(g.alpha - tau), math::Hypot(x, y), Mod2Pi(tau - g.beta) };
    }

    template<typename T>
    auto DubinsPlanner<T>::Lsr(const Geometry& g) -> std::optional<Lengths>
    {
        const auto p{ Straight(T{ -2 } + g.d * g.d + T{ 2 } * g.cab + T{ 2 } * g.d * (g.sa + g.sb)) };
        if (!p)
            return std::nullopt;
        const T tau{ math::Atan2(-g.ca - g.cb, g.d + g.sa + g.sb) - math::Atan2(T{ -2 }, *p) };
        return Lengths{ Mod2Pi(tau - g.alpha), *p, Mod2Pi(tau - g.beta) };
    }

    template<typename T>
    auto DubinsPlanner<T>::Rsl(const Geometry& g) -> std::optional<Lengths>
    {
        const auto p{ Straight(T{ -2 } + g.d * g.d + T{ 2 } * g.cab - T{ 2 } * g.d * (g.sa + g.sb)) };
        if (!p)
            return std::nullopt;
        const T tau{ math::Atan2(g.ca + g.cb, g.d - g.sa - g.sb) - math::Atan2(T{ 2 }, *p) };
        return Lengths{ Mod2Pi(g.alpha - tau), *p, Mod2Pi(g.beta - tau) };
    }

    template<typename T>
    auto DubinsPlanner<T>::Rlr(const Geometry& g) -> std::optional<Lengths>
    {
        const auto p{ Middle((T{ 6 } - g.d * g.d + T{ 2 } * g.cab + T{ 2 } * g.d * (g.sa - g.sb)) / T{ 8 }) };
        if (!p)
            return std::nullopt;
        const T phi{ math::Atan2(g.ca - g.cb, g.d - g.sa + g.sb) };
        const T t{ Mod2Pi(g.alpha - phi + *p / T{ 2 }) };
        return Lengths{ t, *p, Mod2Pi(g.alpha - g.beta - t + *p) };
    }

    template<typename T>
    auto DubinsPlanner<T>::Lrl(const Geometry& g) -> std::optional<Lengths>
    {
        const auto p{ Middle((T{ 6 } - g.d * g.d + T{ 2 } * g.cab + T{ 2 } * g.d * (g.sb - g.sa)) / T{ 8 }) };
        if (!p)
            return std::nullopt;
        const T phi{ math::Atan2(g.ca - g.cb, g.d + g.sa - g.sb) };
        const T t{ Mod2Pi(-g.alpha - phi + *p / T{ 2 }) };
        return Lengths{ t, *p, Mod2Pi(g.beta - g.alpha - t + *p) };
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class DubinsPlanner<float>;
#endif
}
