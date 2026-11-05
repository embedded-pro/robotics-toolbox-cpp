#pragma once

#include "infra/util/ReallyAssert.hpp"
#include "numerical/math/AngleUtilities.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/math/SE2Transform.hpp"
#include <algorithm>
#include <array>
#include <cstddef>
#include <limits>
#include <numbers>
#include <optional>
#include <span>
#include <type_traits>

namespace planning
{
    template<typename T>
    struct DwaParameters
    {
        static_assert(std::is_floating_point_v<T>, "DwaParameters supports floating-point types");

        T minSpeed{};
        T maxSpeed{};
        T maxTurnRate{};
        T maxAcceleration{};
        T maxTurnAcceleration{};
        T controlPeriod{};
        T predictionTime{};
        T robotRadius{};
        T maxClearance{};
        T headingWeight{};
        T clearanceWeight{};
        T speedWeight{};
    };

    template<typename T>
    struct VelocityWindow
    {
        static_assert(std::is_floating_point_v<T>, "VelocityWindow supports floating-point types");

        T vMin{};
        T vMax{};
        T omegaMin{};
        T omegaMax{};
    };

    template<typename T>
    struct DwaCommand
    {
        static_assert(std::is_floating_point_v<T>, "DwaCommand supports floating-point types");

        T v{};
        T omega{};
        T clearance{};
    };

    template<typename T, std::size_t NumV, std::size_t NumW>
    class DynamicWindowApproach
    {
        static_assert(std::is_floating_point_v<T>, "DynamicWindowApproach supports floating-point types");
        static_assert(NumV >= 2 && NumW >= 2, "DynamicWindowApproach needs at least two samples per axis");

    public:
        explicit DynamicWindowApproach(const DwaParameters<T>& parameters);

        VelocityWindow<T> Window(T v, T omega) const;
        OPTIMIZE_FOR_SPEED std::optional<DwaCommand<T>> Compute(T v, T omega, const math::Vector2<T>& goal, std::span<const math::Vector2<T>> obstacles) const;

        static T ArcClearance(T v, T omega, const math::Vector2<T>& obstacle, T robotRadius, T maxClearance);

    private:
        struct Sample
        {
            T v;
            T omega;
            T heading;
            T clearance;
            bool admissible;
        };

        struct Maxima
        {
            T heading;
            T clearance;
            T speed;
        };

        using Samples = std::array<Sample, NumV * NumW>;

        static constexpr T pi{ std::numbers::pi_v<T> };
        static constexpr T straightCurvature{ T{ 1e-6f } };

        static T Mod2Pi(T angle);
        static T Interpolate(T low, T high, std::size_t index, std::size_t count);
        static T Normalise(T value, T maximum);
        Sample Evaluate(T v, T omega, const math::Vector2<T>& goal, std::span<const math::Vector2<T>> obstacles) const;
        DwaCommand<T> Select(const Samples& samples, const Maxima& maxima) const;

        DwaParameters<T> parameters;
    };

    template<typename T, std::size_t NumV, std::size_t NumW>
    DynamicWindowApproach<T, NumV, NumW>::DynamicWindowApproach(const DwaParameters<T>& parameters)
        : parameters{ parameters }
    {
        really_assert(parameters.minSpeed >= T{} && parameters.minSpeed <= parameters.maxSpeed);
        really_assert(parameters.maxTurnRate > T{} && parameters.maxAcceleration > T{} && parameters.maxTurnAcceleration > T{});
        really_assert(parameters.controlPeriod > T{} && parameters.predictionTime > T{} && parameters.robotRadius > T{} && parameters.maxClearance > T{});
    }

    template<typename T, std::size_t NumV, std::size_t NumW>
    VelocityWindow<T> DynamicWindowApproach<T, NumV, NumW>::Window(T v, T omega) const
    {
        const T speedStep{ parameters.maxAcceleration * parameters.controlPeriod };
        const T turnStep{ parameters.maxTurnAcceleration * parameters.controlPeriod };
        return VelocityWindow<T>{ std::max(parameters.minSpeed, v - speedStep), std::min(parameters.maxSpeed, v + speedStep),
            std::max(-parameters.maxTurnRate, omega - turnStep), std::min(parameters.maxTurnRate, omega + turnStep) };
    }

    template<typename T, std::size_t NumV, std::size_t NumW>
    OPTIMIZE_FOR_SPEED std::optional<DwaCommand<T>> DynamicWindowApproach<T, NumV, NumW>::Compute(T v, T omega, const math::Vector2<T>& goal, std::span<const math::Vector2<T>> obstacles) const
    {
        const auto window{ Window(v, omega) };
        Samples samples{};
        Maxima maxima{};
        bool anyAdmissible{ false };

        for (std::size_t k = 0; k < samples.size(); ++k)
        {
            const auto& sample{ samples[k] = Evaluate(Interpolate(window.vMin, window.vMax, k / NumW, NumV), Interpolate(window.omegaMin, window.omegaMax, k % NumW, NumW), goal, obstacles) };
            if (sample.admissible)
            {
                anyAdmissible = true;
                maxima = Maxima{ std::max(maxima.heading, sample.heading), std::max(maxima.clearance, sample.clearance), std::max(maxima.speed, sample.v) };
            }
        }

        if (!anyAdmissible)
            return std::nullopt;

        return Select(samples, maxima);
    }

    template<typename T, std::size_t NumV, std::size_t NumW>
    T DynamicWindowApproach<T, NumV, NumW>::ArcClearance(T v, T omega, const math::Vector2<T>& obstacle, T robotRadius, T maxClearance)
    {
        if (math::Hypot(obstacle[0], obstacle[1]) <= robotRadius)
            return T{};
        if (v <= T{})
            return maxClearance;

        const T curvature{ omega / v };
        if (math::Abs(curvature) < straightCurvature)
        {
            const bool ahead{ math::Abs(obstacle[1]) <= robotRadius && obstacle[0] > T{} };
            return ahead ? std::min(maxClearance, obstacle[0] - math::Sqrt(robotRadius * robotRadius - obstacle[1] * obstacle[1])) : maxClearance;
        }

        const T radius{ T{ 1 } / curvature };
        const T distance{ math::Hypot(obstacle[0], obstacle[1] - radius) };
        if (math::Abs(distance - math::Abs(radius)) > robotRadius)
            return maxClearance;

        const T halfContact{ math::Acos(std::clamp((radius * radius + distance * distance - robotRadius * robotRadius) / (T{ 2 } * math::Abs(radius) * distance), T{ -1 }, T{ 1 })) };
        const T startBearing{ math::Atan2(-radius, T{}) };
        const T obstacleBearing{ math::Atan2(obstacle[1] - radius, obstacle[0]) };
        const T sweep{ Mod2Pi(curvature > T{} ? obstacleBearing - startBearing : startBearing - obstacleBearing) };
        return std::min(maxClearance, math::Abs(radius) * Mod2Pi(sweep - halfContact));
    }

    template<typename T, std::size_t NumV, std::size_t NumW>
    T DynamicWindowApproach<T, NumV, NumW>::Mod2Pi(T angle)
    {
        const T twoPi{ T{ 2 } * pi };
        return angle - twoPi * math::Floor(angle / twoPi);
    }

    template<typename T, std::size_t NumV, std::size_t NumW>
    T DynamicWindowApproach<T, NumV, NumW>::Interpolate(T low, T high, std::size_t index, std::size_t count)
    {
        return low + static_cast<T>(index) * (high - low) / static_cast<T>(count - 1);
    }

    template<typename T, std::size_t NumV, std::size_t NumW>
    T DynamicWindowApproach<T, NumV, NumW>::Normalise(T value, T maximum)
    {
        return maximum > T{} ? value / maximum : value;
    }

    template<typename T, std::size_t NumV, std::size_t NumW>
    auto DynamicWindowApproach<T, NumV, NumW>::Evaluate(T v, T omega, const math::Vector2<T>& goal, std::span<const math::Vector2<T>> obstacles) const -> Sample
    {
        T clearance{ parameters.maxClearance };
        for (const auto& obstacle : obstacles)
            clearance = std::min(clearance, ArcClearance(v, omega, obstacle, parameters.robotRadius, parameters.maxClearance));

        const auto predicted{ math::SE2Transform<T>::Exp(math::Vector3<T>{ v, T{}, omega }, parameters.predictionTime) };
        const T bearing{ math::Atan2(goal[1] - predicted.translation[1], goal[0] - predicted.translation[0]) };
        const T heading{ pi - math::Abs(math::AngleDifference(bearing, predicted.Heading())) };
        return Sample{ v, omega, heading, clearance, v <= math::Sqrt(T{ 2 } * parameters.maxAcceleration * clearance) };
    }

    template<typename T, std::size_t NumV, std::size_t NumW>
    DwaCommand<T> DynamicWindowApproach<T, NumV, NumW>::Select(const Samples& samples, const Maxima& maxima) const
    {
        const Sample* best{ &samples.front() };
        T bestScore{ -std::numeric_limits<T>::infinity() };
        for (const auto& sample : samples)
        {
            if (!sample.admissible)
                continue;

            const T score{ parameters.headingWeight * Normalise(sample.heading, maxima.heading) + parameters.clearanceWeight * Normalise(sample.clearance, maxima.clearance) + parameters.speedWeight * Normalise(sample.v, maxima.speed) };
            if (score > bestScore)
            {
                best = &sample;
                bestScore = score;
            }
        }

        return DwaCommand<T>{ best->v, best->omega, best->clearance };
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class DynamicWindowApproach<float, 5, 11>;
    extern template class DynamicWindowApproach<float, 11, 21>;
#endif
}
