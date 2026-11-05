#pragma once

#include "infra/util/ReallyAssert.hpp"
#include "numerical/math/AngleUtilities.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/math/SE2Transform.hpp"
#include "robotics/planning/CarPath.hpp"
#include <algorithm>
#include <array>
#include <cstddef>
#include <limits>
#include <numbers>
#include <optional>
#include <type_traits>

namespace planning
{
    template<typename T>
    class ReedsSheppPlanner
    {
        static_assert(std::is_floating_point_v<T>, "ReedsSheppPlanner supports floating-point types");

    public:
        explicit ReedsSheppPlanner(T turningRadius);

        CarPath<T> Shortest(const math::SE2Transform<T>& start, const math::SE2Transform<T>& goal) const;

    private:
        using Triple = std::array<T, 3>;
        using Word = std::array<Steering, 5>;

        struct Lengths
        {
            std::array<T, 5> values;
            std::size_t count;
        };

        struct Best
        {
            CarPath<T> path{};
            T length{ std::numeric_limits<T>::infinity() };
        };

        struct Pose
        {
            T x;
            T y;
            T phi;
        };

        static constexpr T pi{ std::numbers::pi_v<T> };
        static constexpr T halfPi{ std::numbers::pi_v<T> / T{ 2 } };
        static constexpr T tolerance{ T{ 1e-5f } };
        static constexpr Word lsl{ Steering::Left, Steering::Straight, Steering::Left, Steering::Straight, Steering::Straight };
        static constexpr Word lsr{ Steering::Left, Steering::Straight, Steering::Right, Steering::Straight, Steering::Straight };
        static constexpr Word lrl{ Steering::Left, Steering::Right, Steering::Left, Steering::Straight, Steering::Straight };
        static constexpr Word lrlr{ Steering::Left, Steering::Right, Steering::Left, Steering::Right, Steering::Straight };
        static constexpr Word lrsl{ Steering::Left, Steering::Right, Steering::Straight, Steering::Left, Steering::Straight };
        static constexpr Word lrsr{ Steering::Left, Steering::Right, Steering::Straight, Steering::Right, Steering::Straight };
        static constexpr Word lrslr{ Steering::Left, Steering::Right, Steering::Straight, Steering::Left, Steering::Right };

        template<typename Formula, typename Builder>
        static void Consider(Best& best, Formula formula, Builder builder, const Word& word, const Pose& pose, bool backwards);
        static Steering Mirror(Steering steering);
        static Triple TauOmega(T u, T v, T xi, T eta, T phi);

        static Lengths Plain(const Triple& r);
        static Lengths OppositeTurns(const Triple& r);
        static Lengths EqualTurns(const Triple& r);
        static Lengths QuarterTurnFirst(const Triple& r);
        static Lengths QuarterTurnsAround(const Triple& r);

        static std::optional<Triple> LpSpLp(T x, T y, T phi);
        static std::optional<Triple> LpSpRp(T x, T y, T phi);
        static std::optional<Triple> LpRmL(T x, T y, T phi);
        static std::optional<Triple> LpRupLumRm(T x, T y, T phi);
        static std::optional<Triple> LpRumLumRp(T x, T y, T phi);
        static std::optional<Triple> LpRmSmLm(T x, T y, T phi);
        static std::optional<Triple> LpRmSmRm(T x, T y, T phi);
        static std::optional<Triple> LpRmSLmRp(T x, T y, T phi);

        T turningRadius;
    };

    template<typename T>
    ReedsSheppPlanner<T>::ReedsSheppPlanner(T turningRadius)
        : turningRadius{ turningRadius }
    {
        really_assert(turningRadius > T{});
    }

    template<typename T>
    CarPath<T> ReedsSheppPlanner<T>::Shortest(const math::SE2Transform<T>& start, const math::SE2Transform<T>& goal) const
    {
        const auto relative{ start.Inverse() * goal };
        const Pose pose{ relative.translation[0] / turningRadius, relative.translation[1] / turningRadius, relative.Heading() };
        const T cosine{ math::Cos(pose.phi) };
        const T sine{ math::Sin(pose.phi) };
        const Pose reversed{ pose.x * cosine + pose.y * sine, pose.x * sine - pose.y * cosine, pose.phi };

        Best best{};
        Consider(best, &LpSpLp, &Plain, lsl, pose, false);
        Consider(best, &LpSpRp, &Plain, lsr, pose, false);
        Consider(best, &LpRmL, &Plain, lrl, pose, false);
        Consider(best, &LpRmL, &Plain, lrl, reversed, true);
        Consider(best, &LpRupLumRm, &OppositeTurns, lrlr, pose, false);
        Consider(best, &LpRumLumRp, &EqualTurns, lrlr, pose, false);
        Consider(best, &LpRmSmLm, &QuarterTurnFirst, lrsl, pose, false);
        Consider(best, &LpRmSmRm, &QuarterTurnFirst, lrsr, pose, false);
        Consider(best, &LpRmSmLm, &QuarterTurnFirst, lrsl, reversed, true);
        Consider(best, &LpRmSmRm, &QuarterTurnFirst, lrsr, reversed, true);
        Consider(best, &LpRmSLmRp, &QuarterTurnsAround, lrslr, pose, false);

        auto path{ best.path };
        path.turningRadius = turningRadius;
        for (auto& segment : path.segments)
            segment.length *= turningRadius;
        return path;
    }

    template<typename T>
    template<typename Formula, typename Builder>
    void ReedsSheppPlanner<T>::Consider(Best& best, Formula formula, Builder builder, const Word& word, const Pose& pose, bool backwards)
    {
        for (std::size_t symmetry = 0; symmetry < 4; ++symmetry)
        {
            const bool timeFlip{ (symmetry & 1U) != 0 };
            const bool reflect{ (symmetry & 2U) != 0 };
            const auto solution{ formula(timeFlip ? -pose.x : pose.x, reflect ? -pose.y : pose.y, timeFlip != reflect ? -pose.phi : pose.phi) };
            if (!solution)
                continue;

            const auto lengths{ builder(*solution) };
            CarPath<T> candidate{};
            candidate.count = lengths.count;
            for (std::size_t k = 0; k < lengths.count; ++k)
            {
                const std::size_t source{ backwards ? lengths.count - 1 - k : k };
                candidate.segments[k] = CarPathSegment<T>{ reflect ? Mirror(word[source]) : word[source], timeFlip ? -lengths.values[source] : lengths.values[source] };
            }

            const T length{ candidate.Length() };
            if (length < best.length)
                best = Best{ candidate, length };
        }
    }

    template<typename T>
    Steering ReedsSheppPlanner<T>::Mirror(Steering steering)
    {
        using enum Steering;

        if (steering == Left)
            return Right;
        if (steering == Right)
            return Left;
        return Straight;
    }

    template<typename T>
    auto ReedsSheppPlanner<T>::TauOmega(T u, T v, T xi, T eta, T phi) -> Triple
    {
        const T delta{ math::WrapToPi(u - v) };
        const T a{ math::Sin(u) - math::Sin(delta) };
        const T b{ math::Cos(u) - math::Cos(delta) - T{ 1 } };
        const T t1{ math::Atan2(eta * a - xi * b, xi * a + eta * b) };
        const T t2{ T{ 2 } * (math::Cos(delta) - math::Cos(v) - math::Cos(u)) + T{ 3 } };
        const T tau{ t2 < T{} ? math::WrapToPi(t1 + pi) : math::WrapToPi(t1) };
        return Triple{ tau, math::WrapToPi(tau - u + v - phi), T{} };
    }

    template<typename T>
    auto ReedsSheppPlanner<T>::Plain(const Triple& r) -> Lengths
    {
        return Lengths{ { r[0], r[1], r[2], T{}, T{} }, 3 };
    }

    template<typename T>
    auto ReedsSheppPlanner<T>::OppositeTurns(const Triple& r) -> Lengths
    {
        return Lengths{ { r[0], r[1], -r[1], r[2], T{} }, 4 };
    }

    template<typename T>
    auto ReedsSheppPlanner<T>::EqualTurns(const Triple& r) -> Lengths
    {
        return Lengths{ { r[0], r[1], r[1], r[2], T{} }, 4 };
    }

    template<typename T>
    auto ReedsSheppPlanner<T>::QuarterTurnFirst(const Triple& r) -> Lengths
    {
        return Lengths{ { r[0], -halfPi, r[1], r[2], T{} }, 4 };
    }

    template<typename T>
    auto ReedsSheppPlanner<T>::QuarterTurnsAround(const Triple& r) -> Lengths
    {
        return Lengths{ { r[0], -halfPi, r[1], -halfPi, r[2] }, 5 };
    }

    template<typename T>
    auto ReedsSheppPlanner<T>::LpSpLp(T x, T y, T phi) -> std::optional<Triple>
    {
        const T xi{ x - math::Sin(phi) };
        const T eta{ y - T{ 1 } + math::Cos(phi) };
        const T t{ math::Atan2(eta, xi) };
        const T v{ math::WrapToPi(phi - t) };
        if (t < -tolerance || v < -tolerance)
            return std::nullopt;
        return Triple{ t, math::Hypot(xi, eta), v };
    }

    template<typename T>
    auto ReedsSheppPlanner<T>::LpSpRp(T x, T y, T phi) -> std::optional<Triple>
    {
        const T xi{ x + math::Sin(phi) };
        const T eta{ y - T{ 1 } - math::Cos(phi) };
        const T squared{ xi * xi + eta * eta };
        if (squared < T{ 4 })
            return std::nullopt;

        const T u{ math::Sqrt(squared - T{ 4 }) };
        const T t{ math::WrapToPi(math::Atan2(eta, xi) + math::Atan2(T{ 2 }, u)) };
        const T v{ math::WrapToPi(t - phi) };
        if (t < -tolerance || v < -tolerance)
            return std::nullopt;
        return Triple{ t, u, v };
    }

    template<typename T>
    auto ReedsSheppPlanner<T>::LpRmL(T x, T y, T phi) -> std::optional<Triple>
    {
        const T xi{ x - math::Sin(phi) };
        const T eta{ y - T{ 1 } + math::Cos(phi) };
        const T distance{ math::Hypot(xi, eta) };
        if (distance > T{ 4 })
            return std::nullopt;

        const T u{ T{ -2 } * math::Asin(distance / T{ 4 }) };
        const T t{ math::WrapToPi(math::Atan2(eta, xi) + u / T{ 2 } + pi) };
        if (t < -tolerance || u > tolerance)
            return std::nullopt;
        return Triple{ t, u, math::WrapToPi(phi - t + u) };
    }

    template<typename T>
    auto ReedsSheppPlanner<T>::LpRupLumRm(T x, T y, T phi) -> std::optional<Triple>
    {
        const T xi{ x + math::Sin(phi) };
        const T eta{ y - T{ 1 } - math::Cos(phi) };
        const T rho{ (T{ 2 } + math::Hypot(xi, eta)) / T{ 4 } };
        if (rho > T{ 1 })
            return std::nullopt;

        const T u{ math::Acos(rho) };
        const auto tauOmega{ TauOmega(u, -u, xi, eta, phi) };
        if (tauOmega[0] < -tolerance || tauOmega[1] > tolerance)
            return std::nullopt;
        return Triple{ tauOmega[0], u, tauOmega[1] };
    }

    template<typename T>
    auto ReedsSheppPlanner<T>::LpRumLumRp(T x, T y, T phi) -> std::optional<Triple>
    {
        const T xi{ x + math::Sin(phi) };
        const T eta{ y - T{ 1 } - math::Cos(phi) };
        const T rho{ (T{ 20 } - xi * xi - eta * eta) / T{ 16 } };
        if (rho < T{} || rho > T{ 1 })
            return std::nullopt;

        const T u{ -math::Acos(rho) };
        const auto tauOmega{ TauOmega(u, u, xi, eta, phi) };
        if (tauOmega[0] < -tolerance || tauOmega[1] < -tolerance)
            return std::nullopt;
        return Triple{ tauOmega[0], u, tauOmega[1] };
    }

    template<typename T>
    auto ReedsSheppPlanner<T>::LpRmSmLm(T x, T y, T phi) -> std::optional<Triple>
    {
        const T xi{ x - math::Sin(phi) };
        const T eta{ y - T{ 1 } + math::Cos(phi) };
        const T distance{ math::Hypot(xi, eta) };
        if (distance < T{ 2 })
            return std::nullopt;

        const T straight{ math::Sqrt(distance * distance - T{ 4 }) };
        const T u{ T{ 2 } - straight };
        const T t{ math::WrapToPi(math::Atan2(eta, xi) + math::Atan2(straight, T{ -2 })) };
        const T v{ math::WrapToPi(phi - halfPi - t) };
        if (t < -tolerance || u > tolerance || v > tolerance)
            return std::nullopt;
        return Triple{ t, u, v };
    }

    template<typename T>
    auto ReedsSheppPlanner<T>::LpRmSmRm(T x, T y, T phi) -> std::optional<Triple>
    {
        const T xi{ x + math::Sin(phi) };
        const T eta{ y - T{ 1 } - math::Cos(phi) };
        const T distance{ math::Hypot(eta, xi) };
        if (distance < T{ 2 })
            return std::nullopt;

        const T t{ math::Atan2(xi, -eta) };
        const T u{ T{ 2 } - distance };
        const T v{ math::WrapToPi(t + halfPi - phi) };
        if (t < -tolerance || u > tolerance || v > tolerance)
            return std::nullopt;
        return Triple{ t, u, v };
    }

    template<typename T>
    auto ReedsSheppPlanner<T>::LpRmSLmRp(T x, T y, T phi) -> std::optional<Triple>
    {
        const T xi{ x + math::Sin(phi) };
        const T eta{ y - T{ 1 } - math::Cos(phi) };
        const T distance{ math::Hypot(xi, eta) };
        if (distance < T{ 2 })
            return std::nullopt;

        const T u{ T{ 4 } - math::Sqrt(distance * distance - T{ 4 }) };
        const T t{ math::WrapToPi(math::Atan2((T{ 4 } - u) * xi - T{ 2 } * eta, T{ -2 } * xi + (u - T{ 4 }) * eta)) };
        const T v{ math::WrapToPi(t - phi) };
        if (u > tolerance || t < -tolerance || v < -tolerance)
            return std::nullopt;
        return Triple{ t, u, v };
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class ReedsSheppPlanner<float>;
#endif
}
