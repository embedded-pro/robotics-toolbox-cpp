#pragma once

#include "numerical/math/Math.hpp"
#include "numerical/math/SE2Transform.hpp"
#include <algorithm>
#include <array>
#include <cstddef>
#include <iterator>
#include <limits>
#include <span>
#include <type_traits>

namespace mobile
{
    template<typename T>
    struct PathProjection
    {
        static_assert(std::is_floating_point_v<T>, "PathProjection supports floating-point types");

        T arcLength{};
        math::Vector2<T> point{};
        T heading{};
        T crossTrack{};
        std::size_t segment{ 0 };
    };

    template<typename T, std::size_t MaxPoints>
    class PlanarPath
    {
        static_assert(std::is_floating_point_v<T>, "PlanarPath supports floating-point types");
        static_assert(MaxPoints >= 2, "A path needs at least two points");

    public:
        using Point2 = math::Vector2<T>;

        bool Assign(std::span<const Point2> waypoints);
        std::size_t Size() const;
        std::size_t Segments() const;
        T Length() const;
        const Point2& Point(std::size_t index) const;
        T ArcLength(std::size_t index) const;
        PathProjection<T> Project(const Point2& query, std::size_t firstSegment, std::size_t segmentWindow) const;
        Point2 PointAt(T arcLength) const;
        T HeadingAt(T arcLength) const;

    private:
        static constexpr T minimumSegment{ T{ 1e-6f } };

        static T Distance(const Point2& from, const Point2& to);
        std::size_t SegmentAt(T arcLength) const;

        std::array<Point2, MaxPoints> points{};
        std::array<T, MaxPoints> cumulative{};
        std::size_t count{ 0 };
    };

    template<typename T, std::size_t MaxPoints>
    bool PlanarPath<T, MaxPoints>::Assign(std::span<const Point2> waypoints)
    {
        if (waypoints.size() < 2 || waypoints.size() > MaxPoints)
            return false;
        for (std::size_t i = 1; i < waypoints.size(); ++i)
            if (Distance(waypoints[i - 1], waypoints[i]) < minimumSegment)
                return false;

        count = waypoints.size();
        cumulative[0] = T{};
        for (std::size_t i = 0; i < count; ++i)
        {
            points[i] = waypoints[i];
            if (i > 0)
                cumulative[i] = cumulative[i - 1] + Distance(points[i - 1], points[i]);
        }
        return true;
    }

    template<typename T, std::size_t MaxPoints>
    std::size_t PlanarPath<T, MaxPoints>::Size() const
    {
        return count;
    }

    template<typename T, std::size_t MaxPoints>
    std::size_t PlanarPath<T, MaxPoints>::Segments() const
    {
        return count - 1;
    }

    template<typename T, std::size_t MaxPoints>
    T PlanarPath<T, MaxPoints>::Length() const
    {
        return cumulative[count - 1];
    }

    template<typename T, std::size_t MaxPoints>
    auto PlanarPath<T, MaxPoints>::Point(std::size_t index) const -> const Point2&
    {
        return points[index];
    }

    template<typename T, std::size_t MaxPoints>
    T PlanarPath<T, MaxPoints>::ArcLength(std::size_t index) const
    {
        return cumulative[index];
    }

    template<typename T, std::size_t MaxPoints>
    PathProjection<T> PlanarPath<T, MaxPoints>::Project(const Point2& query, std::size_t firstSegment, std::size_t segmentWindow) const
    {
        const std::size_t first{ std::min(firstSegment, Segments() - 1) };
        const std::size_t last{ std::min(first + std::max<std::size_t>(segmentWindow, 1), Segments()) };

        PathProjection<T> best{};
        T bestDistance{ std::numeric_limits<T>::infinity() };
        for (std::size_t k = first; k < last; ++k)
        {
            const Point2 delta{ points[k + 1] - points[k] };
            const T length{ cumulative[k + 1] - cumulative[k] };
            const Point2 direction{ delta * (T{ 1 } / length) };
            const Point2 offset{ query - points[k] };
            const T along{ std::clamp(offset[0] * direction[0] + offset[1] * direction[1], T{}, length) };
            const Point2 closest{ points[k] + direction * along };
            const T distance{ Distance(query, closest) };
            if (distance < bestDistance)
            {
                bestDistance = distance;
                best = PathProjection<T>{ cumulative[k] + along, closest, math::Atan2(direction[1], direction[0]), direction[0] * offset[1] - direction[1] * offset[0], k };
            }
        }
        return best;
    }

    template<typename T, std::size_t MaxPoints>
    auto PlanarPath<T, MaxPoints>::PointAt(T arcLength) const -> Point2
    {
        if (arcLength <= T{})
            return points[0];
        if (arcLength >= Length())
            return points[count - 1];

        const std::size_t k{ SegmentAt(arcLength) };
        const T fraction{ (arcLength - cumulative[k]) / (cumulative[k + 1] - cumulative[k]) };
        return Point2{ points[k] + Point2{ points[k + 1] - points[k] } * fraction };
    }

    template<typename T, std::size_t MaxPoints>
    T PlanarPath<T, MaxPoints>::HeadingAt(T arcLength) const
    {
        const std::size_t k{ SegmentAt(std::clamp(arcLength, T{}, Length())) };
        return math::Atan2(points[k + 1][1] - points[k][1], points[k + 1][0] - points[k][0]);
    }

    template<typename T, std::size_t MaxPoints>
    T PlanarPath<T, MaxPoints>::Distance(const Point2& from, const Point2& to)
    {
        return math::Hypot(to[0] - from[0], to[1] - from[1]);
    }

    template<typename T, std::size_t MaxPoints>
    std::size_t PlanarPath<T, MaxPoints>::SegmentAt(T arcLength) const
    {
        const auto upper{ std::upper_bound(cumulative.begin(), cumulative.begin() + static_cast<std::ptrdiff_t>(count), arcLength) };
        const auto index{ static_cast<std::size_t>(std::distance(cumulative.begin(), upper)) };
        return std::min(index == 0 ? 0 : index - 1, Segments() - 1);
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class PlanarPath<float, 8>;
    extern template class PlanarPath<float, 32>;
#endif
}
