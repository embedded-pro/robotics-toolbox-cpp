#pragma once

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/math/SE2Transform.hpp"
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace planning
{
    enum class Steering : std::uint8_t
    {
        Left,
        Straight,
        Right
    };

    template<typename T>
    struct CarPathSegment
    {
        static_assert(std::is_floating_point_v<T>, "CarPathSegment supports floating-point types");

        Steering steering{ Steering::Straight };
        T length{};
    };

    template<typename T>
    struct CarPath
    {
        static_assert(std::is_floating_point_v<T>, "CarPath supports floating-point types");

        T Length() const;
        math::SE2Transform<T> Sample(const math::SE2Transform<T>& start, T s) const;

        std::array<CarPathSegment<T>, 5> segments{};
        std::size_t count{ 0 };
        T turningRadius{};
    };

    template<typename T>
    T CarPath<T>::Length() const
    {
        T total{};
        for (std::size_t k = 0; k < count; ++k)
            total += math::Abs(segments[k].length);
        return total;
    }

    template<typename T>
    math::SE2Transform<T> CarPath<T>::Sample(const math::SE2Transform<T>& start, T s) const
    {
        auto pose{ start };
        T remaining{ std::clamp(s, T{}, Length()) };
        for (std::size_t k = 0; k < count; ++k)
        {
            const auto& segment{ segments[k] };
            const T take{ std::min(remaining, math::Abs(segment.length)) };
            const T direction{ segment.length < T{} ? T{ -1 } : T{ 1 } };
            T curvature{};
            if (segment.steering == Steering::Left)
                curvature = T{ 1 } / turningRadius;
            else if (segment.steering == Steering::Right)
                curvature = -T{ 1 } / turningRadius;

            pose = pose * math::SE2Transform<T>::Exp(math::Vector3<T>{ direction, T{}, direction * curvature }, take);
            remaining -= take;
        }
        return pose;
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template struct CarPath<float>;
#endif
}
