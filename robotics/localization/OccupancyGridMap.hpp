#pragma once

#include "infra/util/ReallyAssert.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/math/SE2Transform.hpp"
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdlib>
#include <span>
#include <type_traits>

namespace localization
{
    template<typename T>
    struct RangeBeam
    {
        static_assert(std::is_floating_point_v<T>, "RangeBeam supports floating-point types");

        T range{};
        T bearing{};
    };

    template<typename T>
    struct OccupancyGridParameters
    {
        static_assert(std::is_floating_point_v<T>, "OccupancyGridParameters supports floating-point types");

        T resolution{};
        math::Vector2<T> origin{};
        T logOddsOccupied{};
        T logOddsFree{};
        T logOddsMin{};
        T logOddsMax{};
        T maxRange{};
    };

    struct GridCell
    {
        int x{};
        int y{};

        friend bool operator==(const GridCell&, const GridCell&) = default;
    };

    template<typename T, std::size_t Width, std::size_t Height>
    class OccupancyGridMap
    {
        static_assert(std::is_floating_point_v<T>, "OccupancyGridMap supports floating-point types");
        static_assert(Width > 0 && Height > 0, "OccupancyGridMap needs at least one cell");

    public:
        explicit OccupancyGridMap(const OccupancyGridParameters<T>& parameters);

        OPTIMIZE_FOR_SPEED void Integrate(const math::SE2Transform<T>& sensorPose, std::span<const RangeBeam<T>> beams);
        GridCell WorldToCell(const math::Vector2<T>& point) const;
        math::Vector2<T> CellCenter(GridCell cell) const;
        bool Contains(GridCell cell) const;
        T LogOdds(GridCell cell) const;
        void SetLogOdds(GridCell cell, T value);
        T Probability(GridCell cell) const;
        bool IsOccupied(GridCell cell, T threshold) const;
        const OccupancyGridParameters<T>& Parameters() const;

    private:
        std::size_t Index(GridCell cell) const;
        void TraceRay(GridCell cell, GridCell end, bool hit);

        std::array<T, Width * Height> logOdds{};
        OccupancyGridParameters<T> parameters;
    };

    template<typename T, std::size_t Width, std::size_t Height>
    OccupancyGridMap<T, Width, Height>::OccupancyGridMap(const OccupancyGridParameters<T>& parameters)
        : parameters{ parameters }
    {
        really_assert(parameters.resolution > T{} && parameters.maxRange > T{});
        really_assert(parameters.logOddsMin <= T{} && parameters.logOddsMax >= T{});
    }

    template<typename T, std::size_t Width, std::size_t Height>
    OPTIMIZE_FOR_SPEED void OccupancyGridMap<T, Width, Height>::Integrate(const math::SE2Transform<T>& sensorPose, std::span<const RangeBeam<T>> beams)
    {
        const auto start{ WorldToCell(sensorPose.translation) };
        for (const auto& beam : beams)
        {
            const bool hit{ beam.range < parameters.maxRange };
            const T range{ std::min(beam.range, parameters.maxRange) };
            const auto end{ WorldToCell(sensorPose.Apply(math::Vector2<T>{ range * math::Cos(beam.bearing), range * math::Sin(beam.bearing) })) };
            TraceRay(start, end, hit);
        }
    }

    template<typename T, std::size_t Width, std::size_t Height>
    GridCell OccupancyGridMap<T, Width, Height>::WorldToCell(const math::Vector2<T>& point) const
    {
        return GridCell{ static_cast<int>(math::Floor((point[0] - parameters.origin[0]) / parameters.resolution)),
            static_cast<int>(math::Floor((point[1] - parameters.origin[1]) / parameters.resolution)) };
    }

    template<typename T, std::size_t Width, std::size_t Height>
    math::Vector2<T> OccupancyGridMap<T, Width, Height>::CellCenter(GridCell cell) const
    {
        return math::Vector2<T>{ parameters.origin[0] + (static_cast<T>(cell.x) + T{ 0.5f }) * parameters.resolution,
            parameters.origin[1] + (static_cast<T>(cell.y) + T{ 0.5f }) * parameters.resolution };
    }

    template<typename T, std::size_t Width, std::size_t Height>
    bool OccupancyGridMap<T, Width, Height>::Contains(GridCell cell) const
    {
        return cell.x >= 0 && cell.y >= 0 && static_cast<std::size_t>(cell.x) < Width && static_cast<std::size_t>(cell.y) < Height;
    }

    template<typename T, std::size_t Width, std::size_t Height>
    T OccupancyGridMap<T, Width, Height>::LogOdds(GridCell cell) const
    {
        return logOdds[Index(cell)];
    }

    template<typename T, std::size_t Width, std::size_t Height>
    void OccupancyGridMap<T, Width, Height>::SetLogOdds(GridCell cell, T value)
    {
        logOdds[Index(cell)] = std::clamp(value, parameters.logOddsMin, parameters.logOddsMax);
    }

    template<typename T, std::size_t Width, std::size_t Height>
    T OccupancyGridMap<T, Width, Height>::Probability(GridCell cell) const
    {
        return T{ 1 } - T{ 1 } / (T{ 1 } + math::Exp(LogOdds(cell)));
    }

    template<typename T, std::size_t Width, std::size_t Height>
    bool OccupancyGridMap<T, Width, Height>::IsOccupied(GridCell cell, T threshold) const
    {
        return LogOdds(cell) > threshold;
    }

    template<typename T, std::size_t Width, std::size_t Height>
    const OccupancyGridParameters<T>& OccupancyGridMap<T, Width, Height>::Parameters() const
    {
        return parameters;
    }

    template<typename T, std::size_t Width, std::size_t Height>
    std::size_t OccupancyGridMap<T, Width, Height>::Index(GridCell cell) const
    {
        really_assert(Contains(cell));
        return static_cast<std::size_t>(cell.y) * Width + static_cast<std::size_t>(cell.x);
    }

    template<typename T, std::size_t Width, std::size_t Height>
    void OccupancyGridMap<T, Width, Height>::TraceRay(GridCell cell, GridCell end, bool hit)
    {
        const int dx{ std::abs(end.x - cell.x) };
        const int dy{ -std::abs(end.y - cell.y) };
        const int sx{ cell.x < end.x ? 1 : -1 };
        const int sy{ cell.y < end.y ? 1 : -1 };
        int error{ dx + dy };

        while (Contains(cell))
        {
            const bool last{ cell == end };
            SetLogOdds(cell, LogOdds(cell) + (last && hit ? parameters.logOddsOccupied : parameters.logOddsFree));
            if (last)
                return;

            const int doubled{ 2 * error };
            if (doubled >= dy)
            {
                error += dy;
                cell.x += sx;
            }
            if (doubled <= dx)
            {
                error += dx;
                cell.y += sy;
            }
        }
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class OccupancyGridMap<float, 10, 10>;
    extern template class OccupancyGridMap<float, 64, 64>;
#endif
}
