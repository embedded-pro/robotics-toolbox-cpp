#pragma once

#include "infra/util/ReallyAssert.hpp"
#include "numerical/filters/active/ParticleFilter.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/math/SE2Transform.hpp"
#include "robotics/localization/OccupancyGridMap.hpp"
#include "robotics/mobile/PlanarOdometry.hpp"
#include <algorithm>
#include <array>
#include <cstddef>
#include <limits>
#include <numbers>
#include <span>
#include <type_traits>

namespace localization
{
    template<typename T>
    struct MclParameters
    {
        static_assert(std::is_floating_point_v<T>, "MclParameters supports floating-point types");

        mobile::OdometryNoise<T> motion{};
        T hitSigma{};
        T hitWeight{};
        T randomWeight{};
        T maxRange{};
        T resampleFraction{};
        std::size_t beamStride{ 1 };
    };

    template<typename T, std::size_t Width, std::size_t Height>
    class LikelihoodField
    {
        static_assert(std::is_floating_point_v<T>, "LikelihoodField supports floating-point types");

    public:
        LikelihoodField(const OccupancyGridMap<T, Width, Height>& map, T occupiedThreshold, T maxDistance);

        T Distance(const math::Vector2<T>& world) const;

    private:
        using Cells = std::array<T, Width * Height>;

        static std::size_t Index(std::size_t x, std::size_t y);
        static void Relax(Cells& cells, std::size_t x, std::size_t y, int dx, int dy, T weight);
        static void ForwardPass(Cells& cells);
        static void BackwardPass(Cells& cells);

        Cells distance{};
        OccupancyGridParameters<T> grid;
        T maxDistance;
    };

    template<typename T, std::size_t NumParticles, std::size_t Width, std::size_t Height>
    class MonteCarloLocalization
    {
        static_assert(std::is_floating_point_v<T>, "MonteCarloLocalization supports floating-point types");

    public:
        using Filter = filters::ParticleFilter<T, 3, NumParticles>;

        MonteCarloLocalization(const LikelihoodField<T, Width, Height>& field, const math::SE2Transform<T>& sensorMount, const MclParameters<T>& parameters);

        template<class Draw>
        void Initialize(Draw&& draw);
        template<class Normal>
        void InitializeAround(const math::SE2Transform<T>& pose, const math::Vector3<T>& sigma, Normal&& normal);
        template<class Normal>
        OPTIMIZE_FOR_SPEED void Predict(const math::SE2Transform<T>& increment, Normal&& normal);
        template<class Uniform>
        OPTIMIZE_FOR_SPEED bool Update(std::span<const RangeBeam<T>> scan, Uniform&& uniform);

        math::SE2Transform<T> Estimate() const;
        const Filter& Particles() const;

    private:
        T LogLikelihood(const math::Vector3<T>& pose, std::span<const RangeBeam<T>> scan) const;

        Filter filter;
        const LikelihoodField<T, Width, Height>& field;
        math::SE2Transform<T> sensorMount;
        MclParameters<T> parameters;
    };

    template<typename T, std::size_t Width, std::size_t Height>
    LikelihoodField<T, Width, Height>::LikelihoodField(const OccupancyGridMap<T, Width, Height>& map, T occupiedThreshold, T maxDistance)
        : grid{ map.Parameters() }
        , maxDistance{ maxDistance }
    {
        for (std::size_t y = 0; y < Height; ++y)
            for (std::size_t x = 0; x < Width; ++x)
                distance[Index(x, y)] = map.IsOccupied(GridCell{ static_cast<int>(x), static_cast<int>(y) }, occupiedThreshold) ? T{} : std::numeric_limits<T>::infinity();

        ForwardPass(distance);
        BackwardPass(distance);

        for (auto& cell : distance)
            cell = std::min(cell * grid.resolution, maxDistance);
    }

    template<typename T, std::size_t Width, std::size_t Height>
    T LikelihoodField<T, Width, Height>::Distance(const math::Vector2<T>& world) const
    {
        const T column{ math::Floor((world[0] - grid.origin[0]) / grid.resolution) };
        const T row{ math::Floor((world[1] - grid.origin[1]) / grid.resolution) };
        const bool inside{ column >= T{} && row >= T{} && column < static_cast<T>(Width) && row < static_cast<T>(Height) };
        return inside ? distance[Index(static_cast<std::size_t>(column), static_cast<std::size_t>(row))] : maxDistance;
    }

    template<typename T, std::size_t Width, std::size_t Height>
    std::size_t LikelihoodField<T, Width, Height>::Index(std::size_t x, std::size_t y)
    {
        return y * Width + x;
    }

    template<typename T, std::size_t Width, std::size_t Height>
    void LikelihoodField<T, Width, Height>::Relax(Cells& cells, std::size_t x, std::size_t y, int dx, int dy, T weight)
    {
        const auto nx{ static_cast<std::ptrdiff_t>(x) + dx };
        const auto ny{ static_cast<std::ptrdiff_t>(y) + dy };
        const bool inside{ nx >= 0 && ny >= 0 && nx < static_cast<std::ptrdiff_t>(Width) && ny < static_cast<std::ptrdiff_t>(Height) };
        const T candidate{ inside ? cells[Index(static_cast<std::size_t>(nx), static_cast<std::size_t>(ny))] + weight : std::numeric_limits<T>::infinity() };
        cells[Index(x, y)] = std::min(cells[Index(x, y)], candidate);
    }

    template<typename T, std::size_t Width, std::size_t Height>
    void LikelihoodField<T, Width, Height>::ForwardPass(Cells& cells)
    {
        const T diagonal{ std::numbers::sqrt2_v<T> };
        for (std::size_t y = 0; y < Height; ++y)
            for (std::size_t x = 0; x < Width; ++x)
            {
                Relax(cells, x, y, -1, 0, T{ 1 });
                Relax(cells, x, y, 0, -1, T{ 1 });
                Relax(cells, x, y, -1, -1, diagonal);
                Relax(cells, x, y, 1, -1, diagonal);
            }
    }

    template<typename T, std::size_t Width, std::size_t Height>
    void LikelihoodField<T, Width, Height>::BackwardPass(Cells& cells)
    {
        const T diagonal{ std::numbers::sqrt2_v<T> };
        for (std::size_t row = Height; row > 0; --row)
            for (std::size_t column = Width; column > 0; --column)
            {
                const std::size_t x{ column - 1 };
                const std::size_t y{ row - 1 };
                Relax(cells, x, y, 1, 0, T{ 1 });
                Relax(cells, x, y, 0, 1, T{ 1 });
                Relax(cells, x, y, 1, 1, diagonal);
                Relax(cells, x, y, -1, 1, diagonal);
            }
    }

    template<typename T, std::size_t NumParticles, std::size_t Width, std::size_t Height>
    MonteCarloLocalization<T, NumParticles, Width, Height>::MonteCarloLocalization(const LikelihoodField<T, Width, Height>& field, const math::SE2Transform<T>& sensorMount,
        const MclParameters<T>& parameters)
        : field{ field }
        , sensorMount{ sensorMount }
        , parameters{ parameters }
    {
        really_assert(parameters.beamStride > 0 && parameters.maxRange > T{} && parameters.hitSigma > T{});
    }

    template<typename T, std::size_t NumParticles, std::size_t Width, std::size_t Height>
    template<class Draw>
    void MonteCarloLocalization<T, NumParticles, Width, Height>::Initialize(Draw&& draw)
    {
        filter.Initialize([&draw](std::size_t i)
            {
                return draw(i).Pose();
            });
    }

    template<typename T, std::size_t NumParticles, std::size_t Width, std::size_t Height>
    template<class Normal>
    void MonteCarloLocalization<T, NumParticles, Width, Height>::InitializeAround(const math::SE2Transform<T>& pose, const math::Vector3<T>& sigma, Normal&& normal)
    {
        filter.Initialize([&pose, &sigma, &normal](std::size_t)
            {
                const T dx{ sigma[0] * normal() };
                const T dy{ sigma[1] * normal() };
                const T dtheta{ sigma[2] * normal() };
                return (pose * math::SE2Transform<T>::FromPose(dx, dy, dtheta)).Pose();
            });
    }

    template<typename T, std::size_t NumParticles, std::size_t Width, std::size_t Height>
    template<class Normal>
    OPTIMIZE_FOR_SPEED void MonteCarloLocalization<T, NumParticles, Width, Height>::Predict(const math::SE2Transform<T>& increment, Normal&& normal)
    {
        const auto delta{ increment.Pose() };
        const auto deviation{ mobile::IncrementDeviation(increment, parameters.motion) };
        filter.Predict([&delta, &deviation, &normal](math::Vector3<T>& particle, std::size_t)
            {
                const T dx{ delta[0] + deviation[0] * normal() };
                const T dy{ delta[1] + deviation[1] * normal() };
                const T dtheta{ delta[2] + deviation[2] * normal() };
                particle = (math::SE2Transform<T>::FromPose(particle[0], particle[1], particle[2]) * math::SE2Transform<T>::FromPose(dx, dy, dtheta)).Pose();
            });
    }

    template<typename T, std::size_t NumParticles, std::size_t Width, std::size_t Height>
    template<class Uniform>
    OPTIMIZE_FOR_SPEED bool MonteCarloLocalization<T, NumParticles, Width, Height>::Update(std::span<const RangeBeam<T>> scan, Uniform&& uniform)
    {
        filter.Update([this, scan](const math::Vector3<T>& particle)
            {
                return LogLikelihood(particle, scan);
            });
        return filter.ResampleIfNeeded(parameters.resampleFraction, uniform());
    }

    template<typename T, std::size_t NumParticles, std::size_t Width, std::size_t Height>
    math::SE2Transform<T> MonteCarloLocalization<T, NumParticles, Width, Height>::Estimate() const
    {
        const auto particles{ filter.Particles() };
        const auto weights{ filter.Weights() };
        T x{};
        T y{};
        T sine{};
        T cosine{};
        for (std::size_t i = 0; i < NumParticles; ++i)
        {
            x += weights[i] * particles[i][0];
            y += weights[i] * particles[i][1];
            sine += weights[i] * math::Sin(particles[i][2]);
            cosine += weights[i] * math::Cos(particles[i][2]);
        }
        return math::SE2Transform<T>::FromPose(x, y, math::Atan2(sine, cosine));
    }

    template<typename T, std::size_t NumParticles, std::size_t Width, std::size_t Height>
    auto MonteCarloLocalization<T, NumParticles, Width, Height>::Particles() const -> const Filter&
    {
        return filter;
    }

    template<typename T, std::size_t NumParticles, std::size_t Width, std::size_t Height>
    T MonteCarloLocalization<T, NumParticles, Width, Height>::LogLikelihood(const math::Vector3<T>& pose, std::span<const RangeBeam<T>> scan) const
    {
        const auto sensor{ math::SE2Transform<T>::FromPose(pose[0], pose[1], pose[2]) * sensorMount };
        const T floor{ parameters.randomWeight / parameters.maxRange };
        const T spread{ T{ 2 } * parameters.hitSigma * parameters.hitSigma };
        T logLikelihood{};
        for (std::size_t k = 0; k < scan.size(); k += parameters.beamStride)
        {
            const auto& beam{ scan[k] };
            if (beam.range >= parameters.maxRange)
                continue;

            const T d{ field.Distance(sensor.Apply(math::Vector2<T>{ beam.range * math::Cos(beam.bearing), beam.range * math::Sin(beam.bearing) })) };
            logLikelihood += math::Log(parameters.hitWeight * math::Exp(-d * d / spread) + floor);
        }
        return logLikelihood;
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class LikelihoodField<float, 40, 40>;
    extern template class LikelihoodField<float, 64, 64>;
    extern template class MonteCarloLocalization<float, 4, 40, 40>;
    extern template class MonteCarloLocalization<float, 27, 40, 40>;
    extern template class MonteCarloLocalization<float, 128, 64, 64>;
    extern template void MonteCarloLocalization<float, 4, 40, 40>::InitializeAround<float (&)()>(const math::SE2Transform<float>&, const math::Vector3<float>&, float (&)());
    extern template void MonteCarloLocalization<float, 4, 40, 40>::Predict<float (&)()>(const math::SE2Transform<float>&, float (&)());
    extern template void MonteCarloLocalization<float, 27, 40, 40>::Initialize<math::SE2Transform<float> (&)(std::size_t)>(math::SE2Transform<float> (&)(std::size_t));
    extern template bool MonteCarloLocalization<float, 27, 40, 40>::Update<float (&)()>(std::span<const RangeBeam<float>>, float (&)());
#endif
}
