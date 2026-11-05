#pragma once

#include "infra/util/ReallyAssert.hpp"
#include "numerical/math/AngleUtilities.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/math/SE2Transform.hpp"
#include "robotics/mobile/CarLikeKinematics.hpp"
#include "robotics/mobile/PlanarPath.hpp"
#include <cstddef>
#include <type_traits>

namespace mobile
{
    template<typename T>
    struct StanleyParameters
    {
        static_assert(std::is_floating_point_v<T>, "StanleyParameters supports floating-point types");

        T gain{};
        T softening{};
        std::size_t searchWindow{ 1 };
    };

    template<typename T>
    struct StanleyError
    {
        static_assert(std::is_floating_point_v<T>, "StanleyError supports floating-point types");

        T crossTrack{};
        T headingError{};
    };

    template<typename T, std::size_t MaxPoints>
    class StanleyController
    {
        static_assert(std::is_floating_point_v<T>, "StanleyController supports floating-point types");

    public:
        using Path = PlanarPath<T, MaxPoints>;

        StanleyController(const Path& path, const CarLikeKinematics<T>& vehicle, const StanleyParameters<T>& parameters);

        OPTIMIZE_FOR_SPEED T Compute(const math::SE2Transform<T>& rearAxlePose, T speed, T previousSteer, T dt);
        StanleyError<T> Error(const math::SE2Transform<T>& rearAxlePose);
        void Restart();

    private:
        const Path& path;
        const CarLikeKinematics<T>& vehicle;
        StanleyParameters<T> parameters;
        T progress{};
        std::size_t segment{ 0 };
    };

    template<typename T, std::size_t MaxPoints>
    StanleyController<T, MaxPoints>::StanleyController(const Path& path, const CarLikeKinematics<T>& vehicle, const StanleyParameters<T>& parameters)
        : path{ path }
        , vehicle{ vehicle }
        , parameters{ parameters }
    {
        really_assert(parameters.gain > T{} && parameters.softening > T{});
    }

    template<typename T, std::size_t MaxPoints>
    OPTIMIZE_FOR_SPEED T StanleyController<T, MaxPoints>::Compute(const math::SE2Transform<T>& rearAxlePose, T speed, T previousSteer, T dt)
    {
        const auto error{ Error(rearAxlePose) };
        const T steer{ error.headingError - math::Atan2(parameters.gain * error.crossTrack, parameters.softening + speed) };
        return vehicle.LimitSteer(steer, previousSteer, dt);
    }

    template<typename T, std::size_t MaxPoints>
    StanleyError<T> StanleyController<T, MaxPoints>::Error(const math::SE2Transform<T>& rearAxlePose)
    {
        really_assert(path.Size() >= 2);

        const auto front{ rearAxlePose.Apply(math::Vector2<T>{ vehicle.Parameters().wheelBase, T{} }) };
        const auto projection{ path.Project(front, segment, parameters.searchWindow) };
        if (projection.arcLength >= progress)
        {
            progress = projection.arcLength;
            segment = projection.segment;
        }

        return StanleyError<T>{ projection.crossTrack, math::AngleDifference(projection.heading, rearAxlePose.Heading()) };
    }

    template<typename T, std::size_t MaxPoints>
    void StanleyController<T, MaxPoints>::Restart()
    {
        progress = T{};
        segment = 0;
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class StanleyController<float, 4>;
    extern template class StanleyController<float, 32>;
#endif
}
