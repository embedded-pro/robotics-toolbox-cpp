#pragma once

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Geometry3D.hpp"
#include "numerical/math/Matrix.hpp"
#include <cstdint>
#include <span>
#include <type_traits>

namespace planning
{
    template<typename T>
    struct PotentialFieldParameters
    {
        static_assert(std::is_floating_point_v<T>, "PotentialFieldParameters supports floating-point types");

        T attractiveGain{};
        T conicDistance{};
        T repulsiveGain{};
        T influenceDistance{};
        T maxSpeed{};
        T stallSpeed{};
        T goalTolerance{};
    };

    template<typename T>
    struct ObstacleProximity
    {
        static_assert(std::is_floating_point_v<T>, "ObstacleProximity supports floating-point types");

        math::Vector3<T> direction{};
        T distance{};
    };

    enum class FieldStatus : std::uint8_t
    {
        Moving,
        Reached,
        Stalled
    };

    template<typename T>
    struct FieldCommand
    {
        static_assert(std::is_floating_point_v<T>, "FieldCommand supports floating-point types");

        math::Vector3<T> velocity{};
        T attractiveMagnitude{};
        T repulsiveMagnitude{};
        FieldStatus status{ FieldStatus::Moving };
    };

    template<typename T>
    class PotentialField
    {
        static_assert(std::is_floating_point_v<T>, "PotentialField supports floating-point types");

    public:
        using Vector3 = math::Vector3<T>;

        explicit PotentialField(const PotentialFieldParameters<T>& parameters);

        OPTIMIZE_FOR_SPEED FieldCommand<T> Compute(const Vector3& position, const Vector3& goal, std::span<const ObstacleProximity<T>> obstacles) const;

    private:
        Vector3 Attraction(const Vector3& error, T distance) const;
        Vector3 Repulsion(std::span<const ObstacleProximity<T>> obstacles) const;
        FieldStatus Status(T goalDistance, T forceMagnitude) const;

        PotentialFieldParameters<T> parameters;
    };

    template<typename T>
    PotentialField<T>::PotentialField(const PotentialFieldParameters<T>& parameters)
        : parameters{ parameters }
    {}

    template<typename T>
    OPTIMIZE_FOR_SPEED FieldCommand<T> PotentialField<T>::Compute(const Vector3& position, const Vector3& goal, std::span<const ObstacleProximity<T>> obstacles) const
    {
        const Vector3 error{ goal - position };
        const T goalDistance{ math::VectorNorm(error) };
        const Vector3 attraction{ Attraction(error, goalDistance) };
        const Vector3 repulsion{ Repulsion(obstacles) };
        const Vector3 force{ attraction + repulsion };
        const T magnitude{ math::VectorNorm(force) };

        const Vector3 velocity{ magnitude <= parameters.maxSpeed ? force : Vector3{ force * (parameters.maxSpeed / magnitude) } };
        return { velocity, math::VectorNorm(attraction), math::VectorNorm(repulsion), Status(goalDistance, magnitude) };
    }

    template<typename T>
    typename PotentialField<T>::Vector3 PotentialField<T>::Attraction(const Vector3& error, T distance) const
    {
        if (distance <= parameters.conicDistance)
            return error * parameters.attractiveGain;

        return error * (parameters.attractiveGain * parameters.conicDistance / distance);
    }

    template<typename T>
    typename PotentialField<T>::Vector3 PotentialField<T>::Repulsion(std::span<const ObstacleProximity<T>> obstacles) const
    {
        Vector3 repulsion{};
        for (const auto& obstacle : obstacles)
        {
            const T rho{ obstacle.distance };
            if (rho < parameters.influenceDistance)
                repulsion = repulsion + obstacle.direction * (parameters.repulsiveGain * (T{ 1 } / rho - T{ 1 } / parameters.influenceDistance) / (rho * rho));
        }

        return repulsion;
    }

    template<typename T>
    FieldStatus PotentialField<T>::Status(T goalDistance, T forceMagnitude) const
    {
        if (goalDistance <= parameters.goalTolerance)
            return FieldStatus::Reached;

        return forceMagnitude < parameters.stallSpeed ? FieldStatus::Stalled : FieldStatus::Moving;
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class PotentialField<float>;
#endif
}
