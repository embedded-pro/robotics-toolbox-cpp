#pragma once

#include "infra/util/ReallyAssert.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/LinearTimeInvariant.hpp"
#include "numerical/math/Math.hpp"
#include <type_traits>

namespace mobile
{
    template<typename T>
    struct WheeledInvertedPendulumParameters
    {
        static_assert(std::is_floating_point_v<T>, "WheeledInvertedPendulumParameters supports floating-point types");

        T bodyMass{};
        T wheelsMass{};
        T centreOfMassHeight{};
        T bodyPitchInertia{};
        T wheelInertia{};
        T yawInertia{};
        T wheelRadius{};
        T trackWidth{};
        T gravity{ T{ 9.81 } };
    };

    template<typename T>
    class WheeledInvertedPendulum
    {
        static_assert(std::is_floating_point_v<T>, "WheeledInvertedPendulum supports floating-point types");

    public:
        using PitchPlant = math::LinearTimeInvariant<T, 4, 1>;
        using YawPlant = math::LinearTimeInvariant<T, 2, 1>;

        explicit WheeledInvertedPendulum(const WheeledInvertedPendulumParameters<T>& parameters);

        PitchPlant Pitch() const;
        YawPlant Yaw() const;
        T FallTimeConstant() const;
        T UnstablePole() const;

    private:
        WheeledInvertedPendulumParameters<T> p;
        T translationalMass;
        T pitchInertia;
        T determinant;
    };

    template<typename T>
    WheeledInvertedPendulum<T>::WheeledInvertedPendulum(const WheeledInvertedPendulumParameters<T>& parameters)
        : p{ parameters }
        , translationalMass{ p.wheelsMass + p.bodyMass + T{ 2 } * p.wheelInertia / (p.wheelRadius * p.wheelRadius) }
        , pitchInertia{ p.bodyPitchInertia + p.bodyMass * p.centreOfMassHeight * p.centreOfMassHeight }
        , determinant{ translationalMass * pitchInertia - p.bodyMass * p.bodyMass * p.centreOfMassHeight * p.centreOfMassHeight }
    {
        really_assert(p.bodyMass > T{} && p.wheelsMass >= T{} && p.centreOfMassHeight > T{});
        really_assert(p.bodyPitchInertia >= T{} && p.wheelInertia >= T{} && p.yawInertia >= T{});
        really_assert(p.wheelRadius > T{} && p.trackWidth > T{} && p.gravity > T{});
        really_assert(determinant > T{});
    }

    template<typename T>
    typename WheeledInvertedPendulum<T>::PitchPlant WheeledInvertedPendulum<T>::Pitch() const
    {
        const T m{ p.bodyMass };
        const T l{ p.centreOfMassHeight };

        typename PitchPlant::StateMatrix a{};
        a.at(0, 1) = T{ 1 };
        a.at(1, 0) = translationalMass * m * p.gravity * l / determinant;
        a.at(2, 3) = T{ 1 };
        a.at(3, 0) = -m * m * l * l * p.gravity / determinant;

        typename PitchPlant::InputMatrix b{};
        b.at(1, 0) = -(translationalMass + m * l / p.wheelRadius) / determinant;
        b.at(3, 0) = (pitchInertia / p.wheelRadius + m * l) / determinant;

        return PitchPlant::WithFullStateOutput(a, b);
    }

    template<typename T>
    typename WheeledInvertedPendulum<T>::YawPlant WheeledInvertedPendulum<T>::Yaw() const
    {
        const T effectiveInertia{ p.yawInertia + p.wheelInertia * p.trackWidth * p.trackWidth / (T{ 2 } * p.wheelRadius * p.wheelRadius) };

        typename YawPlant::StateMatrix a{};
        a.at(0, 1) = T{ 1 };

        typename YawPlant::InputMatrix b{};
        b.at(1, 0) = p.trackWidth / (T{ 2 } * p.wheelRadius * effectiveInertia);

        return YawPlant::WithFullStateOutput(a, b);
    }

    template<typename T>
    T WheeledInvertedPendulum<T>::FallTimeConstant() const
    {
        return math::Sqrt(pitchInertia / (p.bodyMass * p.gravity * p.centreOfMassHeight));
    }

    template<typename T>
    T WheeledInvertedPendulum<T>::UnstablePole() const
    {
        return math::Sqrt(translationalMass * p.bodyMass * p.gravity * p.centreOfMassHeight / determinant);
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class WheeledInvertedPendulum<float>;
#endif
}
