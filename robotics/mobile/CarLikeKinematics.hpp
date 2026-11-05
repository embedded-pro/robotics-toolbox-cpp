#pragma once

#include "infra/util/ReallyAssert.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/math/SE2Transform.hpp"
#include <algorithm>
#include <type_traits>

namespace mobile
{
    template<typename T>
    struct CarLikeParameters
    {
        static_assert(std::is_floating_point_v<T>, "CarLikeParameters supports floating-point types");

        T wheelBase{};
        T trackWidth{};
        T maxSteer{};
        T maxSteerRate{};
    };

    template<typename T>
    struct AckermannAngles
    {
        static_assert(std::is_floating_point_v<T>, "AckermannAngles supports floating-point types");

        T left{};
        T right{};
    };

    template<typename T>
    class CarLikeKinematics
    {
        static_assert(std::is_floating_point_v<T>, "CarLikeKinematics supports floating-point types");

    public:
        explicit CarLikeKinematics(const CarLikeParameters<T>& parameters);

        OPTIMIZE_FOR_SPEED math::Vector3<T> Forward(T v, T steer) const;
        math::Vector3<T> TwistAt(T v, T steer, T offset) const;
        T Curvature(T steer) const;
        T SteerForCurvature(T curvature) const;
        T LimitSteer(T requested, T previous, T dt) const;
        AckermannAngles<T> Ackermann(T steer) const;
        T MinTurningRadius() const;
        const CarLikeParameters<T>& Parameters() const;

    private:
        CarLikeParameters<T> parameters;
    };

    template<typename T>
    CarLikeKinematics<T>::CarLikeKinematics(const CarLikeParameters<T>& parameters)
        : parameters{ parameters }
    {
        really_assert(parameters.wheelBase > T{} && parameters.trackWidth >= T{} && parameters.maxSteer > T{} && parameters.maxSteerRate > T{});
        really_assert(parameters.maxSteer < math::Atan2(T{ 2 } * parameters.wheelBase, parameters.trackWidth));
    }

    template<typename T>
    OPTIMIZE_FOR_SPEED math::Vector3<T> CarLikeKinematics<T>::Forward(T v, T steer) const
    {
        return math::Vector3<T>{ v, T{}, v * Curvature(steer) };
    }

    template<typename T>
    math::Vector3<T> CarLikeKinematics<T>::TwistAt(T v, T steer, T offset) const
    {
        const T omega{ v * Curvature(steer) };
        return math::Vector3<T>{ v, offset * omega, omega };
    }

    template<typename T>
    T CarLikeKinematics<T>::Curvature(T steer) const
    {
        return math::Tan(steer) / parameters.wheelBase;
    }

    template<typename T>
    T CarLikeKinematics<T>::SteerForCurvature(T curvature) const
    {
        return std::clamp(math::Atan(parameters.wheelBase * curvature), -parameters.maxSteer, parameters.maxSteer);
    }

    template<typename T>
    T CarLikeKinematics<T>::LimitSteer(T requested, T previous, T dt) const
    {
        const T bounded{ std::clamp(requested, -parameters.maxSteer, parameters.maxSteer) };
        const T step{ parameters.maxSteerRate * dt };
        return std::clamp(bounded, previous - step, previous + step);
    }

    template<typename T>
    AckermannAngles<T> CarLikeKinematics<T>::Ackermann(T steer) const
    {
        const T lateral{ parameters.wheelBase * math::Tan(steer) };
        const T halfTrack{ parameters.trackWidth / T{ 2 } * math::Tan(steer) };
        return AckermannAngles<T>{ math::Atan(lateral / (parameters.wheelBase - halfTrack)), math::Atan(lateral / (parameters.wheelBase + halfTrack)) };
    }

    template<typename T>
    T CarLikeKinematics<T>::MinTurningRadius() const
    {
        return parameters.wheelBase / math::Tan(parameters.maxSteer);
    }

    template<typename T>
    const CarLikeParameters<T>& CarLikeKinematics<T>::Parameters() const
    {
        return parameters;
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class CarLikeKinematics<float>;
#endif
}
