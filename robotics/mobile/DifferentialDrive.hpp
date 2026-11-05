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
    struct WheelSpeeds
    {
        static_assert(std::is_floating_point_v<T>, "WheelSpeeds supports floating-point types");

        T left{};
        T right{};
    };

    template<typename T>
    WheelSpeeds<T> ScaleToLimit(const WheelSpeeds<T>& wheels, T maxWheelSpeed)
    {
        static_assert(std::is_floating_point_v<T>, "ScaleToLimit supports floating-point types");

        const T largest{ std::max(math::Abs(wheels.left), math::Abs(wheels.right)) };
        if (largest <= maxWheelSpeed)
            return wheels;

        const T scale{ maxWheelSpeed / largest };
        return WheelSpeeds<T>{ wheels.left * scale, wheels.right * scale };
    }

    template<typename T>
    class DifferentialDrive
    {
        static_assert(std::is_floating_point_v<T>, "DifferentialDrive supports floating-point types");

    public:
        DifferentialDrive(T wheelRadius, T trackWidth, T maxWheelSpeed);

        OPTIMIZE_FOR_SPEED math::Vector3<T> Forward(const WheelSpeeds<T>& wheels) const;
        OPTIMIZE_FOR_SPEED WheelSpeeds<T> Inverse(T v, T omega) const;
        WheelSpeeds<T> Limit(const WheelSpeeds<T>& wheels) const;
        math::Vector3<T> Achievable(T v, T omega) const;
        T MaxLinearSpeed() const;
        T MaxAngularSpeed() const;

    private:
        T wheelRadius;
        T trackWidth;
        T maxWheelSpeed;
    };

    template<typename T>
    DifferentialDrive<T>::DifferentialDrive(T wheelRadius, T trackWidth, T maxWheelSpeed)
        : wheelRadius{ wheelRadius }
        , trackWidth{ trackWidth }
        , maxWheelSpeed{ maxWheelSpeed }
    {
        really_assert(wheelRadius > T{} && trackWidth > T{} && maxWheelSpeed > T{});
    }

    template<typename T>
    OPTIMIZE_FOR_SPEED math::Vector3<T> DifferentialDrive<T>::Forward(const WheelSpeeds<T>& wheels) const
    {
        return math::Vector3<T>{ wheelRadius * (wheels.left + wheels.right) / T{ 2 }, T{}, wheelRadius * (wheels.right - wheels.left) / trackWidth };
    }

    template<typename T>
    OPTIMIZE_FOR_SPEED WheelSpeeds<T> DifferentialDrive<T>::Inverse(T v, T omega) const
    {
        const T halfTrack{ omega * trackWidth / T{ 2 } };
        return WheelSpeeds<T>{ (v - halfTrack) / wheelRadius, (v + halfTrack) / wheelRadius };
    }

    template<typename T>
    WheelSpeeds<T> DifferentialDrive<T>::Limit(const WheelSpeeds<T>& wheels) const
    {
        return ScaleToLimit(wheels, maxWheelSpeed);
    }

    template<typename T>
    math::Vector3<T> DifferentialDrive<T>::Achievable(T v, T omega) const
    {
        return Forward(Limit(Inverse(v, omega)));
    }

    template<typename T>
    T DifferentialDrive<T>::MaxLinearSpeed() const
    {
        return wheelRadius * maxWheelSpeed;
    }

    template<typename T>
    T DifferentialDrive<T>::MaxAngularSpeed() const
    {
        return T{ 2 } * wheelRadius * maxWheelSpeed / trackWidth;
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class DifferentialDrive<float>;
    extern template WheelSpeeds<float> ScaleToLimit<float>(const WheelSpeeds<float>&, float);
#endif
}
