#pragma once

#include "infra/util/ReallyAssert.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/SE2Transform.hpp"
#include "robotics/mobile/DifferentialDrive.hpp"
#include <cstddef>
#include <optional>
#include <type_traits>

namespace mobile
{
    template<typename T>
    struct IcrParameters
    {
        static_assert(std::is_floating_point_v<T>, "IcrParameters supports floating-point types");

        T yLeft{};
        T yRight{};
        T xIcr{};
    };

    template<typename T>
    class SkidSteerKinematics
    {
        static_assert(std::is_floating_point_v<T>, "SkidSteerKinematics supports floating-point types");

    public:
        static IcrParameters<T> IdealIcr(T trackWidth);

        SkidSteerKinematics(const IcrParameters<T>& icr, T sprocketRadius, T maxSprocketSpeed);

        OPTIMIZE_FOR_SPEED math::Vector3<T> Forward(const WheelSpeeds<T>& sprockets) const;
        OPTIMIZE_FOR_SPEED WheelSpeeds<T> Inverse(T vx, T omega) const;
        WheelSpeeds<T> Limit(const WheelSpeeds<T>& sprockets) const;

    private:
        IcrParameters<T> icr;
        T sprocketRadius;
        T maxSprocketSpeed;
    };

    template<typename T>
    class IcrEstimator
    {
        static_assert(std::is_floating_point_v<T>, "IcrEstimator supports floating-point types");

    public:
        explicit IcrEstimator(T sprocketRadius);

        void Add(const WheelSpeeds<T>& sprockets, const math::Vector3<T>& measuredTwist);
        std::optional<IcrParameters<T>> Estimate(T minExcitation) const;
        void Reset();
        std::size_t Samples() const;

    private:
        T sprocketRadius;
        T sumOmegaSquared{};
        T sumLeft{};
        T sumRight{};
        T sumLateral{};
        std::size_t samples{ 0 };
    };

    template<typename T>
    IcrParameters<T> SkidSteerKinematics<T>::IdealIcr(T trackWidth)
    {
        return IcrParameters<T>{ trackWidth / T{ 2 }, -trackWidth / T{ 2 }, T{} };
    }

    template<typename T>
    SkidSteerKinematics<T>::SkidSteerKinematics(const IcrParameters<T>& icr, T sprocketRadius, T maxSprocketSpeed)
        : icr{ icr }
        , sprocketRadius{ sprocketRadius }
        , maxSprocketSpeed{ maxSprocketSpeed }
    {
        really_assert(icr.yLeft > icr.yRight && sprocketRadius > T{} && maxSprocketSpeed > T{});
    }

    template<typename T>
    OPTIMIZE_FOR_SPEED math::Vector3<T> SkidSteerKinematics<T>::Forward(const WheelSpeeds<T>& sprockets) const
    {
        const T left{ sprocketRadius * sprockets.left };
        const T right{ sprocketRadius * sprockets.right };
        const T spread{ icr.yLeft - icr.yRight };
        const T omega{ (right - left) / spread };
        return math::Vector3<T>{ (right * icr.yLeft - left * icr.yRight) / spread, -icr.xIcr * omega, omega };
    }

    template<typename T>
    OPTIMIZE_FOR_SPEED WheelSpeeds<T> SkidSteerKinematics<T>::Inverse(T vx, T omega) const
    {
        return WheelSpeeds<T>{ (vx - omega * icr.yLeft) / sprocketRadius, (vx - omega * icr.yRight) / sprocketRadius };
    }

    template<typename T>
    WheelSpeeds<T> SkidSteerKinematics<T>::Limit(const WheelSpeeds<T>& sprockets) const
    {
        return ScaleToLimit(sprockets, maxSprocketSpeed);
    }

    template<typename T>
    IcrEstimator<T>::IcrEstimator(T sprocketRadius)
        : sprocketRadius{ sprocketRadius }
    {}

    template<typename T>
    void IcrEstimator<T>::Add(const WheelSpeeds<T>& sprockets, const math::Vector3<T>& measuredTwist)
    {
        const T omega{ measuredTwist[2] };
        sumOmegaSquared += omega * omega;
        sumLeft += omega * (measuredTwist[0] - sprocketRadius * sprockets.left);
        sumRight += omega * (measuredTwist[0] - sprocketRadius * sprockets.right);
        sumLateral += omega * measuredTwist[1];
        ++samples;
    }

    template<typename T>
    std::optional<IcrParameters<T>> IcrEstimator<T>::Estimate(T minExcitation) const
    {
        if (sumOmegaSquared < minExcitation)
            return std::nullopt;

        return IcrParameters<T>{ sumLeft / sumOmegaSquared, sumRight / sumOmegaSquared, -sumLateral / sumOmegaSquared };
    }

    template<typename T>
    void IcrEstimator<T>::Reset()
    {
        sumOmegaSquared = T{};
        sumLeft = T{};
        sumRight = T{};
        sumLateral = T{};
        samples = 0;
    }

    template<typename T>
    std::size_t IcrEstimator<T>::Samples() const
    {
        return samples;
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class SkidSteerKinematics<float>;
    extern template class IcrEstimator<float>;
#endif
}
