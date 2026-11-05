#pragma once

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/math/SE2Transform.hpp"
#include <type_traits>

namespace mobile
{
    template<typename T>
    struct OdometryNoise
    {
        static_assert(std::is_floating_point_v<T>, "OdometryNoise supports floating-point types");

        T rotationFromRotation{};
        T rotationFromTranslation{};
        T translationFromTranslation{};
        T translationFromRotation{};
    };

    template<typename T>
    class PlanarOdometry
    {
        static_assert(std::is_floating_point_v<T>, "PlanarOdometry supports floating-point types");

    public:
        PlanarOdometry() = default;
        explicit PlanarOdometry(const math::SE2Transform<T>& initial);

        OPTIMIZE_FOR_SPEED void Integrate(const math::Vector3<T>& twist, T dt);
        void Reset(const math::SE2Transform<T>& initial);
        const math::SE2Transform<T>& Pose() const;

        static math::Vector3<T> Increment(const math::SE2Transform<T>& from, const math::SE2Transform<T>& to);

    private:
        math::SE2Transform<T> pose{};
    };

    template<typename T>
    math::Vector3<T> IncrementDeviation(const math::SE2Transform<T>& increment, const OdometryNoise<T>& noise)
    {
        static_assert(std::is_floating_point_v<T>, "IncrementDeviation supports floating-point types");

        const auto delta{ increment.Pose() };
        const T distanceSquared{ delta[0] * delta[0] + delta[1] * delta[1] };
        const T rotationSquared{ delta[2] * delta[2] };
        const T translation{ math::Sqrt(noise.translationFromTranslation * distanceSquared + noise.translationFromRotation * rotationSquared) };
        const T rotation{ math::Sqrt(noise.rotationFromRotation * rotationSquared + noise.rotationFromTranslation * distanceSquared) };
        return math::Vector3<T>{ translation, translation, rotation };
    }

    template<typename T>
    PlanarOdometry<T>::PlanarOdometry(const math::SE2Transform<T>& initial)
        : pose{ initial }
    {}

    template<typename T>
    OPTIMIZE_FOR_SPEED void PlanarOdometry<T>::Integrate(const math::Vector3<T>& twist, T dt)
    {
        pose = (pose * math::SE2Transform<T>::Exp(twist, dt)).Normalized();
    }

    template<typename T>
    void PlanarOdometry<T>::Reset(const math::SE2Transform<T>& initial)
    {
        pose = initial;
    }

    template<typename T>
    const math::SE2Transform<T>& PlanarOdometry<T>::Pose() const
    {
        return pose;
    }

    template<typename T>
    math::Vector3<T> PlanarOdometry<T>::Increment(const math::SE2Transform<T>& from, const math::SE2Transform<T>& to)
    {
        return (from.Inverse() * to).Log();
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class PlanarOdometry<float>;
    extern template math::Vector3<float> IncrementDeviation<float>(const math::SE2Transform<float>&, const OdometryNoise<float>&);
#endif
}
