#pragma once

#include "infra/util/ReallyAssert.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/math/SE2Transform.hpp"
#include <type_traits>

namespace mobile
{
    template<typename T>
    struct KanayamaGains
    {
        static_assert(std::is_floating_point_v<T>, "KanayamaGains supports floating-point types");

        T kx{};
        T ky{};
        T ktheta{};
    };

    template<typename T>
    struct UnicycleCommand
    {
        static_assert(std::is_floating_point_v<T>, "UnicycleCommand supports floating-point types");

        T v{};
        T omega{};
    };

    template<typename T>
    class KanayamaController
    {
        static_assert(std::is_floating_point_v<T>, "KanayamaController supports floating-point types");

    public:
        explicit KanayamaController(const KanayamaGains<T>& gains);

        OPTIMIZE_FOR_SPEED UnicycleCommand<T> Compute(const math::SE2Transform<T>& pose, const math::SE2Transform<T>& reference, T referenceSpeed, T referenceTurnRate) const;

        static math::Vector3<T> TrackingError(const math::SE2Transform<T>& pose, const math::SE2Transform<T>& reference);
        static KanayamaGains<T> CriticallyDamped(T kx, T ky);

    private:
        KanayamaGains<T> gains;
    };

    template<typename T>
    KanayamaController<T>::KanayamaController(const KanayamaGains<T>& gains)
        : gains{ gains }
    {
        really_assert(gains.kx > T{} && gains.ky > T{} && gains.ktheta > T{});
    }

    template<typename T>
    OPTIMIZE_FOR_SPEED UnicycleCommand<T> KanayamaController<T>::Compute(const math::SE2Transform<T>& pose, const math::SE2Transform<T>& reference, T referenceSpeed, T referenceTurnRate) const
    {
        const auto error{ TrackingError(pose, reference) };
        const T v{ referenceSpeed * math::Cos(error[2]) + gains.kx * error[0] };
        const T omega{ referenceTurnRate + referenceSpeed * (gains.ky * error[1] + gains.ktheta * math::Sin(error[2])) };
        return UnicycleCommand<T>{ v, omega };
    }

    template<typename T>
    math::Vector3<T> KanayamaController<T>::TrackingError(const math::SE2Transform<T>& pose, const math::SE2Transform<T>& reference)
    {
        const auto error{ pose.Inverse() * reference };
        return math::Vector3<T>{ error.translation[0], error.translation[1], error.Heading() };
    }

    template<typename T>
    KanayamaGains<T> KanayamaController<T>::CriticallyDamped(T kx, T ky)
    {
        return KanayamaGains<T>{ kx, ky, T{ 2 } * math::Sqrt(ky) };
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class KanayamaController<float>;
#endif
}
