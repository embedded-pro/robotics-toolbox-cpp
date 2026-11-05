#pragma once

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Geometry3D.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/math/Matrix.hpp"
#include <algorithm>
#include <array>
#include <cstddef>
#include <numbers>
#include <optional>
#include <type_traits>

namespace kinematics
{
    template<typename T>
    struct DeltaGeometry
    {
        static_assert(std::is_floating_point_v<T>, "DeltaGeometry supports floating-point types");

        T baseRadius{};
        T effectorRadius{};
        T upperArm{};
        T forearm{};
    };

    template<typename T>
    class DeltaKinematics
    {
        static_assert(std::is_floating_point_v<T>, "DeltaKinematics supports floating-point types");

    public:
        explicit DeltaKinematics(const DeltaGeometry<T>& geometry);

        OPTIMIZE_FOR_SPEED std::optional<math::Vector3<T>> Inverse(const math::Vector3<T>& effector) const;
        std::optional<math::Vector3<T>> Forward(const math::Vector3<T>& angles) const;

    private:
        std::optional<T> ArmAngle(std::size_t arm, const math::Vector3<T>& effector) const;
        math::Vector3<T> ShiftedElbow(std::size_t arm, T angle) const;

        DeltaGeometry<T> geometry;
        std::array<T, 3> cosPhi;
        std::array<T, 3> sinPhi;
    };

    template<typename T>
    DeltaKinematics<T>::DeltaKinematics(const DeltaGeometry<T>& geometry)
        : geometry{ geometry }
        , cosPhi{ T{ 1 }, -T{ 0.5f }, -T{ 0.5f } }
        , sinPhi{ T{}, std::numbers::sqrt3_v<T> / T{ 2 }, -std::numbers::sqrt3_v<T> / T{ 2 } }
    {}

    template<typename T>
    OPTIMIZE_FOR_SPEED std::optional<math::Vector3<T>> DeltaKinematics<T>::Inverse(const math::Vector3<T>& effector) const
    {
        const auto first{ ArmAngle(0, effector) };
        const auto second{ ArmAngle(1, effector) };
        const auto third{ ArmAngle(2, effector) };
        if (!first || !second || !third)
            return std::nullopt;

        return math::Vector3<T>{ *first, *second, *third };
    }

    template<typename T>
    std::optional<math::Vector3<T>> DeltaKinematics<T>::Forward(const math::Vector3<T>& angles) const
    {
        const auto c1{ ShiftedElbow(0, angles[0]) };
        const auto c2{ ShiftedElbow(1, angles[1]) };
        const auto c3{ ShiftedElbow(2, angles[2]) };

        const math::Vector3<T> toSecond{ c2 - c1 };
        const math::Vector3<T> toThird{ c3 - c1 };
        const T d{ math::VectorNorm(toSecond) };
        const math::Vector3<T> ex{ toSecond * (T{ 1 } / d) };
        const T i{ ex[0] * toThird[0] + ex[1] * toThird[1] + ex[2] * toThird[2] };
        const math::Vector3<T> lateral{ toThird - ex * i };
        const math::Vector3<T> ey{ lateral * (T{ 1 } / math::VectorNorm(lateral)) };
        const T j{ ey[0] * toThird[0] + ey[1] * toThird[1] + ey[2] * toThird[2] };
        const math::Vector3<T> ez{ math::CrossProduct(ex, ey) };

        const T x{ d / T{ 2 } };
        const T y{ (i * i + j * j - T{ 2 } * i * x) / (T{ 2 } * j) };
        const T heightSquared{ geometry.forearm * geometry.forearm - x * x - y * y };
        if (heightSquared < T{})
            return std::nullopt;

        const math::Vector3<T> centre{ c1 + ex * x + ey * y };
        const math::Vector3<T> offset{ ez * math::Sqrt(heightSquared) };
        const math::Vector3<T> above{ centre + offset };
        const math::Vector3<T> below{ centre - offset };
        return above[2] < below[2] ? above : below;
    }

    template<typename T>
    std::optional<T> DeltaKinematics<T>::ArmAngle(std::size_t arm, const math::Vector3<T>& effector) const
    {
        const T radial{ cosPhi[arm] * effector[0] + sinPhi[arm] * effector[1] };
        const T tangential{ -sinPhi[arm] * effector[0] + cosPhi[arm] * effector[1] };
        const T z{ effector[2] };
        const T reach{ radial + geometry.effectorRadius - geometry.baseRadius };

        const T a{ T{ 2 } * geometry.upperArm * reach };
        const T b{ T{ -2 } * geometry.upperArm * z };
        const T c{ reach * reach + tangential * tangential + z * z + geometry.upperArm * geometry.upperArm - geometry.forearm * geometry.forearm };
        const T magnitude{ math::Sqrt(a * a + b * b) };
        if (magnitude == T{} || math::Abs(c) > magnitude)
            return std::nullopt;

        return math::Atan2(b, a) - math::Acos(std::clamp(c / magnitude, T{ -1 }, T{ 1 }));
    }

    template<typename T>
    math::Vector3<T> DeltaKinematics<T>::ShiftedElbow(std::size_t arm, T angle) const
    {
        const T radial{ geometry.baseRadius - geometry.effectorRadius + geometry.upperArm * math::Cos(angle) };
        return math::Vector3<T>{ cosPhi[arm] * radial, sinPhi[arm] * radial, -geometry.upperArm * math::Sin(angle) };
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class DeltaKinematics<float>;
#endif
}
