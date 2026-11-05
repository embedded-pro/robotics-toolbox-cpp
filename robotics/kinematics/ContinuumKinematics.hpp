#pragma once

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Geometry3D.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/math/SE3Transform.hpp"
#include <array>
#include <cstddef>
#include <limits>
#include <type_traits>

namespace kinematics
{
    template<typename T>
    struct ArcParameters
    {
        static_assert(std::is_floating_point_v<T>, "ArcParameters supports floating-point types");

        T kappa{};
        T phi{};
        T length{};
    };

    template<typename T, std::size_t NumSections>
    class ContinuumKinematics
    {
        static_assert(std::is_floating_point_v<T>, "ContinuumKinematics supports floating-point types");
        static_assert(NumSections > 0, "At least one section is required");

    public:
        using Sections = std::array<ArcParameters<T>, NumSections>;

        explicit ContinuumKinematics(const Sections& sections);

        OPTIMIZE_FOR_SPEED static math::SE3Transform<T> SectionTransform(const ArcParameters<T>& arc);
        OPTIMIZE_FOR_SPEED math::SE3Transform<T> Forward() const;
        static ArcParameters<T> InverseSection(const math::Vector3<T>& tip);

    private:
        static constexpr T seriesThreshold{ T{ 0.01f } };

        Sections sections;
    };

    template<typename T, std::size_t NumSections>
    ContinuumKinematics<T, NumSections>::ContinuumKinematics(const Sections& sections)
        : sections{ sections }
    {}

    template<typename T, std::size_t NumSections>
    OPTIMIZE_FOR_SPEED math::SE3Transform<T> ContinuumKinematics<T, NumSections>::SectionTransform(const ArcParameters<T>& arc)
    {
        const T theta{ arc.kappa * arc.length };
        T chordial{ theta / T{ 2 } - theta * theta * theta / T{ 24 } };
        T axial{ T{ 1 } - theta * theta / T{ 6 } };
        if (math::Abs(theta) >= seriesThreshold)
        {
            const T halfSin{ math::Sin(theta / T{ 2 }) };
            chordial = T{ 2 } * halfSin * halfSin / theta;
            axial = math::Sin(theta) / theta;
        }

        const T cosPhi{ math::Cos(arc.phi) };
        const T sinPhi{ math::Sin(arc.phi) };
        const math::Vector3<T> position{ arc.length * chordial * cosPhi, arc.length * chordial * sinPhi, arc.length * axial };
        return math::SE3Transform<T>{ math::RotationAboutAxis(math::Vector3<T>{ -sinPhi, cosPhi, T{} }, theta), position };
    }

    template<typename T, std::size_t NumSections>
    OPTIMIZE_FOR_SPEED math::SE3Transform<T> ContinuumKinematics<T, NumSections>::Forward() const
    {
        auto tip{ math::SE3Transform<T>::Identity() };
        for (const auto& arc : sections)
            tip = tip * SectionTransform(arc);
        return tip;
    }

    template<typename T, std::size_t NumSections>
    ArcParameters<T> ContinuumKinematics<T, NumSections>::InverseSection(const math::Vector3<T>& tip)
    {
        const T radial{ math::Sqrt(tip[0] * tip[0] + tip[1] * tip[1]) };
        const T distanceSquared{ radial * radial + tip[2] * tip[2] };
        const T distance{ math::Sqrt(distanceSquared) };
        if (radial <= std::numeric_limits<T>::epsilon() * distance)
            return ArcParameters<T>{ T{}, T{}, tip[2] };

        const T halfTheta{ math::Atan2(radial, tip[2]) };
        T arcOverChord{ T{ 1 } + halfTheta * halfTheta / T{ 6 } };
        if (math::Abs(halfTheta) >= seriesThreshold)
            arcOverChord = halfTheta / math::Sin(halfTheta);

        return ArcParameters<T>{ T{ 2 } * radial / distanceSquared, math::Atan2(tip[1], tip[0]), distance * arcOverChord };
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class ContinuumKinematics<float, 1>;
    extern template class ContinuumKinematics<float, 2>;
#endif
}
