#pragma once

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/math/MatrixNorms.hpp"
#include "robotics/dynamics/JointLink.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <optional>
#include <type_traits>

namespace kinematics
{
    enum class SolutionStatus : std::uint8_t
    {
        found,
        unreachable,
        outsideLimits
    };

    template<typename T, std::size_t Dof>
    struct NearestSolutionResult
    {
        static_assert(std::is_floating_point_v<T>, "NearestSolutionResult supports floating-point types");

        std::optional<math::Vector<T, Dof>> q;
        SolutionStatus status{ SolutionStatus::unreachable };
    };

    template<typename T, std::size_t Dof>
    using JointLimitArray = std::array<dynamics::JointLimits<T>, Dof>;

    template<typename T, std::size_t Dof>
    std::optional<math::Vector<T, Dof>> WrapIntoLimits(const math::Vector<T, Dof>& solution, const math::Vector<T, Dof>& current, const JointLimitArray<T, Dof>& limits)
    {
        static_assert(std::is_floating_point_v<T>, "WrapIntoLimits supports floating-point types");
        constexpr T twoPi{ T{ 2 } * std::numbers::pi_v<T> };

        math::Vector<T, Dof> wrapped{};
        for (std::size_t joint = 0; joint < Dof; ++joint)
        {
            std::optional<T> chosen;
            for (const T shift : { T{}, twoPi, -twoPi })
            {
                const T value{ solution[joint] + shift };
                const bool inside{ limits[joint].positionMin <= value && value <= limits[joint].positionMax };
                if (inside && (!chosen.has_value() || math::Abs(value - current[joint]) < math::Abs(*chosen - current[joint])))
                    chosen = value;
            }

            if (!chosen.has_value())
                return std::nullopt;
            wrapped[joint] = *chosen;
        }

        return wrapped;
    }

    template<typename T, std::size_t Dof, std::size_t Count>
    OPTIMIZE_FOR_SPEED NearestSolutionResult<T, Dof> NearestSolution(const std::array<std::optional<math::Vector<T, Dof>>, Count>& candidates,
        const math::Vector<T, Dof>& current, const JointLimitArray<T, Dof>& limits)
    {
        static_assert(std::is_floating_point_v<T>, "NearestSolution supports floating-point types");

        NearestSolutionResult<T, Dof> result{};
        T best{ std::numeric_limits<T>::max() };
        for (const auto& candidate : candidates)
        {
            if (!candidate.has_value())
                continue;

            if (result.status == SolutionStatus::unreachable)
                result.status = SolutionStatus::outsideLimits;

            const auto wrapped{ WrapIntoLimits(*candidate, current, limits) };
            if (!wrapped.has_value())
                continue;

            const math::Vector<T, Dof> difference{ *wrapped - current };
            const T distance{ math::DotProduct(difference, difference) };
            if (distance < best)
            {
                best = distance;
                result = { wrapped, SolutionStatus::found };
            }
        }

        return result;
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template std::optional<math::Vector<float, 2>> WrapIntoLimits<float, 2>(const math::Vector<float, 2>&, const math::Vector<float, 2>&, const JointLimitArray<float, 2>&);
    extern template std::optional<math::Vector<float, 6>> WrapIntoLimits<float, 6>(const math::Vector<float, 6>&, const math::Vector<float, 6>&, const JointLimitArray<float, 6>&);
    extern template NearestSolutionResult<float, 2> NearestSolution<float, 2, 3>(const std::array<std::optional<math::Vector<float, 2>>, 3>&, const math::Vector<float, 2>&,
        const JointLimitArray<float, 2>&);
    extern template NearestSolutionResult<float, 6> NearestSolution<float, 6, 8>(const std::array<std::optional<math::Vector<float, 6>>, 8>&, const math::Vector<float, 6>&,
        const JointLimitArray<float, 6>&);
#endif
}
