#pragma once

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Geometry3D.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/math/SE3Transform.hpp"
#include "numerical/solvers/ActiveSetQp.hpp"
#include "numerical/solvers/GaussianElimination.hpp"
#include <algorithm>
#include <array>
#include <cstddef>
#include <limits>
#include <optional>
#include <type_traits>

namespace controllers
{
    template<typename T, std::size_t NumCables, std::size_t WrenchDim>
    class CableTensionDistribution
    {
        static_assert(std::is_floating_point_v<T>, "CableTensionDistribution supports floating-point types");
        static_assert(WrenchDim == 6 || (WrenchDim > 0 && WrenchDim <= 3), "CableTensionDistribution supports spatial (6) or point-mass (1 to 3) platforms");
        static_assert(NumCables >= WrenchDim, "At least WrenchDim cables are required");

    public:
        using AnchorArray = std::array<math::Vector3<T>, NumCables>;
        using StructureMatrix = math::Matrix<T, WrenchDim, NumCables>;
        using TensionVector = math::Vector<T, NumCables>;
        using WrenchVector = math::Vector<T, WrenchDim>;

        CableTensionDistribution(const AnchorArray& baseAnchors, const AnchorArray& platformAnchors, T tMin, T tMax);

        std::optional<StructureMatrix> ComputeStructureMatrix(const math::SE3Transform<T>& platformPose) const;
        OPTIMIZE_FOR_SPEED std::optional<TensionVector> Distribute(const WrenchVector& wrench, const math::SE3Transform<T>& platformPose) const;
        std::optional<TensionVector> DistributeExact(const WrenchVector& wrench, const math::SE3Transform<T>& platformPose) const;

    private:
        using Program = solvers::QuadraticProgram<T, NumCables, WrenchDim, 2 * NumCables>;
        using FixedMask = std::array<bool, NumCables>;

        bool SolveFree(const StructureMatrix& structure, const FixedMask& fixed, const WrenchVector& remaining, TensionVector& tensions) const;
        std::optional<std::size_t> WorstViolation(const FixedMask& fixed, const TensionVector& tensions) const;
        T Violation(T tension) const;

        AnchorArray baseAnchors;
        AnchorArray platformAnchors;
        T tMin;
        T tMax;
        T tMid;
    };

    template<typename T, std::size_t NumCables, std::size_t WrenchDim>
    CableTensionDistribution<T, NumCables, WrenchDim>::CableTensionDistribution(const AnchorArray& baseAnchors, const AnchorArray& platformAnchors, T tMin, T tMax)
        : baseAnchors{ baseAnchors }
        , platformAnchors{ platformAnchors }
        , tMin{ tMin }
        , tMax{ tMax }
        , tMid{ (tMin + tMax) / T{ 2 } }
    {}

    template<typename T, std::size_t NumCables, std::size_t WrenchDim>
    auto CableTensionDistribution<T, NumCables, WrenchDim>::ComputeStructureMatrix(const math::SE3Transform<T>& platformPose) const -> std::optional<StructureMatrix>
    {
        StructureMatrix structure{};
        for (std::size_t i = 0; i < NumCables; ++i)
        {
            const math::Vector3<T> arm{ WrenchDim == 6 ? math::Vector3<T>{ platformPose.rotation * platformAnchors[i] } : math::Vector3<T>{} };
            const math::Vector3<T> cable{ baseAnchors[i] - platformPose.translation - arm };
            const T length{ math::VectorNorm(cable) };
            if (length <= std::numeric_limits<T>::epsilon())
                return std::nullopt;

            const math::Vector3<T> direction{ cable * (T{ 1 } / length) };
            const math::Vector3<T> moment{ math::CrossProduct(arm, direction) };
            for (std::size_t r = 0; r < WrenchDim; ++r)
                structure.at(r, i) = r < 3 ? direction[r] : moment[r - 3];
        }
        return structure;
    }

    template<typename T, std::size_t NumCables, std::size_t WrenchDim>
    OPTIMIZE_FOR_SPEED auto CableTensionDistribution<T, NumCables, WrenchDim>::Distribute(const WrenchVector& wrench, const math::SE3Transform<T>& platformPose) const
        -> std::optional<TensionVector>
    {
        const auto structure{ ComputeStructureMatrix(platformPose) };
        if (!structure)
            return std::nullopt;

        FixedMask fixed{};
        WrenchVector remaining{ wrench };
        TensionVector tensions{};
        for (std::size_t freeCables = NumCables; freeCables >= WrenchDim; --freeCables)
        {
            if (!SolveFree(*structure, fixed, remaining, tensions))
                return std::nullopt;

            const auto worst{ WorstViolation(fixed, tensions) };
            if (!worst)
                return tensions;

            tensions[*worst] = std::clamp(tensions[*worst], tMin, tMax);
            fixed[*worst] = true;
            for (std::size_t r = 0; r < WrenchDim; ++r)
                remaining[r] -= structure->at(r, *worst) * tensions[*worst];
        }

        return std::nullopt;
    }

    template<typename T, std::size_t NumCables, std::size_t WrenchDim>
    bool CableTensionDistribution<T, NumCables, WrenchDim>::SolveFree(const StructureMatrix& structure, const FixedMask& fixed, const WrenchVector& remaining, TensionVector& tensions) const
    {
        StructureMatrix active{ structure };
        TensionVector centre{};
        for (std::size_t i = 0; i < NumCables; ++i)
        {
            centre[i] = fixed[i] ? T{} : tMid;
            for (std::size_t r = 0; r < WrenchDim; ++r)
                active.at(r, i) = fixed[i] ? T{} : active.at(r, i);
        }

        const auto multipliers{ solvers::TrySolveSystem(math::SquareMatrix<T, WrenchDim>{ active * active.Transpose() }, WrenchVector{ remaining - active * centre }) };
        if (!multipliers)
            return false;

        const TensionVector correction{ active.Transpose() * *multipliers };
        for (std::size_t i = 0; i < NumCables; ++i)
            tensions[i] = fixed[i] ? tensions[i] : tMid + correction[i];
        return true;
    }

    template<typename T, std::size_t NumCables, std::size_t WrenchDim>
    std::optional<std::size_t> CableTensionDistribution<T, NumCables, WrenchDim>::WorstViolation(const FixedMask& fixed, const TensionVector& tensions) const
    {
        std::optional<std::size_t> worst{};
        T worstViolation{};
        for (std::size_t i = 0; i < NumCables; ++i)
        {
            const T violation{ fixed[i] ? T{} : Violation(tensions[i]) };
            if (violation > worstViolation)
            {
                worst = i;
                worstViolation = violation;
            }
        }
        return worst;
    }

    template<typename T, std::size_t NumCables, std::size_t WrenchDim>
    auto CableTensionDistribution<T, NumCables, WrenchDim>::DistributeExact(const WrenchVector& wrench, const math::SE3Transform<T>& platformPose) const
        -> std::optional<TensionVector>
    {
        const auto structure{ ComputeStructureMatrix(platformPose) };
        if (!structure)
            return std::nullopt;

        Program program{};
        program.hessian = math::SquareMatrix<T, NumCables>::Identity();
        program.equalityMatrix = *structure;
        program.equalityVector = wrench;
        for (std::size_t i = 0; i < NumCables; ++i)
        {
            program.gradient[i] = -tMid;
            program.inequalityMatrix.at(i, i) = T{ 1 };
            program.inequalityMatrix.at(i + NumCables, i) = T{ -1 };
            program.inequalityVector[i] = tMin;
            program.inequalityVector[i + NumCables] = -tMax;
        }

        const auto result{ solvers::ActiveSetQp<T, NumCables, WrenchDim, 2 * NumCables>{}.Solve(program) };
        if (result.status != solvers::QpStatus::Optimal)
            return std::nullopt;

        return result.x;
    }

    template<typename T, std::size_t NumCables, std::size_t WrenchDim>
    T CableTensionDistribution<T, NumCables, WrenchDim>::Violation(T tension) const
    {
        return std::max({ tMin - tension, tension - tMax, T{} });
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class CableTensionDistribution<float, 3, 2>;
    extern template class CableTensionDistribution<float, 2, 2>;
    extern template class CableTensionDistribution<float, 8, 6>;
#endif
}
