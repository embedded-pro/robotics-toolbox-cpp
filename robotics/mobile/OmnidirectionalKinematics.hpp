#pragma once

#include "infra/util/ReallyAssert.hpp"
#include "numerical/math/CholeskyDecomposition.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/math/SE2Transform.hpp"
#include "numerical/math/TriangularSolve.hpp"
#include <algorithm>
#include <array>
#include <cstddef>
#include <type_traits>

namespace mobile
{
    template<typename T>
    struct SwedishWheel
    {
        static_assert(std::is_floating_point_v<T>, "SwedishWheel supports floating-point types");

        math::Vector2<T> position{};
        T driveAngle{};
        T rollerAngle{};
        T radius{};
    };

    template<typename T, std::size_t NumWheels>
    class OmnidirectionalKinematics
    {
        static_assert(std::is_floating_point_v<T>, "OmnidirectionalKinematics supports floating-point types");
        static_assert(NumWheels >= 3, "At least three wheels are needed to span the planar twist");

    public:
        using WheelVector = math::Vector<T, NumWheels>;
        using Jacobian = math::Matrix<T, NumWheels, 3>;
        using Wheels = std::array<SwedishWheel<T>, NumWheels>;

        OmnidirectionalKinematics(const Wheels& wheels, T maxWheelSpeed);

        OPTIMIZE_FOR_SPEED WheelVector Inverse(const math::Vector3<T>& twist) const;
        OPTIMIZE_FOR_SPEED math::Vector3<T> Forward(const WheelVector& speeds) const;
        WheelVector Limit(const WheelVector& speeds) const;
        T SlipResidual(const WheelVector& speeds) const;
        const Jacobian& WheelJacobian() const;

    private:
        static Jacobian BuildJacobian(const Wheels& wheels);
        static math::Matrix<T, 3, NumWheels> BuildPseudoInverse(const Jacobian& jacobian);

        Jacobian jacobian;
        math::Matrix<T, 3, NumWheels> pseudoInverse;
        T maxWheelSpeed;
    };

    template<typename T, std::size_t NumWheels>
    OmnidirectionalKinematics<T, NumWheels>::OmnidirectionalKinematics(const Wheels& wheels, T maxWheelSpeed)
        : jacobian{ BuildJacobian(wheels) }
        , pseudoInverse{ BuildPseudoInverse(jacobian) }
        , maxWheelSpeed{ maxWheelSpeed }
    {}

    template<typename T, std::size_t NumWheels>
    OPTIMIZE_FOR_SPEED auto OmnidirectionalKinematics<T, NumWheels>::Inverse(const math::Vector3<T>& twist) const -> WheelVector
    {
        return jacobian * twist;
    }

    template<typename T, std::size_t NumWheels>
    OPTIMIZE_FOR_SPEED math::Vector3<T> OmnidirectionalKinematics<T, NumWheels>::Forward(const WheelVector& speeds) const
    {
        return pseudoInverse * speeds;
    }

    template<typename T, std::size_t NumWheels>
    auto OmnidirectionalKinematics<T, NumWheels>::Limit(const WheelVector& speeds) const -> WheelVector
    {
        T largest{};
        for (std::size_t i = 0; i < NumWheels; ++i)
            largest = std::max(largest, math::Abs(speeds[i]));

        if (largest <= maxWheelSpeed)
            return speeds;
        return speeds * (maxWheelSpeed / largest);
    }

    template<typename T, std::size_t NumWheels>
    T OmnidirectionalKinematics<T, NumWheels>::SlipResidual(const WheelVector& speeds) const
    {
        const WheelVector residual{ speeds - jacobian * Forward(speeds) };
        T sum{};
        for (std::size_t i = 0; i < NumWheels; ++i)
            sum += residual[i] * residual[i];
        return math::Sqrt(sum);
    }

    template<typename T, std::size_t NumWheels>
    auto OmnidirectionalKinematics<T, NumWheels>::WheelJacobian() const -> const Jacobian&
    {
        return jacobian;
    }

    template<typename T, std::size_t NumWheels>
    auto OmnidirectionalKinematics<T, NumWheels>::BuildJacobian(const Wheels& wheels) -> Jacobian
    {
        Jacobian result{};
        for (std::size_t i = 0; i < NumWheels; ++i)
        {
            const auto& wheel{ wheels[i] };
            const T rollerCosine{ math::Cos(wheel.rollerAngle) };
            really_assert(math::Abs(rollerCosine) > T{ 1e-3f });

            const T angle{ wheel.driveAngle + wheel.rollerAngle };
            const T scale{ T{ 1 } / (wheel.radius * rollerCosine) };
            result.at(i, 0) = math::Cos(angle) * scale;
            result.at(i, 1) = math::Sin(angle) * scale;
            result.at(i, 2) = (wheel.position[0] * math::Sin(angle) - wheel.position[1] * math::Cos(angle)) * scale;
        }
        return result;
    }

    template<typename T, std::size_t NumWheels>
    math::Matrix<T, 3, NumWheels> OmnidirectionalKinematics<T, NumWheels>::BuildPseudoInverse(const Jacobian& jacobian)
    {
        const auto transposed{ jacobian.Transpose() };
        const auto factor{ math::CholeskyDecomposition<T, 3>::TryFactor(math::SquareMatrix<T, 3>{ transposed * jacobian }) };
        really_assert(factor.has_value());

        math::Matrix<T, 3, NumWheels> result{};
        for (std::size_t c = 0; c < NumWheels; ++c)
        {
            const math::Vector3<T> column{ transposed.at(0, c), transposed.at(1, c), transposed.at(2, c) };
            const auto solved{ math::SolveUpperTriangular(factor->Transpose(), math::SolveLowerTriangular(*factor, column)) };
            for (std::size_t r = 0; r < 3; ++r)
                result.at(r, c) = solved[r];
        }
        return result;
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class OmnidirectionalKinematics<float, 3>;
    extern template class OmnidirectionalKinematics<float, 4>;
#endif
}
