#pragma once

#include "numerical/math/AngleUtilities.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/math/SE3Transform.hpp"
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <optional>
#include <type_traits>

namespace kinematics
{
    template<typename T>
    struct OpwParameters
    {
        static_assert(std::is_floating_point_v<T>, "OpwParameters supports floating-point types");

        T a1{};
        T a2{};
        T b{};
        T c1{};
        T c2{};
        T c3{};
        T c4{};
        std::array<T, 6> offsets{};
        std::array<std::int8_t, 6> signs{ 1, 1, 1, 1, 1, 1 };
    };

    template<typename T>
    struct OpwSolution
    {
        static_assert(std::is_floating_point_v<T>, "OpwSolution supports floating-point types");

        math::Vector<T, 6> q{};
        bool wristSingular{ false };
    };

    template<typename T>
    using OpwSolutions = std::array<std::optional<OpwSolution<T>>, 8>;

    template<typename T>
    class AnalyticalIkOpw
    {
        static_assert(std::is_floating_point_v<T>, "AnalyticalIkOpw supports floating-point types");

    public:
        using JointVector = math::Vector<T, 6>;

        explicit AnalyticalIkOpw(const OpwParameters<T>& parameters);

        OPTIMIZE_FOR_SPEED OpwSolutions<T> Solve(const math::SE3Transform<T>& flange) const;
        math::SE3Transform<T> Forward(const JointVector& q) const;

    private:
        using Matrix3 = math::Matrix3<T>;

        struct ArmBranch
        {
            T theta1;
            T theta2;
            T theta3;
        };

        struct WristAngles
        {
            T theta4;
            T theta5;
            T theta6;
        };

        static constexpr T pi{ std::numbers::pi_v<T> };
        static constexpr T acosTolerance{ T{ 100 } * std::numeric_limits<T>::epsilon() };
        static constexpr T wristTolerance{ T{ 100 } * std::numeric_limits<T>::epsilon() };

        static Matrix3 RotationZ(T angle);
        static Matrix3 RotationY(T angle);
        static std::optional<T> SafeAcos(T numerator, T denominator);

        void SolveWrist(OpwSolutions<T>& solutions, std::size_t slot, const ArmBranch& arm, const Matrix3& rotation) const;
        OpwSolution<T> Emit(const ArmBranch& arm, const WristAngles& wrist, bool singular) const;

        OpwParameters<T> parameters;
        T forearm;
        T forearmAngle;
    };

    template<typename T>
    AnalyticalIkOpw<T>::AnalyticalIkOpw(const OpwParameters<T>& parameters)
        : parameters{ parameters }
        , forearm{ math::Sqrt(parameters.a2 * parameters.a2 + parameters.c3 * parameters.c3) }
        , forearmAngle{ math::Atan2(parameters.a2, parameters.c3) }
    {}

    template<typename T>
    OPTIMIZE_FOR_SPEED OpwSolutions<T> AnalyticalIkOpw<T>::Solve(const math::SE3Transform<T>& flange) const
    {
        OpwSolutions<T> solutions{};
        const auto& rotation{ flange.rotation };
        const T cx{ flange.translation[0] - parameters.c4 * rotation.at(0, 2) };
        const T cy{ flange.translation[1] - parameters.c4 * rotation.at(1, 2) };
        const T cz{ flange.translation[2] - parameters.c4 * rotation.at(2, 2) };

        const T radialSquared{ cx * cx + cy * cy - parameters.b * parameters.b };
        if (radialSquared < T{})
            return solutions;

        const T radial{ math::Sqrt(radialSquared) };
        const T n{ radial - parameters.a1 };
        const T z{ cz - parameters.c1 };
        const T azimuth{ math::Atan2(cy, cx) };
        const T lateral{ math::Atan2(parameters.b, radial) };
        const T frontShoulder{ azimuth - lateral };
        const T backShoulder{ azimuth + lateral - pi };

        const T backReach{ n + T{ 2 } * parameters.a1 };
        const T frontDistanceSquared{ n * n + z * z };
        const T backDistanceSquared{ backReach * backReach + z * z };
        const T c2Squared{ parameters.c2 * parameters.c2 };
        const T forearmSquared{ forearm * forearm };
        const T elbowDenominator{ T{ 2 } * parameters.c2 * forearm };

        const auto beta1{ SafeAcos(frontDistanceSquared + c2Squared - forearmSquared, T{ 2 } * math::Sqrt(frontDistanceSquared) * parameters.c2) };
        const auto gamma1{ SafeAcos(frontDistanceSquared - c2Squared - forearmSquared, elbowDenominator) };
        const auto beta2{ SafeAcos(backDistanceSquared + c2Squared - forearmSquared, T{ 2 } * math::Sqrt(backDistanceSquared) * parameters.c2) };
        const auto gamma2{ SafeAcos(backDistanceSquared - c2Squared - forearmSquared, elbowDenominator) };

        if (beta1 && gamma1)
        {
            const T shoulder{ math::Atan2(n, z) };
            SolveWrist(solutions, 0, ArmBranch{ frontShoulder, shoulder - *beta1, *gamma1 - forearmAngle }, rotation);
            SolveWrist(solutions, 1, ArmBranch{ frontShoulder, shoulder + *beta1, -*gamma1 - forearmAngle }, rotation);
        }

        if (beta2 && gamma2)
        {
            const T shoulder{ -math::Atan2(backReach, z) };
            SolveWrist(solutions, 2, ArmBranch{ backShoulder, shoulder - *beta2, *gamma2 - forearmAngle }, rotation);
            SolveWrist(solutions, 3, ArmBranch{ backShoulder, shoulder + *beta2, -*gamma2 - forearmAngle }, rotation);
        }

        return solutions;
    }

    template<typename T>
    math::SE3Transform<T> AnalyticalIkOpw<T>::Forward(const JointVector& q) const
    {
        std::array<T, 6> theta{};
        for (std::size_t i = 0; i < 6; ++i)
            theta[i] = static_cast<T>(parameters.signs[i]) * q[i] - parameters.offsets[i];

        const T elbow{ theta[1] + theta[2] + forearmAngle };
        const T x{ parameters.c2 * math::Sin(theta[1]) + forearm * math::Sin(elbow) + parameters.a1 };
        const T z{ parameters.c2 * math::Cos(theta[1]) + forearm * math::Cos(elbow) };
        const T cos1{ math::Cos(theta[0]) };
        const T sin1{ math::Sin(theta[0]) };
        const math::Vector3<T> wristCentre{ x * cos1 - parameters.b * sin1, x * sin1 + parameters.b * cos1, z + parameters.c1 };

        const Matrix3 rotation{ RotationZ(theta[0]) * RotationY(theta[1] + theta[2]) * RotationZ(theta[3]) * RotationY(theta[4]) * RotationZ(theta[5]) };
        const math::Vector3<T> approach{ rotation.at(0, 2), rotation.at(1, 2), rotation.at(2, 2) };
        return math::SE3Transform<T>{ rotation, wristCentre + approach * parameters.c4 };
    }

    template<typename T>
    auto AnalyticalIkOpw<T>::RotationZ(T angle) -> Matrix3
    {
        const T c{ math::Cos(angle) };
        const T s{ math::Sin(angle) };
        return Matrix3{ { c, -s, T{} }, { s, c, T{} }, { T{}, T{}, T{ 1 } } };
    }

    template<typename T>
    auto AnalyticalIkOpw<T>::RotationY(T angle) -> Matrix3
    {
        const T c{ math::Cos(angle) };
        const T s{ math::Sin(angle) };
        return Matrix3{ { c, T{}, s }, { T{}, T{ 1 }, T{} }, { -s, T{}, c } };
    }

    template<typename T>
    std::optional<T> AnalyticalIkOpw<T>::SafeAcos(T numerator, T denominator)
    {
        if (denominator == T{})
            return std::nullopt;

        const T cosine{ numerator / denominator };
        if (math::Abs(cosine) > T{ 1 } + acosTolerance)
            return std::nullopt;

        return math::Acos(std::min(std::max(cosine, T{ -1 }), T{ 1 }));
    }

    template<typename T>
    void AnalyticalIkOpw<T>::SolveWrist(OpwSolutions<T>& solutions, std::size_t slot, const ArmBranch& arm, const Matrix3& rotation) const
    {
        const Matrix3 wrist{ Matrix3{ RotationZ(arm.theta1) * RotationY(arm.theta2 + arm.theta3) }.Transpose() * rotation };
        const T sin5{ math::Sqrt(wrist.at(0, 2) * wrist.at(0, 2) + wrist.at(1, 2) * wrist.at(1, 2)) };

        if (sin5 > wristTolerance)
        {
            const WristAngles regular{ math::Atan2(wrist.at(1, 2), wrist.at(0, 2)), math::Atan2(sin5, wrist.at(2, 2)), math::Atan2(wrist.at(2, 1), -wrist.at(2, 0)) };
            solutions[slot] = Emit(arm, regular, false);
            solutions[slot + 4] = Emit(arm, WristAngles{ regular.theta4 + pi, -regular.theta5, regular.theta6 - pi }, false);
            return;
        }

        if (wrist.at(2, 2) > T{})
            solutions[slot] = Emit(arm, WristAngles{ T{}, T{}, math::Atan2(wrist.at(1, 0), wrist.at(1, 1)) }, true);
        else
            solutions[slot] = Emit(arm, WristAngles{ T{}, pi, -math::Atan2(-wrist.at(1, 0), wrist.at(1, 1)) }, true);
    }

    template<typename T>
    OpwSolution<T> AnalyticalIkOpw<T>::Emit(const ArmBranch& arm, const WristAngles& wrist, bool singular) const
    {
        const std::array<T, 6> theta{ arm.theta1, arm.theta2, arm.theta3, wrist.theta4, wrist.theta5, wrist.theta6 };

        OpwSolution<T> solution{};
        for (std::size_t i = 0; i < 6; ++i)
            solution.q[i] = math::WrapToPi(static_cast<T>(parameters.signs[i]) * (theta[i] + parameters.offsets[i]));
        solution.wristSingular = singular;
        return solution;
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class AnalyticalIkOpw<float>;
#endif
}
