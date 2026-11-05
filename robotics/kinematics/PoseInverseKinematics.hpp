#pragma once

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/math/SE3Transform.hpp"
#include "numerical/optimization/DampingPolicy.hpp"
#include "numerical/optimization/LeastSquaresProblem.hpp"
#include "numerical/optimization/LevenbergMarquardt.hpp"
#include "numerical/solvers/LuDecomposition.hpp"
#include "robotics/kinematics/JacobianProvider.hpp"
#include <algorithm>
#include <cstddef>
#include <limits>
#include <type_traits>

namespace kinematics
{
    template<typename T, std::size_t Dof>
    struct PoseIkConfig
    {
        static_assert(std::is_floating_point_v<T>, "PoseIkConfig supports floating-point types");

        using JointVector = math::Vector<T, Dof>;

        static JointVector Filled(T value)
        {
            JointVector filled{};
            for (std::size_t i = 0; i < Dof; ++i)
                filled[i] = value;
            return filled;
        }

        T damping{};
        T manipulabilityThreshold{};
        T characteristicLength{ 1 };
        T tolerance{};
        T maxStep{ std::numeric_limits<T>::infinity() };
        std::size_t maxIterations{};
        JointVector lowerLimits{ Filled(-std::numeric_limits<T>::infinity()) };
        JointVector upperLimits{ Filled(std::numeric_limits<T>::infinity()) };
    };

    template<typename T, std::size_t Dof>
    struct PoseIkResult
    {
        static_assert(std::is_floating_point_v<T>, "PoseIkResult supports floating-point types");

        math::Vector<T, Dof> q{};
        T finalError{};
        std::size_t iterations{};
        bool converged{ false };
    };

    template<typename T, std::size_t Dof>
    class PoseIkProblem
        : public optimization::LeastSquaresProblem<T, math::Vector<T, Dof>, 6, Dof>
    {
        static_assert(std::is_floating_point_v<T>, "PoseIkProblem supports floating-point types");

    public:
        using JointVector = math::Vector<T, Dof>;
        using Arm = JacobianProvider<T, 6, Dof>;

        PoseIkProblem(const Arm& arm, const math::SE3Transform<T>& target, T characteristicLength, const JointVector& lowerLimits, const JointVector& upperLimits);

        math::Vector<T, 6> Residual(const JointVector& q) const override;
        math::Matrix<T, 6, Dof> Jacobian(const JointVector& q) const override;
        JointVector Retract(const JointVector& q, const JointVector& delta) const override;

    private:
        const Arm& arm;
        math::SE3Transform<T> target;
        T characteristicLength;
        JointVector lowerLimits;
        JointVector upperLimits;
    };

    template<typename T, std::size_t Dof>
    class ManipulabilityDamping
        : public optimization::DampingPolicy<T, 6, Dof>
    {
        static_assert(std::is_floating_point_v<T>, "ManipulabilityDamping supports floating-point types");

    public:
        ManipulabilityDamping(T damping, T threshold);

        T DampingSquared(const math::Matrix<T, 6, Dof>& jacobian) const override;

    private:
        T damping;
        T threshold;
    };

    template<typename T, std::size_t Dof>
    class PoseInverseKinematics
    {
        static_assert(std::is_floating_point_v<T>, "PoseInverseKinematics supports floating-point types");

    public:
        using JointVector = math::Vector<T, Dof>;
        using Arm = JacobianProvider<T, 6, Dof>;
        using Config = PoseIkConfig<T, Dof>;
        using Result = PoseIkResult<T, Dof>;

        PoseInverseKinematics(const Arm& arm, const Config& config);

        OPTIMIZE_FOR_SPEED Result Solve(const math::SE3Transform<T>& target, const JointVector& q0) const;

    private:
        const Arm& arm;
        Config config;
    };

    template<typename T, std::size_t Dof>
    PoseIkProblem<T, Dof>::PoseIkProblem(const Arm& arm, const math::SE3Transform<T>& target, T characteristicLength, const JointVector& lowerLimits, const JointVector& upperLimits)
        : arm{ arm }
        , target{ target }
        , characteristicLength{ characteristicLength }
        , lowerLimits{ lowerLimits }
        , upperLimits{ upperLimits }
    {}

    template<typename T, std::size_t Dof>
    math::Vector<T, 6> PoseIkProblem<T, Dof>::Residual(const JointVector& q) const
    {
        const auto error{ math::SE3Transform<T>::PoseError(target, arm.ToolPose(q)) };

        math::Vector<T, 6> residual{};
        for (std::size_t i = 0; i < 6; ++i)
            residual[i] = -(i < 3 ? error[i] : characteristicLength * error[i]);
        return residual;
    }

    template<typename T, std::size_t Dof>
    math::Matrix<T, 6, Dof> PoseIkProblem<T, Dof>::Jacobian(const JointVector& q) const
    {
        auto weighted{ arm.Jacobian(q) };
        for (std::size_t r = 3; r < 6; ++r)
            for (std::size_t c = 0; c < Dof; ++c)
                weighted.at(r, c) *= characteristicLength;
        return weighted;
    }

    template<typename T, std::size_t Dof>
    auto PoseIkProblem<T, Dof>::Retract(const JointVector& q, const JointVector& delta) const -> JointVector
    {
        JointVector next{};
        for (std::size_t i = 0; i < Dof; ++i)
            next[i] = std::min(std::max(q[i] + delta[i], lowerLimits[i]), upperLimits[i]);
        return next;
    }

    template<typename T, std::size_t Dof>
    ManipulabilityDamping<T, Dof>::ManipulabilityDamping(T damping, T threshold)
        : damping{ damping }
        , threshold{ threshold }
    {}

    template<typename T, std::size_t Dof>
    T ManipulabilityDamping<T, Dof>::DampingSquared(const math::Matrix<T, 6, Dof>& jacobian) const
    {
        const T maximum{ damping * damping };
        if (threshold == T{})
            return maximum;

        solvers::LuDecomposition<T, 6> lu;
        const T manipulability{ lu.Decompose(jacobian * jacobian.Transpose()) ? math::Sqrt(std::max(T{}, lu.Determinant())) : T{} };
        if (manipulability >= threshold)
            return T{};

        const T ratio{ manipulability / threshold };
        return maximum * (T{ 1 } - ratio * ratio);
    }

    template<typename T, std::size_t Dof>
    PoseInverseKinematics<T, Dof>::PoseInverseKinematics(const Arm& arm, const Config& config)
        : arm{ arm }
        , config{ config }
    {}

    template<typename T, std::size_t Dof>
    OPTIMIZE_FOR_SPEED auto PoseInverseKinematics<T, Dof>::Solve(const math::SE3Transform<T>& target, const JointVector& q0) const -> Result
    {
        const PoseIkProblem<T, Dof> problem{ arm, target, config.characteristicLength, config.lowerLimits, config.upperLimits };
        const ManipulabilityDamping<T, Dof> policy{ config.damping, config.manipulabilityThreshold };
        const optimization::LevenbergMarquardt<T, JointVector, 6, Dof> solver{ problem, policy, { config.tolerance, config.maxStep, config.maxIterations } };

        const auto solution{ solver.Solve(q0) };
        return Result{ solution.x, solution.residualNorm, solution.iterations, solution.converged };
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class PoseIkProblem<float, 6>;
    extern template class PoseIkProblem<float, 7>;
    extern template class ManipulabilityDamping<float, 6>;
    extern template class ManipulabilityDamping<float, 7>;
    extern template class PoseInverseKinematics<float, 6>;
    extern template class PoseInverseKinematics<float, 7>;
#endif
}
