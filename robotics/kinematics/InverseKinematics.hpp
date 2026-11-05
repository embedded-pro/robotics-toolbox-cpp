#pragma once

#include "infra/util/ReallyAssert.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Geometry3D.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/solvers/GaussianElimination.hpp"
#include "robotics/dynamics/JointLink.hpp"
#include "robotics/kinematics/ChainPoseKinematics.hpp"
#include "robotics/kinematics/GeometricJacobian.hpp"
#include <array>
#include <cstddef>

namespace kinematics
{
    template<typename T>
    struct InverseKinematicsConfig
    {
        static_assert(std::is_floating_point_v<T>,
            "InverseKinematicsConfig only supports floating-point types");

        T dampingFactor = T(0.1);
        T tolerance = T(1e-4);
        std::size_t maxIterations = 100;
    };

    template<typename T, std::size_t NumLinks>
    struct InverseKinematicsResult
    {
        math::Vector<T, NumLinks> q;
        T finalError;
        std::size_t iterations;
        bool converged;
    };

    template<typename T, std::size_t NumLinks>
    class InverseKinematics
    {
        static_assert(std::is_floating_point_v<T>,
            "InverseKinematics only supports floating-point types");
        static_assert(NumLinks > 0, "Number of links must be positive");

    public:
        using Vector3 = math::Vector3<T>;
        using Matrix3 = math::Matrix3<T>;
        using JointVector = math::Vector<T, NumLinks>;
        using Jacobian = math::Matrix<T, 3, NumLinks>;
        using LinkArray = std::array<dynamics::JointLink<T>, NumLinks>;

        explicit InverseKinematics(const Vector3& toolOffset, InverseKinematicsConfig<T> cfg = {});

        OPTIMIZE_FOR_SPEED InverseKinematicsResult<T, NumLinks> Solve(
            const LinkArray& links,
            const Vector3& target,
            const JointVector& initialQ) const;

    private:
        JointVector DampedLeastSquaresStep(const Jacobian& jacobian, const Vector3& error) const;

        T PositionError(const LinkArray& links, const JointVector& q, const Vector3& target) const;

        InverseKinematicsConfig<T> config;
        ChainPoseKinematics<T, NumLinks> kinematics;
    };

    template<typename T, std::size_t NumLinks>
    InverseKinematics<T, NumLinks>::InverseKinematics(const Vector3& toolOffset, InverseKinematicsConfig<T> cfg)
        : config{ cfg }
        , kinematics{ math::SE3Transform<T>{ Matrix3::Identity(), toolOffset } }
    {
        really_assert(config.dampingFactor > T(0));
        really_assert(config.tolerance > T(0));
        really_assert(config.maxIterations > 0);
    }

    template<typename T, std::size_t NumLinks>
    OPTIMIZE_FOR_SPEED
        InverseKinematicsResult<T, NumLinks>
        InverseKinematics<T, NumLinks>::Solve(
            const LinkArray& links,
            const Vector3& target,
            const JointVector& initialQ) const
    {
        JointVector q{ initialQ };

        for (std::size_t iteration = 0; iteration < config.maxIterations; ++iteration)
        {
            const auto frames{ kinematics.Compute(links, q) };
            const Vector3 error{ target - frames.tool.translation };
            const T errorNorm{ math::VectorNorm(error) };

            if (errorNorm < config.tolerance)
                return { q, errorNorm, iteration, true };

            const auto jacobian{ GeometricJacobian<T, NumLinks>::template TaskRows<3>(GeometricJacobian<T, NumLinks>::Compute(frames)) };
            q = q + DampedLeastSquaresStep(jacobian, error);
        }

        return { q, PositionError(links, q, target), config.maxIterations, false };
    }

    template<typename T, std::size_t NumLinks>
    typename InverseKinematics<T, NumLinks>::JointVector
    InverseKinematics<T, NumLinks>::DampedLeastSquaresStep(const Jacobian& jacobian, const Vector3& error) const
    {
        const T lambdaSquared{ config.dampingFactor * config.dampingFactor };
        const Matrix3 damped{ jacobian * jacobian.Transpose() + math::ScaledIdentity(lambdaSquared) };

        solvers::GaussianElimination<T, 3> solver;
        const Vector3 y{ solver.Solve(damped, error) };

        return jacobian.Transpose() * y;
    }

    template<typename T, std::size_t NumLinks>
    T InverseKinematics<T, NumLinks>::PositionError(const LinkArray& links, const JointVector& q, const Vector3& target) const
    {
        return math::VectorNorm(Vector3{ target - kinematics.ToolPose(links, q).translation });
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class InverseKinematics<float, 1>;
    extern template class InverseKinematics<float, 2>;
    extern template class InverseKinematics<float, 3>;
#endif
}
