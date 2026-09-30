#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "infra/util/ReallyAssert.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Geometry3D.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/solvers/GaussianElimination.hpp"
#include "robotics/dynamics/RevoluteJointLink.hpp"
#include "robotics/kinematics/ForwardKinematics.hpp"
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
        using LinkArray = std::array<dynamics::RevoluteJointLink<T>, NumLinks>;
        using PositionArray = std::array<Vector3, NumLinks + 1>;

        explicit InverseKinematics(const Vector3& toolOffset, InverseKinematicsConfig<T> cfg = {});

        OPTIMIZE_FOR_SPEED InverseKinematicsResult<T, NumLinks> Solve(
            const LinkArray& links,
            const Vector3& target,
            const JointVector& initialQ) const;

    private:
        Jacobian ComputeJacobian(
            const LinkArray& links,
            const JointVector& q,
            const PositionArray& positions) const;

        JointVector DampedLeastSquaresStep(const Jacobian& jacobian, const Vector3& error) const;

        T PositionError(const LinkArray& links, const JointVector& q, const Vector3& target) const;

        InverseKinematicsConfig<T> config;
        ForwardKinematics<T, NumLinks> fk;
    };

    template<typename T, std::size_t NumLinks>
    InverseKinematics<T, NumLinks>::InverseKinematics(const Vector3& toolOffset, InverseKinematicsConfig<T> cfg)
        : config{ cfg }
        , fk{ toolOffset }
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
            const auto positions = fk.Compute(links, q);
            const Vector3 error{ target - positions[NumLinks] };
            const T errorNorm{ math::VectorNorm(error) };

            if (errorNorm < config.tolerance)
                return { q, errorNorm, iteration, true };

            q = q + DampedLeastSquaresStep(ComputeJacobian(links, q, positions), error);
        }

        return { q, PositionError(links, q, target), config.maxIterations, false };
    }

    template<typename T, std::size_t NumLinks>
    typename InverseKinematics<T, NumLinks>::Jacobian
    InverseKinematics<T, NumLinks>::ComputeJacobian(
        const LinkArray& links,
        const JointVector& q,
        const PositionArray& positions) const
    {
        const Vector3& toolPosition = positions[NumLinks];

        Matrix3 R{ Matrix3::Identity() };
        Jacobian J{};

        for (std::size_t i = 0; i < NumLinks; ++i)
        {
            const Vector3 worldAxis{ R * links[i].jointAxis };
            const Vector3 column{ math::CrossProduct(worldAxis, Vector3{ toolPosition - positions[i] }) };

            J.at(0, i) = column.at(0, 0);
            J.at(1, i) = column.at(1, 0);
            J.at(2, i) = column.at(2, 0);

            R = R * math::RotationAboutAxis(links[i].jointAxis, q.at(i, 0));
        }

        return J;
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
        const auto positions = fk.Compute(links, q);
        return math::VectorNorm(Vector3{ target - positions[NumLinks] });
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class InverseKinematics<float, 1>;
    extern template class InverseKinematics<float, 2>;
    extern template class InverseKinematics<float, 3>;
#endif
}
