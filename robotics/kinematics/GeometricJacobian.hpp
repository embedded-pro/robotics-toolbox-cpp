#pragma once

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Geometry3D.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/math/SE3Transform.hpp"
#include "robotics/dynamics/JointLink.hpp"
#include "robotics/kinematics/ChainPoseKinematics.hpp"
#include <cstddef>
#include <type_traits>

namespace kinematics
{
    template<typename T, std::size_t N>
    class GeometricJacobian
    {
        static_assert(std::is_floating_point_v<T>, "GeometricJacobian supports floating-point types");
        static_assert(N > 0, "Number of joints must be positive");

    public:
        using Jacobian = math::Matrix<T, 6, N>;
        using JointVector = math::Vector<T, N>;
        using Twist = math::Vector6<T>;
        using Vector3 = math::Vector3<T>;

        OPTIMIZE_FOR_SPEED static Jacobian Compute(const FrameChain<T, N>& frames);
        OPTIMIZE_FOR_SPEED static Twist BiasAcceleration(const FrameChain<T, N>& frames, const JointVector& qDot);
        static JointVector JointTorques(const Jacobian& jacobian, const Twist& wrench);

        template<std::size_t TaskDim>
        static math::Matrix<T, TaskDim, N> TaskRows(const Jacobian& jacobian);
        template<std::size_t TaskDim>
        static math::Vector<T, TaskDim> TaskTwist(const Twist& twist);

    private:
        static bool IsRevolute(const FrameChain<T, N>& frames, std::size_t joint);
        static void SetColumn(Jacobian& jacobian, std::size_t column, const Vector3& linear, const Vector3& angular);
    };

    template<typename T, std::size_t N>
    OPTIMIZE_FOR_SPEED typename GeometricJacobian<T, N>::Jacobian GeometricJacobian<T, N>::Compute(const FrameChain<T, N>& frames)
    {
        const Vector3& tool{ frames.tool.translation };

        Jacobian jacobian{};
        for (std::size_t i = 0; i < N; ++i)
        {
            const Vector3& axis{ frames.jointAxes[i] };
            if (IsRevolute(frames, i))
                SetColumn(jacobian, i, math::CrossProduct(axis, Vector3{ tool - frames.jointOrigins[i] }), axis);
            else
                SetColumn(jacobian, i, axis, Vector3{});
        }

        return jacobian;
    }

    template<typename T, std::size_t N>
    OPTIMIZE_FOR_SPEED typename GeometricJacobian<T, N>::Twist GeometricJacobian<T, N>::BiasAcceleration(const FrameChain<T, N>& frames, const JointVector& qDot)
    {
        const Vector3& tool{ frames.tool.translation };
        const Vector3 toolVelocity{ TaskTwist<3>(Twist{ Compute(frames) * qDot }) };

        Vector3 omega{};
        Vector3 originVelocity{};
        Vector3 linear{};
        Vector3 angular{};
        for (std::size_t i = 0; i < N; ++i)
        {
            const Vector3& origin{ frames.jointOrigins[i] };
            const Vector3& axis{ frames.jointAxes[i] };
            originVelocity = originVelocity + math::CrossProduct(omega, Vector3{ origin - frames.jointOrigins[i == 0 ? 0 : i - 1] });
            const Vector3 axisRate{ math::CrossProduct(omega, axis) };

            if (IsRevolute(frames, i))
            {
                linear = linear + (math::CrossProduct(axisRate, Vector3{ tool - origin }) + math::CrossProduct(axis, Vector3{ toolVelocity - originVelocity })) * qDot[i];
                angular = angular + axisRate * qDot[i];
                omega = omega + axis * qDot[i];
            }
            else
            {
                linear = linear + axisRate * qDot[i];
                originVelocity = originVelocity + axis * qDot[i];
            }
        }

        Twist bias{};
        bias.SetBlock(linear, 0, 0);
        bias.SetBlock(angular, 3, 0);
        return bias;
    }

    template<typename T, std::size_t N>
    typename GeometricJacobian<T, N>::JointVector GeometricJacobian<T, N>::JointTorques(const Jacobian& jacobian, const Twist& wrench)
    {
        return jacobian.Transpose() * wrench;
    }

    template<typename T, std::size_t N>
    template<std::size_t TaskDim>
    math::Matrix<T, TaskDim, N> GeometricJacobian<T, N>::TaskRows(const Jacobian& jacobian)
    {
        static_assert(TaskDim == 3 || TaskDim == 6, "Task dimension must be 3 (position) or 6 (pose)");
        return jacobian.template GetBlock<TaskDim, N>(0, 0);
    }

    template<typename T, std::size_t N>
    template<std::size_t TaskDim>
    math::Vector<T, TaskDim> GeometricJacobian<T, N>::TaskTwist(const Twist& twist)
    {
        static_assert(TaskDim == 3 || TaskDim == 6, "Task dimension must be 3 (position) or 6 (pose)");
        return twist.template GetBlock<TaskDim, 1>(0, 0);
    }

    template<typename T, std::size_t N>
    bool GeometricJacobian<T, N>::IsRevolute(const FrameChain<T, N>& frames, std::size_t joint)
    {
        return frames.jointTypes[joint] == dynamics::JointType::Revolute;
    }

    template<typename T, std::size_t N>
    void GeometricJacobian<T, N>::SetColumn(Jacobian& jacobian, std::size_t column, const Vector3& linear, const Vector3& angular)
    {
        jacobian.SetBlock(linear, 0, column);
        jacobian.SetBlock(angular, 3, column);
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class GeometricJacobian<float, 1>;
    extern template class GeometricJacobian<float, 2>;
    extern template class GeometricJacobian<float, 3>;
    extern template class GeometricJacobian<float, 6>;
#endif
}
