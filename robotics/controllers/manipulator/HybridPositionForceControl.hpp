#pragma once

#include "infra/util/ReallyAssert.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/math/SE3Transform.hpp"
#include "robotics/controllers/manipulator/TaskError.hpp"
#include "robotics/dynamics/EulerLagrangeDynamics.hpp"
#include "robotics/kinematics/JacobianProvider.hpp"
#include <algorithm>
#include <cstddef>
#include <type_traits>

namespace controllers
{
    template<typename T, std::size_t Dof, std::size_t TaskDim>
    class HybridPositionForceControl
    {
        static_assert(std::is_floating_point_v<T>, "HybridPositionForceControl supports floating-point types");

    public:
        using JointVector = math::Vector<T, Dof>;
        using TaskVector = math::Vector<T, TaskDim>;
        using TaskMatrix = math::SquareMatrix<T, TaskDim>;
        using Model = dynamics::EulerLagrangeDynamics<T, Dof>;
        using Provider = kinematics::JacobianProvider<T, TaskDim, Dof>;

        struct Gains
        {
            TaskMatrix kp;
            TaskMatrix kd;
            TaskMatrix kf;
            TaskMatrix ki;
            TaskMatrix kdf;
            TaskVector integralLimit;
        };

        HybridPositionForceControl(const Model& model, const Provider& jacobian, const TaskMatrix& constraintRotation, const TaskMatrix& selection, const Gains& gains, T dt);

        OPTIMIZE_FOR_SPEED JointVector ComputeTorque(const JointVector& q, const JointVector& qDot, const math::SE3Transform<T>& desiredPose, const TaskVector& xdDot,
            const TaskVector& fMeasured, const TaskVector& fd);
        void Reset();

    private:
        const Model& model;
        const Provider& jacobian;
        TaskMatrix constraintRotation;
        TaskMatrix selection;
        TaskMatrix complement;
        Gains gains;
        TaskVector forceIntegral{};
        T dt;
    };

    template<typename T, std::size_t Dof, std::size_t TaskDim>
    HybridPositionForceControl<T, Dof, TaskDim>::HybridPositionForceControl(const Model& model, const Provider& jacobian, const TaskMatrix& constraintRotation, const TaskMatrix& selection,
        const Gains& gains, T dt)
        : model{ model }
        , jacobian{ jacobian }
        , constraintRotation{ constraintRotation }
        , selection{ selection }
        , complement{ TaskMatrix::Identity() - selection }
        , gains{ gains }
        , dt{ dt }
    {
        for (std::size_t r = 0; r < TaskDim; ++r)
            for (std::size_t c = 0; c < TaskDim; ++c)
                really_assert(r == c ? (selection.at(r, c) == T{} || selection.at(r, c) == T{ 1 }) : selection.at(r, c) == T{});
    }

    template<typename T, std::size_t Dof, std::size_t TaskDim>
    OPTIMIZE_FOR_SPEED auto HybridPositionForceControl<T, Dof, TaskDim>::ComputeTorque(const JointVector& q, const JointVector& qDot, const math::SE3Transform<T>& desiredPose,
        const TaskVector& xdDot, const TaskVector& fMeasured, const TaskVector& fd) -> JointVector
    {
        const auto taskJacobian{ jacobian.Jacobian(q) };
        const TaskVector taskVelocity{ taskJacobian * qDot };
        const auto toConstraint{ constraintRotation.Transpose() };

        const TaskVector error{ toConstraint * TaskError<T, TaskDim>(desiredPose, jacobian.ToolPose(q)) };
        const TaskVector errorRate{ toConstraint * TaskVector{ xdDot - taskVelocity } };
        const TaskVector constraintVelocity{ toConstraint * taskVelocity };
        const TaskVector desiredForce{ toConstraint * fd };
        const TaskVector forceError{ desiredForce - toConstraint * fMeasured };

        const TaskVector motionWrench{ gains.kp * error + gains.kd * errorRate };

        forceIntegral = forceIntegral + complement * forceError * dt;
        for (std::size_t i = 0; i < TaskDim; ++i)
            forceIntegral[i] = std::min(std::max(forceIntegral[i], -gains.integralLimit[i]), gains.integralLimit[i]);
        const TaskVector forceWrench{ desiredForce + gains.kf * forceError + gains.ki * forceIntegral - gains.kdf * constraintVelocity };

        const TaskVector wrench{ constraintRotation * TaskVector{ selection * motionWrench + complement * forceWrench } };
        return taskJacobian.Transpose() * wrench + model.ComputeCoriolisTerms(q, qDot) + model.ComputeGravityTerms(q);
    }

    template<typename T, std::size_t Dof, std::size_t TaskDim>
    void HybridPositionForceControl<T, Dof, TaskDim>::Reset()
    {
        forceIntegral = TaskVector{};
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class HybridPositionForceControl<float, 2, 2>;
    extern template class HybridPositionForceControl<float, 6, 6>;
#endif
}
