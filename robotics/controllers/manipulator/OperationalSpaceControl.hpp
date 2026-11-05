#pragma once

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/math/SE3Transform.hpp"
#include "numerical/solvers/GaussianElimination.hpp"
#include "robotics/controllers/manipulator/TaskError.hpp"
#include "robotics/dynamics/EulerLagrangeDynamics.hpp"
#include "robotics/kinematics/JacobianProvider.hpp"
#include <cstddef>
#include <type_traits>

namespace controllers
{
    template<typename T, std::size_t Dof, std::size_t TaskDim>
    class OperationalSpaceControl
    {
        static_assert(std::is_floating_point_v<T>, "OperationalSpaceControl supports floating-point types");
        static_assert(TaskDim <= Dof, "Task dimension cannot exceed the degrees of freedom");

    public:
        using JointVector = math::Vector<T, Dof>;
        using JointMatrix = math::SquareMatrix<T, Dof>;
        using TaskVector = math::Vector<T, TaskDim>;
        using TaskMatrix = math::SquareMatrix<T, TaskDim>;
        using Model = dynamics::EulerLagrangeDynamics<T, Dof>;
        using Provider = kinematics::JacobianProvider<T, TaskDim, Dof>;

        OperationalSpaceControl(const Model& model, const Provider& jacobian, const TaskMatrix& kp, const TaskMatrix& kd, T sigma);

        OPTIMIZE_FOR_SPEED JointVector ComputeTorque(const JointVector& q, const JointVector& qDot, const math::SE3Transform<T>& desiredPose, const TaskVector& xdDot,
            const TaskVector& xdDdot, const JointVector& tauSecondary) const;

    private:
        const Model& model;
        const Provider& jacobian;
        TaskMatrix kp;
        TaskMatrix kd;
        T sigma;
    };

    template<typename T, std::size_t Dof, std::size_t TaskDim>
    OperationalSpaceControl<T, Dof, TaskDim>::OperationalSpaceControl(const Model& model, const Provider& jacobian, const TaskMatrix& kp, const TaskMatrix& kd, T sigma)
        : model{ model }
        , jacobian{ jacobian }
        , kp{ kp }
        , kd{ kd }
        , sigma{ sigma }
    {}

    template<typename T, std::size_t Dof, std::size_t TaskDim>
    OPTIMIZE_FOR_SPEED auto OperationalSpaceControl<T, Dof, TaskDim>::ComputeTorque(const JointVector& q, const JointVector& qDot, const math::SE3Transform<T>& desiredPose,
        const TaskVector& xdDot, const TaskVector& xdDdot, const JointVector& tauSecondary) const -> JointVector
    {
        const auto taskJacobian{ jacobian.Jacobian(q) };
        const auto transposed{ taskJacobian.Transpose() };
        const TaskVector error{ TaskError<T, TaskDim>(desiredPose, jacobian.ToolPose(q)) };
        const TaskVector errorRate{ xdDot - taskJacobian * qDot };
        const TaskVector acceleration{ xdDdot + kd * errorRate + kp * error };

        const auto inverseMassTimesTransposed{ solvers::SolveSystem(model.ComputeMassMatrix(q), transposed) };
        const TaskMatrix taskInertia{ solvers::SolveSystem(TaskMatrix{ taskJacobian * inverseMassTimesTransposed + TaskMatrix::Identity() * (sigma * sigma) }, TaskMatrix::Identity()) };
        const JointMatrix nullSpace{ JointMatrix::Identity() - transposed * TaskMatrix{ taskInertia } * inverseMassTimesTransposed.Transpose() };

        return transposed * TaskVector{ taskInertia * TaskVector{ acceleration - jacobian.BiasAcceleration(q, qDot) } } + model.ComputeCoriolisTerms(q, qDot) + model.ComputeGravityTerms(q) +
               nullSpace * tauSecondary;
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class OperationalSpaceControl<float, 2, 2>;
    extern template class OperationalSpaceControl<float, 3, 2>;
    extern template class OperationalSpaceControl<float, 7, 6>;
#endif
}
