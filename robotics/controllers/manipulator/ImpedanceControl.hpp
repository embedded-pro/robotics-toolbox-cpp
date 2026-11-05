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
    class ImpedanceControl
    {
        static_assert(std::is_floating_point_v<T>, "ImpedanceControl supports floating-point types");

    public:
        using JointVector = math::Vector<T, Dof>;
        using TaskVector = math::Vector<T, TaskDim>;
        using TaskMatrix = math::SquareMatrix<T, TaskDim>;
        using Model = dynamics::EulerLagrangeDynamics<T, Dof>;
        using Provider = kinematics::JacobianProvider<T, TaskDim, Dof>;

        ImpedanceControl(const Model& model, const Provider& jacobian, const TaskMatrix& stiffness, const TaskMatrix& damping, const TaskMatrix& targetInertia);

        OPTIMIZE_FOR_SPEED JointVector ComputeTorque(const JointVector& q, const JointVector& qDot, const math::SE3Transform<T>& desiredPose, const TaskVector& xdDot) const;

        OPTIMIZE_FOR_SPEED JointVector ComputeTorqueWithInertiaShaping(const JointVector& q, const JointVector& qDot, const math::SE3Transform<T>& desiredPose,
            const TaskVector& xdDot, const TaskVector& xdDdot, const TaskVector& fExternal) const;

    private:
        const Model& model;
        const Provider& jacobian;
        TaskMatrix stiffness;
        TaskMatrix damping;
        TaskMatrix targetInertiaInverse;
    };

    template<typename T, std::size_t Dof, std::size_t TaskDim>
    ImpedanceControl<T, Dof, TaskDim>::ImpedanceControl(const Model& model, const Provider& jacobian, const TaskMatrix& stiffness, const TaskMatrix& damping, const TaskMatrix& targetInertia)
        : model{ model }
        , jacobian{ jacobian }
        , stiffness{ stiffness }
        , damping{ damping }
        , targetInertiaInverse{ solvers::SolveSystem(targetInertia, TaskMatrix::Identity()) }
    {}

    template<typename T, std::size_t Dof, std::size_t TaskDim>
    OPTIMIZE_FOR_SPEED auto ImpedanceControl<T, Dof, TaskDim>::ComputeTorque(const JointVector& q, const JointVector& qDot, const math::SE3Transform<T>& desiredPose, const TaskVector& xdDot) const -> JointVector
    {
        const auto taskJacobian{ jacobian.Jacobian(q) };
        const TaskVector error{ TaskError<T, TaskDim>(desiredPose, jacobian.ToolPose(q)) };
        const TaskVector errorRate{ xdDot - taskJacobian * qDot };

        return taskJacobian.Transpose() * TaskVector{ stiffness * error + damping * errorRate } + model.ComputeGravityTerms(q);
    }

    template<typename T, std::size_t Dof, std::size_t TaskDim>
    OPTIMIZE_FOR_SPEED auto ImpedanceControl<T, Dof, TaskDim>::ComputeTorqueWithInertiaShaping(const JointVector& q, const JointVector& qDot, const math::SE3Transform<T>& desiredPose,
        const TaskVector& xdDot, const TaskVector& xdDdot, const TaskVector& fExternal) const -> JointVector
    {
        const auto taskJacobian{ jacobian.Jacobian(q) };
        const auto transposed{ taskJacobian.Transpose() };
        const TaskVector error{ TaskError<T, TaskDim>(desiredPose, jacobian.ToolPose(q)) };
        const TaskVector errorRate{ xdDot - taskJacobian * qDot };
        const TaskVector acceleration{ xdDdot + targetInertiaInverse * TaskVector{ damping * errorRate + stiffness * error + fExternal } };

        const auto inverseMassTimesTransposed{ solvers::SolveSystem(model.ComputeMassMatrix(q), transposed) };
        const TaskVector force{ solvers::SolveSystem(TaskMatrix{ taskJacobian * inverseMassTimesTransposed }, TaskVector{ acceleration - jacobian.BiasAcceleration(q, qDot) }) - fExternal };

        return transposed * force + model.ComputeCoriolisTerms(q, qDot) + model.ComputeGravityTerms(q);
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class ImpedanceControl<float, 2, 2>;
    extern template class ImpedanceControl<float, 6, 6>;
#endif
}
