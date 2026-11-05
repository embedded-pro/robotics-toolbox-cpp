#pragma once

#include "numerical/math/Matrix.hpp"
#include "numerical/math/SE3Transform.hpp"
#include "robotics/dynamics/JointLink.hpp"
#include "robotics/kinematics/ChainPoseKinematics.hpp"
#include "robotics/kinematics/GeometricJacobian.hpp"
#include "robotics/kinematics/JacobianProvider.hpp"
#include <array>
#include <cstddef>

namespace kinematics
{
    template<typename T, std::size_t TaskDim, std::size_t Dof>
    class ChainTaskJacobian
        : public JacobianProvider<T, TaskDim, Dof>
    {
    public:
        using typename JacobianProvider<T, TaskDim, Dof>::JointVector;
        using typename JacobianProvider<T, TaskDim, Dof>::TaskJacobian;
        using typename JacobianProvider<T, TaskDim, Dof>::TaskVector;
        using LinkArray = std::array<dynamics::JointLink<T>, Dof>;

        ChainTaskJacobian(const LinkArray& links, const math::SE3Transform<T>& toolInLastLink);

        TaskJacobian Jacobian(const JointVector& q) const override;
        TaskVector BiasAcceleration(const JointVector& q, const JointVector& qDot) const override;
        math::SE3Transform<T> ToolPose(const JointVector& q) const override;

    private:
        using Geometric = GeometricJacobian<T, Dof>;

        const LinkArray& links;
        ChainPoseKinematics<T, Dof> kinematics;
    };

    template<typename T, std::size_t TaskDim, std::size_t Dof>
    ChainTaskJacobian<T, TaskDim, Dof>::ChainTaskJacobian(const LinkArray& links, const math::SE3Transform<T>& toolInLastLink)
        : links{ links }
        , kinematics{ toolInLastLink }
    {}

    template<typename T, std::size_t TaskDim, std::size_t Dof>
    typename ChainTaskJacobian<T, TaskDim, Dof>::TaskJacobian ChainTaskJacobian<T, TaskDim, Dof>::Jacobian(const JointVector& q) const
    {
        return Geometric::template TaskRows<TaskDim>(Geometric::Compute(kinematics.Compute(links, q)));
    }

    template<typename T, std::size_t TaskDim, std::size_t Dof>
    typename ChainTaskJacobian<T, TaskDim, Dof>::TaskVector ChainTaskJacobian<T, TaskDim, Dof>::BiasAcceleration(const JointVector& q, const JointVector& qDot) const
    {
        return Geometric::template TaskTwist<TaskDim>(Geometric::BiasAcceleration(kinematics.Compute(links, q), qDot));
    }

    template<typename T, std::size_t TaskDim, std::size_t Dof>
    math::SE3Transform<T> ChainTaskJacobian<T, TaskDim, Dof>::ToolPose(const JointVector& q) const
    {
        return kinematics.ToolPose(links, q);
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class ChainTaskJacobian<float, 3, 2>;
    extern template class ChainTaskJacobian<float, 6, 2>;
#endif
}
