#pragma once

#include "numerical/math/SE3Transform.hpp"
#include "robotics/kinematics/DenavitHartenberg.hpp"
#include "robotics/kinematics/GeometricJacobian.hpp"
#include "robotics/kinematics/JacobianProvider.hpp"
#include <cstddef>

namespace kinematics
{
    template<typename T, std::size_t TaskDim, std::size_t N>
    class DhTaskJacobian
        : public JacobianProvider<T, TaskDim, N>
    {
    public:
        using typename JacobianProvider<T, TaskDim, N>::JointVector;
        using typename JacobianProvider<T, TaskDim, N>::TaskJacobian;
        using typename JacobianProvider<T, TaskDim, N>::TaskVector;

        explicit DhTaskJacobian(const DenavitHartenberg<T, N>& model);

        TaskJacobian Jacobian(const JointVector& q) const override;
        TaskVector BiasAcceleration(const JointVector& q, const JointVector& qDot) const override;
        math::SE3Transform<T> ToolPose(const JointVector& q) const override;

    private:
        using Geometric = GeometricJacobian<T, N>;

        const DenavitHartenberg<T, N>& model;
    };

    template<typename T, std::size_t TaskDim, std::size_t N>
    DhTaskJacobian<T, TaskDim, N>::DhTaskJacobian(const DenavitHartenberg<T, N>& model)
        : model{ model }
    {}

    template<typename T, std::size_t TaskDim, std::size_t N>
    typename DhTaskJacobian<T, TaskDim, N>::TaskJacobian DhTaskJacobian<T, TaskDim, N>::Jacobian(const JointVector& q) const
    {
        return Geometric::template TaskRows<TaskDim>(Geometric::Compute(model.Frames(q)));
    }

    template<typename T, std::size_t TaskDim, std::size_t N>
    typename DhTaskJacobian<T, TaskDim, N>::TaskVector DhTaskJacobian<T, TaskDim, N>::BiasAcceleration(const JointVector& q, const JointVector& qDot) const
    {
        return Geometric::template TaskTwist<TaskDim>(Geometric::BiasAcceleration(model.Frames(q), qDot));
    }

    template<typename T, std::size_t TaskDim, std::size_t N>
    math::SE3Transform<T> DhTaskJacobian<T, TaskDim, N>::ToolPose(const JointVector& q) const
    {
        return model.Forward(q);
    }

#ifdef ROBOTICS_TOOLBOX_COVERAGE_BUILD
    extern template class DhTaskJacobian<float, 6, 6>;
#endif
}
