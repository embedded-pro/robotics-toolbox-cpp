#pragma once

#include "numerical/math/Matrix.hpp"
#include "numerical/math/SE3Transform.hpp"
#include <cstddef>
#include <type_traits>

namespace kinematics
{
    template<typename T, std::size_t TaskDim, std::size_t Dof>
    class JacobianProvider
    {
        static_assert(std::is_floating_point_v<T>, "JacobianProvider supports floating-point types");

    public:
        using JointVector = math::Vector<T, Dof>;
        using TaskJacobian = math::Matrix<T, TaskDim, Dof>;
        using TaskVector = math::Vector<T, TaskDim>;

        virtual TaskJacobian Jacobian(const JointVector& q) const = 0;
        virtual TaskVector BiasAcceleration(const JointVector& q, const JointVector& qDot) const = 0;
        virtual math::SE3Transform<T> ToolPose(const JointVector& q) const = 0;

    protected:
        ~JacobianProvider() = default;
    };
}
