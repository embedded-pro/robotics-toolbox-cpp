#pragma once

#include "robotics/kinematics/JacobianProvider.hpp"
#include <gmock/gmock.h>

namespace kinematics
{
    template<typename T, std::size_t TaskDim, std::size_t Dof>
    class MockJacobianProvider
        : public JacobianProvider<T, TaskDim, Dof>
    {
    public:
        using typename JacobianProvider<T, TaskDim, Dof>::JointVector;
        using typename JacobianProvider<T, TaskDim, Dof>::TaskJacobian;
        using typename JacobianProvider<T, TaskDim, Dof>::TaskVector;

        MOCK_METHOD(TaskJacobian, Jacobian, (const JointVector& q), (const, override));
        MOCK_METHOD(TaskVector, BiasAcceleration, (const JointVector& q, const JointVector& qDot), (const, override));
        MOCK_METHOD(math::SE3Transform<T>, ToolPose, (const JointVector& q), (const, override));
    };
}
