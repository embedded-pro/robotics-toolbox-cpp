#pragma once

#include "robotics/dynamics/InertialRegressor.hpp"
#include <gmock/gmock.h>

namespace dynamics
{
    template<typename T, std::size_t Dof, std::size_t NumParams>
    class MockInertialRegressor
        : public InertialRegressor<T, Dof, NumParams>
    {
    public:
        using typename InertialRegressor<T, Dof, NumParams>::JointVector;
        using typename InertialRegressor<T, Dof, NumParams>::Regressor;

        MOCK_METHOD(Regressor, Compute, (const JointVector& q, const JointVector& qDot, const JointVector& qDotReference, const JointVector& qDDotReference), (const, override));
    };
}
