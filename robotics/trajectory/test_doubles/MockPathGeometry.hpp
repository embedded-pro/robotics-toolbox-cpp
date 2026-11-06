#pragma once

#include "robotics/trajectory/TimeOptimalPathParameterization.hpp"
#include <gmock/gmock.h>

namespace trajectory
{
    template<typename T, std::size_t Dof>
    class MockPathGeometry
        : public PathGeometry<T, Dof>
    {
    public:
        using typename PathGeometry<T, Dof>::JointVector;

        MOCK_METHOD(JointVector, Position, (T s), (const, override));
        MOCK_METHOD(JointVector, FirstDerivative, (T s), (const, override));
        MOCK_METHOD(JointVector, SecondDerivative, (T s), (const, override));
    };
}
