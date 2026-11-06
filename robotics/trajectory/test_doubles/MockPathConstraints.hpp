#pragma once

#include "robotics/trajectory/TimeOptimalPathParameterization.hpp"
#include <gmock/gmock.h>

namespace trajectory
{
    template<typename T, std::size_t Dof>
    class MockPathConstraints
        : public PathConstraints<T, Dof>
    {
    public:
        MOCK_METHOD((PathCoefficients<T, Dof>), Coefficients, (T s), (const, override));
        MOCK_METHOD((const JointBounds<T, Dof>&), Bounds, (), (const, override));
    };
}
