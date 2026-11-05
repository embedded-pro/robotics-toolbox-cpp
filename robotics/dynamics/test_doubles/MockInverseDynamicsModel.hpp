#pragma once

#include "robotics/dynamics/InverseDynamicsModel.hpp"
#include <gmock/gmock.h>

namespace dynamics
{
    template<typename T, std::size_t Dof>
    class MockInverseDynamicsModel
        : public InverseDynamicsModel<T, Dof>
    {
    public:
        using typename InverseDynamicsModel<T, Dof>::StateVector;

        MOCK_METHOD(StateVector, ComputeInverseDynamics, (const StateVector& q, const StateVector& qDot, const StateVector& qDDot), (const, override));
    };
}
