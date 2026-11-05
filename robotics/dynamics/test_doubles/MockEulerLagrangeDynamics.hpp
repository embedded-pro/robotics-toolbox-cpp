#pragma once

#include "robotics/dynamics/EulerLagrangeDynamics.hpp"
#include <gmock/gmock.h>

namespace dynamics
{
    template<typename T, std::size_t Dof>
    class MockEulerLagrangeDynamics
        : public EulerLagrangeDynamics<T, Dof>
    {
    public:
        using typename EulerLagrangeDynamics<T, Dof>::StateVector;
        using typename EulerLagrangeDynamics<T, Dof>::MassMatrix;

        MOCK_METHOD(MassMatrix, ComputeMassMatrix, (const StateVector& q), (const, override));
        MOCK_METHOD(StateVector, ComputeCoriolisTerms, (const StateVector& q, const StateVector& qDot), (const, override));
        MOCK_METHOD(StateVector, ComputeGravityTerms, (const StateVector& q), (const, override));
    };
}
