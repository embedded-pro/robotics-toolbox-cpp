#include "robotics/dynamics/ForwardDynamicsIntegrator.hpp"

namespace dynamics
{
    template class ChainAccelerationSystem<float, 1>;
    template class ChainAccelerationSystem<float, 2>;
    template class ForwardDynamicsIntegrator<float, 1, IntegrationMethod::RungeKutta4>;
    template class ForwardDynamicsIntegrator<float, 2, IntegrationMethod::SemiImplicitEuler>;
}
