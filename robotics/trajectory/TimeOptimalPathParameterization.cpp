#include "robotics/trajectory/TimeOptimalPathParameterization.hpp"

namespace trajectory
{
    template class InverseDynamicsPathConstraints<float, 2>;
    template class TimeOptimalPathParameterization<float, 2, 64>;
}
