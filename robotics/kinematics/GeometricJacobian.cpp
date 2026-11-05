#include "robotics/kinematics/GeometricJacobian.hpp"
#include "robotics/kinematics/ChainTaskJacobian.hpp"

namespace kinematics
{
    template class GeometricJacobian<float, 1>;
    template class GeometricJacobian<float, 2>;
    template class GeometricJacobian<float, 3>;
    template class GeometricJacobian<float, 6>;
    template class ChainTaskJacobian<float, 3, 2>;
    template class ChainTaskJacobian<float, 6, 2>;
}
