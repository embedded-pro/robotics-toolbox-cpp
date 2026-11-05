#include "robotics/kinematics/DenavitHartenberg.hpp"
#include "robotics/kinematics/DhTaskJacobian.hpp"

namespace kinematics
{
    template class DenavitHartenberg<float, 1>;
    template class DenavitHartenberg<float, 2>;
    template class DenavitHartenberg<float, 6>;
    template class DhTaskJacobian<float, 6, 6>;
}
