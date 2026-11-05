#include "robotics/kinematics/RedundancyResolution.hpp"

namespace kinematics
{
    template class RedundancyResolution<float, 6, 7>;
    template class RedundancyResolution<float, 3, 7>;
}
